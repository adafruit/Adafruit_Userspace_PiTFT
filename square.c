// Userspace ST7789 display handler for 240x240 pixel display.
// Must be run as root (e.g. sudo ./square) because hardware.

// Stuff that's REQUIRED in /boot/config.txt:
// disable_overscan=1
// hdmi_force_hotplug=1
// hdmi_group=2
// hdmi_mode=87
// hdmi_cvt=480 480 60 1 0 0 0
// dtparam=spi=on

// NOT ALL HDMI MONITORS SUPPORT THIS RESOLUTION. THIS IS OK. You might
// see a "no signal" or "resolution not supported" message, but the Pi is
// in fact generating the HDMI signal and all is well, you just can't see
// what it's doing. The framebuffer is there, intact and operating at
// 480x480 resolution, and this is what the 'square' utility wants.
// (2x2 downsampling is performed, this is why 480x480 vs 240x240)

// RECOMMENDED: To enable smooth interp in MAME,
// edit /opt/retropie/configs/all/retroarch.cfg:
// video_smooth = "true"
// (This line is present but set to "false" by default)

// Written by Phil Burgess / Paint Your Dragon for Adafruit Industries.
// MIT license. Some insights came from Tasanakorn's fbcp utility:
// https://github.com/tasanakorn/rpi-fbcp

#include <stdio.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <sys/ioctl.h>
#include <linux/spi/spidev.h>
#include <bcm_host.h>


// CONFIGURATION AND GLOBAL STUFF ------------------------------------------

#define WIDTH     240
#define HEIGHT    240
#define RESET_PIN 5
#define DC_PIN    6
#define BITRATE   96000000 // 96 MHz = OK
#define SPI_MODE  SPI_MODE_0
#define FPS       30

#define ST77XX_SWRESET        0x01
#define ST77XX_SLPOUT         0x11
#define ST77XX_NORON          0x13
#define ST77XX_INVON          0x21
#define ST77XX_GAMSET         0x26
#define ST77XX_DISPON         0x29
#define ST77XX_CASET          0x2A
#define ST77XX_RASET          0x2B
#define ST77XX_RAMWR          0x2C
#define ST77XX_COLMOD         0x3A
#define ST77XX_MADCTL         0x36
#define ST77XX_DGMEN          0xBA
#define ST77XX_MADCTL_MY      0x80
#define ST77XX_MADCTL_MX      0x40
#define ST77XX_MADCTL_MV      0x20
#define ST77XX_MADCTL_ML      0x10
#define ST77XX_MADCTL_RGB     0x00
#define ST7789_240x240_XSTART 0
#define ST7789_240x240_YSTART 0

// From GPIO example code by Dom and Gert van Loo on elinux.org:
#define PI1_BCM2708_PERI_BASE 0x20000000
#define PI1_GPIO_BASE         (PI1_BCM2708_PERI_BASE + 0x200000)
#define PI2_BCM2708_PERI_BASE 0x3F000000
#define PI2_GPIO_BASE         (PI2_BCM2708_PERI_BASE + 0x200000)
#define BLOCK_SIZE            (4*1024)
#define INP_GPIO(g)          *(gpio+((g)/10)) &= ~(7<<(((g)%10)*3))
#define OUT_GPIO(g)          *(gpio+((g)/10)) |=  (1<<(((g)%10)*3))

static volatile unsigned
  *gpio = NULL, // Memory-mapped GPIO peripheral
  *gpioSet,     // Write bitmask of GPIO pins to set here
  *gpioClr;     // Write bitmask of GPIO pins to clear here

uint32_t resetMask, dcMask; // GPIO pin bitmasks
int      fdSPI0;            // /dev/spidev0.0 file descriptor

static struct spi_ioc_transfer
 cmd = { // SPI transfer structure used when issuing commands to screen
  .rx_buf        = 0,
  .delay_usecs   = 0,
  .bits_per_word = 8,
  .pad           = 0,
  .speed_hz      = BITRATE,
  .tx_nbits      = 0,
  .rx_nbits      = 0,
  .cs_change     = 0 },
 dat = { // SPI transfer structure used when issuing data
  .rx_buf        = 0,
  .delay_usecs   = 0,
  .bits_per_word = 8,
  .len           = 4096,
  .pad           = 0,
  .speed_hz      = BITRATE,
  .tx_nbits      = 0,
  .rx_nbits      = 0,
  .cs_change     = 0 };

// UTILITY FUNCTIONS -------------------------------------------------------

// Detect Pi board type.  Doesn't return super-granular details,
// just the most basic distinction needed for GPIO compatibility:
// 0: Pi 1 Model B revision 1
// 1: Pi 1 Model B revision 2, Model A, Model B+, Model A+
// 2: Pi 2 Model B
static int boardType(void) {
	FILE *fp;
	char  buf[1024], *ptr;
	int   n, board = 1; // Assume Pi1 Rev2 by default

	// Relies on info in /proc/cmdline.  If this becomes unreliable
	// in the future, alt code below uses /proc/cpuinfo if any better.
#if 1
	if((fp = fopen("/proc/cmdline", "r"))) {
		while(fgets(buf, sizeof(buf), fp)) {
			if((ptr = strstr(buf, "mem_size=")) &&
			   (sscanf(&ptr[9], "%x", &n) == 1) &&
			   ((n == 0x3F000000) || (n == 0x40000000))) {
				board = 2; // Appears to be a Pi 2
				break;
			} else if((ptr = strstr(buf, "boardrev=")) &&
			          (sscanf(&ptr[9], "%x", &n) == 1) &&
			          ((n == 0x02) || (n == 0x03))) {
				board = 0; // Appears to be an early Pi
				break;
			}
		}
		fclose(fp);
	}
#else
	char s[8];
	if((fp = fopen("/proc/cpuinfo", "r"))) {
		while(fgets(buf, sizeof(buf), fp)) {
			if((ptr = strstr(buf, "Hardware")) &&
			   (sscanf(&ptr[8], " : %7s", s) == 1) &&
			   (!strcmp(s, "BCM2709"))) {
				board = 2; // Appears to be a Pi 2
				break;
			} else if((ptr = strstr(buf, "Revision")) &&
			          (sscanf(&ptr[8], " : %x", &n) == 1) &&
			          ((n == 0x02) || (n == 0x03))) {
				board = 0; // Appears to be an early Pi
				break;
			}
		}
		fclose(fp);
	}
#endif

	return board;
}

// Crude error 'handler' (prints message, returns same code as passed in)
static int err(int code, char *string) {
	(void)puts(string);
	return code;
}

static void spiWrite(uint8_t value) {
	*gpioSet = dcMask; // 0/low = command, 1/high = data
	cmd.tx_buf = (unsigned long)&value;
	cmd.len    = 1;
	(void)ioctl(fdSPI0, SPI_IOC_MESSAGE(1), &cmd);
}

static void SPI_WRITE16(uint16_t value) {
	uint8_t foo[2];
	*gpioSet = dcMask; // 0/low = command, 1/high = data
	foo[0] = value >> 8; // MSB
	foo[1] = value;      // LSB
	cmd.tx_buf = (unsigned long)&foo;
	cmd.len    = 2;
	(void)ioctl(fdSPI0, SPI_IOC_MESSAGE(1), &cmd);
}

static void writeCommand(uint8_t c) {
	*gpioClr = dcMask; // 0/low = command, 1/high = data
	cmd.tx_buf = (unsigned long)&c;
	cmd.len    = 1;
	(void)ioctl(fdSPI0, SPI_IOC_MESSAGE(1), &cmd);
}


// MAIN CODE ---------------------------------------------------------------

int main(int argc, char *argv[]) {

	uint16_t pixelBuf[WIDTH * HEIGHT]; // 16-bit pixel buffer
	uint8_t  isPi2 = 0;

	// DISPMANX INIT ---------------------------------------------------

	bcm_host_init();

	DISPMANX_DISPLAY_HANDLE_T  display; // Primary framebuffer display
	DISPMANX_RESOURCE_HANDLE_T screen_resource; // Intermediary buf
	uint32_t                   handle;
	VC_RECT_T                  rect;

	if(!(display = vc_dispmanx_display_open(0))) {
		return err(1, "Can't open primary display");
	}

	// screen_resource is an intermediary between framebuffer and
	// main RAM -- VideoCore will copy the primary framebuffer
	// contents to this resource while providing interpolated
	// scaling plus 8/8/8 -> 5/6/5 dithering.
	if(!(screen_resource = vc_dispmanx_resource_create(
	  VC_IMAGE_RGB565, WIDTH, HEIGHT, &handle))) {
		vc_dispmanx_display_close(display);
		return err(2, "Can't create screen buffer");
	}
	vc_dispmanx_rect_set(&rect, 0, 0, WIDTH, HEIGHT);

	// GPIO AND OLED SCREEN INIT ---------------------------------------

	int fd;
	if((fd = open("/dev/mem", O_RDWR | O_SYNC)) < 0) {
		return err(3, "Can't open /dev/mem (try 'sudo')\n");
	}
	isPi2 = (boardType() == 2);
	gpio  = (volatile unsigned *)mmap( // Memory-map I/O
	  NULL,                 // Any adddress will do
	  BLOCK_SIZE,           // Mapped block length
	  PROT_READ|PROT_WRITE, // Enable read+write
	  MAP_SHARED,           // Shared w/other processes
	  fd,                   // File to map
	  isPi2 ?
	   PI2_GPIO_BASE :      // -> GPIO registers
	   PI1_GPIO_BASE);
	close(fd);              // Not needed after mmap()
	if(gpio == MAP_FAILED) {
		return err(4, "Can't mmap()");
	}
	gpioSet   = &gpio[7];
	gpioClr   = &gpio[10];
	resetMask = 1 << RESET_PIN;
	dcMask    = 1 << DC_PIN;

	if((fdSPI0 = open("/dev/spidev0.0", O_WRONLY | O_NONBLOCK)) < 0) {
		return err(5, "Can't open /dev/spidev0.0 (try 'sudo')\n");
	}
	uint8_t mode = SPI_MODE;
	ioctl(fdSPI0, SPI_IOC_WR_MODE, &mode);
	ioctl(fdSPI0, SPI_IOC_WR_MAX_SPEED_HZ, BITRATE);

	// Set 2 pins as outputs.  Must use INP before OUT.
	INP_GPIO(RESET_PIN); OUT_GPIO(RESET_PIN);
	INP_GPIO(DC_PIN)   ; OUT_GPIO(DC_PIN);

	*gpioSet = resetMask; usleep(50); // Reset high,
	*gpioClr = resetMask; usleep(50); // low,
	*gpioSet = resetMask; usleep(50); // high

	// Initialize display
	writeCommand(ST77XX_SWRESET);
	usleep(150000);
	writeCommand(ST77XX_SLPOUT);
	usleep(500000);
	writeCommand(ST77XX_COLMOD);
	spiWrite(0x55);
	usleep(10000);
	writeCommand(ST77XX_MADCTL);
	spiWrite(0x00); // RGB
	writeCommand(ST77XX_GAMSET);
	spiWrite(0x02); // Gamma curve 2 (G1.8)
	writeCommand(ST77XX_DGMEN);
	spiWrite(0x04); // Enable gamma
	writeCommand(ST77XX_INVON);
	usleep(10000);
	writeCommand(ST77XX_NORON);
	usleep(10000);
	writeCommand(ST77XX_DISPON);
	usleep(500000);

	// Get SPI buffer size from /sys/module/spidev/parameters/bufsiz
	// Default is 4096.

	FILE *fp;
	char  buf[32];
	int   n, bufsiz = 4096;

	if((fp = fopen("/sys/module/spidev/parameters/bufsiz", "r"))) {
		if(fscanf(fp, "%d", &n) == 1) bufsiz = n;
		fclose(fp);
	}

	// MAIN LOOP -------------------------------------------------------

	struct timeval tv;
	uint32_t       timeNow, timePrev = 0, timeDelta;

	for(;;) {
		// Throttle transfer to approx FPS frames/sec.
		// usleep() avoids heavy CPU load of time polling.
		gettimeofday(&tv, NULL);
		timeNow = tv.tv_sec * 1000000 + tv.tv_usec;
		timeDelta = timeNow - timePrev;
		if(timeDelta < (1000000 / FPS)) {
			usleep((1000000 / FPS) - timeDelta);
		}
		timePrev = timeNow;

		// Framebuffer -> intermediary (w/scale & 565 dithering)
		vc_dispmanx_snapshot(display, screen_resource, 0);
		// Intermediary -> main RAM
		vc_dispmanx_resource_read_data(screen_resource, &rect,
		  pixelBuf, WIDTH * 2);

		// Before pushing data to SPI screen, column and row
		// ranges are reset every frame to force screen data
		// pointer back to (0,0).  Though the pointer will
		// automatically 'wrap' when the end of the screen is
		// reached, this is extra insurance in case there's
		// a glitch where a byte doesn't get through to the
		// display (which would then be out of sync in all
		// subsequent frames).
		writeCommand(ST77XX_CASET);
		SPI_WRITE16(ST7789_240x240_XSTART);
		SPI_WRITE16(ST7789_240x240_XSTART + WIDTH - 1);
		writeCommand(ST77XX_RASET);
		SPI_WRITE16(ST7789_240x240_YSTART);
		SPI_WRITE16(ST7789_240x240_YSTART + HEIGHT - 1);
		writeCommand(ST77XX_RAMWR);

		*gpioSet = dcMask; // DC high

		// 16-bit conversion is done by GPU, but still need byte swap
		int i;
		for(i=0; i<WIDTH*HEIGHT; i++) {
			pixelBuf[i] = __builtin_bswap16(pixelBuf[i]);
		}

		// Max SPI transfer size is 4096 bytes
		uint32_t bytes_remaining = WIDTH * HEIGHT * 2;
		dat.tx_buf = (uint32_t)pixelBuf;
		while(bytes_remaining > 0) {
			dat.len = (bytes_remaining > bufsiz) ?
			bufsiz : bytes_remaining;
			(void)ioctl(fdSPI0, SPI_IOC_MESSAGE(1), &dat);
			bytes_remaining -= dat.len;
			dat.tx_buf      += dat.len;
		}
	}

	vc_dispmanx_resource_delete(screen_resource);
	vc_dispmanx_display_close(display);
	return 0;
}
