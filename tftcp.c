// Userspace PiTFT display handler.
// Must be run as root (e.g. sudo ./tftcp) because hardware.
// This is a replacement for the PiTFT overlay + fbcp combo.
// WILL NOT WORK if PiTFT overlay is enabled!
// Use touchmouse.py as a substitute touchscreen driver.

// Written by Phil Burgess / Paint Your Dragon for Adafruit Industries.
// MIT license. Some insights came from Tasanakorn's fbcp utility:
// https://github.com/tasanakorn/rpi-fbcp

#include <stdio.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <sys/ioctl.h>
#include <linux/spi/spidev.h>
#include <bcm_host.h>

// Shit. TFT reset is done through STMPE GPIO :/
// Will need to add a bunch of spidev1 calls for that.

#define ILI9341_TFTWIDTH   240
#define ILI9341_TFTHEIGHT  320

#define ILI9341_NOP        0x00
#define ILI9341_SWRESET    0x01
#define ILI9341_RDDID      0x04
#define ILI9341_RDDST      0x09

#define ILI9341_SLPIN      0x10
#define ILI9341_SLPOUT     0x11
#define ILI9341_PTLON      0x12
#define ILI9341_NORON      0x13

#define ILI9341_RDMODE     0x0A
#define ILI9341_RDMADCTL   0x0B
#define ILI9341_RDPIXFMT   0x0C
#define ILI9341_RDIMGFMT   0x0D
#define ILI9341_RDSELFDIAG 0x0F

#define ILI9341_INVOFF     0x20
#define ILI9341_INVON      0x21
#define ILI9341_GAMMASET   0x26
#define ILI9341_DISPOFF    0x28
#define ILI9341_DISPON     0x29

#define ILI9341_CASET      0x2A
#define ILI9341_PASET      0x2B
#define ILI9341_RAMWR      0x2C
#define ILI9341_RAMRD      0x2E

#define ILI9341_PTLAR      0x30
#define ILI9341_MADCTL     0x36
#define ILI9341_VSCRSADD   0x37
#define ILI9341_PIXFMT     0x3A

#define ILI9341_FRMCTR1    0xB1
#define ILI9341_FRMCTR2    0xB2
#define ILI9341_FRMCTR3    0xB3
#define ILI9341_INVCTR     0xB4
#define ILI9341_DFUNCTR    0xB6

#define ILI9341_PWCTR1     0xC0
#define ILI9341_PWCTR2     0xC1
#define ILI9341_PWCTR3     0xC2
#define ILI9341_PWCTR4     0xC3
#define ILI9341_PWCTR5     0xC4
#define ILI9341_VMCTR1     0xC5
#define ILI9341_VMCTR2     0xC7

#define ILI9341_RDID1      0xDA
#define ILI9341_RDID2      0xDB
#define ILI9341_RDID3      0xDC
#define ILI9341_RDID4      0xDD

#define ILI9341_GMCTRP1    0xE0
#define ILI9341_GMCTRN1    0xE1

#define MADCTL_MY  0x80
#define MADCTL_MX  0x40
#define MADCTL_MV  0x20
#define MADCTL_ML  0x10
#define MADCTL_RGB 0x00
#define MADCTL_BGR 0x08
#define MADCTL_MH  0x04


// CONFIGURATION AND GLOBAL STUFF ------------------------------------------

#define WIDTH     320
#define HEIGHT    240
#define DC_PIN    25
#define BITRATE   80000000
#define SPI_MODE  SPI_MODE_0
#define FPS       30

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
 cmd = { // SPI transfer structure used when issuing commands to OLED
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
			   (n == 0x3F000000)) {
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

// Issue command (not data) to OLED
static void writeCommands(uint8_t *c, uint8_t len) {
	*gpioClr   = dcMask; // 0/low = command, 1/high = data
	cmd.tx_buf = (unsigned long)c;
	cmd.len    = len;
	(void)ioctl(fdSPI0, SPI_IOC_MESSAGE(1), &cmd);
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

static void SPI_WRITE32(uint32_t value) {
	uint8_t foo[4];
	*gpioSet = dcMask; // 0/low = command, 1/high = data
	foo[0] = value >> 24; // MSB
	foo[1] = value >> 16;
	foo[2] = value >>  8;
	foo[3] = value;       // LSB
	cmd.tx_buf = (unsigned long)&foo;
	cmd.len    = 4;
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
	// contents to this resource. It's 4X the OLED pixel dimensions
	// (ideally the framebuffer resolution will match this) to allow
	// for quality downsampling (see notes in code below).
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
//	resetMask = 1 << RESET_PIN;
	dcMask    = 1 << DC_PIN;

	if((fdSPI0 = open("/dev/spidev0.0", O_WRONLY | O_NONBLOCK)) < 0) {
		return err(5, "Can't open /dev/spidev0.0 (try 'sudo')\n");
	}
	uint8_t mode = SPI_MODE;
	ioctl(fdSPI0, SPI_IOC_WR_MODE, &mode);
	ioctl(fdSPI0, SPI_IOC_WR_MAX_SPEED_HZ, BITRATE);

	// Set 2 pins as outputs.  Must use INP before OUT.
//	INP_GPIO(RESET_PIN); OUT_GPIO(RESET_PIN);
	INP_GPIO(DC_PIN)   ; OUT_GPIO(DC_PIN);

//	*gpioSet = resetMask; usleep(50); // Reset high,
//	*gpioClr = resetMask; usleep(50); // low,
//	*gpioSet = resetMask; usleep(50); // high

	// Initialize OLED
//	writeCommands(initCommands, sizeof(initCommands));

	writeCommand(0xEF);
	spiWrite(0x03);
	spiWrite(0x80);
	spiWrite(0x02);

	writeCommand(0xCF);
	spiWrite(0x00);
	spiWrite(0XC1);
	spiWrite(0X30);

	writeCommand(0xED);
	spiWrite(0x64);
	spiWrite(0x03);
	spiWrite(0X12);
	spiWrite(0X81);

	writeCommand(0xE8);
	spiWrite(0x85);
	spiWrite(0x00);
	spiWrite(0x78);

	writeCommand(0xCB);
	spiWrite(0x39);
	spiWrite(0x2C);
	spiWrite(0x00);
	spiWrite(0x34);
	spiWrite(0x02);

	writeCommand(0xF7);
	spiWrite(0x20);

	writeCommand(0xEA);
	spiWrite(0x00);
	spiWrite(0x00);

	writeCommand(ILI9341_PWCTR1);    //Power control
	spiWrite(0x23);   //VRH[5:0]

	writeCommand(ILI9341_PWCTR2);    //Power control
	spiWrite(0x10);   //SAP[2:0];BT[3:0]

	writeCommand(ILI9341_VMCTR1);    //VCM control
	spiWrite(0x3e);
	spiWrite(0x28);

	writeCommand(ILI9341_VMCTR2);    //VCM control2
	spiWrite(0x86);  //--

	writeCommand(ILI9341_MADCTL);    // Memory Access Control
	spiWrite(MADCTL_MV | MADCTL_BGR);

	writeCommand(ILI9341_VSCRSADD); // Vertical scroll
	SPI_WRITE16(0);                 // Zero

	writeCommand(ILI9341_PIXFMT);
	spiWrite(0x55);

	writeCommand(ILI9341_FRMCTR1);
	spiWrite(0x00);
	spiWrite(0x18);

	writeCommand(ILI9341_DFUNCTR);    // Display Function Control
	spiWrite(0x08);
	spiWrite(0x82);
	spiWrite(0x27);

	writeCommand(0xF2);    // 3Gamma Function Disable
	spiWrite(0x00);

	writeCommand(ILI9341_GAMMASET);    //Gamma curve selected
	spiWrite(0x01);

	writeCommand(ILI9341_GMCTRP1);    //Set Gamma
	spiWrite(0x0F);
	spiWrite(0x31);
	spiWrite(0x2B);
	spiWrite(0x0C);
	spiWrite(0x0E);
	spiWrite(0x08);
	spiWrite(0x4E);
	spiWrite(0xF1);
	spiWrite(0x37);
	spiWrite(0x07);
	spiWrite(0x10);
	spiWrite(0x03);
	spiWrite(0x0E);
	spiWrite(0x09);
	spiWrite(0x00);

	writeCommand(ILI9341_GMCTRN1);    //Set Gamma
	spiWrite(0x00);
	spiWrite(0x0E);
	spiWrite(0x14);
	spiWrite(0x03);
	spiWrite(0x11);
	spiWrite(0x07);
	spiWrite(0x31);
	spiWrite(0xC1);
	spiWrite(0x48);
	spiWrite(0x08);
	spiWrite(0x0F);
	spiWrite(0x0C);
	spiWrite(0x31);
	spiWrite(0x36);
	spiWrite(0x0F);

	writeCommand(ILI9341_SLPOUT);    //Exit Sleep
	usleep(120000);
	writeCommand(ILI9341_DISPON);    //Display on
	usleep(120000);

	// Get SPI buffer size from /sys/module/spidev/parameters/bufsiz
	// Default is 4096.  For lower CPU load, add a new parameter to
	// /boot/cmdline.txt: spidev.bufsiz=65536

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

		// Framebuffer -> intermediary
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
		writeCommand(ILI9341_CASET); // Column addr set
		SPI_WRITE32(WIDTH-1);
		writeCommand(ILI9341_PASET); // Row addr set
		SPI_WRITE32(HEIGHT-1);
		writeCommand(ILI9341_RAMWR); // write to RAM

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

