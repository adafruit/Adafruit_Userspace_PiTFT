#!/usr/bin/python

# Userspace PiTFT touch driver.
# Prerequisites:
# sudo apt-get install python-pip python-smbus python-dev
# sudo pip install evdev
# Run as root (sudo python touchmouse.py).
# Be sure to enable SPI via raspi-config.
# WILL NOT WORK if PiTFT overlay is enabled!
# Use tftcp as a substitude PiTFT driver (also works in userspace).

import os
import time
import RPi.GPIO as gpio
import spidev
from evdev import UInputError, UInput, AbsInfo, ecodes as e
import argparse


# DEFINE WHAT IS A VALID ROTATION
def isValidRotation(r):
	if int(r) not in [0, 90, 180, 270]:
		raise argparse.ArgumentTypeError("%s is not one of [0, 90, 180, 270]" % r)
	return int(r)

# ARGUMENT PARSER ----------------------------------------------------------
parser = argparse.ArgumentParser(description='Touchmouse configurations')
parser.add_argument('--chip', default="STMPE", help='select what chip to use - only STMPE supported at this time')
parser.add_argument('-d', '--debug', action='store_true', help='print debug message')
parser.add_argument('-r', '--rotation', default=90, type=isValidRotation, help='Rotate the touchscreen 0/90/180 or 270 degrees')
parser.add_argument('-c', '--calibration', default=[500, 300, 3600, 3800], nargs=4, type=int, help='Set the 4-part calibration')
parser.add_argument('-o', '--outrange', default=[0, 0, 4095, 4095],  nargs=4, type=int, help='Set the 4-par output range')
args = parser.parse_args()
print(args)

# UINPUT INIT --------------------------------------------------------------

#os.system("sudo modprobe uinput")

EVENT_X_MAX              = args.outrange[2]
EVENT_Y_MAX              = args.outrange[3]

events = {
  e.EV_KEY : [e.BTN_TOUCH],
  e.EV_ABS : [
   (e.ABS_X, AbsInfo(value=0, min=0, max=EVENT_X_MAX, fuzz=0, flat=0, resolution=0)),
   (e.ABS_Y, AbsInfo(value=0, min=0, max=EVENT_Y_MAX, fuzz=0, flat=0, resolution=0)),
   (e.ABS_PRESSURE, AbsInfo(value=0, min=0, max=255, fuzz=0, flat=0, resolution=0))
   ]
}
try:
  ui = UInput(events, name="touchmouse", bustype=e.BUS_USB)
except UInputError:
  print("Couldn't create uinput - make sure you are running with sudo!")
  exit(0)

# GPIO INIT ----------------------------------------------------------------

irqPin = 24

gpio.setwarnings(False)
gpio.setmode(gpio.BCM)

# Enable pullup on IRQ pin
gpio.setup(irqPin, gpio.IN, pull_up_down=gpio.PUD_UP)

# SPI INIT -----------------------------------------------------------------

spi = spidev.SpiDev()
spi.open(0, 1)
spi.bits_per_word = 8
spi.cshigh        = False
spi.max_speed_hz  = 1000000
spi.mode          = 1

# STMPE610 INIT ------------------------------------------------------------

STMPE_SYS_CTRL1          = 0x03
STMPE_SYS_CTRL2          = 0x04
STMPE_INT_CTRL           = 0x09
STMPE_INT_EN             = 0x0A
STMPE_INT_STA            = 0x0B
STMPE_ADC_CTRL1          = 0x20
STMPE_ADC_CTRL2          = 0x21
STMPE_TSC_CTRL           = 0x40
STMPE_TSC_CFG            = 0x41
STMPE_FIFO_TH            = 0x4A
STMPE_FIFO_STA           = 0x4B
STMPE_TSC_FRACTION_Z     = 0x56
STMPE_TSC_I_DRIVE        = 0x58

STMPE_SYS_CTRL1_RESET    = 0x02
STMPE_INT_CTRL_POL_HIGH  = 0x04
STMPE_INT_CTRL_EDGE      = 0x02
STMPE_INT_CTRL_ENABLE    = 0x01
STMPE_INT_EN_TOUCHDET    = 0x01
STMPE_ADC_CTRL1_10BIT    = 0x00
STMPE_ADC_CTRL2_6_5MHZ   = 0x02
STMPE_TSC_CTRL_EN        = 0x01
STMPE_TSC_CTRL_XYZ       = 0x00
STMPE_TSC_CFG_4SAMPLE    = 0x80
STMPE_TSC_CFG_8SAMPLE    = 0xC0
STMPE_TSC_CFG_DELAY_1MS  = 0x20
STMPE_TSC_CFG_SETTLE_1MS = 0x03
STMPE_TSC_CFG_SETTLE_5MS = 0x04
STMPE_FIFO_STA_RESET     = 0x01
STMPE_TSC_I_DRIVE_50MA   = 0x01

CALIBRATION_MIN_X        = 0
CALIBRATION_MIN_Y        = 1
CALIBRATION_MAX_X        = 2
CALIBRATION_MAX_Y        = 3

def writeRegister8(addr, value):
	spi.xfer2([addr, value])

def readRegister8(addr):
	return spi.xfer2([0x80 + addr, 0])[1]

def readRegister16(addr):
	return ((spi.xfer2([0x80 + addr, 0])[1] << 8) +
			 spi.xfer2([0x81 + addr, 0])[1])

id = readRegister16(0)
if id != 0x811:
	spi.mode = 0
	id = readRegister16(0)

if args.debug:
	print("Found Chip ID: 0x%x" % id)
if id != 0x811:
	print("Not a valid touch chip I know! Check your wiring")
	exit
if args.debug:
	print("Recognized touch chip. Let's go!")

# Issue soft reset
writeRegister8(STMPE_SYS_CTRL1, STMPE_SYS_CTRL1_RESET)
time.sleep(0.01)

# Initialize
writeRegister8(STMPE_SYS_CTRL2, 0x0) # turn on clocks!
writeRegister8(STMPE_TSC_CTRL, STMPE_TSC_CTRL_XYZ | STMPE_TSC_CTRL_EN) # XYZ and enable!
#Serial.println(readRegister8(STMPE_TSC_CTRL), HEX)
writeRegister8(STMPE_ADC_CTRL1, STMPE_ADC_CTRL1_10BIT | (0x6 << 4)) # 96 clocks per conversion
writeRegister8(STMPE_ADC_CTRL2, STMPE_ADC_CTRL2_6_5MHZ)
writeRegister8(STMPE_TSC_CFG, STMPE_TSC_CFG_8SAMPLE | STMPE_TSC_CFG_DELAY_1MS | STMPE_TSC_CFG_SETTLE_1MS)
writeRegister8(STMPE_TSC_FRACTION_Z, 0x6)
writeRegister8(STMPE_FIFO_TH, 1)
writeRegister8(STMPE_FIFO_STA, STMPE_FIFO_STA_RESET)
writeRegister8(STMPE_FIFO_STA, 0)    # unreset
writeRegister8(STMPE_TSC_I_DRIVE, STMPE_TSC_I_DRIVE_50MA)
writeRegister8(STMPE_INT_CTRL, STMPE_INT_CTRL_EDGE | STMPE_INT_CTRL_ENABLE)
writeRegister8(STMPE_INT_EN, STMPE_INT_EN_TOUCHDET)
writeRegister8(STMPE_INT_STA, 0xFF) # reset all ints

# MAIN LOOP ----------------------------------------------------------------

foo = [ 0,0,0,0 ]

while True:
	r = gpio.wait_for_edge(irqPin, gpio.FALLING, timeout=1)
	if r is None:     # no edge detected
		continue
	
	blat = True
	while readRegister8(STMPE_TSC_CTRL) & 0x80:

		# Allow conversion time before 1st reading,
		# also prevents heavy CPU load; 40 fps is ample
		time.sleep(1.0 / 30.0) # 30 FPS avoids glitches

		for i in range(4):
			foo[i] = spi.xfer2([0xD7, 0])[1]

		x = ( foo[0]         << 4) | (foo[1] >> 4)
		y = ((foo[1] & 0x0F) << 8) |  foo[2]
		z =  foo[3]

		#Configure the rotation, if args.rotation is not in this set, it is assumed to be 0
		xFrac = float(x - args.calibration[CALIBRATION_MIN_X]) / (args.calibration[CALIBRATION_MAX_X] - args.calibration[CALIBRATION_MIN_X])
		yFrac = float(y - args.calibration[CALIBRATION_MIN_Y]) / (args.calibration[CALIBRATION_MAX_Y] - args.calibration[CALIBRATION_MIN_Y])

		if args.rotation == 90:
			t = xFrac
			xFrac = yFrac
			yFrac = 1.0 - t
		elif args.rotation == 180:
			xFrac = 1.0 - xFrac
			yFrac = 1.0 - yFrac
		if args.rotation == 270:
			t = xFrac
			xFrac = 1.0 - yFrac
			yFrac = t

			if args.debug:
				print("{:04d}({:01.3f}), {:04d}({:01.3f}), {:02d}".format(x, xFrac, y, yFrac, z))

		if z:
			# Convert to screen space
			y1 = int(yFrac * (EVENT_Y_MAX+1))
			y1 = min(max(y1, 0), EVENT_Y_MAX)

			x1 = int(xFrac * (EVENT_X_MAX+1))
			x1 = min(max(x1, 0), EVENT_X_MAX)
			
			ui.write(e.EV_ABS, e.ABS_X, x1)
			ui.write(e.EV_ABS, e.ABS_Y, y1)
			ui.write(e.EV_ABS, e.ABS_PRESSURE, z)
			if blat:
				ui.write(e.EV_KEY, e.BTN_TOUCH, 1)
				blat = False
			ui.syn()
			writeRegister8(STMPE_FIFO_STA, 1) # Clear FIFO
			writeRegister8(STMPE_FIFO_STA, 0)

	ui.write(e.EV_ABS, e.ABS_PRESSURE, 0)
	ui.write(e.EV_KEY, e.BTN_TOUCH, 0)
	ui.syn()

	writeRegister8(STMPE_INT_STA, 0xFF) # Reset interrupts

# Top left:
# 3803 188 81
# Lower right:
# 293 3696 61
# First number is vertical, second is horizontal
# because screen is rotated with respective to 'native' orientation
# (it's a portrait display for phones & such)

