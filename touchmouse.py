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
from evdev import UInput, AbsInfo, ecodes as e

# UINPUT INIT --------------------------------------------------------------

#os.system("sudo modprobe uinput")

events = {
  e.EV_KEY : [e.BTN_LEFT],
  e.EV_ABS : [
   (e.ABS_X, AbsInfo(value=0, min=0, max=1023, fuzz=0, flat=0, resolution=0)),
   (e.ABS_Y, AbsInfo(value=0, min=0, max=1023, fuzz=0, flat=0, resolution=0))]
}
ui = UInput(events, name="touchmouse", bustype=e.BUS_USB)

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
STMPE_TSC_CFG_DELAY_1MS  = 0x20
STMPE_TSC_CFG_SETTLE_5MS = 0x04
STMPE_FIFO_STA_RESET     = 0x01
STMPE_TSC_I_DRIVE_50MA   = 0x01

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

print "ID: ", id

# Issue soft reset
writeRegister8(STMPE_SYS_CTRL1, STMPE_SYS_CTRL1_RESET)
time.sleep(0.01)

# Initialize
writeRegister8(STMPE_SYS_CTRL2, 0x0) # turn on clocks!
writeRegister8(STMPE_TSC_CTRL, STMPE_TSC_CTRL_XYZ | STMPE_TSC_CTRL_EN) # XYZ and enable!
#Serial.println(readRegister8(STMPE_TSC_CTRL), HEX)
writeRegister8(STMPE_ADC_CTRL1, STMPE_ADC_CTRL1_10BIT | (0x6 << 4)) # 96 clocks per conversion
writeRegister8(STMPE_ADC_CTRL2, STMPE_ADC_CTRL2_6_5MHZ)
writeRegister8(STMPE_TSC_CFG, STMPE_TSC_CFG_4SAMPLE | STMPE_TSC_CFG_DELAY_1MS | STMPE_TSC_CFG_SETTLE_5MS)
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
	gpio.wait_for_edge(irqPin, gpio.FALLING)

	blat = True
	while readRegister8(STMPE_TSC_CTRL) & 0x80:

		# Allow conversion time before 1st reading,
		# also prevents heavy CPU load; 40 fps is ample
#		time.sleep(0.025)
		time.sleep(1.0 / 30.0) # 30 FPS avoids glitches

		for i in range(4):
			foo[i] = spi.xfer2([0xD7, 0])[1]

		x = ( foo[0]         << 4) | (foo[1] >> 4)
		y = ((foo[1] & 0x0F) << 8) |  foo[2]
		z =  foo[3]
#		print x, y, z

		if z:
			# Convert to screen space
			x1 = (y - 188) * 1024 / 3508;
			if   x1 > 1023: x1 = 1023;
			elif x1 <    0: x1 = 0;
			y1 = (3803 - x) * 1024 / 3510;
			if   y1 > 1023: y1 = 1023;
			elif y1 <    0: y1 = 0;
			ui.write(e.EV_ABS, e.ABS_X, x1)
			ui.write(e.EV_ABS, e.ABS_Y, y1)
			if blat:
				ui.write(e.EV_KEY, e.BTN_LEFT, 1)
				blat = False
			ui.syn()
			writeRegister8(STMPE_FIFO_STA, 1) # Clear FIFO
			writeRegister8(STMPE_FIFO_STA, 0)

	ui.write(e.EV_KEY, e.BTN_LEFT, 0)
	ui.syn()

	writeRegister8(STMPE_INT_STA, 0xFF) # Reset interrupts

# Top left:
# 3803 188 81
# Lower right:
# 293 3696 61
# First number is vertical, second is horizontal
# because screen is rotated with respective to 'native' orientation
# (it's a portrait display for phones & such)

