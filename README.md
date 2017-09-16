# Adafruit_Userspace_PiTFT

Raspberry Pi userspace handlers for PiTFT display and touch. These could be substituted for the current PiTFT device tree overlay and touchscreen driver (which is in the kernel and causes headaches whenver there's a new Raspbian kernel).

This is currently UGLY and EXPERIMENTAL just to test the concept. Works ONLY on the 2.8 inch RESISTIVE PiTFT, and only in a SINGLE ORIENTATION.

tftcp handles the display (as PiTFT + fbcp did before). This is a compiled executable and must be run as root. touchmouse.py handles the touchscreen (converting to mouse events), it's written in Python and also must be run as root. Be sure to ENABLE SPI via raspi-config. This WILL NOT WORK if PiTFT overlay is enabled!

Install prerequisite software:
```
sudo apt-get install python-pip python-smbus python-dev
sudo pip install evdev
```

OPTIONAL: enable large SPI transfers (reducing CPU load) by adding this to /boot/cmdline.txt:
```
spidev.bufsiz=65536
```
EDIT: don't do this. Yes it reduces CPU load, but at the expense of frame rate. Just leave at the default (4096) and a reasonably steady-ish 30-ish FPS is possible.

OPTIONAL: set HDMI output resolution to 320x240 or 640x480 (makes the scaled output look somewhat legible) in /boot/config.txt:
```
disable_overscan=1
hdmi_force_hotplug=1
hdmi_group=2
hdmi_mode=87
hdmi_cvt=640 480 60 1 0 0 0
```

OPTIONAL, for 'portrait' video:
```
    display_rotate=3
```

(For 'square' utility, use 480x480 resolution instead)

(For 'nanoscreen' utility, use 384x256 resolution instead)
