# RAK811-GPS-FORWARDER

You need arm compiler at /usr/bin or change Makefile

use: make 

to build bin

Switch flash jumper

Connect board

use: sudo stm32flash -w ./latest_build/gpsCubeMx.bin -v -g 0x0 -e 255 /dev/ttyUSB0

to flash

Swith flash jumper back

Bridge PB12 to GND

Download u-center from u-box and test

[x] Get data from GPS and send to USB-SERIAL
[x] Get data from USB-SERIAL and send to GPS

I still do not know how to check end of GPS config message
