# RAK831-GPS-FORWARDER

You need arm compiler at /usr/bin or change Makefile

use: make to build bin

Switch flash jumper

Connect board

use: sudo stm32flash -w ./build/gpsCubeMx.bin -v -g 0x0 -e 255 /dev/ttyUSB0

To flash

