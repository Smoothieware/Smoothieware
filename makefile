#!/usr/bin/make

DIRS = mbed src
DIRSCLEAN = $(addsuffix .clean,$(DIRS))

all:
	@ $(MAKE) -C mbed
	@echo Building Smoothie
	@ $(MAKE) -C src

clean: $(DIRSCLEAN)

$(DIRSCLEAN): %.clean:
	@echo Cleaning $*
	@ $(MAKE) -C $*  clean

debug-store:
	@ $(MAKE) -C src debug-store

flash:
	@ $(MAKE) -C src flash

dfu:
	@ $(MAKE) -C src dfu

upload:
	@ $(MAKE) -C src upload

debug:
	@ $(MAKE) -C src debug

console:
	@ $(MAKE) -C src console

opentrons:
	make all AXIS=6 PAXIS=4 CNC=1 DISABLEMSD=1
	./build/osx64/lpc21isp -wipe -donotstart ./LPC1768/main.hex /dev/tty.usbserial-AL0158RY 115200 12000

.PHONY: all $(DIRS) $(DIRSCLEAN) debug-store flash upload debug console dfu
