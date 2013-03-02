#!/usr/bin/make

DEVICES = mbed-lpc1768.mk mbed-lpc11u24.mk
DEVICESCLEAN = $(addsuffix .clean,$(DEVICES))

all: $(DEVICES)

clean: $(DEVICESCLEAN)

$(DEVICES):
	@echo Building $@
	@ $(MAKE) -f $@

$(DEVICESCLEAN): %.clean:
	@echo Cleaning $*
	@ $(MAKE) -f $*  clean

.PHONY: all $(DEVICES) $(DEVICESCLEAN)
