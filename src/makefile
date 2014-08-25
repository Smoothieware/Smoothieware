PROJECT=main
BUILD_DIR=../build
DEVICES=lpc1768

# BUILD_TYPE can be set to the following values:
#  Checked - Optimizations enabled with MRI debug monitor support. (Recommended Type)
#  Release - Optimizations enabled.
#  Debug - Optimization disabled with MRI debug monitor support.
BUILD_TYPE=Checked

# Set to 1 to tag each heap allocation with the caller's return address.
# NOTE: Can't be enabled with latest build as not compatible with newlib nano.
HEAP_TAGS=0

# Set to 1 configure MPU to disable write buffering and eliminate imprecise bus faults.
WRITE_BUFFER_DISABLE=0

# Set to non zero value if you want checks to be enabled which reserve a
# specific amount of space for the stack.  The heap's growth will be
# constrained to reserve this much space for the stack and the stack won't be
# able to grow larger than this amount.
STACK_SIZE=3072

# Set to 1 to allow MRI debug monitor to take full control of UART0 and use it
# as a dedicated debug channel.  If you are using the USB based serial port for
# the console then this should cause you no problems.  Set MRI_BREAK_ON_INIT to
# 0 if you don't want to break into GDB at startup.
ENABLE_DEBUG_MONITOR?=0

# this is the default UART baud rate used if it is not set in config
# it is also the baud rate used to report any errors found while parsing the config file
DEFAULT_SERIAL_BAUD_RATE?=9600

ifeq "$(ENABLE_DEBUG_MONITOR)" "1"
# Can add MRI_UART_BAUD=115200 to next line if GDB fails to connect to MRI.
# Tends to happen on some Linux distros but not Windows and OS X.
MRI_UART=MRI_UART_0
MRI_BREAK_ON_INIT=1
MRI_SEMIHOST_STDIO=1
else
MRI_UART=MRI_UART_0 MRI_UART_SHARE
MRI_BREAK_ON_INIT=0
MRI_SEMIHOST_STDIO=0
endif

# generate a git version string, sorry only on linux and mac osx
ifneq "$(OS)" "Windows_NT"
DEFINES += -D__GITVERSIONSTRING__=\"$(shell ./generate-version.sh)\"
else
DEFINES += -D__GITVERSIONSTRING__=\"$(shell generate-version.bat)\"
endif

# use c++11 features for the checksums and set default baud rate for serial uart
DEFINES += -DCHECKSUM_USE_CPP -DDEFAULT_SERIAL_BAUD_RATE=$(DEFAULT_SERIAL_BAUD_RATE)

# add any modules that you do not want included in the build
export EXCLUDED_MODULES = tools/touchprobe
# e.g for a CNC machine
#export EXCLUDED_MODULES = tools/touchprobe tools/laser tools/temperaturecontrol tools/extruder

# set to not compile in any network support
#export NONETWORK = 1

include $(BUILD_DIR)/build.mk

CONSOLE?=/dev/arduino
BAUD?=1000000

.PHONY: debug-store flash upload debug console dfu

debug-store: ../LPC1768/$(PROJECT).elf
	cp ../LPC1768/$(PROJECT).elf ../LPC1768/$(PROJECT)_lastupload.elf

flash: ../LPC1768/$(PROJECT).hex debug-store
	lpc21isp $< $(CONSOLE) 115200 12000

dfu: ../LPC1768/$(PROJECT).bin
	dfu-util -D $< -R

upload: ../LPC1768/$(PROJECT).bin debug-store
	dfu-util -R -d 1d50:6015 -D $<

debug: ../LPC1768/$(PROJECT)_lastupload.elf
	arm-none-eabi-gdb $< -ex  "set target-charset ASCII" -ex "set remotelogfile mri.log" -ex "target remote $(CONSOLE)"

console:
	stty raw ignbrk -echo $(BAUD) < $(CONSOLE)
	( cat <&3 & cat >&3 ; kill %% ) 3<>$(CONSOLE)
