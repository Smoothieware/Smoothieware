# Copyright 2011 Adam Green (http://mbed.org/users/AdamGreen/)
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
PROJECT=main
GCC4MBED_DIR=./gcc4mbed
INCDIRS=
LIBS_PREFIX=
LIBS_SUFFIX=
SRC=./src

#GCC4MBED_TYPE=Debug
#GCC4MBED_TYPE=Release
GCC4MBED_TYPE=Checked

# Set to 1 to tag each heap allocation with the caller's return address.
HEAP_TAGS=1

# Set to 1 configure MPU to disable write buffering and eliminate imprecise bus faults.
WRITE_BUFFER_DISABLE=1

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

CONSOLE=/dev/ttyUSB0

include ./gcc4mbed/build/gcc4mbed.mk

.PHONY: debug-store flash upload debug console

debug-store: $(PROJECT).elf
	cp $(PROJECT).elf $(PROJECT)_lastupload.elf

flash: $(PROJECT).hex debug-store
	lpc21isp $< $(CONSOLE) 115200 12000

upload: $(PROJECT).bin debug-store
	dfu-util -d 1d50:6015 -R -D $<

debug: $(PROJECT)_lastupload.elf
	arm-none-eabi-gdb $< -ex  "set target-charset ASCII" -ex "set remotelogfile mri.log" -ex "target remote $(CONSOLE)"

console:
	stty raw ignbrk -echo 2000000 < $(CONSOLE)
	( cat <&3 & cat >&3 ; kill %% ) 3<>$(CONSOLE)
