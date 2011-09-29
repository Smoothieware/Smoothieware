#/*Copyright (C) 2011 by Sagar G V
#
#Permission is hereby granted, free of charge, to any person obtaining a copy
#of this software and associated documentation files (the "Software"), to deal
#in the Software without restriction, including without limitation the rights
#to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
#copies of the Software, and to permit persons to whom the Software is
#furnished to do so, subject to the following conditions:
#
#The above copyright notice and this permission notice shall be included in
#all copies or substantial portions of the Software.
#
#THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
#IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
#FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
#AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
#LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
#OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
#THE SOFTWARE.*/
#
# Updates: 
#    Arthur Wolf & Adam Green in 2011 - Updated to work with mbed.


#  Project Name
PROJECT=agutil

# List of sources to be compiled/assembled
CSRCS = $(wildcard *.c)
ASRCS =  $(wildcard *.S)
CPPSRCS = $(wildcard *.cpp)

# List of the objects files to be compiled/assembled
OBJECTS= $(CSRCS:.c=.o) $(ASRCS:.S=.o) $(CPPSRCS:.cpp=.o)

# Include path
INCDIRS = ../../external/mbed ../../external/mbed/LPC1768 ../../external/FATFileSystem

# DEFINEs to be used when building C/C++ code
DEFINES = -DTARGET_LPC1768 

# Maximum optimization level
OPTIMIZATION = 2

#  Compiler Options
GCFLAGS = -O$(OPTIMIZATION) -gdwarf-2 -mcpu=cortex-m3 -mthumb -mthumb-interwork -fshort-wchar -ffunction-sections -fdata-sections -fpromote-loop-indices -Wall -Wextra -Wimplicit -Wcast-align -Wpointer-arith -Wredundant-decls -Wshadow -Wcast-qual -Wcast-align -Wnested-externs --std=gnu99 
GCFLAGS += $(patsubst %,-I%,$(INCDIRS)) -I.
GCFLAGS += $(DEFINES)

GPFLAGS = -O$(OPTIMIZATION) -gdwarf-2 -mcpu=cortex-m3 -mthumb -mthumb-interwork -fshort-wchar -ffunction-sections -fdata-sections -fpromote-loop-indices -Wall -Wextra -Wimplicit -Wcast-align -Wpointer-arith -Wredundant-decls -Wshadow -Wcast-qual -Wcast-align -fno-exceptions
GPFLAGS += $(patsubst %,-I%,$(INCDIRS)) -I.
GPFLAGS += $(DEFINES)

LDFLAGS = -mcpu=cortex-m3 -mthumb -O$(OPTIMIZATION) -Wl,-Map=$(PROJECT).map,--cref,--gc-sections,--no-wchar-size-warning

ASFLAGS = $(LISTING) -mcpu=cortex-m3 -mthumb -x assembler-with-cpp
ASFLAGS += $(patsubst %,-I%,$(INCDIRS)) -I.

#  Compiler/Assembler/Linker Paths
GCC = arm-none-eabi-gcc
GPP = arm-none-eabi-g++
AS = arm-none-eabi-gcc
LD = arm-none-eabi-g++
AR = arm-none-eabi-ar
OBJCOPY = arm-none-eabi-objcopy
OBJDUMP = arm-none-eabi-objdump
SIZE = arm-none-eabi-size
REMOVE = rm

# Switch to cs-rm on Windows.
ifeq "$(MAKE)" "cs-make"
REMOVE = cs-rm
endif

#########################################################################

all:: $(PROJECT).ar

$(PROJECT).ar: $(OBJECTS)
	$(AR) -rc $(PROJECT).ar $(OBJECTS)

clean:
	$(REMOVE) -f $(OBJECTS)
	$(REMOVE) -f $(PROJECT).ar

#########################################################################
#  Default rules to compile .c and .cpp file to .o
#  and assemble .s files to .o

.c.o :
	$(GCC) $(GCFLAGS) -c $< -o $(<:.c=.o)

.cpp.o :
	$(GPP) $(GPFLAGS) -c $< -o $(<:.cpp=.o)

.S.o :
	$(AS) $(ASFLAGS) -c $< -o $(<:.S=.o)

#########################################################################