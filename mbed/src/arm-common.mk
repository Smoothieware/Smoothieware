# Copyright (C) 2012 - Adam Green (http://mbed.org/users/AdamGreen/)
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.

# Set VERBOSE make variable to 1 to output all tool commands.
VERBOSE?=0
ifeq "$(VERBOSE)" "0"
Q=@
else
Q=
endif


# Directories where source files are found and output files should be placed.
ROOT_DIR              =..
SRC_DIR               =.
VENDOR_SRC            =$(SRC_DIR)/vendor/$(VENDOR)
VENDOR_CAPI_SRC       =$(VENDOR_SRC)/capi
VENDOR_CAPI_DEVICE_SRC=$(VENDOR_CAPI_SRC)/$(DEVICE)
VENDOR_CMSIS_SRC      =$(VENDOR_SRC)/cmsis/$(DEVICE)
VENDOR_CMSIS_GCC_SRC  =$(VENDOR_CMSIS_SRC)/GCC_ARM
MBED_CAPI_SRC         =$(SRC_DIR)/capi
MBED_CPP_SRC          =$(SRC_DIR)/cpp
DROP_DIR              =$(ROOT_DIR)/drop
DEVICE_DROP           =$(DROP_DIR)/$(DEVICE)
RELEASE_DROP          =$(DEVICE_DROP)/GCC_ARM
DEBUG_DROP            =$(DEVICE_DROP)/GCC_ARM/Debug
RELEASE_OBJDIR        =$(SRC_DIR)/Release/$(DEVICE)
DEBUG_OBJDIR          =$(SRC_DIR)/Debug/$(DEVICE)


# Release and Debug target libraries for C and C++ portions of mbed SDK.
LIB_MBED      =libmbed.a
RELEASE_MBED  =$(RELEASE_DROP)/$(LIB_MBED)
DEBUG_MBED    =$(DEBUG_DROP)/$(LIB_MBED)


# Copy linker script to device drop directory.
LD_FILE=$(DEVICE).ld
LD_SRC =$(VENDOR_CMSIS_GCC_SRC)/$(LD_FILE)
LD_DEST=$(RELEASE_DROP)/$(LD_FILE)


# Copy headers to device drop directory.
DEVICE_HEADER_SRCS =$(notdir $(wildcard $(VENDOR_CAPI_DEVICE_SRC)/*.h))
DEVICE_HEADER_SRCS+=$(notdir $(wildcard $(VENDOR_CMSIS_SRC)/*.h))
DEVICE_HEADERS     =$(patsubst %.h,$(DEVICE_DROP)/%.h,$(DEVICE_HEADER_SRCS))


# Build up list of all C, C++, and Assembly Language files to be compiled/assembled.
CAPI_SRCS     =$(wildcard $(VENDOR_CAPI_SRC)/*.c)
CMSIS_SRCS    =$(wildcard $(VENDOR_CMSIS_SRC)/*.c)
CMSIS_ASM_SRCS=$(wildcard $(VENDOR_CMSIS_GCC_SRC)/*.s)
MBED_CAPI_SRCS=$(wildcard $(MBED_CAPI_SRC)/*.c)
MBED_CPP_SRCS =$(wildcard $(MBED_CPP_SRC)/*.cpp)


# Convert list of source files to corresponding list of object files to be generated.
# Debug and Release object files go into separate sub-directories.
CAPI_OBJECTS  =$(patsubst %.c,__Output__/%.o,$(CAPI_SRCS))
CAPI_OBJECTS+=$(patsubst %.c,__Output__/%.o,$(CMSIS_SRCS))
CAPI_OBJECTS+=$(patsubst %.c,__Output__/%.o,$(MBED_CAPI_SRCS))
CAPI_OBJECTS+=$(patsubst %.s,__Output__/%.o,$(CMSIS_ASM_SRCS))

DEBUG_CAPI_OBJECTS  =$(patsubst __Output__%,$(DEBUG_OBJDIR)%,$(CAPI_OBJECTS))
RELEASE_CAPI_OBJECTS=$(patsubst __Output__%,$(RELEASE_OBJDIR)%,$(CAPI_OBJECTS))

MBED_OBJECTS=$(patsubst %.cpp,__Output__/%.o,$(MBED_CPP_SRCS))

DEBUG_MBED_OBJECTS  =$(patsubst __Output__%,$(DEBUG_OBJDIR)%,$(MBED_OBJECTS))
RELEASE_MBED_OBJECTS=$(patsubst __Output__%,$(RELEASE_OBJDIR)%,$(MBED_OBJECTS))

DEBUG_OBJECTS  =$(DEBUG_CAPI_OBJECTS) $(DEBUG_MBED_OBJECTS)
RELEASE_OBJECTS=$(RELEASE_CAPI_OBJECTS) $(RELEASE_MBED_OBJECTS)


# List of the header dependency files, one per object file.
DEBUG_DEPFILES  =$(patsubst %.o,%.d,$(DEBUG_OBJECTS))
RELEASE_DEPFILES=$(patsubst %.o,%.d,$(RELEASE_OBJECTS))


# UNDONE: The mbed is just here to get the code to compile for now.  Another solution is required.
# Include directory list.
INCLUDE_DIRS =$(MBED_CPP_SRC)
INCLUDE_DIRS =$(MBED_CAPI_SRC)
INCLUDE_DIRS+=$(VENDOR_CAPI_DEVICE_SRC)
INCLUDE_DIRS+=$(VENDOR_CMSIS_SRC)


# Optimization levels to be used for Debug and Release versions of the library.
DEBUG_OPTIMIZATION=0
RELEASE_OPTIMIZATION=2


# Compiler flags used to enable creation of header dependency files.
DEP_FLAGS = -MMD -MP


# UNDONE: Add -Werror
# Flags to be used with C/C++ compiler that are shared between Debug and Release builds.
C_FLAGS =-g3 $(DEVICE_C_FLAGS) 
C_FLAGS+=-ffunction-sections -fdata-sections -fno-exceptions -fno-delete-null-pointer-checks
C_FLAGS+=-Wall -Wextra
C_FLAGS+=-Wno-unused-parameter -Wcast-align -Wpointer-arith -Wredundant-decls -Wcast-qual -Wcast-align
C_FLAGS+=$(patsubst %,-I%,$(INCLUDE_DIRS))
C_FLAGS+=-DTARGET_$(DEVICE) -DTOOLCHAIN_GCC_ARM
C_FLAGS+=$(DEP_FLAGS)

CPP_FLAGS:=$(C_FLAGS) -fno-rtti
C_FLAGS +=-std=c99


# Customize C/C++ flags for Debug and Release builds.
DEBUG_C_FLAGS    =$(C_FLAGS) -O$(DEBUG_OPTIMIZATION)
DEBUG_CPP_FLAGS  =$(CPP_FLAGS) -O$(DEBUG_OPTIMIZATION)

RELEASE_C_FLAGS  =$(C_FLAGS) -O$(RELEASE_OPTIMIZATION) -DNDEBUG
RELEASE_CPP_FLAGS=$(CPP_FLAGS) -O$(RELEASE_OPTIMIZATION) -DNDEBUG


# Flags used to assemble assembly languages sources.
AS_FLAGSv=-g3 $(DEVICE_AS_FLAGS) -x assembler-with-cpp
AS_FLAGS+=$(patsubst %,-I%,$(INCDIRS))


# GNU Tools for ARM Embedded filenames.
GCC     =arm-none-eabi-gcc
GPP     =arm-none-eabi-g++
LD      =arm-none-eabi-g++
AR      =arm-none-eabi-ar
OBJCOPY =arm-none-eabi-objcopy
OBJDUMP =arm-none-eabi-objdump
SIZE    =arm-none-eabi-size


# Command line tools that are different between *nix and Windows.
ifeq "$(OS)" "Windows_NT"
SHELL      =cmd.exe
REMOVE     =del /q
REMOVE_DIR =rd /s /q
COPY       =copy
CAT        =type
MKDIR      =mkdir
QUIET      =>nul 2>nul & exit 0
NOSTDOUT   =>nul
else
REMOVE     =rm
REMOVE_DIR =rm -r -f
COPY       =cp
CAT        =cat
MKDIR      =mkdir -p
QUIET      => /dev/null 2>&1 ; exit 0
NOSTDOUT   => /dev/null
endif


# Macro which will convert / to \ on Windows.
ifeq "$(OS)" "Windows_NT"
define convert-slash
$(subst /,\,$1)
endef
else
define convert-slash
$1
endef
endif


#########################################################################
# High level rules for building Debug and Release versions of library.
#########################################################################
.PHONY: all clean

all: $(RELEASE_MBED) $(DEBUG_MBED) $(DEVICE_HEADERS) $(LD_DEST)

$(RELEASE_MBED): $(RELEASE_OBJECTS)
	@echo Linking release library $@
	$(Q) $(MKDIR) $(call convert-slash,$(dir $@)) $(QUIET)
	$(Q) $(AR) -rc $@ $?

$(DEBUG_MBED): $(DEBUG_OBJECTS)
	@echo Linking debug library $@
	$(Q) $(MKDIR) $(call convert-slash,$(dir $@)) $(QUIET)
	$(Q) $(AR) -rc $@ $?

$(LD_DEST): $(LD_SRC)
	@echo Exporting linker script to $@
	$(Q) $(MKDIR) $(call convert-slash,$(dir $@)) $(QUIET)
	$(Q) $(COPY) $(call convert-slash,$?) $(call convert-slash,$@)

clean:
	$(Q) $(REMOVE_DIR) $(call convert-slash,$(DEBUG_OBJDIR)) $(QUIET)
	$(Q) $(REMOVE_DIR) $(call convert-slash,$(RELEASE_OBJDIR)) $(QUIET)

-include $(DEBUG_DEPFILES)
-include $(RELEASE_DEPFILES)


#########################################################################
#  Default rules to compile c/c++/assembly language sources to objects.
#########################################################################
$(DEBUG_OBJDIR)/%.o : %.c
	@echo Compiling $<
	$(Q) $(MKDIR) $(call convert-slash,$(dir $@)) $(QUIET)
	$(Q) $(GCC) $(DEBUG_C_FLAGS) -c $< -o $@

$(RELEASE_OBJDIR)/%.o : %.c
	@echo Compiling $<
	$(Q) $(MKDIR) $(call convert-slash,$(dir $@)) $(QUIET)
	$(Q) $(GCC) $(RELEASE_C_FLAGS) -c $< -o $@

$(DEBUG_OBJDIR)/%.o : %.cpp
	@echo Compiling $<
	$(Q) $(MKDIR) $(call convert-slash,$(dir $@)) $(QUIET)
	$(Q) $(GPP) $(DEBUG_CPP_FLAGS) -c $< -o $@

$(RELEASE_OBJDIR)/%.o : %.cpp
	@echo Compiling $<
	$(Q) $(MKDIR) $(call convert-slash,$(dir $@)) $(QUIET)
	$(Q) $(GPP) $(RELEASE_CPP_FLAGS) -c $< -o $@

$(DEBUG_OBJDIR)/%.o : %.s
	@echo Assembling $<
	$(Q) $(MKDIR) $(call convert-slash,$(dir $@)) $(QUIET)
	$(Q) $(GCC) $(AS_FLAGS) -c $< -o $@

$(RELEASE_OBJDIR)/%.o : %.s
	@echo Assembling $<
	$(Q) $(MKDIR) $(call convert-slash,$(dir $@)) $(QUIET)
	$(Q) $(GCC) $(AS_FLAGS) -c $< -o $@

$(DEVICE_DROP)/%.h : $(VENDOR_CAPI_DEVICE_SRC)/%.h
	@echo Deploying $? to drop
	$(Q) $(MKDIR) $(call convert-slash,$(dir $@)) $(QUIET)
	$(Q) $(COPY) $(call convert-slash,$?) $(call convert-slash,$@) $(NOSTDOUT)

$(DEVICE_DROP)/%.h : $(VENDOR_CMSIS_SRC)/%.h
	@echo Deploying $? to drop
	$(Q) $(MKDIR) $(call convert-slash,$(dir $@)) $(QUIET)
	$(Q) $(COPY) $(call convert-slash,$?) $(call convert-slash,$@) $(NOSTDOUT)
