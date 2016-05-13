#Copyright (C) 2011 by Sagar G V
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
#THE SOFTWARE.
#
# Updates:
#    Arthur Wolf & Adam Green in 2011 - 2012 - Updated to work with mbed.
###############################################################################
# Check for undefined variables.
ifndef PROJECT
$(error makefile must set PROJECT variable.)
endif

ifndef BUILD_DIR
$(error makefile must set BUILD_DIR.)
endif

# Set VERBOSE make variable to 1 to output all tool commands.
VERBOSE?=0
ifeq "$(VERBOSE)" "0"
Q=@
else
Q=
endif


# Default variables.
SRC ?= .
BUILD_TYPE ?= Release
MRI_BREAK_ON_INIT ?= 1
MRI_UART ?= MRI_UART_MBED_USB
HEAP_TAGS ?= 0
WRITE_BUFFER_DISABLE ?= 0
STACK_SIZE ?= 0


# Configure MRI variables based on BUILD_TYPE build type variable.
ifeq "$(BUILD_TYPE)" "Release"
OPTIMIZATION ?= 2
MRI_ENABLE = 0
MRI_SEMIHOST_STDIO ?= 0
endif


ifeq "$(BUILD_TYPE)" "Debug"
OPTIMIZATION = 0
MRI_ENABLE ?= 1
MRI_SEMIHOST_STDIO ?= 1
endif


ifeq "$(BUILD_TYPE)" "Checked"
OPTIMIZATION ?= 2
MRI_ENABLE = 1
MRI_SEMIHOST_STDIO ?= 1
endif

MRI_INIT_PARAMETERS=$(MRI_UART)

# Output Object Directory
OUTDIR=../$(DEVICE)

# List of sources to be compiled/assembled
CSRCS1 = $(wildcard $(SRC)/*.c $(SRC)/*/*.c $(SRC)/*/*/*.c $(SRC)/*/*/*/*.c $(SRC)/*/*/*/*/*.c $(SRC)/*/*/*/*/*/*.c)
# Totally exclude network if NONETWORK is defined
ifeq "$(NONETWORK)" "1"
CSRCS2 = $(filter-out $(SRC)/libs/Network/%,$(CSRCS1))
DEFINES += -DNONETWORK
else
CSRCS2 = $(CSRCS1)
endif

# do not compile the src/testframework as that can only be done with rake
CSRCS = $(filter-out $(SRC)/testframework/%,$(CSRCS2))

ifeq "$(DISABLEMSD)" "1"
DEFINES += -DDISABLEMSD
endif

ASRCS =  $(wildcard $(SRC)/*.S $(SRC)/*/*.S $(SRC)/*/*/*.S $(SRC)/*/*/*/*.S $(SRC)/*/*/*/*/*.S)
ifneq "$(OS)" "Windows_NT"
ASRCS +=  $(wildcard $(SRC)/*.s $(SRC)/*/*.s $(SRC)/*/*/*.s $(SRC)/*/*/*/*.s $(SRC)/*/*/*/*/*.s)
endif

CPPSRCS1 = $(wildcard $(SRC)/*.cpp $(SRC)/*/*.cpp $(SRC)/*/*/*.cpp $(SRC)/*/*/*/*.cpp $(SRC)/*/*/*/*/*.cpp $(SRC)/*/*/*/*/*/*.cpp)
ifeq "$(NONETWORK)" "1"
	CPPSRCS2 = $(filter-out $(SRC)/libs/Network/%,$(CPPSRCS1))
else
	ifneq "$(PLAN9)" "1"
		DEFINES += -DNOPLAN9
		CPPSRCS2 = $(filter-out $(SRC)/libs/Network/uip/plan9/%,$(CPPSRCS1))
	else
		CPPSRCS2 = $(CPPSRCS1)
	endif
endif

# CNC build
ifeq "$(CNC)" "1"
	CPPSRCS21 = $(filter-out $(SRC)/modules/utils/panel/screens/3dprinter/%,$(CPPSRCS2))
	DEFINES += -DCNC
else
	CPPSRCS21 = $(filter-out $(SRC)/modules/utils/panel/screens/cnc/%,$(CPPSRCS2))
endif

# Totally exclude any modules listed in EXCLUDE_MODULES
# uppercase function
uc = $(subst a,A,$(subst b,B,$(subst c,C,$(subst d,D,$(subst e,E,$(subst f,F,$(subst g,G,$(subst h,H,$(subst i,I,$(subst j,J,$(subst k,K,$(subst l,L,$(subst m,M,$(subst n,N,$(subst o,O,$(subst p,P,$(subst q,Q,$(subst r,R,$(subst s,S,$(subst t,T,$(subst u,U,$(subst v,V,$(subst w,W,$(subst x,X,$(subst y,Y,$(subst z,Z,$1))))))))))))))))))))))))))
EXL = $(patsubst %,$(SRC)/modules/%/%,$(EXCLUDED_MODULES))
CPPSRCS3 = $(filter-out $(EXL),$(CPPSRCS21))
DEFINES += $(call uc, $(subst /,_,$(patsubst %,-DNO_%,$(EXCLUDED_MODULES))))

# do not compile the src/testframework as that can only be done with rake
CPPSRCS = $(filter-out $(SRC)/testframework/%,$(CPPSRCS3))

# List of the objects files to be compiled/assembled
OBJECTS = $(patsubst %.c,$(OUTDIR)/%.o,$(CSRCS)) $(patsubst %.s,$(OUTDIR)/%.o,$(patsubst %.S,$(OUTDIR)/%.o,$(ASRCS))) $(patsubst %.cpp,$(OUTDIR)/%.o,$(CPPSRCS))

# Add in the MBED customization stubs which allow hooking in the MRI debug monitor.
OBJECTS += $(OUTDIR)/mbed_custom.o

OBJECTS += $(OUTDIR)/configdefault.o

# List of the header dependency files, one per object file.
DEPFILES = $(patsubst %.o,%.d,$(OBJECTS))

# Linker script to be used.  Indicates what code should be placed where in memory.
LSCRIPT=$(MBED_DIR)/$(DEVICE)/GCC_ARM/$(DEVICE).ld

# Location of external library and header dependencies.
MBED_DIR = $(BUILD_DIR)/../mbed/drop
MRI_DIR  = $(BUILD_DIR)/../mri

# Include path which points to external library headers and to subdirectories of this project which contain headers.
SUBDIRS = $(wildcard $(SRC)/* $(SRC)/*/* $(SRC)/*/*/* $(SRC)/*/*/*/* $(SRC)/*/*/*/*/* $(SRC)/*/*/*/*/*/*)
PROJINCS = $(sort $(dir $(SUBDIRS)))
INCDIRS += $(SRC) $(PROJINCS) $(MRI_DIR) $(MBED_DIR) $(MBED_DIR)/$(DEVICE)

# DEFINEs to be used when building C/C++ code
DEFINES += -DTARGET_$(DEVICE)
DEFINES += -DMRI_ENABLE=$(MRI_ENABLE) -DMRI_INIT_PARAMETERS='"$(MRI_INIT_PARAMETERS)"'
DEFINES += -DMRI_BREAK_ON_INIT=$(MRI_BREAK_ON_INIT) -DMRI_SEMIHOST_STDIO=$(MRI_SEMIHOST_STDIO)
DEFINES += -DWRITE_BUFFER_DISABLE=$(WRITE_BUFFER_DISABLE) -DSTACK_SIZE=$(STACK_SIZE)

ifeq "$(OPTIMIZATION)" "0"
DEFINES += -DDEBUG
endif

# Libraries to be linked into final binary
MBED_LIBS = $(MBED_DIR)/$(DEVICE)/GCC_ARM/libmbed.a
#SYS_LIBS = -lstdc++_s -lsupc++_s -lm -lgcc -lc_s -lgcc -lc_s -lnosys
SYS_LIBS = -specs=nano.specs -lstdc++ -lsupc++ -lm -lgcc -lc -lnosys
LIBS = $(LIBS_PREFIX)

ifeq "$(MRI_ENABLE)" "1"
LIBS += $(MRI_DIR)/mri.ar
endif

LIBS += $(MBED_LIBS)
LIBS += $(SYS_LIBS)
LIBS += $(LIBS_SUFFIX)

# Compiler flags used to enable creation of header dependencies.
DEPFLAGS = -MMD -MP

# Setup wraps for newlib read/writes to redirect to MRI debugger.
ifeq "$(MRI_ENABLE)" "1"
MRI_WRAPS=,--wrap=_read,--wrap=_write,--wrap=semihost_connected
else
MRI_WRAPS=
endif

# Setup wraps to memory allocations routines if we want to tag heap allocations.
ifeq "$(HEAP_TAGS)" "1"
DEFINES += -DHEAP_TAGS
endif

# Compiler Options
GCFLAGS += -O$(OPTIMIZATION) -g3 $(DEVICE_CFLAGS)
GCFLAGS += -ffunction-sections -fdata-sections  -fno-exceptions -fno-delete-null-pointer-checks
GCFLAGS += $(patsubst %,-I%,$(INCDIRS))
GCFLAGS += $(DEFINES)
GCFLAGS += $(DEPFLAGS)
GCFLAGS += -Wall -Wextra -Wno-unused-parameter -Wcast-align -Wpointer-arith -Wredundant-decls -Wcast-qual -Wcast-align

GPFLAGS += $(GCFLAGS) -fno-rtti -std=gnu++11

AS_GCFLAGS += -g3 $(DEVICE_FLAGS) -x assembler-with-cpp
AS_GCFLAGS += $(patsubst %,-I%,$(INCDIRS))
AS_FLAGS += -g3 $(DEVICE_FLAGS)


# Linker Options.
LDFLAGS = $(DEVICE_FLAGS) -specs=$(BUILD_DIR)/startfile.spec
LDFLAGS += -Wl,-Map=$(OUTDIR)/$(PROJECT).map,--cref,--gc-sections,--wrap=_isatty,--wrap=malloc,--wrap=realloc,--wrap=free$(MRI_WRAPS)
LDFLAGS += -T$(LSCRIPT)  -L $(EXTERNAL_DIR)/gcc/LPC1768
ifneq "$(NO_FLOAT_SCANF)" "1"
LDFLAGS += -u _scanf_float
endif
ifneq "$(NO_FLOAT_PRINTF)" "1"
LDFLAGS += -u _printf_float
endif


#  Compiler/Assembler/Linker Paths
GCC = arm-none-eabi-gcc
GPP = arm-none-eabi-g++
AS = arm-none-eabi-as
LD = arm-none-eabi-g++
OBJCOPY = arm-none-eabi-objcopy
OBJDUMP = arm-none-eabi-objdump
SIZE = arm-none-eabi-size

# Some tools are different on Windows in comparison to Unix.
ifeq "$(OS)" "Windows_NT"
REMOVE = del
SHELL=cmd.exe
REMOVE_DIR = rd /s /q
MKDIR = mkdir
QUIET=>nul 2>nul & exit 0
BLANK_LINE=echo -
else
REMOVE = rm
REMOVE_DIR = rm -r -f
MKDIR = mkdir -p
QUIET=> /dev/null 2>&1 ; exit 0
BLANK_LINE=echo
endif

# Create macro which will convert / to \ on Windows.
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
.PHONY: all clean size

all:: $(OUTDIR)/$(PROJECT).hex $(OUTDIR)/$(PROJECT).bin $(OUTDIR)/$(PROJECT).disasm size

$(OUTDIR)/$(PROJECT).bin: $(OUTDIR)/$(PROJECT).elf
	@echo Extracting $@
	$(Q) $(MKDIR) $(call convert-slash,$(dir $@)) $(QUIET)
	$(Q) $(OBJCOPY) -O binary $< $@

$(OUTDIR)/$(PROJECT).hex: $(OUTDIR)/$(PROJECT).elf
	@echo Extracting $@
	$(Q) $(MKDIR) $(call convert-slash,$(dir $@)) $(QUIET)
	$(Q) $(OBJCOPY) -R .stack -O ihex $< $@

$(OUTDIR)/$(PROJECT).disasm: $(OUTDIR)/$(PROJECT).elf
	@echo Extracting disassembly to $@
	$(Q) $(MKDIR) $(call convert-slash,$(dir $@)) $(QUIET)
	$(Q) $(OBJDUMP) -d -f -M reg-names-std --demangle $< >$@

$(OUTDIR)/$(PROJECT).elf: $(LSCRIPT) $(OBJECTS)
	@echo Linking $@
	$(Q) $(MKDIR) $(call convert-slash,$(dir $@)) $(QUIET)
	$(Q) $(LD) $(LDFLAGS) $(OBJECTS) $(LIBS) -o $@

size: $(OUTDIR)/$(PROJECT).elf
	$(Q) $(SIZE) $<
	@$(BLANK_LINE)

clean:
	@echo Cleaning up all build generated files
	$(Q) $(REMOVE_DIR) $(OUTDIR) $(QUIET)

-include $(DEPFILES)

#########################################################################
#  Default rules to compile .c and .cpp file to .o
#  and assemble .s files to .o

$(OUTDIR)/mbed_custom.o : $(BUILD_DIR)/mbed_custom.cpp makefile
	@echo Compiling $<
	$(Q) $(MKDIR) $(call convert-slash,$(dir $@)) $(QUIET)
	$(Q) $(GPP) $(GPFLAGS) -c $< -o $@

$(OUTDIR)/%.o : %.cpp makefile
	@echo Compiling $<
	$(Q) $(MKDIR) $(call convert-slash,$(dir $@)) $(QUIET)
	$(Q) $(GPP) $(GPFLAGS) -c $< -o $@

$(OUTDIR)/%.o : %.c makefile
	@echo Compiling $<
	$(Q) $(MKDIR) $(call convert-slash,$(dir $@)) $(QUIET)
	$(Q) $(GCC) $(GCFLAGS) -c $< -o $@

$(OUTDIR)/%.o : %.S makefile
	@echo Assembling $<
	$(Q) $(MKDIR) $(call convert-slash,$(dir $@)) $(QUIET)
	$(Q) $(GCC) $(AS_GCFLAGS) -c $< -o $@

$(OUTDIR)/%.o : %.s makefile
	@echo Assembling $<
	$(Q) $(MKDIR) $(call convert-slash,$(dir $@)) $(QUIET)
	$(Q) $(AS) $(AS_FLAGS) -o $@ $<

$(OUTDIR)/configdefault.o : config.default
	$(Q) $(OBJCOPY) -I binary -O elf32-littlearm -B arm --readonly-text --rename-section .data=.rodata.configdefault $< $@

#########################################################################
