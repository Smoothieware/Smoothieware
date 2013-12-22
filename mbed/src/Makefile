#!/usr/bin/make

DEVICES      =mbed-lpc1768.mk mbed-lpc11u24.mk
DEVICESCLEAN =$(addsuffix .clean,$(DEVICES))


# Set VERBOSE make variable to 1 to output all tool commands.
VERBOSE?=0
ifeq "$(VERBOSE)" "0"
Q=@
else
Q=
endif


# Directories where source files are found and output files should be placed.
ROOT_DIR      =..
SRC_DIR       =.
DROP_DIR      =$(ROOT_DIR)/drop
MBED_CAPI_SRC =$(SRC_DIR)/capi
MBED_CPP_SRC  =$(SRC_DIR)/cpp


# Copy headers to drop directory.
COMMON_HEADER_SRCS =$(notdir $(wildcard $(MBED_CPP_SRC)/*.h))
COMMON_HEADER_SRCS+=$(notdir $(wildcard $(MBED_CAPI_SRC)/*.h))
COMMON_HEADERS     =$(patsubst %.h,$(DROP_DIR)/%.h,$(COMMON_HEADER_SRCS))


# Command line tools that are different between *nix and Windows.
ifeq "$(OS)" "Windows_NT"
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


# Make rules.
all: $(COMMON_HEADERS) $(DEVICES)

clean: $(DEVICESCLEAN) clean_drop

clean_drop:
	@echo Cleaning $(DROP_DIR)
	$(Q) $(REMOVE_DIR) $(call convert-slash,$(DROP_DIR)) $(QUIET)

$(DEVICES):
	@echo Building $@
	@ $(MAKE) -f $@

$(DEVICESCLEAN): %.clean:
	@echo Cleaning $*
	@ $(MAKE) -f $*  clean

$(DROP_DIR)/%.h : $(MBED_CPP_SRC)/%.h
	@echo Deploying $? to drop
	$(Q) $(MKDIR) $(call convert-slash,$(dir $@)) $(QUIET)
	$(Q) $(COPY) $(call convert-slash,$?) $(call convert-slash,$@) $(NOSTDOUT)

$(DROP_DIR)/%.h : $(MBED_CAPI_SRC)/%.h
	@echo Deploying $? to drop
	$(Q) $(MKDIR) $(call convert-slash,$(dir $@)) $(QUIET)
	$(Q) $(COPY) $(call convert-slash,$?) $(call convert-slash,$@) $(NOSTDOUT)


.PHONY: all clean clean_drop $(DEVICES) $(DEVICESCLEAN)
