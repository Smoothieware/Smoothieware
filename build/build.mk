# Copyright 2012 Adam Green (http://mbed.org/users/AdamGreen/)
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
###############################################################################
# USAGE:
# Variables that must be defined in including makefile.
#   PROJECT: Name to be given to the output binary for this project.
#   BUILD_DIR: The directory containing this build.mk file.
#
# Variables that may be optionally set in makefile.
#   SRC: The root directory for the sources of your project.  Defaults to '.'.
#   LIBS_PREFIX: List of library/object files to prepend to mbed.ar capi.ar libs.
#   LIBS_SUFFIX: List of library/object files to append to mbed.ar capi.ar libs.
#   BUILD_TYPE: Type of build to produce.  Allowed values are:
#                  Debug - Build for debugging.  Disables optimizations and
#                          links in debug MRI runtime.  Best debugging 
#                          experience.
#                  Release - Build for release with no debug support.
#                  Checked - Release build with debug support.  Due to
#                            optimizations, debug experience won't be as good
#                            as Debug but might be needed when bugs don't
#                            reproduce in Debug builds.
#                  default: Release
#   DEVICES: List of devices for which to build.  This list be space delimited
#            and can include the following devices:
#            LPC1768, LPC11U24
#            default: LPC1768
#   GPFLAGS: Additional compiler flags used when building C++ sources.
#   GCFLAGS: Additional compiler flags used when building C sources.
#   AS_GCFLAGS: Additional compiler flags used by GCC when building
#               preprocessed assembly language sources.
#   AS_FLAGS: Additional assembler flags used when building assembly language
#             sources.
#   NO_FLOAT_SCANF: When set to 1, scanf() will not support %f specifier to
#                   input floating point values.  Reduces code size.
#   NO_FLOAT_PRINTF: When set to 1, scanf() will not support %f specifier to
#                    output floating point values.  Reduces code size.
#   VERBOSE: When set to 1, all build commands will be displayed to console.
#            It defaults to 0 which suppresses the output of the build tool
#            command lines themselves.
#   MRI_BREAK_ON_INIT: Should the program halt before calling into main(),
#                      allowing the developer time to set breakpoints in main()
#                      or in code run from within global constructors.
#                      default: 1 - break on init.
#   MRI_SEMIHOST_STDIO: Set to non-zero value to allow debug monitor to use
#                       semi-host calls to redirect stdin/stdout/stderr to the
#                       gdb console.
#                       default: 1 for Debug/Checked builds and 0 for Release.
#   MRI_UART: Select the UART to be used by the debugger.  See mri.h for
#             allowed values.
#             default: MRI_UART_MBED_USB - Use USB based UART on the mbed.
# Example makefile:
#       PROJECT=HelloWorld
#       BUILD_DIR=../..
#
#       include $(BUILD_DIR)/build.mk
#      
###############################################################################

# Check for undefined variables.
ifndef PROJECT
$(error makefile must set PROJECT variable.)
endif

ifndef BUILD_DIR
$(error makefile must set BUILD_DIR.)
endif


# Default DEVICES to LPC1768 if nothing else has been specified.
DEVICES?=lpc1768


# Use DEVICES variable to determine which builds to perform.
CLEAN_DEVICES=$(addsuffix .clean,$(DEVICES))
.PHONY: all clean $(DEVICES) $(CLEAN_DEVICES) deploy deploy-lpc1768 deploy-lpc11u24

all: $(DEVICES)

clean: $(CLEAN_DEVICES)

export
$(DEVICES):
	@echo Building for device $@
	@$(MAKE) -f $(BUILD_DIR)/$@.mk all

$(CLEAN_DEVICES): %.clean:
	@echo Cleaning up for device $*
	@$(MAKE) -f $(BUILD_DIR)/$*.mk clean

ifdef LPC_DEPLOY
deploy deploy-lpc1768:
	@echo Deploying to target.
	$(subst PROJECT,LPC1768/$(PROJECT),$(LPC_DEPLOY))

deploy-lpc11u24:
	@echo Deploying to target.
	$(subst PROJECT,LPC11U24/$(PROJECT),$(LPC_DEPLOY))
endif
