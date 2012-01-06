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
#
# Makefile to install external dependencies like mbed and FATFileSystem
# libraries from mbed.
ifeq "$(MAKE)" "cs-make"
SVN=win32\svn
MOVE=@echo Skipping move
DELTREE=-rd /s /q
PATCH=win32\applydiff --binary
QUIET=>nul 2>nul & exit 0
SHELL=cmd.exe
else
SVN=svn
MOVE=mv
DELTREE=rm -f -r
PATCH=patch
QUIET=> /dev/null 2>&1 ; exit 0
endif

.PHONY: install install_mbed install_fat patch_mbed patch_fat strip_libs

install: install_mbed install_fat patch_mbed patch_fat strip_libs

install_mbed:
	$(DELTREE) mbed $(QUIET)
	$(SVN) checkout --revision 28  http://mbed.org/projects/libraries/svn/mbed/trunk mbed

install_fat:
	$(DELTREE) FATFileSystem $(QUIET)
	$(SVN) checkout --revision 28  http://mbed.org/projects/libraries/svn/FATFileSystem/trunk FATFileSystem

patch_mbed:
	$(MOVE) mbed/LPC1768/LPC1768.sct  mbed/LPC1768/LPC1768.scatter
	$(MOVE) mbed/LPC2368/LPC2368.sct  mbed/LPC2368/LPC2368.scatter
	$(PATCH) -p1 <mbed.patch

patch_fat:
	$(PATCH) -p1 <FATFileSystem.patch

strip_libs:
	arm-none-eabi-objcopy -g mbed/LPC1768/mbed.ar
	arm-none-eabi-objcopy -g mbed/LPC1768/capi.ar
	arm-none-eabi-objcopy -g FATFileSystem/LPC1768/FATFileSystem.ar
	