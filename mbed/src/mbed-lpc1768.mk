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

# Vendor/device for which the library should be built.
VENDOR=NXP
DEVICE=LPC1768


# Compiler flags which are specifc to this device.
DEVICE_C_FLAGS =-mcpu=cortex-m3 -mthumb -mthumb-interwork
DEVICE_AS_FLAGS=-mcpu=cortex-m3 -mthumb

# If compiling for newer smoothie boards set clock to 120MHz
SMOOTHIEBETA?=1
ifeq "$(SMOOTHIEBETA)" "0"
DEVICE_C_FLAGS += -DUSE120MHZ
endif

include arm-common.mk
