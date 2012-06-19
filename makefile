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
GCC4MBED_TYPE=Checked
MRI_UART=MRI_UART_MBED_USB MRI_UART_SHARE
MRI_BREAK_ON_INIT=0
MRI_SEMIHOST_STDIO=0

include ./gcc4mbed/build/gcc4mbed.mk

flash: 
	lpc21isp $(PROJECT).hex /dev/ttyACM0 115200 14746


