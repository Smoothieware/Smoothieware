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
# -----------------------------------------------------------------------------
# LPC11U24 device specific makefile.
# -----------------------------------------------------------------------------
ifndef GCC4MBED_DIR
$(error makefile must set GCC4MBED_DIR.)
endif


# Set build customizations for this device.
DEVICE=LPC11U24
ARCHITECTURE=armv6-m
DEVICE_FLAGS=-mcpu=cortex-m0 -mthumb
DEVICE_CFLAGS=$(DEVICE_FLAGS)
NO_FLOAT_SCANF?=1
NO_FLOAT_PRINTF?=1


# MRI enabled builds aren't supported on LPC11U24 so force Release build.
GCC4MBED_TYPE=Release


# Now include the rest which is generic across devices.
include $(GCC4MBED_DIR)/build/common.mk
