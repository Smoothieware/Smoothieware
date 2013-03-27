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
# LPC1768 device specific makefile.
# -----------------------------------------------------------------------------
ifndef BUILD_DIR
$(error makefile must set BUILD_DIR.)
endif


# Set build customizations for this device.
DEVICE=LPC1768
ARCHITECTURE=armv7-m
DEVICE_FLAGS=-mcpu=cortex-m3 -mthumb
DEVICE_CFLAGS=$(DEVICE_FLAGS) -mthumb-interwork
NO_FLOAT_SCANF?=0
NO_FLOAT_PRINTF?=0
DEFINES+=-D__LPC17XX__


# Now include the rest which is generic across devices.
include $(BUILD_DIR)/common.mk
