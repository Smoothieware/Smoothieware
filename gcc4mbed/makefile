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
# Directories to be built
DIRS=samples

.PHONY: subdirs $(DIRS)

subdirs:$(DIRS)

clean: $(DIRS)

# Recurse into each of the specified directories and perform a make
$(DIRS):
	$(MAKE) $(MAKECMDGOALS) -C $@

# Run makefile which pulls mbed libraries down from mbed.org when necessary.
install_mbed:
	$(MAKE) -C external
