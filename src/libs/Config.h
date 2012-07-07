/*  
      This file is part of Smoothie (http://smoothieware.org/). The motion control part is heavily based on Grbl (https://github.com/simen/grbl).
      Smoothie is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
      Smoothie is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
      You should have received a copy of the GNU General Public License along with Smoothie. If not, see <http://www.gnu.org/licenses/>. 
*/

#ifndef CONFIG_H
#define CONFIG_H
#include "libs/Kernel.h"
#include "libs/utils.h"
#include "libs/Pin.h"
#include "ConfigValue.h"
#include "ConfigCache.h"
#include "ConfigSource.h"
#include "libs/ConfigSources/FileConfigSource.h"

#define error(...) (fprintf(stderr, __VA_ARGS__), exit(1))

using namespace std;
#include <vector>
#include <string>
#include <stdio.h>

#define LOCAL_CONFIGSOURCE_CHECKSUM     13581
#define SD_CONFIGSOURCE_CHECKSUM        19415


class Config : public Module {
    public:
        Config();

        void on_module_loaded();
        void on_console_line_received( void* argument );
        void config_cache_load();
        void config_cache_clear();
        void set_string( string setting , string value);

        ConfigValue* value(uint16_t check_sum);
        ConfigValue* value(uint16_t check_sum_a, uint16_t check_sum_b);
        ConfigValue* value(uint16_t check_sum_a, uint16_t check_sum_b, uint16_t check_sum_c );
        ConfigValue* value(vector<uint16_t> check_sums );

        void get_module_list(vector<uint16_t>* list, uint16_t family);

        bool   has_characters(uint16_t check_sum, string str );

        ConfigCache config_cache;             // A cache in which ConfigValues are kept
        vector<ConfigSource*> config_sources; // A list of all possible coniguration sources
        bool   config_cache_loaded;           // Whether or not the cache is currently popluated
};

#endif
