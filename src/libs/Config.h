/*
      This file is part of Smoothie (http://smoothieware.org/). The motion control part is heavily based on Grbl (https://github.com/simen/grbl).
      Smoothie is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
      Smoothie is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
      You should have received a copy of the GNU General Public License along with Smoothie. If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef CONFIG_H
#define CONFIG_H


using namespace std;
#include <vector>
#include <string>

class ConfigValue;
class ConfigSource;
class ConfigCache;

class Config : public Module {
    public:
        Config();

        void on_module_loaded();
        void on_console_line_received( void* argument );
        void config_cache_load(bool parse= true);
        void config_cache_clear();
        void set_string( string setting , string value);

        ConfigValue* value(uint16_t check_sum_a, uint16_t check_sum_b= 0, uint16_t check_sum_c= 0 );
        ConfigValue* value(uint16_t check_sums[3] );

        void get_module_list(vector<uint16_t>* list, uint16_t family);
        bool is_config_cache_loaded() { return config_cache != NULL; };    // Whether or not the cache is currently popluated

        friend class  Configurator;

    private:
        bool   has_characters(uint16_t check_sum, string str );

        ConfigCache* config_cache;            // A cache in which ConfigValues are kept
        vector<ConfigSource*> config_sources; // A list of all possible coniguration sources
};

#endif
