/*
      This file is part of Smoothie (http://smoothieware.org/). The motion control part is heavily based on Grbl (https://github.com/simen/grbl).
      Smoothie is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
      Smoothie is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
      You should have received a copy of the GNU General Public License along with Smoothie. If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef CONFIGSOURCE_H
#define CONFIGSOURCE_H

using namespace std;
#include <vector>
#include <string>
#include "ConfigValue.h"
#include "ConfigCache.h"

class ConfigValue;

class ConfigSource {
    public:
        ConfigSource(){}

        // Read each value, and append it as a ConfigValue to the config_cache we were passed
        virtual void transfer_values_to_cache( ConfigCache* ) = 0;
        virtual bool is_named( uint16_t check_sum ) = 0;
        virtual void write( string setting, string value ) = 0;
        virtual string read( uint16_t check_sums[3] ) = 0;

        uint16_t name_checksum;
    protected:
        virtual string process_char_from_ascii_config(int c, ConfigCache* cache) {
            static string buffer;
            if (c == '\n' || c == EOF)
            {
                // We have a new line
                if( buffer[0] == '#' ){ buffer.clear(); return ""; } // Ignore comments
                if( buffer.length() < 3 ){ buffer.clear(); return ""; } //Ignore empty lines
                size_t begin_key = buffer.find_first_not_of(" ");
                size_t begin_value = buffer.find_first_not_of(" ", buffer.find_first_of(" ", begin_key));

                uint16_t check_sums[3];
                get_checksums(check_sums, buffer.substr(begin_key,  buffer.find_first_of(" ", begin_key) - begin_key).append(" "));

                ConfigValue* result = new ConfigValue;

                result->found = true;
                result->check_sums[0] = check_sums[0];
                result->check_sums[1] = check_sums[1];
                result->check_sums[2] = check_sums[2];

                result->value = buffer.substr(begin_value, buffer.find_first_of("\r\n# ", begin_value+1)-begin_value);
                // Append the newly found value to the cache we were passed
                cache->replace_or_push_back(result);

                buffer.clear();

                return result->value;
            }
            else
                buffer += c;

            return "";
        };
        virtual string process_char_from_ascii_config(int c, uint16_t line_checksums[3]) {
            static string buffer;
            string value;
            if (c == '\n' || c == EOF)
            {
                // We have a new line
                if( buffer[0] == '#' ){ buffer.clear(); return ""; } // Ignore comments
                if( buffer.length() < 3 ){ buffer.clear(); return ""; } //Ignore empty lines
                size_t begin_key = buffer.find_first_not_of(" ");
                size_t begin_value = buffer.find_first_not_of(" ", buffer.find_first_of(" ", begin_key));

                uint16_t check_sums[3];
                get_checksums(check_sums, buffer.substr(begin_key,  buffer.find_first_of(" ", begin_key) - begin_key).append(" "));

                if(check_sums[0] == line_checksums[0] && check_sums[1] == line_checksums[1] && check_sums[2] == line_checksums[2] ){
                    value = buffer.substr(begin_value, buffer.find_first_of("\r\n# ", begin_value+1)-begin_value);
                    buffer.clear();
                    return value;
                }

                buffer.clear();
            }
            else
                buffer += c;
            return value;
        };
};



#endif
