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

class ConfigValue;
class ConfigCache;

class ConfigSource {
    public:
        ConfigSource(){}

        // Read each value, and append it as a ConfigValue to the config_cache we were passed
        virtual void transfer_values_to_cache( ConfigCache* ) = 0;
        virtual bool is_named( uint16_t check_sum ) = 0;
        virtual bool write( string setting, string value ) = 0;
        virtual string read( uint16_t check_sums[3] ) = 0;

    protected:
        virtual ConfigValue* process_line_from_ascii_config(const string& line, ConfigCache* cache);
        virtual string process_line_from_ascii_config(const string& line, uint16_t line_checksums[3]);
        uint16_t name_checksum;

    private:
        ConfigValue* process_line(const string &buffer);
};



#endif
