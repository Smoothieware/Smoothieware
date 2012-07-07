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
        virtual void transfer_values_to_cache( ConfigCache* ){}
        virtual bool is_named( uint16_t check_sum ){}
        virtual void write( string setting, string value ){}
        virtual string read( vector<uint16_t> check_sums ){}

        uint16_t name_checksum;
};



#endif
