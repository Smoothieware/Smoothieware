/*
      This file is part of Smoothie (http://smoothieware.org/). The motion control part is heavily based on Grbl (https://github.com/simen/grbl).
      Smoothie is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
      Smoothie is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
      You should have received a copy of the GNU General Public License along with Smoothie. If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef CONFIGVALUE_H
#define CONFIGVALUE_H

#include <string>
using std::string;

class ConfigValue{
    public:
        ConfigValue();
        ConfigValue(uint16_t *check_sums);
        ConfigValue(const ConfigValue& to_copy);
        ConfigValue& operator= (const ConfigValue& to_copy);
        void clear();
        ConfigValue* required();
        float as_number();
        int as_int();
        bool as_bool();
        string as_string();

        ConfigValue* by_default(float val);
        ConfigValue* by_default(string val);
        ConfigValue* by_default(int val);
        bool is_inverted();


        friend class ConfigCache;
        friend class Config;
        friend class ConfigSource;
        friend class Configurator;
        friend class FileConfigSource;

    private:
        bool has_characters( const char* mask );
        string value;
        int default_int;
        float default_double;
        uint16_t check_sums[3];
        bool found;
        bool default_set;
};








#endif
