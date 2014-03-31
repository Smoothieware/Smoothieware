/*
      This file is part of Smoothie (http://smoothieware.org/). The motion control part is heavily based on Grbl (https://github.com/simen/grbl).
      Smoothie is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
      Smoothie is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
      You should have received a copy of the GNU General Public License along with Smoothie. If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef CONFIGVALUE_H
#define CONFIGVALUE_H

using namespace std;
#include <string>

class ConfigValue{
    public:
        ConfigValue();

        ConfigValue* required();
        float as_number();
        int as_int();
        string as_string();
        bool as_bool();

        ConfigValue* by_default(float val);
        ConfigValue* by_default(string val);
        ConfigValue* by_default(int val);
        bool has_characters( string mask );
        bool is_inverted();

        int default_int;
        float default_double;
        uint16_t check_sums[3];
        string value;
        bool found;
        bool default_set;
};








#endif
