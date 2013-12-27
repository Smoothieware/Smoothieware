/*
      This file is part of Smoothie (http://smoothieware.org/). The motion control part is heavily based on Grbl (https://github.com/simen/grbl).
      Smoothie is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
      Smoothie is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
      You should have received a copy of the GNU General Public License along with Smoothie. If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef CONFIGVALUE_H
#define CONFIGVALUE_H

#include "libs/Kernel.h"
#include "libs/utils.h"
#include "libs/Pin.h"
#include "Pwm.h"


#define error(...) (fprintf(stderr, __VA_ARGS__), exit(1))

using namespace std;
#include <vector>
#include <string>
#include <stdio.h>


class ConfigValue{
    public:
        ConfigValue(){
            this->found = false;
            this->default_set = false;
            this->check_sums[0] = 0x0000;
            this->check_sums[1] = 0x0000;
            this->check_sums[2] = 0x0000;
        };

        ConfigValue* required(){
            if( this->found == true ){
                return this;
            }else{
                error("could not find config setting, please see http://smoothieware.org/configuring-smoothie\r\n");
            }
        }

        float as_number(){
            if( this->found == false && this->default_set == true ){
                return this->default_double;
            }else{
                char* endptr = NULL;
                float result = strtof(remove_non_number(this->value).c_str(), &endptr);
                if( endptr <= remove_non_number(this->value).c_str() ){
                    error("config setting with value '%s' and checksums[%u,%u,%u] is not a valid number, please see http://smoothieware.org/configuring-smoothie\r\n", this->value.c_str(), this->check_sums[0], this->check_sums[1], this->check_sums[2] );
                }
                return result;
            }
        }

        int as_int()
        {
            if( this->found == false && this->default_set == true ){
                return this->default_int;
            }else{
                char* endptr = NULL;
                int result = strtol(remove_non_number(this->value).c_str(), &endptr, 10);
                if( endptr <= remove_non_number(this->value).c_str() ){
                    error("config setting with value '%s' and checksums[%u,%u,%u] is not a valid number, please see http://smoothieware.org/configuring-smoothie\r\n", this->value.c_str(), this->check_sums[0], this->check_sums[1], this->check_sums[2] );
                }
                return result;
            }
        }

        std::string as_string(){
            return this->value;
        }

        bool as_bool(){
            if( this->found == false && this->default_set == true ){
                return this->default_double;
            }else{
                if( this->value.find_first_of("ty1") != string::npos ){
                    return true;
                }else{
                    return false;
                }
            }
        }

        ConfigValue* by_default(int val)
        {
            this->default_set = true;
            this->default_int = val;
            this->default_double = val;
            return this;
        }

        ConfigValue* by_default(float val){
            this->default_set = true;
            this->default_double = val;
            return this;
        }

        ConfigValue* by_default(std::string val){
            if( this->found ){ return this; }
            this->default_set = true;
            this->value = val;
            return this;
        }

        bool has_characters( string mask ){
            if( this->value.find_first_of(mask) != string::npos ){ return true; }else{ return false; }
        }

        bool is_inverted(){
            return this->has_characters(string("!"));
        }

        int default_int;
        float default_double;
        uint16_t check_sums[3];
        string value;
        bool found;
        bool default_set;
};








#endif
