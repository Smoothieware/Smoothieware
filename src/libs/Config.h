/*  
      This file is part of Smoothie (http://smoothieware.org/). The motion control part is heavily based on Grbl (https://github.com/simen/grbl).
      Smoothie is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
      Smoothie is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
      You should have received a copy of the GNU General Public License along with Smoothie. If not, see <http://www.gnu.org/licenses/>. 
*/

#ifndef CONFIG_H
#define CONFIG_H
#include "mbed.h"
#include "libs/Kernel.h"
#include "libs/utils.h"
#include <string>
using std::string;

#define config_get_checksum        46310
#define config_set_checksum        55538
#define config_load_checksum       3143

class ConfigValue{
    public:
        ConfigValue(){
            this->found = false;
            this->default_set = false; 
        };

        ConfigValue* required(){
            if( this->found == true ){
                return this;
            }else{
                error("could not find config setting with checksum %u, please see http://smoothieware.org/configuring-smoothie\r\n", this->check_sum );
            }
        }

        double as_number(){
            if( this->found == false && this->default_set == true ){
                return this->default_double;
            }else{
                double result = atof(remove_non_number(value).c_str());
                if( result == 0.0 && this->value.find_first_not_of("0.") != string::npos ){
                    error("config setting '%s' with value '%s' is not a valid number, please see http://smoothieware.org/configuring-smoothie\r\n", this->key.c_str(), this->value.c_str() );
                }
                return result; 
            }
        }

        ConfigValue* by_default(double value){
            this->default_set = true;
            this->default_double = value;
            return this; 
        }

        bool has_characters( string mask ){
            if( this->value.find_first_of(mask) != string::npos ){ return true; }else{ return false; } 
        }

        bool is_inverted(){
            return this->has_characters(string("!"));
        }

        string value;
        string key;
        uint16_t check_sum; 
        bool found;
        bool default_set;
        double default_double; 
};


class Config : public Module {
    public:
        Config();

        void on_module_loaded();
        void on_console_line_received( void* argument );
        void config_get_command( string parameters ); 
        void config_set_command( string parameters ); 
        void config_load_command(string parameters );
        void set_string( uint16_t check_sum, string value);
        ConfigValue* value(uint16_t check_sum);
        bool   has_characters(uint16_t check_sum, string str );
        string get_config_file();
        void try_config_file(string candidate);

        string config_file;         // Path to the config file
        bool   config_file_found;   // Wether or not the config file's location is known
};

#endif
