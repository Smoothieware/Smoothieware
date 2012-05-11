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

#define error(...) (fprintf(stderr, __VA_ARGS__), exit(1))

using namespace std;
#include <vector>
#include <string>
#include <stdio.h>

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
                double result = atof(remove_non_number(this->value).c_str());
                if( result == 0.0 && this->value.find_first_not_of("0.") != string::npos ){
                    error("config setting '%s' with value '%s' is not a valid number, please see http://smoothieware.org/configuring-smoothie\r\n", this->key.c_str(), this->value.c_str() );
                }
                return result; 
            }
           
        }

        std::string as_string(){
            if( this->found == false && this->default_set == true ){
                return this->default_string;
            }else{
                return this->value; 
            }
        }

        bool as_bool(){
            if( this->found == false && this->default_set == true ){
                return this->default_double;
            }else{
                if( this->value.find_first_of("t1") != string::npos ){
                    return true;
                }else{
                    return false;
                } 
            }
        }

        Pin* as_pin(){
            Pin* pin = new Pin();
            pin->from_string(this->as_string());
            return pin;
        }

        ConfigValue* by_default(double val){
            this->default_set = true;
            this->default_double = val;
            return this; 
        }

        ConfigValue* by_default(std::string val){
            this->default_set = true;
            this->default_string = val;
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
        string default_string;
};


class Config : public Module {
    public:
        Config();

        void on_module_loaded();
        void on_console_line_received( void* argument );
        void config_get_command( string parameters ); 
        void config_set_command( string parameters ); 
        void config_load_command(string parameters );
        void set_string( string setting , string value);
        
        ConfigValue* value(uint16_t check_sum);
        ConfigValue* value(uint16_t check_sum_a, uint16_t check_sum_b);
        ConfigValue* value(uint16_t check_sum_a, uint16_t check_sum_b, uint16_t check_sum_c );
        ConfigValue* value(vector<uint16_t> check_sums );

        void get_module_list(vector<uint16_t>* list, uint16_t family);

        bool   has_characters(uint16_t check_sum, string str );
        string get_config_file();
        bool has_config_file();
        void try_config_file(string candidate);

        string config_file;         // Path to the config file
        bool   config_file_found;   // Wether or not the config file's location is known
};

#endif
