/*  
      This file is part of Smoothie (http://smoothieware.org/). The motion control part is heavily based on Grbl (https://github.com/simen/grbl).
      Smoothie is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
      Smoothie is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
      You should have received a copy of the GNU General Public License along with Smoothie. If not, see <http://www.gnu.org/licenses/>. 
*/


using namespace std;
#include <vector>
#include <string>

#include "libs/Kernel.h"
#include "Config.h"
#include "libs/nuts_bolts.h"
#include "libs/utils.h"
#include "libs/SerialMessage.h"

Config::Config(){
    config_file_found = false; 
}

void Config::on_module_loaded(){
    this->register_for_event(ON_CONSOLE_LINE_RECEIVED);
}

// When a new line is received, check if it is a command, and if it is, act upon it
void Config::on_console_line_received( void* argument ){
    string possible_command = (*static_cast<SerialMessage*>(argument)).message;

    // We don't compare to a string but to a checksum of that string, this saves some space in flash memory
    unsigned short check_sum = get_checksum( possible_command.substr(0,possible_command.find_first_of(" \r\n")) );  // todo: put this method somewhere more convenient
    //this->kernel->serial->printf("checksum: %u \r\n", check_sum);
    // Act depending on command
    switch( check_sum ){
        case config_get_checksum: this->config_get_command( get_arguments(possible_command))    ; break; 
        case config_set_checksum: this->config_set_command( get_arguments(possible_command))    ; break;
        case config_load_checksum:this->config_load_command(get_arguments(possible_command))    ; break; 
    }
}

// Command to retreive the value of a specific configuration setting
void Config::config_get_command( string parameter ){
    this->kernel->serial->printf("%s\r\n", this->value( get_checksum( parameter ) )->value.c_str() ); 
}

// Command to set the value of a specific configuration setting
void Config::config_set_command( string parameters ){
    string setting = shift_parameter(parameters);
    string value   = shift_parameter(parameters);
    this->set_string( setting, value );
}

// Command to reload configuration in all modules ( usefull if you changed one )
void Config::config_load_command( string parameters ){
    if(parameters.size() > 0) {
	this->config_file_found = false;
        //this->try_config_file("/local/" + parameters;
        this->try_config_file("/sd/" + parameters);
        if( !this->config_file_found ){
            this->kernel->serial->printf("ERROR: config file \'%s\' not found, loading default config\r\n", parameters.c_str());
        }
    }
    this->kernel->call_event(ON_CONFIG_RELOAD);
}

// Set a value from the configuration as a string
// Because we don't like to waste space in Flash with lengthy config parameter names, we take a checksum instead so that the name does not have to be stored
// See get_checksum
void Config::set_string( string setting, string value ){
    // Open the config file ( find it if we haven't already found it ) 
    FILE *lp = fopen(this->get_config_file().c_str(), "r+");
    string buffer;
    int c; 
    // For each line 
    do {
        c = fgetc (lp);
        if (c == '\n'){
            // We have a new line
            if( buffer[0] == '#' ){ buffer.clear(); continue; } // Ignore comments
            if( buffer.length() < 3 ){ buffer.clear(); continue; } //Ignore empty lines
            size_t begin_key = buffer.find_first_not_of(" ");
            size_t begin_value = buffer.find_first_not_of(" ", buffer.find_first_of(" ", begin_key));
            // If this line matches the checksum 
            string candidate = buffer.substr(begin_key,  buffer.find_first_of(" ", begin_key) - begin_key);
            if( candidate.compare(setting) != 0 ){ buffer.clear(); continue; }
            int free_space = int(int(buffer.find_first_of("\r\n#", begin_value+1))-begin_value); 
            if( int(value.length()) >= free_space ){ this->kernel->serial->printf("ERROR: Not enough room for value\r\n"); fclose(lp); return; }
            // Update value
            for( int i = value.length(); i < free_space; i++){ value += " "; }
            fpos_t pos;
            fgetpos( lp, &pos );
            int start = pos - buffer.length() + begin_value - 1;
            fseek(lp, start, SEEK_SET); 
            fputs(value.c_str(), lp);
            fclose(lp);
            return;
        }else{
            buffer += c;                    
        }
    } while (c != EOF);  
    fclose(lp);
    this->kernel->serial->printf("ERROR: configuration key not found\r\n");
}

ConfigValue* Config::value(uint16_t check_sum_a, uint16_t check_sum_b, uint16_t check_sum_c ){
    vector<uint16_t> check_sums;
    check_sums.push_back(check_sum_a); 
    check_sums.push_back(check_sum_b); 
    check_sums.push_back(check_sum_c); 
    return this->value(check_sums);
}    

ConfigValue* Config::value(uint16_t check_sum_a, uint16_t check_sum_b){
    vector<uint16_t> check_sums;
    check_sums.push_back(check_sum_a); 
    check_sums.push_back(check_sum_b); 
    return this->value(check_sums);
}    

ConfigValue* Config::value(uint16_t check_sum){
    vector<uint16_t> check_sums;
    check_sums.push_back(check_sum); 
    return this->value(check_sums);
}    
    
// Get a value from the configuration as a string
// Because we don't like to waste space in Flash with lengthy config parameter names, we take a checksum instead so that the name does not have to be stored
// See get_checksum
ConfigValue* Config::value(vector<uint16_t> check_sums){
    ConfigValue* result = new ConfigValue;
    //uint16_t check_sum = 0;
    //result->check_sum = check_sum; 
    if( this->has_config_file() == false ){
       return result;
    } 
    // Open the config file ( find it if we haven't already found it ) 
    FILE *lp = fopen(this->get_config_file().c_str(), "r");
    string buffer;
    int c; 
    // For each line 
    do {
        c = fgetc (lp);
        if (c == '\n'){
            // We have a new line
            if( buffer[0] == '#' ){ buffer.clear(); continue; } // Ignore comments
            if( buffer.length() < 3 ){ buffer.clear(); continue; } //Ignore empty lines
            size_t begin_key = buffer.find_first_not_of(" ");
            size_t begin_value = buffer.find_first_not_of(" ", buffer.find_first_of(" ", begin_key));
            string key = buffer.substr(begin_key,  buffer.find_first_of(" ", begin_key) - begin_key);
            
            // If this line matches the checksum 
            bool match = true;
            for( unsigned int i = 0; i < check_sums.size(); i++ ){
                uint16_t checksum_node = check_sums[i];
                size_t end_key =  buffer.find_first_of(" .", begin_key);
                string key_node = buffer.substr(begin_key, end_key - begin_key);
               
                //printf("%u(%s) against %u\r\n", get_checksum(key_node), key_node.c_str(), checksum_node);
                if(get_checksum(key_node) != checksum_node ){ 
                    buffer.clear(); 
                    match = false;
                    //printf("no match\r\n");
                    break; 
                }
                //printf("test:<%s>\r\n",key_node.c_str());
                begin_key = end_key + 1;
            } 
            if( match == false ){ 
                //printf("continue\r\n");
                continue; 
            }           

            result->found = true;
            result->key = key;
            result->value = buffer.substr(begin_value, buffer.find_first_of("\r\n# ", begin_value+1)-begin_value);
            break;            
        }else{
            buffer += c;
        }
    } while (c != EOF);  
    fclose(lp);
    return result;
}

bool Config::has_config_file(){
    if( this->config_file_found ){ return true; }
    this->try_config_file("/local/config");
    this->try_config_file("/sd/config");
    if( this->config_file_found ){
        return true; 
    }else{
        return false; 
    }

}

// Get the filename for the config file
string Config::get_config_file(){
    if( this->config_file_found ){ return this->config_file; }
    if( this->has_config_file() ){
        return this->config_file;
    }else{
        printf("ERROR: no config file found\r\n"); 
    }
}

// Tool function for get_config_file
inline void Config::try_config_file(string candidate){
    FILE *lp = fopen(candidate.c_str(), "r");
    if(lp){ this->config_file_found = true; this->config_file = candidate; }
    fclose(lp);
}


void Config::get_module_list(vector<uint16_t>* list, uint16_t family){
    if( this->has_config_file() == false ){ return; } 
    
    // Open the config file ( find it if we haven't already found it ) 
    FILE *lp = fopen(this->get_config_file().c_str(), "r");
    string buffer;
    int c; 
    
    // For each line 
    do {
        c = fgetc (lp);
        if (c == '\n'){
            
            // We have a new line
            if( buffer[0] == '#' ){ buffer.clear(); continue; } // Ignore comments
            if( buffer.length() < 3 ){ buffer.clear(); continue; } //Ignore empty lines
            size_t begin_key = buffer.find_first_not_of(" ");
            size_t begin_value = buffer.find_first_not_of(" ", buffer.find_first_of(" ", begin_key));
            string key = buffer.substr(begin_key,  buffer.find_first_of(" ", begin_key) - begin_key);
            
            // If this line matches the checksum 
            bool match = true;
            uint16_t match_checksum = 0;
            for( unsigned int i = 0; i <= 2; i++ ){
                size_t end_key =  buffer.find_first_of(" .", begin_key);
                string key_node = buffer.substr(begin_key, end_key - begin_key);
                if( i == 0 ){
                    if( family != get_checksum(key_node) ){
                        buffer.clear();
                        match = false;
                        continue;
                    }
                }else if( i == 1 ){
                   match_checksum = get_checksum(key_node);
                }else if( i == 2 ){
                    if( get_checksum(key_node) != 29545 ){ // enable
                        buffer.clear();
                        match = false;
                        continue;
                    } 
                }
                begin_key = end_key + 1;
            } 
            if( match == false ){ continue; }           
            list->push_back(match_checksum); 
            buffer.clear();
        }else{
            buffer += c;
        }

    } while (c != EOF);  
    fclose(lp);
}



