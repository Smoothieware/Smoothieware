/*  
      This file is part of Smoothie (http://smoothieware.org/). The motion control part is heavily based on Grbl (https://github.com/simen/grbl).
      Smoothie is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
      Smoothie is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
      You should have received a copy of the GNU General Public License along with Smoothie. If not, see <http://www.gnu.org/licenses/>. 
*/

#include "mbed.h"
#include <string>
using std::string;
#include "libs/Kernel.h"
#include "Config.h"

Config::Config(){
    config_file_found = false; 
}

// Get a value from the configuration as a string
// Because we don't like to waste space in Flash with lengthy config parameter names, we take a checksum instead so that the name does not have to be stored
// See get_checksum
string Config::get_string(uint16_t checksum){
    // Open the config file ( find it if we haven't already found it ) 
    FILE *lp = fopen(this->get_config_file().c_str(), "r");
    string buffer;
    char c; 
    // For each line 
    do {
        c = fgetc (lp);
        if (c == '\n'){
            // We have a new line
            if( buffer[0] == '#' ){ buffer.clear(); continue; } // Ignore comments
            size_t begin_key = buffer.find_first_not_of(" ");
            size_t begin_value = buffer.find_first_not_of(" ", buffer.find_first_of(" ", begin_key));
            // If this line matches the checksum 
            if(this->get_checksum(buffer.substr(begin_key,  buffer.find_first_of(" ", begin_key) - begin_key)) != checksum){ buffer.clear(); continue; }
            fclose(lp);
            return buffer.substr(begin_value, buffer.find_first_of("\r\n# ", begin_value+1)-begin_value);
        }else{
            buffer += c;                    
        }
    } while (c != EOF);  
    fclose(lp);
    printf("ERROR: configuration key not found\r\n");
}

double Config::get(uint16_t checksum){
    return atof(this->get_string( checksum ).c_str());
}


uint16_t Config::get_checksum(string to_check){
   // From: http://en.wikipedia.org/wiki/Fletcher%27s_checksum 
   uint16_t sum1 = 0;
   uint16_t sum2 = 0;
   for( int index = 0; index < to_check.length(); ++index ){
      sum1 = (sum1 + to_check[index]) % 255;
      sum2 = (sum2 + sum1) % 255;
   }
   return (sum2 << 8) | sum1;
}

string Config::get_config_file(){
    if( this->config_file_found ){ return this->config_file; }
    this->try_config_file("/local/config");
    this->try_config_file("/sd/config");
    if( this->config_file_found ){
        return this->config_file;
    }else{
        printf("ERROR: no config file found\r\n"); 
    }
}

void Config::try_config_file(string candidate){
    FILE *lp = fopen(candidate.c_str(), "r");
    if(lp){ this->config_file_found = true; this->config_file = candidate; }
    fclose(lp);
}



