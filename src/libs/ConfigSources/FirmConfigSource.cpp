/*
      This file is part of Smoothie (http://smoothieware.org/). The motion control part is heavily based on Grbl (https://github.com/simen/grbl).
      Smoothie is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
      Smoothie is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
      You should have received a copy of the GNU General Public License along with Smoothie. If not, see <http://www.gnu.org/licenses/>.
*/

#include "libs/Kernel.h"
#include "ConfigValue.h"
#include "FirmConfigSource.h"
#include "ConfigCache.h"
#include <malloc.h>


using namespace std;
#include <string>

extern char _binary_config_default_start;
extern char _binary_config_default_end;


FirmConfigSource::FirmConfigSource(uint16_t name_checksum){
    this->name_checksum = name_checksum;
}

// Transfer all values found in the file to the passed cache
void FirmConfigSource::transfer_values_to_cache( ConfigCache* cache ){

    // Default empty value
    ConfigValue* result = new ConfigValue;

    char* p = &_binary_config_default_start;
    string buffer;
    int c;
    // For each line
    while( p != &_binary_config_default_end ){
        c = *p++;
        if (c == '\n' || c == '\r' || c == EOF){

            // We have a new line
            if( buffer[0] == '#' ){ buffer.clear(); continue; } // Ignore comments
            if( buffer.length() < 3 ){ buffer.clear(); continue; } //Ignore empty lines
            size_t begin_key = buffer.find_first_not_of(" ");
            size_t begin_value = buffer.find_first_not_of(" ", buffer.find_first_of(" ", begin_key));

            uint16_t check_sums[3];
            get_checksums(check_sums, buffer.substr(begin_key,  buffer.find_first_of(" ", begin_key) - begin_key).append(" "));

            result = new ConfigValue;

            result->found = true;
            result->check_sums[0] = check_sums[0];
            result->check_sums[1] = check_sums[1];
            result->check_sums[2] = check_sums[2];

            result->value = buffer.substr(begin_value, buffer.find_first_of("\r\n# ", begin_value+1)-begin_value);
            // Append the newly found value to the cache we were passed
            cache->replace_or_push_back(result);

            buffer.clear();

        }else{
            buffer += c;
        }
    }
}

// Return true if the check_sums match
bool FirmConfigSource::is_named( uint16_t check_sum ){
    return check_sum == this->name_checksum;
}

// Write a config setting to the file *** FirmConfigSource is read only ***
void FirmConfigSource::write( string setting, string value ){
    //this->kernel->streams->printf("ERROR: FirmConfigSource is read only\r\n");
}

// Return the value for a specific checksum
string FirmConfigSource::read( uint16_t check_sums[3] ){

    string value = "";

    char* p = &_binary_config_default_start;
    string buffer;
    int c;
    // For each line
    while( p != &_binary_config_default_end ){
        c = *p++;
        if (c == '\n' || c == '\r' || c == EOF){
            // We have a new line
            if( buffer[0] == '#' ){ buffer.clear(); continue; } // Ignore comments
            if( buffer.length() < 3 ){ buffer.clear(); continue; } //Ignore empty lines
            size_t begin_key = buffer.find_first_not_of(" \t");
            size_t begin_value = buffer.find_first_not_of(" \t", buffer.find_first_of(" \t", begin_key));
            string key = buffer.substr(begin_key,  buffer.find_first_of(" \t", begin_key) - begin_key).append(" ");

            uint16_t line_checksums[3];
            get_checksums(line_checksums, key);

            if(check_sums[0] == line_checksums[0] && check_sums[1] == line_checksums[1] && check_sums[2] == line_checksums[2] ){
                value = buffer.substr(begin_value, buffer.find_first_of("\r\n# ", begin_value+1)-begin_value);
                break;
            }

            buffer.clear();
        }else{
            buffer += c;
        }
    }

    return value;
}

