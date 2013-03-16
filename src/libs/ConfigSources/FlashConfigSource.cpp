/*
      This file is part of Smoothie (http://smoothieware.org/). The motion control part is heavily based on Grbl (https://github.com/simen/grbl).
      Smoothie is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
      Smoothie is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
      You should have received a copy of the GNU General Public License along with Smoothie. If not, see <http://www.gnu.org/licenses/>.
*/

#include "libs/Kernel.h"
#include "ConfigValue.h"
#include "FlashConfigSource.h"
#include "ConfigCache.h"
#include <malloc.h>


using namespace std;
#include <string>



FlashConfigSource::FlashConfigSource(uint16_t name_checksum){
    this->name_checksum = name_checksum;
}

// Transfer all values found in the file to the passed cache
void FlashConfigSource::transfer_values_to_cache( ConfigCache* cache ){
    // start by selecting the last sector
    char* sector = (char*)FLASH_SECTOR_29;
    int c;
    // if the first byte is 0x00, assume that sector data is invalid and select the second to last sector instead
    if( *sector == 0x00 ) {
        sector = (char*)FLASH_SECTOR_28;
        // if the first byte of the selected sector is also 0x00, then assume the second sector is invalid and bail out
        if( *sector == 0x00 ) {
            return;
        }
    }

    do {
        c = *sector;
        process_char_from_ascii_config(c, cache);
        sector++;
    } while (c != EOF);
}

// Return true if the check_sums match
bool FlashConfigSource::is_named( uint16_t check_sum ){
    return check_sum == this->name_checksum;
}

// Write a config setting to the file
void FlashConfigSource::write( string setting, string value ){
/*
    // Open the config file ( find it if we haven't already found it )
    FILE *lp = fopen(this->get_config_file().c_str(), "r+");
    string buffer;
    int c;
    // For each line
    do {
        c = fgetc (lp);
        if (c == '\n' || c == EOF){
            // We have a new line
            if( buffer[0] == '#' ){ buffer.clear(); continue; } // Ignore comments
            if( buffer.length() < 3 ){ buffer.clear(); continue; } //Ignore empty lines
            size_t begin_key = buffer.find_first_not_of(" \t");
            size_t begin_value = buffer.find_first_not_of(" \t", buffer.find_first_of(" \t", begin_key));
            // If this line matches the checksum
            string candidate = buffer.substr(begin_key,  buffer.find_first_of(" \t", begin_key) - begin_key);
            if( candidate.compare(setting) != 0 ){ buffer.clear(); continue; }
            int free_space = int(int(buffer.find_first_of("\r\n#", begin_value+1))-begin_value);
            if( int(value.length()) >= free_space ){
                //this->kernel->streams->printf("ERROR: Not enough room for value\r\n");
                fclose(lp);
                return;
            }
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
    //this->kernel->streams->printf("ERROR: configuration key not found\r\n");
*/
}

// Return the value for a specific checksum
string FlashConfigSource::read( uint16_t check_sums[3] ){
    string value = "";
    // start by selecting the last sector
    char* sector = (char*)FLASH_SECTOR_29;
    int c;
    // if the first byte is 0x00, assume that sector data is invalid and select the second to last sector instead
    if( *sector == 0x00 ) {
        sector = (char*)FLASH_SECTOR_28;
        // if the first byte of the selected sector is also 0x00, then assume the second sector is invalid and bail out
        if( *sector == 0x00 ) {
            return value;
        }
    }

    do {
        c = *sector;
        value = process_char_from_ascii_config(c, check_sums);
        sector++;
        if(value.length())
            return value;
    } while (c != EOF);

    return value;
}


