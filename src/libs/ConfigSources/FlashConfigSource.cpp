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

#define FLASH_BUFFER_SIZE       256
#define FLASH_CONFIG_SIZE       ((32 * 1024) - FLASH_BUFFER_SIZE)
#define BLANK_SECTOR            0x00
#define VALID_SECTOR            0x01


FlashConfigSource::FlashConfigSource(uint16_t name_checksum){
    this->name_checksum = name_checksum;
    this->iap = new IAP();
}

// Transfer all values found in the file to the passed cache
void FlashConfigSource::transfer_values_to_cache( ConfigCache* cache ){
    // start by selecting the last sector
    char* sector = (char*)FLASH_SECTOR_29;
    int c;
    // if the first byte is 0x00, assume that sector data is invalid and select the second to last sector instead
    if( *sector != VALID_SECTOR ) {
        sector = (char*)FLASH_SECTOR_28;
    }
    // if the sector isn't valid bail out
    if( *sector != VALID_SECTOR ) {
        return;
    }
    char* address = sector + FLASH_BUFFER_SIZE;

    int t = 0;
    do {
        c = *address;
        process_char_from_ascii_config(c, cache);
        address++;
        t++;
    } while (c != EOF && t < FLASH_CONFIG_SIZE);
}

// Return true if the check_sums match
bool FlashConfigSource::is_named( uint16_t check_sum ){
    return check_sum == this->name_checksum;
}

// Write a config setting to the file
void FlashConfigSource::write( string setting, string value ){
    char flash_buffer[FLASH_BUFFER_SIZE];
    for(int i=0;i<FLASH_BUFFER_SIZE;i++){
        flash_buffer[i] = 0x00;
    }
    // start by assuming that the last sector is the current valid sector and target second to last
    int current_sector = 29;
    int target_sector = 28;
    char* current_base = (char*)FLASH_SECTOR_29;
    char* target_base = (char*)FLASH_SECTOR_28;
    string buffer;
    int c;
    int n;
    // if first byte is 0x00 then see if second to last is valid
    if( *current_base == BLANK_SECTOR ) {
        current_sector = 28;
        target_sector = 29;
        current_base = (char*)FLASH_SECTOR_28;
        target_base = (char*)FLASH_SECTOR_29;
    }
    char* current_address = current_base + FLASH_BUFFER_SIZE;
    char* target_address = target_base + FLASH_BUFFER_SIZE;
    // if the first byte is not 0x01 then assume both sectors are invalid and start a new empty config
    if( *current_base != VALID_SECTOR ) {
        this->iap->prepare(current_sector, current_sector);
        this->iap->erase(current_sector, current_sector);
        flash_buffer[0] = VALID_SECTOR;
        this->iap->write(flash_buffer, current_base, FLASH_BUFFER_SIZE);
        flash_buffer[0] = EOF;
        this->iap->write(flash_buffer, current_address, FLASH_BUFFER_SIZE);
    }

    this->iap->prepare(target_sector, target_sector);
    this->iap->erase(target_sector, target_sector);
    target_address += FLASH_BUFFER_SIZE;

    n = 0;
    // copy all current values to the target sector
    do {
        c = *current_address;
        current_address++;
        if (c == '\n' || c == EOF){
            if( buffer[0] == '#' ){ buffer.clear(); continue; } // ignore comments
            if( buffer.length() < 3 ){ buffer.clear(); continue; } // ignore empty lines

            size_t begin_key = buffer.find_first_not_of(" \t");
            //size_t begin_value = buffer.find_first_not_of(" \t", buffer.find_first_of(" \t", begin_key));
            // If this line matches the checksum
            string candidate = buffer.substr(begin_key,  buffer.find_first_of(" \t", begin_key) - begin_key);
            if( candidate.compare(setting) != 0 ){ buffer.clear(); continue; } // drop old setting being overwritten
            
            // Copy current value to flash_buffer
            c = '\n';
            buffer += c;
            const char* buf = buffer.c_str();
            for(unsigned int i=0;i<buffer.length();i++){
                flash_buffer[n] = buf[i];
                n++;
                if (n == FLASH_BUFFER_SIZE) {
                    this->iap->write(flash_buffer, target_address, FLASH_BUFFER_SIZE);
                    target_address += FLASH_BUFFER_SIZE;
                    n = 0;
                }
            }
            buffer.clear();
            continue;
        }else{
            buffer += c;
        }
    } while (c != EOF);

    // append the new value to the end
    buffer = setting + " " + value + "\n";
    const char* buf = buffer.c_str();
    for(unsigned int i=0;i<buffer.length();i++){
        flash_buffer[n] = buf[i];
        n++;
        if (n == FLASH_BUFFER_SIZE) {
            this->iap->write(flash_buffer, target_address, FLASH_BUFFER_SIZE);
            target_address += FLASH_BUFFER_SIZE;
            n = 0;
        }
    }
    flash_buffer[n] = EOF;
    for(n++;n<FLASH_BUFFER_SIZE;n++){
        flash_buffer[n] = 0x00;
    }
    this->iap->write(flash_buffer, target_address, FLASH_BUFFER_SIZE);

    // finally set the target sector live and disable the current sector
    for(int i=0;i<FLASH_BUFFER_SIZE;i++){
        flash_buffer[n] = 0x00;
    }
    flash_buffer[0] = VALID_SECTOR; 
    this->iap->write(flash_buffer, target_base, FLASH_BUFFER_SIZE);
    flash_buffer[0] = BLANK_SECTOR;
    this->iap->prepare(current_sector, current_sector);
    this->iap->write(flash_buffer, current_base, FLASH_BUFFER_SIZE);
}

// Return the value for a specific checksum
string FlashConfigSource::read( uint16_t check_sums[3] ){
    string value = "";
    // start by selecting the last sector
    char* sector = (char*)FLASH_SECTOR_29;
    int c;
    // if the first byte is 0x00, assume that sector data is invalid and select the second to last sector instead
    if( *sector != VALID_SECTOR ) {
        sector = (char*)FLASH_SECTOR_28;
    }
    // if the sector isn't valid bail out
    if( *sector != VALID_SECTOR ) {
        return value;
    }
    char* address = sector + FLASH_BUFFER_SIZE;

    int t = 0;
    do {
        c = *address;
        value = process_char_from_ascii_config(c, check_sums);
        address++;
        t++;
        if(value.length())
            return value;
    } while (c != EOF && t < FLASH_CONFIG_SIZE);
    return value;
}


