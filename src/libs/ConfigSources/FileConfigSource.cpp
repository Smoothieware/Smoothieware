/*
      This file is part of Smoothie (http://smoothieware.org/). The motion control part is heavily based on Grbl (https://github.com/simen/grbl).
      Smoothie is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
      Smoothie is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
      You should have received a copy of the GNU General Public License along with Smoothie. If not, see <http://www.gnu.org/licenses/>.
*/

#include "libs/Kernel.h"
#include "ConfigValue.h"
#include "FileConfigSource.h"
#include "ConfigCache.h"
#include <malloc.h>


using namespace std;
#include <string>


FileConfigSource::FileConfigSource(string config_file, uint16_t name_checksum){
    this->name_checksum = name_checksum;
    this->config_file = config_file;
    this->config_file_found = false;
}

// Transfer all values found in the file to the passed cache
void FileConfigSource::transfer_values_to_cache( ConfigCache* cache ){

    if( this->has_config_file() == false ){return;}
    // Open the config file ( find it if we haven't already found it )
    FILE *lp = fopen(this->get_config_file().c_str(), "r");
    int c;
    // For each line
    do {
        process_char_from_ascii_config(c = fgetc(lp), cache);
    } while (c != EOF);
    fclose(lp);
}

// Return true if the check_sums match
bool FileConfigSource::is_named( uint16_t check_sum ){
    return check_sum == this->name_checksum;
}

// Write a config setting to the file
void FileConfigSource::write( string setting, string value ){
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
                //THEKERNEL->streams->printf("ERROR: Not enough room for value\r\n");
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
    //THEKERNEL->streams->printf("ERROR: configuration key not found\r\n");
}

// Return the value for a specific checksum
string FileConfigSource::read( uint16_t check_sums[3] ){

    string value = "";

    if( this->has_config_file() == false ){return value;}
    // Open the config file ( find it if we haven't already found it )
    FILE *lp = fopen(this->get_config_file().c_str(), "r");
    int c;
    // For each line
    do {
        c = fgetc (lp);
        process_char_from_ascii_config(c, check_sums);
    } while (c != EOF);
    fclose(lp);

    return value;
}

// Return wether or not we have a readable config file
bool FileConfigSource::has_config_file(){
    if( this->config_file_found ){ return true; }
    this->try_config_file(this->config_file);
    if( this->config_file_found ){
        return true;
    }else{
        return false;
    }

}

// Tool function for get_config_file
inline void FileConfigSource::try_config_file(string candidate){
    if(file_exists(candidate)){ this->config_file_found = true; }
}

// Get the filename for the config file
string FileConfigSource::get_config_file(){
    if( this->config_file_found ){ return this->config_file; }
    if( this->has_config_file() ){
        return this->config_file;
    }else{
        printf("ERROR: no config file found\r\n");
		return "";
    }
}






