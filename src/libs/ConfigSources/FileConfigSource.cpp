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

using namespace std;
#include <string>


FileConfigSource::FileConfigSource(string config_file, uint16_t name_checksum){
    this->name_checksum = name_checksum;
    this->config_file = config_file;
    this->config_file_found = false;
}

// Transfer all values found in the file to the passed cache
void FileConfigSource::transfer_values_to_cache( ConfigCache* cache ){
    
    // Default empty value
    ConfigValue* result = new ConfigValue;
    
    if( this->has_config_file() == false ){return;}
    // Open the config file ( find it if we haven't already found it ) 
    FILE *lp = fopen(this->get_config_file().c_str(), "r");
    string buffer;
    int c; 
    // For each line 
    do {
        c = fgetc (lp);
        if (c == '\n' || c == EOF){
            // We have a new line
            if( buffer[0] == '#' ){ buffer.clear(); continue; } // Ignore comments
            if( buffer.length() < 3 ){ buffer.clear(); continue; } //Ignore empty lines
            size_t begin_key = buffer.find_first_not_of(" ");
            size_t begin_value = buffer.find_first_not_of(" ", buffer.find_first_of(" ", begin_key));
            string key = buffer.substr(begin_key,  buffer.find_first_of(" ", begin_key) - begin_key).append(" ");
            vector<uint16_t> check_sums = get_checksums(key);
            
            result = new ConfigValue;
            result->found = true;
            result->check_sums = check_sums;
            result->value = buffer.substr(begin_value, buffer.find_first_of("\r\n# ", begin_value+1)-begin_value);
            
            // Append the newly found value to the cache we were passed 
            cache->replace_or_push_back(result);
            
            buffer.clear();
        }else{
            buffer += c;
        }
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
            size_t begin_key = buffer.find_first_not_of(" ");
            size_t begin_value = buffer.find_first_not_of(" ", buffer.find_first_of(" ", begin_key));
            // If this line matches the checksum
            string candidate = buffer.substr(begin_key,  buffer.find_first_of(" ", begin_key) - begin_key);
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
}

// Return the value for a specific checksum
string FileConfigSource::read( vector<uint16_t> check_sums ){

    string value = "";

    if( this->has_config_file() == false ){return value;}
    // Open the config file ( find it if we haven't already found it ) 
    FILE *lp = fopen(this->get_config_file().c_str(), "r");
    string buffer;
    int c; 
    // For each line 
    do {
        c = fgetc (lp);
        if (c == '\n' || c == EOF){
            // We have a new line
            if( buffer[0] == '#' ){ buffer.clear(); continue; } // Ignore comments
            if( buffer.length() < 3 ){ buffer.clear(); continue; } //Ignore empty lines
            size_t begin_key = buffer.find_first_not_of(" ");
            size_t begin_value = buffer.find_first_not_of(" ", buffer.find_first_of(" ", begin_key));
            string key = buffer.substr(begin_key,  buffer.find_first_of(" ", begin_key) - begin_key).append(" ");
            vector<uint16_t> line_checksums = get_checksums(key);

            if(check_sums == line_checksums){
                value = buffer.substr(begin_value, buffer.find_first_of("\r\n# ", begin_value+1)-begin_value);
                break;
            }
            
            buffer.clear();
        }else{
            buffer += c;
        }
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
    }
}






