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
#include "checksumm.h"
#include "utils.h"
#include <malloc.h>

using namespace std;
#include <string>
#include <string.h>

#define include_checksum     CHECKSUM("include")

FileConfigSource::FileConfigSource(string config_file, const char *name)
{
    this->name_checksum = get_checksum(name);
    this->config_file = config_file;
    this->config_file_found = false;
}

bool FileConfigSource::readLine(string& line, int lineno, FILE *fp)
{
    char buf[132];
    char *l= fgets(buf, sizeof(buf)-1, fp);
    if(l != NULL) {
        if(buf[strlen(l)-1] != '\n') {
            // truncate long lines
            if(lineno != 0) {
                // report if it is not truncating a comment
                if(strchr(buf, '#') == NULL)
                    printf("Truncated long line %d in: %s\n", lineno, config_file.c_str());
            }
            // read until the next \n or eof
            int c;
            while((c=fgetc(fp)) != '\n' && c != EOF) /* discard */;
        }
        line.assign(buf);
        return true;
    }

    return false;
}

// Transfer all values found in the file to the passed cache
void FileConfigSource::transfer_values_to_cache( ConfigCache *cache )
{
    if( !this->has_config_file() ) {
        return;
    }
    transfer_values_to_cache( cache, this->get_config_file().c_str());
}

void FileConfigSource::transfer_values_to_cache( ConfigCache *cache, const char * file_name )
{
    if( !file_exists(file_name) ) {
        return;
    }

    // Open the config file ( find it if we haven't already found it )
    FILE *lp = fopen(file_name, "r");

    int ln= 1;
    // For each line
    while(!feof(lp)) {
        string line;
        if(readLine(line, ln++, lp)) {
            // process the config line and store the value in cache
            ConfigValue* cv = process_line_from_ascii_config(line, cache);

            // if this line is an include directive then attempt to read the included file
            if(cv->check_sums[0] == include_checksum) {
                string inc_file_name = cv->value.c_str();
                if(!file_exists(inc_file_name)) {
                    // if the file is not found at the location entered then look around for it a bit
                    if(inc_file_name[0] != '/') inc_file_name = "/" + inc_file_name;
                    string path(file_name);
                    path = path.substr(0,path.find_last_of('/'));

                    // first check the path of the current config file
                    if(file_exists(path + inc_file_name)) inc_file_name = path + inc_file_name;
                    // then check root locations
                    else if(file_exists("/sd" + inc_file_name)) inc_file_name = "/sd" + inc_file_name;
                    else if(file_exists("/local" + inc_file_name)) inc_file_name = "/local" + inc_file_name;
                }
                if(file_exists(inc_file_name)) {
                    // save position in current config file
                    fpos_t pos;
                    fgetpos(lp, &pos);

                    // open and read the included file
                    freopen(inc_file_name.c_str(), "r", lp);
                    this->transfer_values_to_cache(cache, inc_file_name.c_str());

                    // reopen the current config file and restore position
                    freopen(file_name, "r", lp);
                    fsetpos(lp, &pos);
                }
            }
        }else break;
    }
    fclose(lp);
}

// Return true if the check_sums match
bool FileConfigSource::is_named( uint16_t check_sum )
{
    return check_sum == this->name_checksum;
}

// OverWrite or append a config setting to the file
bool FileConfigSource::write( string setting, string value )
{
    if( !this->has_config_file() ) {
        return false;
    }

    uint16_t setting_checksums[3];
    get_checksums(setting_checksums, setting );

    // Open the config file ( find it if we haven't already found it )
    FILE *lp = fopen(this->get_config_file().c_str(), "r+");

    // search each line for a match
    while(!feof(lp)) {
        string line;
        fpos_t bol, eol;
        fgetpos( lp, &bol ); // get start of line
        if(readLine(line, 0, lp)) {
            fgetpos( lp, &eol ); // get end of line
            if(!process_line_from_ascii_config(line, setting_checksums).empty()) {
                // found it
                unsigned int free_space = eol - bol - 4; // length of line
                // check we have enough space for this insertion
                if( (setting.length() + value.length() + 3) > free_space ) {
                    //THEKERNEL->streams->printf("ERROR: Not enough room for value\r\n");
                    fclose(lp);
                    return false;
                }

                // Update line, leaves whatever was at end of line there just overwrites the key and value
                fseek(lp, bol, SEEK_SET);
                fputs(setting.c_str(), lp);
                fputs(" ", lp);
                fputs(value.c_str(), lp);
                fputs(" #", lp);
                fclose(lp);
                return true;
            }
        }else break;
    }

    // not found so append the new value
    fclose(lp);
    lp = fopen(this->get_config_file().c_str(), "a");
    fputs("\n", lp);
    fputs(setting.c_str(), lp);
    fputs("         ", lp);
    fputs(value.c_str(), lp);
    fputs("         # added\n", lp);
    fclose(lp);

    return true;
}

// Return the value for a specific checksum
string FileConfigSource::read( uint16_t check_sums[3] )
{
    string value = "";

    if( this->has_config_file() == false ) {
        return value;
    }

    // Open the config file ( find it if we haven't already found it )
    FILE *lp = fopen(this->get_config_file().c_str(), "r");
    // For each line
    while(!feof(lp)) {
        string line;
         if(readLine(line, 0, lp)) {
            value = process_line_from_ascii_config(line, check_sums);
            if(!value.empty()) break; // found it
        }else break;
    }
    fclose(lp);

    return value;
}

// Return wether or not we have a readable config file
bool FileConfigSource::has_config_file()
{
    if( this->config_file_found ) {
        return true;
    }
    this->try_config_file(this->config_file);
    if( this->config_file_found ) {
        return true;
    } else {
        return false;
    }

}

// Tool function for get_config_file
inline void FileConfigSource::try_config_file(string candidate)
{
    if(file_exists(candidate)) {
        this->config_file_found = true;
    }
}

// Get the filename for the config file
string FileConfigSource::get_config_file()
{
    if( this->config_file_found ) {
        return this->config_file;
    }
    if( this->has_config_file() ) {
        return this->config_file;
    } else {
        printf("ERROR: no config file found\r\n");
        return "";
    }
}






