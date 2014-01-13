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
#include "ConfigValue.h"
#include "ConfigSource.h"
#include "ConfigCache.h"
#include "libs/nuts_bolts.h"
#include "libs/utils.h"
#include "libs/SerialMessage.h"
#include "libs/ConfigSources/FileConfigSource.h"
#include "libs/ConfigSources/FirmConfigSource.h"

// Add various config sources. Config can be fetched from several places.
// All values are read into a cache, that is then used by modules to read their configuration
Config::Config(){
    this->config_cache_loaded = false;

    // Config source for firm config found in src/config.default
    this->config_sources.push_back( new FirmConfigSource() );

    // Config source for */config files
    FileConfigSource* fcs = NULL;
    if( file_exists("/local/config") )
        fcs = new FileConfigSource("/local/config", LOCAL_CONFIGSOURCE_CHECKSUM);
    else if( file_exists("/local/config.txt") )
        fcs = new FileConfigSource("/local/config.txt", LOCAL_CONFIGSOURCE_CHECKSUM);
    if( fcs != NULL ){
        this->config_sources.push_back( fcs );
        fcs = NULL;
    }
    if( file_exists("/sd/config") )
       fcs = new FileConfigSource("/sd/config",    SD_CONFIGSOURCE_CHECKSUM   );
    else if( file_exists("/sd/config.txt") )
       fcs = new FileConfigSource("/sd/config.txt",    SD_CONFIGSOURCE_CHECKSUM   );
    if( fcs != NULL )
        this->config_sources.push_back( fcs );

    // Pre-load the config cache
    this->config_cache_load();

}

void Config::on_module_loaded(){}

void Config::on_console_line_received( void* argument ){}

// Set a value in the config cache, but not in any config source
void Config::set_string( string setting, string value ){
    ConfigValue* cv = new ConfigValue;
    cv->found = true;
    get_checksums(cv->check_sums, setting);
    cv->value = value;

    this->config_cache.replace_or_push_back(cv);

    THEKERNEL->call_event(ON_CONFIG_RELOAD);
}

// Get a list of modules, used by module "pools" that look for the "enable" keyboard to find things like "moduletype.modulename.enable" as the marker of a new instance of a module
void Config::get_module_list(vector<uint16_t>* list, uint16_t family){
    for( unsigned int i=1; i<this->config_cache.size(); i++){
        ConfigValue* value = this->config_cache.at(i);
        //if( value->check_sums.size() == 3 && value->check_sums.at(2) == CHECKSUM("enable") && value->check_sums.at(0) == family ){
        if( value->check_sums[2] == CHECKSUM("enable") && value->check_sums[0] == family ){
            // We found a module enable for this family, add it's number
            list->push_back(value->check_sums[1]);
        }
    }
}


// Command to load config cache into buffer for multiple reads during init
void Config::config_cache_load(){

    // First clear the cache
    this->config_cache_clear();

    // First element is a special empty ConfigValue for values not found
    ConfigValue* result = new ConfigValue;
    this->config_cache.push_back(result);

    // For each ConfigSource in our stack
    for( unsigned int i = 0; i < this->config_sources.size(); i++ ){
        ConfigSource* source = this->config_sources[i];
        source->transfer_values_to_cache(&this->config_cache);
    }

    this->config_cache_loaded = true;
}

// Command to clear the config cache after init
void Config::config_cache_clear(){
    while( ! this->config_cache.empty() ){
        delete this->config_cache.back();
        this->config_cache.pop_back();
    }
    this->config_cache_loaded = false;
}

// Three ways to read a value from the config, depending on adress length
ConfigValue* Config::value(uint16_t check_sum_a, uint16_t check_sum_b, uint16_t check_sum_c ){
    uint16_t check_sums[3];
    check_sums[0] = check_sum_a;
    check_sums[1] = check_sum_b;
    check_sums[2] = check_sum_c;
    return this->value(check_sums);
}

ConfigValue* Config::value(uint16_t check_sum_a, uint16_t check_sum_b){
    uint16_t check_sums[3];
    check_sums[0] = check_sum_a;
    check_sums[1] = check_sum_b;
    check_sums[2] = 0x0000;
    return this->value(check_sums);
}

ConfigValue* Config::value(uint16_t check_sum){
    uint16_t check_sums[3];
    check_sums[0] = check_sum;
    check_sums[1] = 0x0000;
    check_sums[2] = 0x0000;
    return this->value(check_sums);
}

// Get a value from the configuration as a string
// Because we don't like to waste space in Flash with lengthy config parameter names, we take a checksum instead so that the name does not have to be stored
// See get_checksum
ConfigValue* Config::value(uint16_t check_sums[]){
    ConfigValue* result = this->config_cache[0];
    bool cache_preloaded = this->config_cache_loaded;
    if( !cache_preloaded ){ this->config_cache_load(); }

    for( unsigned int i=1; i<this->config_cache.size(); i++){
        // If this line matches the checksum
        bool match = true;
        unsigned int counter = 0;
        while(check_sums[counter] != 0x0000 && counter <= 2 ){
             if(this->config_cache[i]->check_sums[counter] != check_sums[counter] ){
                match = false;
                break;
            }
            counter++;
        }
        if( match == false ){
            continue;
        }
        result = this->config_cache[i];
        break;
    }

    if( !cache_preloaded ){
        this->config_cache_clear();
    }
    return result;
}



