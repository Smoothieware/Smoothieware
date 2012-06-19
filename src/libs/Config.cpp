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

Config::Config(){
    this->config_cache_loaded = false;

    // Config source for */config files
    this->config_sources.push_back(new FileConfigSource());

    // Pre-load the config cache
    this->config_cache_load();

}

void Config::on_module_loaded(){}

void Config::on_console_line_received( void* argument ){}
void Config::config_get_command( string parameter ){}
void Config::config_set_command( string parameters ){}
void Config::config_load_command( string parameters ){}
void Config::set_string( string setting, string value ){}
void Config::get_module_list(vector<uint16_t>* list, uint16_t family){ }


// Command to load config cache into buffer for multiple reads during init
void Config::config_cache_load(){

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
    ConfigValue* result = this->config_cache[0];
    //if( this->has_config_file() == false ){
    //   return result;
    //} 
    // Check if the config is cached, and load it temporarily if it isn't
    bool cache_preloaded = this->config_cache_loaded;
    if( !cache_preloaded ){ this->config_cache_load(); }
     
    for( int i=1; i<this->config_cache.size(); i++){
        // If this line matches the checksum 
        bool match = true;
        for( unsigned int j = 0; j < check_sums.size(); j++ ){
            uint16_t checksum_node = check_sums[j];
           
            //printf("%u(%s) against %u\r\n", get_checksum(key_node), key_node.c_str(), checksum_node);
            if(this->config_cache[i]->check_sums[j] != checksum_node ){
                match = false;
                break; 
            }
        } 
        if( match == false ){ 
            //printf("continue\r\n");
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



