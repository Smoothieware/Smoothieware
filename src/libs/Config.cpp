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
#include "StreamOutputPool.h"

// Add various config sources. Config can be fetched from several places.
// All values are read into a cache, that is then used by modules to read their configuration
Config::Config()
{
    this->config_cache = NULL;

    // Config source for firm config found in src/config.default
    this->config_sources.push_back( new FirmConfigSource("firm") );

    // Config source for */config files
    FileConfigSource *fcs = NULL;
    if( file_exists("/local/config") )
        fcs = new FileConfigSource("/local/config", "local");
    else if( file_exists("/local/config.txt") )
        fcs = new FileConfigSource("/local/config.txt", "local");
    if( fcs != NULL ) {
        this->config_sources.push_back( fcs );
        fcs = NULL;
    }
    if( file_exists("/sd/config") )
        fcs = new FileConfigSource("/sd/config", "sd");
    else if( file_exists("/sd/config.txt") )
        fcs = new FileConfigSource("/sd/config.txt", "sd");
    if( fcs != NULL )
        this->config_sources.push_back( fcs );
}

void Config::on_module_loaded() {}

void Config::on_console_line_received( void *argument ) {}

// Get a list of modules, used by module "pools" that look for the "enable" keyboard to find things like "moduletype.modulename.enable" as the marker of a new instance of a module
void Config::get_module_list(vector<uint16_t> *list, uint16_t family)
{
    this->config_cache->collect(family, CHECKSUM("enable"), list);
}

// Command to load config cache into buffer for multiple reads during init
void Config::config_cache_load(bool parse)
{
    // First clear the cache
    this->config_cache_clear();

    this->config_cache= new ConfigCache;
    if(parse) {
        // For each ConfigSource in our stack
        for( ConfigSource *source : this->config_sources ) {
            source->transfer_values_to_cache(this->config_cache);
        }
    }
}

// Command to clear the config cache after init
void Config::config_cache_clear()
{
    delete this->config_cache;
    this->config_cache= NULL;
}

// Three ways to read a value from the config, depending on adress length
ConfigValue *Config::value(uint16_t check_sum_a, uint16_t check_sum_b, uint16_t check_sum_c )
{
    uint16_t check_sums[3];
    check_sums[0] = check_sum_a;
    check_sums[1] = check_sum_b;
    check_sums[2] = check_sum_c;
    return this->value(check_sums);
}

static ConfigValue dummyValue;

// Get a value from the configuration as a string
// Because we don't like to waste space in Flash with lengthy config parameter names, we take a checksum instead so that the name does not have to be stored
// See get_checksum
ConfigValue *Config::value(uint16_t check_sums[])
{
    if( !is_config_cache_loaded() ) {
        THEKERNEL->streams->printf("ERROR: calling value after config cache has been cleared\n");
        // note this will cause whatever called it to blow up!
        return NULL;
    }

    ConfigValue *result = this->config_cache->lookup(check_sums);

    if(result == NULL) {
        // create a dummy value for this to play with, each call requires it's own value not a shared one
        // result= new ConfigValue(check_sums);
        // config_cache->add(result);
        dummyValue.clear();
        result = &dummyValue;
    }

    return result;
}



