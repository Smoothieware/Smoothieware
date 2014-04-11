#include "ConfigCache.h"
#include "ConfigValue.h"

#include "libs/StreamOutput.h"

ConfigCache::ConfigCache()
{
}

ConfigCache::~ConfigCache()
{
    clear();
}

void ConfigCache::clear()
{
    for (ConfigValue *i : store)  {
        delete i;
    }
    vector<ConfigValue*>().swap(store);   //  makes sure the vector releases its memory
}

// If we find an existing value, replace it, otherwise, push it at the back of the list
void ConfigCache::replace_or_push_back(ConfigValue *new_value)
{

    bool value_exists = false;

    // For each already existing element
    for(ConfigValue*& i: store) {
        // If this configvalue matches the checksum
        bool match = true;
        unsigned int counter = 0;
        while( counter < 3 && new_value->check_sums[counter] != 0x0000 ) {
            if(i->check_sums[counter] != new_value->check_sums[counter]  ) {
                match = false;
                break;
            }
            counter++;
        }
        if( !match ) {
            continue;
        }
        value_exists = true;

        // Replace with the provided value
        i=  new_value;
        break;
    }

    // Value does not already exists, add to the list
    if( !value_exists ) {
        store.push_back(new_value);
    }

}

ConfigValue *ConfigCache::lookup(const uint16_t *check_sums) const
{
   for( ConfigValue *cv : store) {
        if(memcmp(check_sums, cv->check_sums, sizeof(cv->check_sums)) == 0)
            return cv;
    }

    return NULL;
}

void ConfigCache::collect(uint16_t family, uint16_t cs, vector<uint16_t> *list)
{
    for( ConfigValue *value : store ) {
        if( value->check_sums[2] == cs && value->check_sums[0] == family ) {
            // We found a module enable for this family, add it's number
            list->push_back(value->check_sums[1]);
        }
    }
}

void ConfigCache::dump(StreamOutput *stream)
{
    int l= 1;
    for( ConfigValue *v : store ) {
        stream->printf("%3d - %04X %04X %04X : '%s' - found: %d, default: %d, default-double: %f, default-int: %d\n",
            l++, v->check_sums[0], v->check_sums[1], v->check_sums[2], v->value.c_str(), v->found, v->default_set, v->default_double, v->default_int );
    }
}
