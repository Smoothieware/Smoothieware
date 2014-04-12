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
    for (auto &kv : store)  {
        delete kv;
    }
    store.clear();
    storage_t().swap(store);   //  makes sure the vector releases its memory
}

void ConfigCache::add(ConfigValue *v)
{
    store.push_back(v);
}

// If we find an existing value, replace it, otherwise, push it at the back of the list
void ConfigCache::replace_or_push_back(ConfigValue *new_value)
{
    // For each already existing element
    for(auto &cv : store) {
        // If this configvalue matches the checksum
        if(memcmp(new_value->check_sums, cv->check_sums, sizeof(cv->check_sums)) == 0) {
            // Replace with the provided value
            cv =  new_value;
            printf("WARNING: duplicate config line replaced\n");
            return;
        }
    }

    // Value does not already exists, add to the list
    store.push_back(new_value);
}

ConfigValue *ConfigCache::lookup(const uint16_t *check_sums) const
{
    for( auto &cv : store) {
        if(memcmp(check_sums, cv->check_sums, sizeof(cv->check_sums)) == 0)
            return cv;
    }

    return NULL;
}

void ConfigCache::collect(uint16_t family, uint16_t cs, vector<uint16_t> *list)
{
    for( auto &kv : store ) {
        if( kv->check_sums[2] == cs && kv->check_sums[0] == family ) {
            // We found a module enable for this family, add it's number
            list->push_back(kv->check_sums[1]);
        }
    }
}

void ConfigCache::dump(StreamOutput *stream)
{
    int l = 1;
    for( auto &kv : store ) {
        ConfigValue *v = kv;
        stream->printf("%3d - %04X %04X %04X : '%s' - found: %d, default: %d, default-double: %f, default-int: %d\n",
                       l++, v->check_sums[0], v->check_sums[1], v->check_sums[2], v->value.c_str(), v->found, v->default_set, v->default_double, v->default_int );
    }
}
