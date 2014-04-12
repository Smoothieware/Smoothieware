#include "ConfigCache.h"
#include "ConfigValue.h"

#include "libs/StreamOutput.h"
#include "md5.h"

ConfigCache::ConfigCache()
{
}

ConfigCache::~ConfigCache()
{
    clear();
}

static uint32_t md5sum(const uint16_t *array)
{
    MD5 md5;
    md5.update((const unsigned char *)array, (size_t)3*sizeof(uint16_t));
    md5.finalize();
    uint32_t r;
    md5.bindigest(&r, sizeof(r));
    return r;
}

void ConfigCache::clear()
{
    for (auto &kv : store)  {
        ConfigValue *i = kv.second;
        delete i;
    }
    store.clear();
}

void ConfigCache::add(ConfigValue *v)
{
    uint32_t k = md5sum(v->check_sums);
    store[k] = v;
}

// If we find an existing value, replace it, otherwise, push it at the back of the list
void ConfigCache::replace_or_push_back(ConfigValue *new_value)
{
    add(new_value);
}

ConfigValue *ConfigCache::lookup(const uint16_t *check_sums) const
{
    uint32_t k = md5sum(check_sums);
    storage_t::const_iterator i = store.find(k);
    if(i == store.end())
        return NULL;
    return i->second;
}

void ConfigCache::collect(uint16_t family, uint16_t cs, vector<uint16_t> *list)
{
    for( auto& kv : store ) {
        ConfigValue *i = kv.second;
        if( i->check_sums[2] == cs && i->check_sums[0] == family ) {
            // We found a module enable for this family, add it's number
            list->push_back(i->check_sums[1]);
        }
    }
}

void ConfigCache::dump(StreamOutput *stream)
{
    int l = 1;
    for( auto& kv : store ) {
        ConfigValue *v= kv.second;
        stream->printf("%3d - %04X %04X %04X : '%s' - found: %d, default: %d, default-double: %f, default-int: %d\n",
                       l++, v->check_sums[0], v->check_sums[1], v->check_sums[2], v->value.c_str(), v->found, v->default_set, v->default_double, v->default_int );
    }
}
