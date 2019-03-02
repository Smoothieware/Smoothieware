#include "utils.h"
#include "ConfigSource.h"
#include "ConfigValue.h"
#include "ConfigCache.h"

#include "stdio.h"

ConfigValue* ConfigSource::process_line(const string &buffer)
{
    if( buffer[0] == '#' ) {
        return NULL;
    }
    if( buffer.length() < 3 ) {
        return NULL;
    }

    size_t begin_key = buffer.find_first_not_of(" \t");
    if(begin_key == string::npos || buffer[begin_key] == '#') return NULL; // comment line or blank line

    size_t end_key = buffer.find_first_of(" \t", begin_key);
    if(end_key == string::npos) {
        printf("ERROR: config file line %s is invalid, no key value pair found\r\n", buffer.c_str());
        return NULL;
    }

    size_t begin_value = buffer.find_first_not_of(" \t", end_key);
    if(begin_value == string::npos || buffer[begin_value] == '#') {
        printf("ERROR: config file line %s has no value\r\n", buffer.c_str());
        return NULL;
    }

    string key= buffer.substr(begin_key,  end_key - begin_key);
    uint16_t check_sums[3];
    get_checksums(check_sums, key);

    ConfigValue *result = new ConfigValue;

    result->found = true;
    result->check_sums[0] = check_sums[0];
    result->check_sums[1] = check_sums[1];
    result->check_sums[2] = check_sums[2];

    size_t end_value = buffer.find_first_of("\r\n# \t", begin_value + 1);
    size_t vsize = end_value == string::npos ? end_value : end_value - begin_value;
    result->value = buffer.substr(begin_value, vsize);

    //printf("key: %s, value: %s\n\n", key.c_str(), result->value.c_str());
    return result;
}

ConfigValue* ConfigSource::process_line_from_ascii_config(const string &buffer, ConfigCache *cache)
{
    ConfigValue *result = process_line(buffer);
    if(result != NULL) {
        // Append the newly found value to the cache we were passed
        cache->replace_or_push_back(result);
        return result;
    }
    return NULL;
}

string ConfigSource::process_line_from_ascii_config(const string &buffer, uint16_t line_checksums[3])
{
    string value= "";
    ConfigValue *result = process_line(buffer);
    if(result != NULL) {
        if(result->check_sums[0] == line_checksums[0] && result->check_sums[1] == line_checksums[1] && result->check_sums[2] == line_checksums[2]) {
            value= result->value;
        }
        delete result;
    }
    return value;
}
