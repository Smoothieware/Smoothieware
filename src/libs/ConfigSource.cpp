#include "utils.h"
#include "ConfigSource.h"
#include "ConfigValue.h"
#include "ConfigCache.h"

#include "stdio.h"

string ConfigSource::process_char_from_ascii_config(int c, ConfigCache *cache)
{
    static string buffer;
    if (c == '\n' || c == EOF) {
        // We have a new line
        if( buffer[0] == '#' ) {
            buffer.clear();    // Ignore comments
            return "";
        }
        if( buffer.length() < 3 ) {
            buffer.clear();    //Ignore empty lines
            return "";
        }
        size_t begin_key = buffer.find_first_not_of(" \t");
        size_t begin_value = buffer.find_first_not_of(" \t", buffer.find_first_of(" \t", begin_key));

        uint16_t check_sums[3];
        get_checksums(check_sums, buffer.substr(begin_key,  buffer.find_first_of(" \t", begin_key) - begin_key).append(" "));

        ConfigValue *result = new ConfigValue;

        result->found = true;
        result->check_sums[0] = check_sums[0];
        result->check_sums[1] = check_sums[1];
        result->check_sums[2] = check_sums[2];

        result->value = buffer.substr(begin_value, buffer.find_first_of("\r\n# \t", begin_value + 1) - begin_value);
        // Append the newly found value to the cache we were passed
        cache->replace_or_push_back(result);

        buffer.clear();


        return result->value;
    } else
        buffer += c;

    return "";
}

string ConfigSource::process_char_from_ascii_config(int c, uint16_t line_checksums[3])
{
    static string buffer;
    string value;
    if (c == '\n' || c == EOF) {
        // We have a new line
        if( buffer[0] == '#' ) {
            buffer.clear();    // Ignore comments
            return "";
        }
        if( buffer.length() < 3 ) {
            buffer.clear();    //Ignore empty lines
            return "";
        }
        size_t begin_key = buffer.find_first_not_of(" \t");
        size_t begin_value = buffer.find_first_not_of(" \t", buffer.find_first_of(" \t", begin_key));

        uint16_t check_sums[3];
        get_checksums(check_sums, buffer.substr(begin_key,  buffer.find_first_of(" \t", begin_key) - begin_key).append(" "));

        if(check_sums[0] == line_checksums[0] && check_sums[1] == line_checksums[1] && check_sums[2] == line_checksums[2] ) {
            value = buffer.substr(begin_value, buffer.find_first_of("\r\n# \t", begin_value + 1) - begin_value);
            buffer.clear();
            return value;
        }

        buffer.clear();
    } else
        buffer += c;
    return value;
}
