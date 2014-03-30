#include "ConfigValue.h"
#include "StreamOutputPool.h"

#include "libs/Kernel.h"
#include "libs/utils.h"
#include "libs/Pin.h"
#include "Pwm.h"


#define printErrorandExit(...) THEKERNEL->streams->printf(__VA_ARGS__); exit(1);

#include <vector>
#include <stdio.h>

ConfigValue::ConfigValue()
{
    this->found = false;
    this->default_set = false;
    this->check_sums[0] = 0x0000;
    this->check_sums[1] = 0x0000;
    this->check_sums[2] = 0x0000;
}

ConfigValue *ConfigValue::required()
{
    if( this->found == true ) {
        return this;
    } else {
        printErrorandExit("could not find config setting, please see http://smoothieware.org/configuring-smoothie\r\n");
    }
}

float ConfigValue::as_number()
{
    if( this->found == false && this->default_set == true ) {
        return this->default_double;
    } else {
        char *endptr = NULL;
        string str = remove_non_number(this->value);
        float result = strtof(str.c_str(), &endptr);
        if( endptr <= str.c_str() ) {
            printErrorandExit("config setting with value '%s' and checksums[%u,%u,%u] is not a valid number, please see http://smoothieware.org/configuring-smoothie\r\n", this->value.c_str(), this->check_sums[0], this->check_sums[1], this->check_sums[2] );
        }
        return result;
    }
}

int ConfigValue::as_int()
{
    if( this->found == false && this->default_set == true ) {
        return this->default_int;
    } else {
        char *endptr = NULL;
        string str = remove_non_number(this->value);
        int result = strtol(str.c_str(), &endptr, 10);
        if( endptr <= str.c_str() ) {
            printErrorandExit("config setting with value '%s' and checksums[%u,%u,%u] is not a valid number, please see http://smoothieware.org/configuring-smoothie\r\n", this->value.c_str(), this->check_sums[0], this->check_sums[1], this->check_sums[2] );
        }
        return result;
    }
}

std::string ConfigValue::as_string()
{
    return this->value;
}

bool ConfigValue::as_bool()
{
    if( this->found == false && this->default_set == true ) {
        return this->default_double;
    } else {
        if( this->value.find_first_of("ty1") != string::npos ) {
            return true;
        } else {
            return false;
        }
    }
}

ConfigValue *ConfigValue::by_default(int val)
{
    this->default_set = true;
    this->default_int = val;
    this->default_double = val;
    return this;
}

ConfigValue *ConfigValue::by_default(float val)
{
    this->default_set = true;
    this->default_double = val;
    return this;
}

ConfigValue *ConfigValue::by_default(string val)
{
    if( this->found ) {
        return this;
    }
    this->default_set = true;
    this->value = val;
    return this;
}

bool ConfigValue::has_characters( string mask )
{
    if( this->value.find_first_of(mask) != string::npos ) {
        return true;
    } else {
        return false;
    }
}

bool ConfigValue::is_inverted()
{
    return this->has_characters(string("!"));
}

