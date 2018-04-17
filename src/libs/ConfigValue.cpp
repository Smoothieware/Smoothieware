#include "ConfigValue.h"
#include "StreamOutputPool.h"

#include "libs/Kernel.h"
#include "libs/utils.h"
#include "libs/Pin.h"
#include "Pwm.h"


#define printErrorandExit(...) THEKERNEL->streams->printf(__VA_ARGS__); // exit(1);

#include <vector>
#include <stdio.h>

ConfigValue::ConfigValue()
{
    clear();
}

void ConfigValue:: clear()
{
    this->found = false;
    this->default_set = false;
    this->check_sums[0] = 0x0000;
    this->check_sums[1] = 0x0000;
    this->check_sums[2] = 0x0000;
    this->default_double= 0.0F;
    this->default_int= 0;
    this->value= "";
}

ConfigValue::ConfigValue(uint16_t *cs) {
    memcpy(this->check_sums, cs, sizeof(this->check_sums));
    this->found = false;
    this->default_set = false;
    this->value= "";
}

ConfigValue::ConfigValue(const ConfigValue& to_copy)
{
    this->found = to_copy.found;
    this->default_set = to_copy.default_set;
    memcpy(this->check_sums, to_copy.check_sums, sizeof(this->check_sums));
    this->value.assign(to_copy.value);
}

ConfigValue& ConfigValue::operator= (const ConfigValue& to_copy)
{
    if( this != &to_copy ){
        this->found = to_copy.found;
        this->default_set = to_copy.default_set;
        memcpy(this->check_sums, to_copy.check_sums, sizeof(this->check_sums));
        this->value.assign(to_copy.value);
    }
    return *this;
}

ConfigValue *ConfigValue::required()
{
    if( !this->found ) {
        printErrorandExit("could not find config setting, please see http://smoothieware.org/configuring-smoothie\r\n");
    }
    return this;
}

float ConfigValue::as_number()
{
    if( this->found == false && this->default_set == true ) {
        return this->default_double;
    } else {
        char *endptr = NULL;
        string str = remove_non_number(this->value);
        const char *cp= str.c_str();
        float result = strtof(cp, &endptr);
        if( endptr <= cp ) {
            printErrorandExit("config setting with value '%s' and checksums[%04X,%04X,%04X] is not a valid number, please see http://smoothieware.org/configuring-smoothie\r\n", this->value.c_str(), this->check_sums[0], this->check_sums[1], this->check_sums[2] );
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
        const char *cp= str.c_str();
        int result = strtol(cp, &endptr, 10);
        if( endptr <= cp ) {
            printErrorandExit("config setting with value '%s' and checksums[%04X,%04X,%04X] is not a valid int, please see http://smoothieware.org/configuring-smoothie\r\n", this->value.c_str(), this->check_sums[0], this->check_sums[1], this->check_sums[2] );
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
        return this->default_int;
    } else {
        return this->value.find_first_of("ty1") != string::npos;
    }
}

ConfigValue *ConfigValue::by_default(int val)
{
    this->default_set = true;
    this->default_int = val;
    this->default_double = val; // we need to set both becuase sometimes an integer is passed when it should be a float
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

bool ConfigValue::has_characters( const char *mask )
{
    if( this->value.find_first_of(mask) != string::npos ) {
        return true;
    } else {
        return false;
    }
}

bool ConfigValue::is_inverted()
{
    return this->has_characters("!");
}

