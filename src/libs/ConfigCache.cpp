#include "ConfigCache.h"
#include "ConfigValue.h"

// If we find an existing value, replace it, otherwise, push it at the back of the list
void ConfigCache::replace_or_push_back(ConfigValue *new_value)
{

    bool value_exists = false;
    // For each already existing element
    for( unsigned int i = 1; i < this->size(); i++) {
        // If this configvalue matches the checksum
        bool match = true;
        unsigned int counter = 0;
        while( counter < 3 && new_value->check_sums[counter] != 0x0000 ) {
            if(this->at(i)->check_sums[counter] != new_value->check_sums[counter]  ) {
                match = false;
                break;
            }
            counter++;
        }
        if( match == false ) {
            continue;
        }
        value_exists = true;

        // Replace with the provided value
        delete this->at(i);
        this->at(i) = new_value;
        break;
    }

    // Value does not already exists, add to the list
    if( value_exists == false ) {
        this->push_back(new_value);
    }

}
