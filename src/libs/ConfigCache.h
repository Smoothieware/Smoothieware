/*
      This file is part of Smoothie (http://smoothieware.org/). The motion control part is heavily based on Grbl (https://github.com/simen/grbl).
      Smoothie is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
      Smoothie is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
      You should have received a copy of the GNU General Public License along with Smoothie. If not, see <http://www.gnu.org/licenses/>. 
*/

#ifndef CONFIGCACHE_H
#define CONFIGCACHE_H

using namespace std;
#include <vector>

#include "ConfigValue.h"

class ConfigCache : public std::vector<ConfigValue*> {
    public:
        ConfigCache(){}
        
        // If we find an existing value, replace it, otherwise, push it at the back of the list 
        void replace_or_push_back(ConfigValue* new_value){
       
           bool value_exists = false; 
            // For each already existing element 
            for( int i=1; i<this->size(); i++){
                // If this configvalue matches the checksum 
                bool match = true;
                for( unsigned int j = 0; j < new_value->check_sums.size(); j++ ){
                    uint16_t checksum_node = new_value->check_sums[j];
                    if(this->at(i)->check_sums[j] != checksum_node ){
                        match = false;
                        break; 
                    }
                } 
                if( match == false ){ continue; }
                value_exists = true;
                // Replace with the provided value 
                delete this->at(i);
                this->at(i) = new_value;
                break;
            }

            // Value does not already exists, add to the list
            if( value_exists == false ){ 
                this->push_back(new_value);
            }

        }

};



#endif
