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
#include <stdint.h>
#include <map>

class ConfigValue;
class StreamOutput;

class ConfigCache {
    public:
        ConfigCache();
        ~ConfigCache();
        void clear();

        void add(ConfigValue* v);

        // lookup and return the entru that matches the check sums,return NULL if not found
        ConfigValue *lookup(const uint16_t *check_sums) const;

        // collect enabled checksums of the given family
        void collect(uint16_t family, uint16_t cs, vector<uint16_t> *list);

        // If we find an existing value, replace it, otherwise, push it at the back of the list
        void replace_or_push_back(ConfigValue* new_value);

        // used for debugging, dumps the cache to a stream
        void dump(StreamOutput *stream);

    private:
        typedef vector<ConfigValue*> storage_t;
        storage_t store;
};



#endif
