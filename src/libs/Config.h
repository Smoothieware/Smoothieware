/*  
      This file is part of Smoothie (http://smoothieware.org/). The motion control part is heavily based on Grbl (https://github.com/simen/grbl).
      Smoothie is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
      Smoothie is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
      You should have received a copy of the GNU General Public License along with Smoothie. If not, see <http://www.gnu.org/licenses/>. 
*/

#ifndef CONFIG_H
#define CONFIG_H
#include "mbed.h"
#include "libs/Kernel.h"
#include <string>
using std::string;

#define config_get_checksum        46310
#define config_set_checksum        55538
#define config_load_checksum       3143

class Config : public Module {
    public:
        Config();

        void on_module_loaded();
        void on_console_line_received( void* argument );
        string get_string(uint16_t check_sum);
        double get(uint16_t check_sum);
        string get_config_file();
        void try_config_file(string candidate);

        string config_file;         // Path to the config file
        bool   config_file_found;   // Wether or not the config file's location is known
};

#endif
