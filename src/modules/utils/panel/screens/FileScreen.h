/*  
      This file is part of Smoothie (http://smoothieware.org/). The motion control part is heavily based on Grbl (https://github.com/simen/grbl).
      Smoothie is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
      Smoothie is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
      You should have received a copy of the GNU General Public License along with Smoothie. If not, see <http://www.gnu.org/licenses/>. 
*/

#ifndef FILESCREEN_H
#define FILESCREEN_H

#include "libs/Kernel.h"
#include "libs/nuts_bolts.h"
#include "libs/utils.h"
#include "libs/Pin.h"
#include "LcdBase.h"
#include "Panel.h"
#include "PanelScreen.h"

#include <string>
using namespace std;

class FileScreen : public PanelScreen {
    public:
        FileScreen();
        void on_enter();
        void on_refresh(); 
        void on_main_loop();
        void enter_folder(std::string folder);
        uint16_t count_folder_content(std::string folder);
        void clicked_line(uint16_t line);
        void display_menu_line(uint16_t line);
        bool is_a_folder( string path );
        string file_at(uint16_t line);

        std::string current_folder;

    private:
        void play(string path);
        bool start_play;
        string play_path;
};






#endif
