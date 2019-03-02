/*
      This file is part of Smoothie (http://smoothieware.org/). The motion control part is heavily based on Grbl (https://github.com/simen/grbl).
      Smoothie is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
      Smoothie is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
      You should have received a copy of the GNU General Public License along with Smoothie. If not, see <http://www.gnu.org/licenses/>.
*/

#include "libs/Kernel.h"
#include "Panel.h"
#include "PanelScreen.h"
#include "LcdBase.h"
#include "MainMenuScreen.h"
#include "FileScreen.h"
#include "libs/nuts_bolts.h"
#include "libs/utils.h"
#include <string>
#include "libs/SerialMessage.h"
#include "StreamOutput.h"
#include "DirHandle.h"
#include "mri.h"

using std::string;

FileScreen::FileScreen()
{
    this->start_play = false;
}

// When entering this screen
void FileScreen::on_enter()
{
    THEPANEL->lcd->clear();

    // Default folder to enter
    this->enter_folder(THEKERNEL->current_path.c_str());
}

void FileScreen::on_exit()
{
    // reset to root directory, I think this is less confusing
    THEKERNEL->current_path= "/";
}

// For every ( potential ) refresh of the screen
void FileScreen::on_refresh()
{
    if ( THEPANEL->menu_change() ) {
        this->refresh_menu();
    }
    if ( THEPANEL->click() ) {
        this->clicked_line(THEPANEL->get_menu_current_line());
    }
}

// Enter a new folder
void FileScreen::enter_folder(const char *folder)
{
    // Remember where we are
    THEKERNEL->current_path= folder;

    // We need the number of lines to setup the menu
    uint16_t number_of_files_in_folder = this->count_folder_content();

    // Setup menu
    THEPANEL->setup_menu(number_of_files_in_folder + 1); // same number of files as menu items
    THEPANEL->enter_menu_mode();

    // Display menu
    this->refresh_menu();
}

// Called by the panel when refreshing the menu, display .. then all files in the current dir
void FileScreen::display_menu_line(uint16_t line)
{
    if ( line == 0 ) {
        THEPANEL->lcd->printf("..");
    } else {
        bool isdir;
        string fn= this->file_at(line - 1, isdir).substr(0, 18);
        if(isdir) {
            if(fn.size() >= 18) fn.back()= '/';
            else fn.append("/");
        }
        THEPANEL->lcd->printf("%s", fn.c_str());
    }
}

// When a line is clicked in the menu, act
void FileScreen::clicked_line(uint16_t line)
{
    if ( line == 0 ) {
        string path= THEKERNEL->current_path;
        if(path == "/") {
            // Exit file navigation
            THEPANEL->enter_screen(this->parent);
        } else {
            // Go up one folder
            path = path.substr(0, path.find_last_of('/'));
            if (path.empty()) {
                path= "/";
            }
            this->enter_folder(path.c_str());
        }
    } else {
        // Enter file or dir
        bool isdir;
        string path= absolute_from_relative(this->file_at(line - 1, isdir));
        if(isdir) {
            this->enter_folder(path.c_str());
            return;
        }

        // start printing that file...
        this->play_path = path;
        this->start_play = true;
    }
}

// only filter files that have a .g, .ngc or .nc in them and does not start with a .
bool FileScreen::filter_file(const char *f)
{
    string fn= lc(f);
    return (fn.at(0) != '.') &&
             ((fn.find(".g") != string::npos) ||
              (fn.find(".ngc") != string::npos) ||
              (fn.find(".nc") != string::npos));
}

// Find the "line"th file in the current folder
string FileScreen::file_at(uint16_t line, bool& isdir)
{
    DIR *d;
    struct dirent *p;
    uint16_t count = 0;
    d = opendir(THEKERNEL->current_path.c_str());
    if (d != NULL) {
        while ((p = readdir(d)) != NULL) {
            // only filter files that have a .g in them and directories not starting with a .
          if(((p->d_isdir && p->d_name[0] != '.') || filter_file(p->d_name)) && count++ == line ) {
                isdir= p->d_isdir;
                string fn= p->d_name;
                closedir(d);
                return fn;
            }
        }
    }

    if (d != NULL) closedir(d);
    isdir= false;
    return "";
}

// Count how many files there are in the current folder that have a .g in them and does not start with a .
uint16_t FileScreen::count_folder_content()
{
    DIR *d;
    struct dirent *p;
    uint16_t count = 0;
    d = opendir(THEKERNEL->current_path.c_str());
    if (d != NULL) {
        while ((p = readdir(d)) != NULL) {
            if((p->d_isdir && p->d_name[0] != '.') || filter_file(p->d_name)) count++;
        }
        closedir(d);
        return count;
    }
    return 0;
}

void FileScreen::on_main_loop()
{
    if (this->start_play) {
        this->start_play = false;
        THEPANEL->set_playing_file(this->play_path);
        this->play(this->play_path.c_str());
        THEPANEL->enter_screen(this->parent);
        return;
    }
}

void FileScreen::play(const char *path)
{
    struct SerialMessage message;
    message.message = string("play ") + path;
    message.stream = &(StreamOutput::NullStream);
    THEKERNEL->call_event(ON_CONSOLE_LINE_RECEIVED, &message );
}
