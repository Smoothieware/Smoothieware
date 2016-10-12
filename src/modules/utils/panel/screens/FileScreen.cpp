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
#include "FileSorter.h"


FileScreen::FileScreen()
{
    this->start_play = false;
    this->file_sorter = NULL;
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
    delete_file_sorter();
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

    // read the folder contents into the file sorter.
    if ( this->file_sorter == NULL ){
        this->file_sorter = new FileSorter(string(folder), filter_file);
    } else {
        this->file_sorter->open_directory(string(folder));
    }

    // We need the number of lines to setup the menu
    //
    // if there was an error reading the directory, set
    // the count to the raw directory count.  the file
    // list will revert to unsorted in this case.
    uint16_t number_of_files_in_folder = 0;
    if ( this->file_sorter != NULL ) {
        if ( this->file_sorter->is_sorted() ) {
            number_of_files_in_folder = this->file_sorter->get_file_count();
        } else {
            number_of_files_in_folder = this->file_sorter->get_total_file_count();
        }
    }

    // Setup menu
    THEPANEL->setup_menu(number_of_files_in_folder + 1);// same number of files as menu items
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

// Find the "line"th file in the current folder
string FileScreen::file_at(uint16_t line, bool& isdir)
{
    if ( this->file_sorter != NULL ) {
        const char* fn = this->file_sorter->file_at(line, isdir);
        if ( fn != NULL ) {
            return string(fn);
        }
    }

    isdir= false;
    return "";
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

void FileScreen::delete_file_sorter(void)
{
    delete this->file_sorter;
    this->file_sorter = NULL;
}

// only filter files that have a .g, .ngc or .nc in them and does not start with a .
bool FileScreen::filter_file(struct dirent* file_info)
{
    string fn = lc(file_info->d_name);
    return ((file_info->d_isdir && file_info->d_name[0] != '.') ||  // match all non-hidden directories (no .dir_name)
            ((fn.at(0) != '.') &&                                   // match non-hidden files (no .file_name) with one of the extensions below:
                    ((fn.find(".g") != string::npos) ||             // files with .gXXX extensions
                    (fn.find(".ngc") != string::npos) ||            // files with .nacXXX extensions
                    (fn.find(".nc") != string::npos))));            // files with .ncXXX extensions
}
