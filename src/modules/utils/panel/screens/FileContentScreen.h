/*
 * FileContentScreen.h
 *
 *  Created on: 20/04/2015
 *      Author: DelbenAfonso
 */

#ifndef SRC_MODULES_UTILS_PANEL_SCREENS_FILECONTENTSCREEN_H_
#define SRC_MODULES_UTILS_PANEL_SCREENS_FILECONTENTSCREEN_H_

#include <stdio.h>
#include "PanelScreen.h"


class FileContentScreen : public PanelScreen {
	public:
		FileContentScreen();
		void on_enter();
		void on_exit();
		void on_refresh();
        void clicked_line(uint16_t line);
        void display_menu_line(uint16_t line);
        void setCurrentPath(std::string path);
	private:
        uint16_t count_file_content();
        std::string current_path;
        FILE *current_file;
};

#endif /* SRC_MODULES_UTILS_PANEL_SCREENS_FILECONTENTSCREEN_H_ */
