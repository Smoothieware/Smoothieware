/*
 * FileContentScreen.cpp
 *
 *  Created on: 20/04/2015
 *      Author: DelbenAfonso
 */

#include "FileContentScreen.h"
#include "libs/Kernel.h"
#include "libs/SerialMessage.h"
#include "Panel.h"
#include "PanelScreen.h"
#include "MainMenuScreen.h"
#include "ControlScreen.h"
#include "libs/nuts_bolts.h"
#include "libs/utils.h"
#include <string>
#include "modules/robot/RobotPublicAccess.h"
#include "PublicData.h"
#include "checksumm.h"
#include "LcdBase.h"

#include "SerialConsole.h"

#include <math.h>
#include <stdio.h>

FileContentScreen::FileContentScreen() {
	// TODO Auto-generated constructor stub

}

// When entering this screen
void FileContentScreen::on_enter(){
    THEPANEL->enter_menu_mode();
    THEPANEL->setup_menu(5);
    this->refresh_menu();
}

// For every ( potential ) refresh of the screen
void FileContentScreen::on_refresh()
{
    if ( THEPANEL->menu_change() ) {
        this->refresh_menu();
    }
    if ( THEPANEL->click() ) {
        this->clicked_line(THEPANEL->get_menu_current_line());
    }

}

void FileContentScreen::on_exit()
{

}

void FileContentScreen::clicked_line(uint16_t line)
{
	switch ( line ) {
	        case 0: THEPANEL->enter_screen(this->parent); break;
	}

}

void FileContentScreen::display_menu_line(uint16_t line)
{
    // in menu mode
    switch ( line ) {
		case 0: THEPANEL->lcd->printf("Back");  break;
		case 1: THEPANEL->lcd->printf("Teste0"); break;
		case 2: THEPANEL->lcd->printf("Teste1");  break;
		case 3: THEPANEL->lcd->printf("Teste2");  break;
		case 4: THEPANEL->lcd->printf("Teste3"); break;
    }

}

uint16_t FileContentScreen::count_file_content()
{
	uint16_t lineNumber = 0;
	this->current_file = fopen(this->current_path.c_str(),"r");
	 if(this->current_file != NULL) {
		// fread(this->current_file);
		 THEKERNEL->serial->printf("Reading to file: %s\r\nok\r\n", this->current_path.c_str());
	} else {
		 THEKERNEL->serial->printf("open failed, File: %s.\r\nok\r\n", this->current_path.c_str());
	}
	return 5;
}

void FileContentScreen::setCurrentPath(std::string path)
{
	this->current_path = path;
	THEKERNEL->serial->printf("Caminho: %s",this->current_path.c_str());
}
