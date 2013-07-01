/*  
      This file is part of Smoothie (http://smoothieware.org/). The motion control part is heavily based on Grbl (https://github.com/simen/grbl).
      Smoothie is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
      Smoothie is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
      You should have received a copy of the GNU General Public License along with Smoothie. If not, see <http://www.gnu.org/licenses/>. 
*/

#include "libs/Kernel.h"
#include "Panel.h"
#include "libs/nuts_bolts.h"
#include "libs/utils.h"
#include <string>
using namespace std;
#include "Button.h"
#include "PanelScreen.h"
#include "screens/MainMenuScreen.h"
#include "modules/utils/player/PlayerPublicAccess.h"

#include "panels/I2CLCD.h"
#include "panels/VikiLCD.h"
#include "panels/Smoothiepanel.h"

Panel::Panel(){
    this->counter_changed = false;
    this->click_changed = false;
    this->refresh_flag = false;
    this->enter_menu_mode();
    this->lcd= NULL;
    this->do_buttons = false;
    this->idle_time= 0;
    strcpy(this->playing_file, "Playing file");
}

Panel::~Panel() {
    delete this->lcd;
}

void Panel::on_module_loaded(){
    // Exit if this module is not enabled
    if( !this->kernel->config->value( panel_checksum, enable_checksum )->by_default(false)->as_bool() ){
        delete this;
        return;
    } 

    // Register for events
    this->register_for_event(ON_IDLE);
    this->register_for_event(ON_MAIN_LOOP);

    // Initialise the LCD, see which LCD to use
    if (this->lcd != NULL) delete this->lcd;
    int lcd_cksm = get_checksum(this->kernel->config->value(panel_checksum, lcd_checksum)->by_default("i2c")->as_string());

    // Note checksums are not const expressions when in debug mode, so don't use switch
    if(lcd_cksm == i2c_lcd_checksum) {
        this->lcd = new I2CLCD();
    }else if(lcd_cksm == viki_lcd_checksum) {
        this->lcd = new VikiLCD();
    }else if(lcd_cksm == smoothiepanel_checksum) {
        this->lcd = new Smoothiepanel();
    }else{
        // no lcd type defined
        return;
    }

    // some panels may need access to this global info
    this->lcd->setPanel(this);

    // some encoders may need more clicks to move menu, this is a divisor and is in config as it is
    // an end user usability issue
    this->menu_offset = this->kernel->config->value( panel_checksum, menu_offset_checksum )->by_default(0)->as_number();

    // load jogging feedrates in mm/min
    jogging_speed_mm_min[0]= this->kernel->config->value( panel_checksum, jog_x_feedrate_checksum )->by_default(3000.0)->as_number();
    jogging_speed_mm_min[1]= this->kernel->config->value( panel_checksum, jog_y_feedrate_checksum )->by_default(3000.0)->as_number();
    jogging_speed_mm_min[2]= this->kernel->config->value( panel_checksum, jog_z_feedrate_checksum )->by_default(300.0)->as_number();

    // load the default preset temeratures
    default_hotend_temperature= this->kernel->config->value( panel_checksum, hotend_temp_checksum )->by_default(185.0)->as_number();
    default_bed_temperature= this->kernel->config->value( panel_checksum, bed_temp_checksum )->by_default(60.0)->as_number();

    this->encoder_click_resolution= this->lcd->getEncoderResolution();
    this->lcd->init();
    this->lcd->printf("Starting...");
    
    this->up_button.up_attach(    this, &Panel::on_up );
    this->down_button.up_attach(  this, &Panel::on_down );
    this->click_button.up_attach( this, &Panel::on_click_release );
    this->back_button.up_attach(  this, &Panel::on_back );

    this->kernel->slow_ticker->attach( 100,  this, &Panel::button_tick );
    this->kernel->slow_ticker->attach( 1000, this, &Panel::encoder_check );

    // Default top screen
    this->top_screen = new MainMenuScreen();
    this->top_screen->set_panel(this);
    this->enter_screen(this->top_screen->watch_screen); // default first screen is watch screen even though its parent is Mainmenu
    
    // Refresh timer
    this->kernel->slow_ticker->attach( 20, this, &Panel::refresh_tick );

}

// Enter a screen, we only care about it now
void Panel::enter_screen(PanelScreen* screen){
    screen->panel = this;
    this->current_screen = screen;
    this->reset_counter();
    this->current_screen->on_enter();
}

// Reset the counter
void Panel::reset_counter(){
    *this->counter = 0;
    this->counter_changed = false;
}

// Indicate the idle loop we want to call the refresh hook in the current screen
uint32_t Panel::refresh_tick(uint32_t dummy){
    this->refresh_flag = true;
    this->idle_time++;
    return 0;
}

// Encoder pins changed
uint32_t Panel::encoder_check(uint32_t dummy){
    // TODO if encoder reads go through i2c like on smoothie panel this needs to be
    // optionally done in idle loop, however when reading encoder directly it needs to be done
    // frequently, smoothie panel will return an actual delta count so won't miss any if polled slowly
    static int encoder_counter = 0;
    int change = lcd->readEncoderDelta();
    encoder_counter += change;
    // TODO divisor needs to be configurable
    if( change != 0 && encoder_counter % this->encoder_click_resolution == 0 ){
        this->counter_changed = true;
        (*this->counter) += change;
        this->idle_time= 0;
    }
    return 0;
}

// Read and update each button
uint32_t Panel::button_tick(uint32_t dummy){
    this->do_buttons = true;
    return 0;
}

// on main loop, we can send gcodes or do anything that waits in this loop
void Panel::on_main_loop(void* argument){
    this->current_screen->on_main_loop();
    this->lcd->on_main_loop();
}

// On idle things, we don't want to do shit in interrupts
// don't queue gcodes in this
void Panel::on_idle(void* argument){

    // after being idle for a while switch to Watch screen
    if(this->idle_time > 20*5) { // 5 seconds
        this->idle_time= 0;
        if(this->top_screen->watch_screen != this->current_screen) {
            this->enter_screen(this->top_screen->watch_screen);
            // TODO do we need to reset any state?
        }
        
        return;
    }
    
    if(this->do_buttons) {
        // we don't want to do I2C in interrupt mode
        this->do_buttons = false;

        // read the actual buttons
        int but= lcd->readButtons();
        if(but != 0) this->idle_time= 0;
        
        // fire events if the buttons are active and debounce is satisfied
        this->up_button.check_signal(but&BUTTON_UP);
        this->down_button.check_signal(but&BUTTON_DOWN);
        this->back_button.check_signal(but&BUTTON_LEFT);
        this->click_button.check_signal(but&BUTTON_SELECT);
    }
    
    // If we are in menu mode and the position has changed
    if( this->mode == MENU_MODE && this->counter_change() ){
        this->menu_update();
    }

    // If we are in control mode
    if( this->mode == CONTROL_MODE && this->counter_change() ){
        this->control_value_update();
    }

    // If we must refresh
    if( this->refresh_flag ){
        this->refresh_flag = false;
        this->current_screen->on_refresh();
        this->lcd->on_refresh();
    }
}

// Hooks for button clicks
uint32_t Panel::on_up(uint32_t dummy){
    // this is simulating encoder clicks, but needs to be inverted to
    // increment values on up
    int inc= (this->mode == CONTROL_MODE) ? 1 : -1;
    *this->counter += inc;
    this->counter_changed = true;
    return 0;
}
uint32_t Panel::on_down(uint32_t dummy){
    int inc= (this->mode == CONTROL_MODE) ? -1 : 1;
    *this->counter += inc;
    this->counter_changed = true;
    return 0;
}

// on most menu screens will go back to previous higher menu
uint32_t Panel::on_back(uint32_t dummy){
    if(this->mode == MENU_MODE && this->current_screen->parent != NULL) {
        this->enter_screen(this->current_screen->parent);
    }
    return 0;
}

uint32_t Panel::on_click_release(uint32_t dummy){
    // TODO make configurable, including turning off
    // buzz is ignored on panels that do not support buzz
    lcd->buzz(60,300); // 50ms 300Hz
    this->click_changed = true;
    this->idle_time= 0;
    return 0;
}

bool Panel::counter_change(){ if( this->counter_changed ){ this->counter_changed = false; return true; }else{ return false; } }
bool Panel::click(){ if( this->click_changed ){ this->click_changed = false; return true; }else{ return false; } }


// Enter menu mode
void Panel::enter_menu_mode(){
    this->mode = MENU_MODE;
    this->counter = &this->menu_selected_line;
    this->menu_changed = false;
}

void Panel::setup_menu(uint16_t rows, uint16_t lines){
    this->menu_selected_line = 0;
    this->menu_start_line = 0; 
    this->menu_rows = rows;
    this->menu_lines = lines;
}

uint16_t Panel::menu_current_line(){
    return this->menu_selected_line >> this->menu_offset;
}

void Panel::menu_update(){

    // Limits, up and down 
    this->menu_selected_line = this->menu_selected_line % ( this->menu_rows<<this->menu_offset );
    while( this->menu_selected_line < 0 ){ this->menu_selected_line += this->menu_rows << this->menu_offset; }

    // What to display
    this->menu_start_line = 0;
    if( this->menu_rows > this->menu_lines ){
        if( this->menu_current_line() >= 2 ){
            this->menu_start_line = this->menu_current_line() - 1;
        }
        if( this->menu_current_line() > this->menu_rows - this->menu_lines ){
            this->menu_start_line = this->menu_rows - this->menu_lines;
        }
    }

    this->menu_changed = true;
}

bool Panel::menu_change(){
    if( this->menu_changed ){ this->menu_changed = false; return true; }else{ return false; }
}

bool Panel::control_value_change(){
    if( this->control_value_changed ){ this->control_value_changed = false; return true; }else{ return false; }
}


bool Panel::enter_control_mode(double passed_normal_increment, double passed_pressed_increment){
    this->mode = CONTROL_MODE;
    this->normal_increment  = passed_normal_increment;
    this->pressed_increment = passed_pressed_increment;
    this->counter = &this->control_normal_counter;
    this->control_normal_counter = 0;
    this->control_pressed_counter = 0;
    this->control_base_value = 0;
    return true;
}

void Panel::control_value_update(){
    // TODO what do we do here?
    this->control_value_changed = true;
}

void Panel::set_control_value(double value){
    this->control_base_value = value;
}

double Panel::get_control_value(){
    return this->control_base_value + (this->control_normal_counter*this->normal_increment);
}

bool Panel::is_playing() const {
    void *returned_data;

    bool ok= THEKERNEL->public_data->get_value( player_checksum, is_playing_checksum, &returned_data );
    if(ok) {
        bool b= *static_cast<bool*>(returned_data);
        return b;
    }
    return false;
}

void  Panel::set_playing_file(string f) {
    // just copy the first 20 characters after the first / if there
    size_t n= f.find_last_of('/');
    if(n == string::npos) n= 0;
    strncpy(playing_file, f.substr(n+1, 19).c_str(), sizeof(playing_file));
    playing_file[sizeof(playing_file)-1]= 0;
}
