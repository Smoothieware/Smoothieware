/*
      This file is part of Smoothie (http://smoothieware.org/). The motion control part is heavily based on Grbl (https://github.com/simen/grbl).
      Smoothie is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
      Smoothie is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
      You should have received a copy of the GNU General Public License along with Smoothie. If not, see <http://www.gnu.org/licenses/>.
*/

#include "libs/Kernel.h"
#include "Panel.h"
#include "PanelScreen.h"

#include "libs/nuts_bolts.h"
#include "libs/utils.h"
#include "Button.h"

#include "modules/utils/player/PlayerPublicAccess.h"
#include "screens/CustomScreen.h"
#include "screens/MainMenuScreen.h"
#include "SlowTicker.h"
#include "Gcode.h"
#include "Pauser.h"
#include "TemperatureControlPublicAccess.h"
#include "ModifyValuesScreen.h"
#include "PublicDataRequest.h"
#include "PublicData.h"

#include "panels/I2CLCD.h"
#include "panels/VikiLCD.h"
#include "panels/Smoothiepanel.h"
#include "panels/ReprapDiscountGLCD.h"
#include "panels/ST7565.h"
#include "panels/UniversalAdapter.h"

#include "version.h"
#include "checksumm.h"
#include "ConfigValue.h"

#define panel_checksum             CHECKSUM("panel")
#define enable_checksum            CHECKSUM("enable")
#define lcd_checksum               CHECKSUM("lcd")
#define i2c_lcd_checksum           CHECKSUM("i2c_lcd")
#define viki_lcd_checksum          CHECKSUM("viki_lcd")
#define smoothiepanel_checksum     CHECKSUM("smoothiepanel")
#define panelolu2_checksum         CHECKSUM("panelolu2")
#define rrd_glcd_checksum          CHECKSUM("reprap_discount_glcd")
#define st7565_glcd_checksum       CHECKSUM("st7565_glcd")
#define universal_adapter_checksum CHECKSUM("universal_adapter")

#define menu_offset_checksum        CHECKSUM("menu_offset")
#define encoder_resolution_checksum CHECKSUM("encoder_resolution")
#define jog_x_feedrate_checksum     CHECKSUM("alpha_jog_feedrate")
#define jog_y_feedrate_checksum     CHECKSUM("beta_jog_feedrate")
#define jog_z_feedrate_checksum     CHECKSUM("gamma_jog_feedrate")
#define	longpress_delay_checksum	CHECKSUM("longpress_delay")

#define hotend_temp_checksum CHECKSUM("hotend_temperature")
#define bed_temp_checksum    CHECKSUM("bed_temperature")

Panel* Panel::instance= nullptr;

Panel::Panel()
{
    instance= this;
    this->counter_changed = false;
    this->click_changed = false;
    this->refresh_flag = false;
    this->enter_menu_mode();
    this->lcd = NULL;
    this->do_buttons = false;
    this->do_encoder = false;
    this->idle_time = 0;
    this->start_up = true;
    this->current_screen = NULL;
    strcpy(this->playing_file, "Playing file");
}

Panel::~Panel()
{
    delete this->lcd;
}

void Panel::on_module_loaded()
{
    // Exit if this module is not enabled
    if ( !THEKERNEL->config->value( panel_checksum, enable_checksum )->by_default(false)->as_bool() ) {
        delete this;
        return;
    }

    // Initialise the LCD, see which LCD to use
    if (this->lcd != NULL) delete this->lcd;
    int lcd_cksm = get_checksum(THEKERNEL->config->value(panel_checksum, lcd_checksum)->by_default("i2c")->as_string());

    // Note checksums are not const expressions when in debug mode, so don't use switch
    if (lcd_cksm == i2c_lcd_checksum) {
        this->lcd = new I2CLCD();
    } else if (lcd_cksm == viki_lcd_checksum) {
        this->lcd = new VikiLCD();
        this->lcd->set_variant(0);
    } else if (lcd_cksm == panelolu2_checksum) {
        this->lcd = new VikiLCD();
        this->lcd->set_variant(1);
    } else if (lcd_cksm == smoothiepanel_checksum) {
        this->lcd = new Smoothiepanel();
    } else if (lcd_cksm == rrd_glcd_checksum) {
        this->lcd = new ReprapDiscountGLCD();
    } else if (lcd_cksm == st7565_glcd_checksum) {
        this->lcd = new ST7565();
    } else if (lcd_cksm == universal_adapter_checksum) {
        this->lcd = new UniversalAdapter();
    } else {
        // no lcd type defined
        return;
    }

    // these need to be called here as they need the config cache loaded as they enumerate modules
    this->custom_screen= new CustomScreen();
    setup_temperature_screen();

    // some panels may need access to this global info
    this->lcd->setPanel(this);

    // the number of screen lines the panel supports
    this->screen_lines = this->lcd->get_screen_lines();

    // some encoders may need more clicks to move menu, this is a divisor and is in config as it is
    // an end user usability issue
    this->menu_offset = THEKERNEL->config->value( panel_checksum, menu_offset_checksum )->by_default(0)->as_number();

    // override default encoder resolution if needed
    this->encoder_click_resolution = THEKERNEL->config->value( panel_checksum, encoder_resolution_checksum )->by_default(this->lcd->getEncoderResolution())->as_number();

    // load jogging feedrates in mm/min
    jogging_speed_mm_min[0] = THEKERNEL->config->value( panel_checksum, jog_x_feedrate_checksum )->by_default(3000.0f)->as_number();
    jogging_speed_mm_min[1] = THEKERNEL->config->value( panel_checksum, jog_y_feedrate_checksum )->by_default(3000.0f)->as_number();
    jogging_speed_mm_min[2] = THEKERNEL->config->value( panel_checksum, jog_z_feedrate_checksum )->by_default(300.0f )->as_number();

    // load the default preset temeratures
    default_hotend_temperature = THEKERNEL->config->value( panel_checksum, hotend_temp_checksum )->by_default(185.0f )->as_number();
    default_bed_temperature    = THEKERNEL->config->value( panel_checksum, bed_temp_checksum    )->by_default(60.0f  )->as_number();


    this->up_button.up_attach(    this, &Panel::on_up );
    this->down_button.up_attach(  this, &Panel::on_down );
    this->click_button.up_attach( this, &Panel::on_select );
    this->back_button.up_attach(  this, &Panel::on_back );
    this->pause_button.up_attach( this, &Panel::on_pause );


    //setting longpress_delay
    int longpress_delay =  THEKERNEL->config->value( panel_checksum, longpress_delay_checksum )->by_default(0)->as_number();
    this->up_button.set_longpress_delay(longpress_delay);
    this->down_button.set_longpress_delay(longpress_delay);
//    this->click_button.set_longpress_delay(longpress_delay);
//    this->back_button.set_longpress_delay(longpress_delay);
//    this->pause_button.set_longpress_delay(longpress_delay);


    THEKERNEL->slow_ticker->attach( 50,  this, &Panel::button_tick );
    if(lcd->encoderReturnsDelta()) {
        // panel handles encoder pins and returns a delta
        THEKERNEL->slow_ticker->attach( 10, this, &Panel::encoder_tick );
    }else{
        // read encoder pins
        THEKERNEL->slow_ticker->attach( 1000, this, &Panel::encoder_check );
    }

    // Register for events
    this->register_for_event(ON_IDLE);
    this->register_for_event(ON_MAIN_LOOP);
    this->register_for_event(ON_GCODE_RECEIVED);

    // Refresh timer
    THEKERNEL->slow_ticker->attach( 20, this, &Panel::refresh_tick );
}

// Enter a screen, we only care about it now
void Panel::enter_screen(PanelScreen *screen)
{
    this->current_screen = screen;
    this->reset_counter();
    this->current_screen->on_enter();
}

// Reset the counter
void Panel::reset_counter()
{
    *this->counter = 0;
    this->counter_changed = false;
}

// Indicate the idle loop we want to call the refresh hook in the current screen
// called 20 times a second
uint32_t Panel::refresh_tick(uint32_t dummy)
{
    this->refresh_flag = true;
    this->idle_time++;
    return 0;
}

// Encoder pins changed in interrupt or call from on_idle
uint32_t Panel::encoder_check(uint32_t dummy)
{
    // if encoder reads go through SPI like on universal panel adapter this needs to be
    // done in idle loop, this is indicated by lcd->encoderReturnsDelta()
    // however when reading encoder directly it needs to be done
    // frequently, universal panel adapter will return an actual delta count so won't miss any if polled slowly
    // NOTE FIXME (WHY is it in menu only?) this code will not work if change is not 1,0,-1 anything greater (as in above case) will not work properly
    static int encoder_counter = 0; // keeps track of absolute encoder position
    static int last_encoder_click= 0; // last readfing of divided encoder count
    int change = lcd->readEncoderDelta();
    encoder_counter += change;
    int clicks= encoder_counter/this->encoder_click_resolution;
    int delta= clicks - last_encoder_click; // the number of clicks this time
    last_encoder_click= clicks;

    if ( delta != 0 ) {
        this->counter_changed = true;
        (*this->counter) += delta;
        this->idle_time = 0;
    }
    return 0;
}

// Read and update each button
uint32_t Panel::button_tick(uint32_t dummy)
{
    this->do_buttons = true;
    return 0;
}

// Read and update encoder
uint32_t Panel::encoder_tick(uint32_t dummy)
{
    this->do_encoder = true;
    return 0;
}

void Panel::on_gcode_received(void *argument)
{
    Gcode *gcode = static_cast<Gcode *>(argument);
    if ( gcode->has_m) {
        if ( gcode->m == 117 ) { // set LCD message
            this->message = get_arguments(gcode->get_command());
            if (this->message.size() > 20) this->message = this->message.substr(0, 20);
            gcode->mark_as_taken();
        }
    }
}

// on main loop, we can send gcodes or do anything that waits in this loop
void Panel::on_main_loop(void *argument)
{
    if (this->current_screen != NULL) {
        this->current_screen->on_main_loop();
        this->lcd->on_main_loop();
    }
}


#define ohw_logo_antipixel_width 80
#define ohw_logo_antipixel_height 15
static const uint8_t ohw_logo_antipixel_bits[] = {
    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x01, 0x80, 0x0C, 0x00, 0x33, 0x18, 0xBB, 0xFF, 0xFF, 0xFF, 0xFD, 0x80, 0x5E,
    0x80, 0x2D, 0x6B, 0x9B, 0xFF, 0xFF, 0xFF, 0xFD, 0x80, 0xFF, 0xC0, 0x2D, 0x18, 0xAB, 0xFF, 0xFF,
    0xFF, 0xFD, 0x80, 0xFF, 0xC0, 0x2D, 0x7B, 0xB3, 0xFF, 0xFF, 0xFF, 0xFD, 0x80, 0x7F, 0x80, 0x33,
    0x78, 0xBB, 0xFF, 0xFF, 0xFF, 0xFD, 0x81, 0xF3, 0xE0, 0x3F, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFD,
    0x81, 0xF3, 0xE0, 0x3F, 0xFD, 0xB3, 0x18, 0xDD, 0x98, 0xC5, 0x81, 0xF3, 0xE0, 0x3F, 0xFD, 0xAD,
    0x6B, 0x5D, 0x6B, 0x5D, 0x80, 0x73, 0x80, 0x3F, 0xFC, 0x21, 0x1B, 0x55, 0x08, 0xC5, 0x80, 0xF3,
    0xC0, 0x3F, 0xFD, 0xAD, 0x5B, 0x49, 0x6A, 0xDD, 0x80, 0xE1, 0xC0, 0x3F, 0xFD, 0xAD, 0x68, 0xDD,
    0x6B, 0x45, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0xFF, 0xFF, 0xFF, 0xFF,
    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF
};

// On idle things, we don't want to do shit in interrupts
// don't queue gcodes in this
void Panel::on_idle(void *argument)
{
    if (this->start_up) {
        this->lcd->init();

        Version v;
        string build(v.get_build());
        string date(v.get_build_date());
        this->lcd->clear();
        this->lcd->setCursor(0, 0); this->lcd->printf("Welcome to Smoothie");
        this->lcd->setCursor(0, 1); this->lcd->printf("%s", build.substr(0, 20).c_str());
        this->lcd->setCursor(0, 2); this->lcd->printf("%s", date.substr(0, 20).c_str());
        this->lcd->setCursor(0, 3); this->lcd->printf("Please wait....");

        if (this->lcd->hasGraphics()) {
            this->lcd->bltGlyph(24, 40, ohw_logo_antipixel_width, ohw_logo_antipixel_height, ohw_logo_antipixel_bits);
        }

        this->lcd->on_refresh(true); // tell lcd to display now

        // Default top screen
        this->top_screen= new MainMenuScreen();
        this->custom_screen->set_parent(this->top_screen);
        this->start_up = false;
        return;
    }

    MainMenuScreen *mms= static_cast<MainMenuScreen*>(this->top_screen);
    // after being idle for a while switch to Watch screen
    if (this->current_screen != NULL && this->idle_time > this->current_screen->idle_timeout_secs()*20) {
        this->idle_time = 0;
        if (mms->watch_screen != this->current_screen) {
            this->enter_screen(mms->watch_screen);
            // TODO do we need to reset any state?
        }

        return;
    }

    if(current_screen == NULL && this->idle_time > 20*4) {
        this->enter_screen(mms->watch_screen);
        return;
    }

    if(this->do_encoder) {
        this->do_encoder= false;
        encoder_check(0);
    }

    if (this->do_buttons) {
        // we don't want to do SPI in interrupt mode
        this->do_buttons = false;

        // read the actual buttons
        int but = lcd->readButtons();
        if (but != 0) {
            this->idle_time = 0;
            if(current_screen == NULL) {
                // we were in startup screen so go to watch screen
                this->enter_screen(mms->watch_screen);
                return;
            }
        }

        // fire events if the buttons are active and debounce is satisfied
        this->up_button.check_signal(but & BUTTON_UP);
        this->down_button.check_signal(but & BUTTON_DOWN);
        this->back_button.check_signal(but & BUTTON_LEFT);
        this->click_button.check_signal(but & BUTTON_SELECT);
        this->pause_button.check_signal(but & BUTTON_PAUSE);
    }

    // If we are in menu mode and the position has changed
    if ( this->mode == MENU_MODE && this->counter_change() ) {
        this->menu_update();
    }

    // If we are in control mode
    if ( this->mode == CONTROL_MODE && this->counter_change() ) {
        this->control_value_update();
    }

    // If we must refresh
    if ( this->refresh_flag ) {
        this->refresh_flag = false;
        if (this->current_screen != NULL) {
            this->current_screen->on_refresh();
            this->lcd->on_refresh();
        }
    }
}

// Hooks for button clicks
uint32_t Panel::on_up(uint32_t dummy)
{
    // this is simulating encoder clicks, but needs to be inverted to
    // increment values on up,increment by
    int inc = (this->mode == CONTROL_MODE) ? 1 : -(this->menu_offset+1);
    *this->counter += inc;
    this->counter_changed = true;
    return 0;
}

uint32_t Panel::on_down(uint32_t dummy)
{
    int inc = (this->mode == CONTROL_MODE) ? -1 : (this->menu_offset+1);
    *this->counter += inc;
    this->counter_changed = true;
    return 0;
}

// on most menu screens will go back to previous higher menu
uint32_t Panel::on_back(uint32_t dummy)
{
    if (this->mode == MENU_MODE && this->current_screen != NULL && this->current_screen->parent != NULL) {
        this->enter_screen(this->current_screen->parent);
    }
    return 0;
}

uint32_t Panel::on_select(uint32_t dummy)
{
    // TODO make configurable, including turning off
    // buzz is ignored on panels that do not support buzz
    this->click_changed = true;
    this->idle_time = 0;
    lcd->buzz(60, 300); // 50ms 300Hz
    return 0;
}

uint32_t Panel::on_pause(uint32_t dummy)
{
    if (!paused) {
        THEKERNEL->pauser->take();
        paused = true;
    } else {
        THEKERNEL->pauser->release();
        paused = false;
    }
    return 0;
}

bool Panel::counter_change()
{
    if ( this->counter_changed ) {
        this->counter_changed = false;
        return true;
    } else {
        return false;
    }
}
bool Panel::click()
{
    if ( this->click_changed ) {
        this->click_changed = false;
        return true;
    } else {
        return false;
    }
}


// Enter menu mode
void Panel::enter_menu_mode(bool force)
{
    this->mode = MENU_MODE;
    this->counter = &this->menu_selected_line;
    this->menu_changed = force;
}

void Panel::setup_menu(uint16_t rows)
{
    this->setup_menu(rows, min(rows, this->max_screen_lines()));
}

void Panel::setup_menu(uint16_t rows, uint16_t lines)
{
    this->menu_selected_line = 0;
    this->menu_current_line = 0;
    this->menu_start_line = 0;
    this->menu_rows = rows;
    this->panel_lines = lines;
}

void Panel::menu_update()
{
    // Limits, up and down
    // NOTE menu_selected_line is changed in an interrupt and can change at any time
    int msl = this->menu_selected_line; // hopefully this is atomic

    #if 0
    // this allows it to wrap but with new method we dont; want to wrap
    msl = msl % ( this->menu_rows << this->menu_offset );
    while ( msl < 0 ) {
        msl += this->menu_rows << this->menu_offset;
    }
    #else
        // limit selected line to screen lines available
        if(msl >= this->menu_rows<<this->menu_offset){
            msl= (this->menu_rows-1)<<this->menu_offset;
        }else if(msl < 0) msl= 0;
    #endif

    this->menu_selected_line = msl; // update atomically we hope
    // figure out which actual line to select, if we have a menu offset it means we want to move one line per two clicks
    if(msl % (this->menu_offset+1) == 0) { // only if divisible by offset
        this->menu_current_line = msl >> this->menu_offset;
    }

    // What to display
    if ( this->menu_rows > this->panel_lines ) {
        #if 0
        // old way of scrolling not nice....
        if ( this->menu_current_line >= 2 ) {
            this->menu_start_line = this->menu_current_line - 1;
        }
        if ( this->menu_current_line > this->menu_rows - this->panel_lines ) {
            this->menu_start_line = this->menu_rows - this->panel_lines;
        }
        #else
        // new way we only scroll the lines when the cursor hits the bottom of the screen or the top of the screen
        // do we want to scroll up?
        int sl= this->menu_current_line - this->menu_start_line; // screen line we are on
        if(sl >= this->panel_lines) {
            this->menu_start_line += ((sl+1)-this->panel_lines); // scroll up to keep it on the screen

        }else if(sl < 0 ) { // do we want to scroll down?
            this->menu_start_line += sl; // scroll down
        }
        #endif

    }else{
        this->menu_start_line = 0;
    }

    this->menu_changed = true;
}

bool Panel::menu_change()
{
    if ( this->menu_changed ) {
        this->menu_changed = false;
        return true;
    } else {
        return false;
    }
}

bool Panel::control_value_change()
{
    if ( this->control_value_changed ) {
        this->control_value_changed = false;
        return true;
    } else {
        return false;
    }
}

bool Panel::enter_control_mode(float passed_normal_increment, float passed_pressed_increment)
{
    this->mode = CONTROL_MODE;
    this->normal_increment  = passed_normal_increment;
    this->counter = &this->control_normal_counter;
    this->control_normal_counter = 0;
    this->control_base_value = 0;
    return true;
}

void Panel::control_value_update()
{
    // TODO what do we do here?
    this->control_value_changed = true;
}

void Panel::set_control_value(float value)
{
    this->control_base_value = value;
}

float Panel::get_control_value()
{
    return this->control_base_value + (this->control_normal_counter * this->normal_increment);
}

bool Panel::is_playing() const
{
    void *returned_data;

    bool ok = PublicData::get_value( player_checksum, is_playing_checksum, &returned_data );
    if (ok) {
        bool b = *static_cast<bool *>(returned_data);
        return b;
    }
    return false;
}

void  Panel::set_playing_file(string f)
{
    // just copy the first 20 characters after the first / if there
    size_t n = f.find_last_of('/');
    if (n == string::npos) n = 0;
    strncpy(playing_file, f.substr(n + 1, 19).c_str(), sizeof(playing_file));
    playing_file[sizeof(playing_file) - 1] = 0;
}

static float getTargetTemperature(uint16_t heater_cs)
{
    void *returned_data;
    bool ok = PublicData::get_value( temperature_control_checksum, heater_cs, current_temperature_checksum, &returned_data );

    if (ok) {
        struct pad_temperature temp =  *static_cast<struct pad_temperature *>(returned_data);
        return temp.target_temperature;
    }

    return 0.0F;
}

void Panel::setup_temperature_screen()
{
    // setup temperature screen
    auto mvs= new ModifyValuesScreen();
    this->temperature_screen= mvs;

    // enumerate heaters and add a menu item for each one
    vector<uint16_t> modules;
    THEKERNEL->config->get_module_list( &modules, temperature_control_checksum );

    for(auto i : modules) {
        if (!THEKERNEL->config->value(temperature_control_checksum, i, enable_checksum )->as_bool()) continue;
        void *returned_data;
        bool ok = PublicData::get_value( temperature_control_checksum, i, current_temperature_checksum, &returned_data );
        if (!ok) continue;

        struct pad_temperature t =  *static_cast<struct pad_temperature *>(returned_data);

        // rename if two of the known types
        const char *name;
        if(t.designator == "T") name= "Hotend";
        else if(t.designator == "B") name= "Bed";
        else name= t.designator.c_str();

        mvs->addMenuItem(name, // menu name
            [i]() -> float { return getTargetTemperature(i); }, // getter
            [i](float t) { PublicData::set_value( temperature_control_checksum, i, &t ); }, // setter
            1.0F, // increment
            0.0F, // Min
            500.0F // Max
            );
    }
}
