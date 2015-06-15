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
#include "libs/USBDevice/USBMSD/SDCard.h"
#include "libs/SDFAT.h"

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
#include "StreamOutputPool.h"
#include "platform_memory.h"

#include "panels/ReprapDiscountGLCD.h"
#include "panels/ST7565.h"
#include "panels/UniversalAdapter.h"

#include "version.h"
#include "checksumm.h"
#include "ConfigValue.h"
#include "Config.h"
#include "TemperatureControlPool.h"

// for parse_pins in mbed
#include "pinmap.h"

#define panel_checksum             CHECKSUM("panel")
#define enable_checksum            CHECKSUM("enable")
#define lcd_checksum               CHECKSUM("lcd")
#define rrd_glcd_checksum          CHECKSUM("reprap_discount_glcd")
#define st7565_glcd_checksum       CHECKSUM("st7565_glcd")
#define viki2_checksum             CHECKSUM("viki2")
#define mini_viki2_checksum        CHECKSUM("mini_viki2")
#define universal_adapter_checksum CHECKSUM("universal_adapter")

#define menu_offset_checksum        CHECKSUM("menu_offset")
#define encoder_resolution_checksum CHECKSUM("encoder_resolution")
#define jog_x_feedrate_checksum     CHECKSUM("alpha_jog_feedrate")
#define jog_y_feedrate_checksum     CHECKSUM("beta_jog_feedrate")
#define jog_z_feedrate_checksum     CHECKSUM("gamma_jog_feedrate")
#define	longpress_delay_checksum	CHECKSUM("longpress_delay")

#define ext_sd_checksum            CHECKSUM("external_sd")
#define sdcd_pin_checksum          CHECKSUM("sdcd_pin")
#define spi_channel_checksum       CHECKSUM("spi_channel")
#define spi_cs_pin_checksum        CHECKSUM("spi_cs_pin")

#define hotend_temp_checksum CHECKSUM("hotend_temperature")
#define bed_temp_checksum    CHECKSUM("bed_temperature")
#define panel_display_message_checksum CHECKSUM("display_message")

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
    this->sd= nullptr;
    this->extmounter= nullptr;
    this->external_sd_enable= false;
    this->halted= false;
    strcpy(this->playing_file, "Playing file");
}

Panel::~Panel()
{
    delete this->lcd;
    delete this->extmounter;
    delete this->sd;
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
    int lcd_cksm = get_checksum(THEKERNEL->config->value(panel_checksum, lcd_checksum)->by_default("reprap_discount_glcd")->as_string());

    // Note checksums are not const expressions when in debug mode, so don't use switch
    if (lcd_cksm == rrd_glcd_checksum) {
        this->lcd = new ReprapDiscountGLCD();
    } else if (lcd_cksm == st7565_glcd_checksum) {
        this->lcd = new ST7565();
    } else if (lcd_cksm == viki2_checksum) {
        this->lcd = new ST7565(1); // variant 1
    } else if (lcd_cksm == mini_viki2_checksum) {
        this->lcd = new ST7565(2); // variant 2
    } else if (lcd_cksm == universal_adapter_checksum) {
        this->lcd = new UniversalAdapter();
    } else {
        // no known lcd type defined
        delete this;
        return;
    }

    // external sd
    if(THEKERNEL->config->value( panel_checksum, ext_sd_checksum )->by_default(false)->as_bool()) {
        this->external_sd_enable= true;
        // external sdcard detect
        this->sdcd_pin.from_string(THEKERNEL->config->value( panel_checksum, ext_sd_checksum, sdcd_pin_checksum )->by_default("nc")->as_string())->as_input();
        this->extsd_spi_channel = THEKERNEL->config->value(panel_checksum, ext_sd_checksum, spi_channel_checksum)->by_default(0)->as_number();
        string s= THEKERNEL->config->value( panel_checksum, ext_sd_checksum, spi_cs_pin_checksum)->by_default("2.8")->as_string();
        s= "P" + s; // Pinnames need to be Px_x
        this->extsd_spi_cs= parse_pins(s.c_str());
        this->register_for_event(ON_SECOND_TICK);
    }

    // these need to be called here as they need the config cache loaded as they enumerate modules
    this->custom_screen= new CustomScreen();

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
    this->register_for_event(ON_HALT);
    this->register_for_event(ON_SET_PUBLIC_DATA);

    // Refresh timer
    THEKERNEL->slow_ticker->attach( 20, this, &Panel::refresh_tick );
}

// Enter a screen, we only care about it now
void Panel::enter_screen(PanelScreen *screen)
{
    if(this->current_screen != nullptr)
        this->current_screen->on_exit();

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

void Panel::on_set_public_data(void *argument)
{
     PublicDataRequest *pdr = static_cast<PublicDataRequest *>(argument);

    if(!pdr->starts_with(panel_checksum)) return;

    if(!pdr->second_element_is(panel_display_message_checksum)) return;

    string *s = static_cast<string *>(pdr->get_data_ptr());
    if (s->size() > 20) {
        this->message = s->substr(0, 20);
    } else {
        this->message= *s;
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
    if (!THEKERNEL->pauser->paused()) {
        THEKERNEL->pauser->take();
    } else {
        THEKERNEL->pauser->release();
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

bool Panel::is_suspended() const
{
    void *returned_data;

    bool ok = PublicData::get_value( player_checksum, is_suspended_checksum, &returned_data );
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

bool Panel::mount_external_sd(bool on)
{
    // now setup the external sdcard if we have one and mount it
    if(on) {
        if(this->sd == nullptr) {
            PinName mosi, miso, sclk, cs= this->extsd_spi_cs;
            if(extsd_spi_channel == 0) {
                mosi = P0_18; miso = P0_17; sclk = P0_15;
            } else if(extsd_spi_channel == 1) {
                mosi = P0_9; miso = P0_8; sclk = P0_7;
            } else{
                this->external_sd_enable= false;
                THEKERNEL->streams->printf("Bad SPI channel for external SDCard\n");
                return false;
            }
            size_t n= sizeof(SDCard);
            void *v = AHB0.alloc(n);
            memset(v, 0, n); // clear the allocated memory
            this->sd= new(v) SDCard(mosi, miso, sclk, cs); // allocate object using zeroed memory
        }
        delete this->extmounter; // if it was not unmounted before
        size_t n= sizeof(SDFAT);
        void *v = AHB0.alloc(n);
        memset(v, 0, n); // clear the allocated memory
        this->extmounter= new(v) SDFAT("ext", this->sd); // use cleared allocated memory
        this->sd->disk_initialize(); // first one seems to fail, but works next time
        THEKERNEL->streams->printf("External SDcard mounted as /ext\n");
    }else{
        delete this->extmounter;
        this->extmounter= nullptr;
        THEKERNEL->streams->printf("External SDcard unmounted\n");
    }
    return true;
}

void Panel::on_second_tick(void *arg)
{
    if(!this->external_sd_enable || this->start_up) return;

    // sd insert detect, mount sdcard if inserted, unmount if removed
    if(this->sdcd_pin.connected()) {
        if(this->extmounter == nullptr && this->sdcd_pin.get()) {
            mount_external_sd(true);
            // go to the play screen and the /ext directory
            // TODO we don't want to do this if we just booted and card was already in
            THEKERNEL->current_path= "/ext";
            MainMenuScreen *mms= static_cast<MainMenuScreen*>(this->top_screen);
            THEPANEL->enter_screen(mms->file_screen);

        }else if(this->extmounter != nullptr && !this->sdcd_pin.get()){
            mount_external_sd(false);
        }
    }else{
        // TODO for panels with no sd card detect we need to poll to see if card is inserted - or not
    }
}

void Panel::on_halt(void *arg)
{
    halted= (arg == nullptr);
}
