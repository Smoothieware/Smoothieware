#include "MotorDriverControl.h"
#include "libs/Kernel.h"
#include "libs/nuts_bolts.h"
#include "libs/utils.h"
#include "ConfigValue.h"
#include "libs/StreamOutput.h"
#include "libs/StreamOutputPool.h"


#include "Gcode.h"
#include "Config.h"
#include "checksumm.h"
#include "DigipotBase.h"

#include "mbed.h" // for SPI

#include "drivers/TMC26X/TMC26X.h"

#include <string>

#define motor_driver_control_checksum  CHECKSUM("motor_driver_control")
#define enable_checksum                CHECKSUM("enable")
#define chip_checksum                  CHECKSUM("chip")

#define current_checksum               CHECKSUM("current")
#define max_current_checksum           CHECKSUM("max_current")
#define current_factor_checksum        CHECKSUM("current_factor")

#define microsteps_checksum            CHECKSUM("microsteps")
#define decay_mode_checksum            CHECKSUM("decay_mode")
#define torque_checksum                CHECKSUM("torque")
#define gain_checksum                  CHECKSUM("gain")


#define spi_channel_checksum           CHECKSUM("spi_channel")
#define spi_cs_pin_checksum            CHECKSUM("spi_cs_pin")
#define spi_frequency_checksum         CHECKSUM("spi_frequency")

MotorDriverControl::MotorDriverControl(uint16_t cs, uint8_t id) : cs(cs), id(id)
{
}

MotorDriverControl::~MotorDriverControl()
{
}

// this will load all motor driver controls defined in config, called from main
void MotorDriverControl::on_module_loaded()
{
    vector<uint16_t> modules;
    THEKERNEL->config->get_module_list( &modules, motor_driver_control_checksum );
    uint8_t cnt = 0;
    for( auto cs : modules ) {
        // If module is enabled create an instance and initialize it
        if( THEKERNEL->config->value(motor_driver_control_checksum, cs, enable_checksum )->as_bool() ) {
            MotorDriverControl *controller = new MotorDriverControl(cs, cnt++);
            if(!controller->config_module()) delete controller;
        }
    }

    // we don't need this instance anymore
    delete this;
}

bool MotorDriverControl::config_module()
{
    spi_cs_pin.from_string(THEKERNEL->config->value( motor_driver_control_checksum, cs, spi_cs_pin_checksum)->by_default("nc")->as_string())->as_output();
    if(!spi_cs_pin.connected()) {
        THEKERNEL->streams->printf("MotorDriverControl ERROR: chip select not defined\n");
        return false; // if not defined then we can't use this instance
    }
    spi_cs_pin.set(1);

    std::string str= THEKERNEL->config->value( motor_driver_control_checksum, cs, designator)->by_default("")->as_string();
    if(str.empty()) {
        THEKERNEL->streams->printf("MotorDriverControl ERROR: designator not defined\n");
        return false; // designator required
    }
    designator= str[0];

    str= THEKERNEL->config->value( motor_driver_control_checksum, cs, chip)->by_default("")->as_string();
    if(str.empty()) {
        THEKERNEL->streams->printf("MotorDriverControl ERROR: chip type not defined\n");
        return false; // chip type required
    }

    using std::placeholders::_1;
    using std::placeholders::_2;
    using std::placeholders::_3;

    if(str == "DRV8711") {
        chip= DRV8711;
        //drv8711= new DRV8711DRV(sendSPI);

    }else if(str == "TMC2660") {
        chip= TMC2660;
        tmc26x= new TMC26X(std::bind( &MotorDriverControl::sendSPI, this, _1, _2, _3));

    }else{
        THEKERNEL->streams->printf("MotorDriverControl ERROR: Unknown chip type: %s\n", str.c_str());
        return false;
    }

    // select which SPI channel to use
    int spi_channel = THEKERNEL->config->value(motor_driver_control_checksum, cs, spi_channel_checksum)->by_default(1)->as_number();
    int spi_frequency = THEKERNEL->config->value(motor_driver_control_checksum, cs, spi_frequency_checksum)->by_default(1000000)->as_number();

    // select SPI channel to use
    PinName mosi, miso, sclk;
    if(spi_channel == 0) {
        mosi = P0_18; miso = P0_17; sclk = P0_15;
    } else if(spi_channel == 1) {
        mosi = P0_9; miso = P0_8; sclk = P0_7;
    } else {
        THEKERNEL->streams->printf("MotorDriverControl ERROR: Unknown SPI Channel: %d\n", spi_channel);
        return false;
    }

    this->spi = new mbed::SPI(mosi, miso, sclk);
    this->spi->frequency(spi_frequency);


    max_current= THEKERNEL->config->value(motor_driver_control_checksum, cs, max_current_checksum )->by_default(2.0f)->as_number();
    current_factor= THEKERNEL->config->value(motor_driver_control_checksum, cs, current_factor_checksum )->by_default(1.0F)->as_number();

    current= THEKERNEL->config->value(motor_driver_control_checksum, cs, current_checksum )->by_default(1.0F)->as_number();
    microsteps= THEKERNEL->config->value(motor_driver_control_checksum, cs, microsteps_checksum )->by_default(4)->as_number(); // 2^n
    decay_mode= THEKERNEL->config->value(motor_driver_control_checksum, cs, decay_mode_checksum )->by_default(1)->as_number();

    if(chip == DRV8711) {
        torque= THEKERNEL->config->value(motor_driver_control_checksum, cs, torque_checksum )->by_default(-1)->as_number();
        gain= THEKERNEL->config->value(motor_driver_control_checksum, cs, gain_checksum )->by_default(-1)->as_number();
    }

    // setup the chip via SPI
    initialize_chip();

    this->register_for_event(ON_GCODE_RECEIVED);
    this->register_for_event(ON_GCODE_EXECUTE);
    return true;
}

// React to enable/disable gcodes
void MotorDriverControl::on_gcode_execute(void *argument)
{
    Gcode *gcode = static_cast<Gcode *>(argument);

    if( gcode->has_m) {
        if( gcode->m == 17 ) {
            enable(true);
        }
        if( (gcode->m == 84 || gcode->m == 18) && !gcode->has_letter('E') ) {
            enable(false);
        }
    }
}


void MotorDriverControl::on_gcode_received(void *argument)
{
    Gcode *gcode = static_cast<Gcode*>(argument);

    if (gcode->has_m) {
        if(gcode->m == 906) {
            if(gcode->subcode == 1) {
                // M906.1 dump status for all drivers
                dump_status(gcode->stream);

            }else if (gcode->has_letter(designator)) {
                // set motor currents in mA (Note not using M907 as digipots use that)
                current= gcode->get_value(designator);
                set_current(current);
            }

        } else if(gcode->m == 909) { // set microstepping
            if (gcode->has_letter(designator)) {
                microsteps= gcode->get_value(designator);
                set_microstep(microsteps);
            }

        } else if(gcode->m == 910) { // set decay mode
            if (gcode->has_letter(designator)) {
                decay_mode= gcode->get_value(designator);
                set_decay_mode(decay_mode);
            }

        } else if(chip == DRV8711 && gcode->m == 911) { // M911.1 set torque, M911.2 set gain (MAYBE could be subcodes for 906 as DRV8711 doesn't set current directly)
            if (gcode->has_letter(designator)) {
                if(gcode->subcode == 1) {
                    torque= gcode->get_value(designator);

                }else  if(gcode->subcode == 2) {
                    gain= gcode->get_value(designator);

                }else{
                    return;
                }
                set_torque(torque, gain);
            }

        } else if(gcode->m == 500 || gcode->m == 503) {
            gcode->stream->printf(";Motor id %d current mA, microsteps, decay mode:\n", id);
            gcode->stream->printf("M906 %c%lu\n", designator, current);
            gcode->stream->printf("M909 %c%d\n", designator, microsteps);
            gcode->stream->printf("M910 %c%d\n", designator, decay_mode);
            if(torque >= 0 && gain >= 0) {
                gcode->stream->printf("M911.1 %c%1.5f\n", designator, torque);
                gcode->stream->printf("M911.2 %c%1.5f\n", designator, gain);
            }
         }
    }
}

void MotorDriverControl::initialize_chip()
{
    // send initialization sequence to chips
    if(chip == DRV8711) {
        set_torque(torque, gain);

    }else if(chip == TMC2660){
        //setResistor(150);
        tmc26x->init();
        set_current(current);
    }

    set_microstep(microsteps);
    set_decay_mode(decay_mode);
}

// set curent in milliamps
void MotorDriverControl::set_current(uint32_t c)
{
    switch(chip) {
        case DRV8711: break; // doesn't use current

        case TMC2660:
            tmc26x->setCurrent(c);
            break;
    }
}

// set microsteps where n is the number of microsteps eg 64 for 1/64
void MotorDriverControl::set_microstep( uint32_t n )
{
    switch(chip) {
        case DRV8711: break;

        case TMC2660:
            tmc26x->setMicrosteps(n);
            break;
    }
}

void MotorDriverControl::set_decay_mode( uint8_t dm )
{
    switch(chip) {
        case DRV8711: break;
        case TMC2660: break;
    }
}

void MotorDriverControl::set_torque(float torque, float gain)
{

}

void MotorDriverControl::enable(bool on)
{
    switch(chip) {
        case DRV8711: break;
        case TMC2660:
            tmc26x->setEnabled(on);
            break;
    }
}

void MotorDriverControl::dump_status(StreamOutput *stream)
{
    switch(chip) {
        case DRV8711: break;
        case TMC2660:
            tmc26x->dumpStatus(stream);
            break;
    }
}


int MotorDriverControl::sendSPI(uint8_t *b, int cnt, uint8_t *r)
{
    spi_cs_pin.set(0);
    for (int i = 0; i < cnt; ++i) {
        r[i]= spi->write(b[i]);
    }
    spi_cs_pin.set(1);
    return cnt;
}

