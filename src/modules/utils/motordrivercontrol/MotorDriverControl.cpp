#include "MotorDriverControl.h"
#include "libs/Kernel.h"
#include "libs/nuts_bolts.h"
#include "libs/utils.h"
#include "ConfigValue.h"
#include "libs/StreamOutput.h"

#include "Gcode.h"
#include "Config.h"
#include "checksumm.h"
#include "DigipotBase.h"

// add new digipot chips here
#include "mcp4451.h"
#include "ad5206.h"

#include <string>
using namespace std;


#define motor_driver_control_checksum           CHECKSUM("motor_driver_control")
#define enable_checksum                         CHECKSUM("enable")

#define current_checksum                        CHECKSUM("current")
#define max_current_checksum                    CHECKSUM("max_current")
#define current_factor_checksum                 CHECKSUM("current_factor")

#define spi_channel_checksum       CHECKSUM("spi_channel")
#define spi_cs_pin_checksum        CHECKSUM("spi_cs_pin")
#define spi_frequency_checksum     CHECKSUM("spi_frequency")

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
        // If module is enabled
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
    if(!spi_cs_pin.connected()) return false; // if not defined then we can't use this instance

    std::string str= THEKERNEL->config->value( motor_driver_control_checksum, cs, designator)->by_default("")->as_string();
    if(str.empty()) return false; // designator required
    designator= str[0];

    // select which SPI channel to use
    int spi_channel = THEKERNEL->config->value(motor_driver_control_checksum, cs, spi_channel_checksum)->by_default(0)->as_number();
    int spi_frequency = THEKERNEL->config->value(motor_driver_control_checksum, cs, spi_frequency_checksum)->by_default(1000000)->as_number();

    PinName mosi, miso, sclk;
    if(spi_channel == 0) {
        mosi = P0_18; miso = P0_17; sclk = P0_15;
    } else if(spi_channel == 1) {
        mosi = P0_9; miso = P0_8; sclk = P0_7;
    } else {
        mosi = P0_18; miso = P0_17; sclk = P0_15;
    }

    this->spi = new mbed::SPI(mosi, miso, sclk);
    this->spi->frequency(spi_frequency);

    // chip select
    this->spi_cs_pin.set(0);

    max_current= THEKERNEL->config->value(motor_driver_control_checksum, cs, max_current )->by_default(2.0f)->as_number();
    current_factor= THEKERNEL->config->value(motor_driver_control_checksum, cs, current_factor )->by_default(1.0F)->as_number();

    this->register_for_event(ON_GCODE_RECEIVED);
    return true;
}

void MotorDriverControl::on_gcode_received(void *argument)
{
    Gcode *gcode = static_cast<Gcode*>(argument);

    if (gcode->has_m) {
        if(gcode->m == 907) {
            if (gcode->has_letter(designator)) {
                current= gcode->get_value(designator);
                set_current(current);
            }

        } else if(gcode->m == 500 || gcode->m == 503) {
            gcode->stream->printf(";Motor current:\nM907 ");
            gcode->stream->printf("%c%1.5f\n", designator, current);
        }
    }
}

void MotorDriverControl::set_current(float c)
{
    // TODO set current for chip type
}
