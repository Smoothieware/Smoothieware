#include "ColdExtrusionPrevention.h"

#include "libs/Kernel.h"
#include "libs/PublicData.h"
#include "libs/StreamOutputPool.h"
#include "Config.h"
#include "ConfigValue.h"
#include "Gcode.h"
#include "checksumm.h"

#include "SecurityPool.h"
#include "TemperatureControlPool.h"
#include "TemperatureControlPublicAccess.h"

#define designator_checksum CHECKSUM("designator")
#define min_temp_checksum   CHECKSUM("min_extrusion_temp")

// TODO:
// * Don't hold the instances in the SecurityPool. Delete it.

ColdExtrusionPrevention::ColdExtrusionPrevention(uint16_t identifier) {
    this->flags = 0x00;
    this->identifier = identifier;
}

void ColdExtrusionPrevention::on_module_loaded() {
    this->on_config_reload(this);
}

void ColdExtrusionPrevention::on_config_reload(void* argument) {
    bool enabled = THEKERNEL->config->value(coldextrusionprevention_checksum, this->identifier, enable_checksum)->by_default(false)->as_bool();

    set_flag(FLAG_ENABLE, enabled);
    if(!enabled) {
        //return;
    }

    string designator = THEKERNEL->config->value(coldextrusionprevention_checksum, this->identifier, designator_checksum)->by_default("")->as_string();

    auto& availableControllers = THEKERNEL->temperature_control_pool->get_controllers();

    temperatureController = 0;

    // Iterate over the list of available temperature controllers and remove the
    // ones which does not match the requested designator
    void* returned_temp;
    for(auto ctrl: availableControllers) {
        bool temp_ok = PublicData::get_value(temperature_control_checksum, ctrl, current_temperature_checksum, &returned_temp);
        if(temp_ok) {
            struct pad_temperature temp =  *static_cast<struct pad_temperature *>(returned_temp);
            if(designator.compare(temp.designator) == 0) {
                // OK, we found the desired temperature controller
                temperatureController = ctrl;
            }
        }
    }

    if(temperatureController == 0) {
        // We need at least one temperature controller
        //return;
    }

    minExtrusionTemperature = static_cast<uint8_t>(THEKERNEL->config->value(coldextrusionprevention_checksum, this->identifier, min_temp_checksum)->by_default(0)->as_number());
    if(minExtrusionTemperature == 0) {
        set_flag(FLAG_ENABLE, false);
        //return;
    }

    set_flag(FLAG_TEMP_OK, false);

    this->register_for_event(ON_GCODE_RECEIVED);
    this->register_for_event(ON_GCODE_EXECUTE);
    this->register_for_event(ON_IDLE);

    // Read the temperature every second
    this->register_for_event(ON_SECOND_TICK);
}

void ColdExtrusionPrevention::on_gcode_received(void* argument) {
    Gcode* gcode = static_cast<Gcode*>(argument);

    THEKERNEL->streams->printf("OnGcodeReceived\r\n");

    if(gcode->has_m) {
        THEKERNEL->streams->printf("HasM\r\n");
        if(gcode->m == 302) {
            THEKERNEL->streams->printf("Is 302\r\n");
            if(gcode->has_letter('P')) {
                THEKERNEL->streams->printf("Has P\r\n");
                // Enable the prevention if the value of the P argument is 0 (or
                // if get_uint returns an error, which has an value of 0).
                // This enables cold extrusion with M302 P1 (disables the
                // prevention) and disables cold extrusion with M302 P0 (enables
                // the prevention).
                // This is compatible to the definition on reprap.org
                set_flag(FLAG_ENABLE, (gcode->get_uint('P', nullptr) == 0));
            } else {
                THEKERNEL->streams->printf("Cold extrusions are %s.\r\n", read_flag(FLAG_ENABLE) ? "not allowed" : "allowed");
            }

        }
    }
}

void ColdExtrusionPrevention::on_gcode_execute(void* argument) {
    if(!read_flag(FLAG_ENABLE)) {
        return;
    }

    Gcode* gcode = static_cast<Gcode*>(argument);

    THEKERNEL->streams->printf("Got Gcode: %s\r\n", gcode->get_command());

    if(gcode->has_g && gcode->g != 92) {
        if(!read_flag(FLAG_TEMP_OK)) {
            // Only prevent cold extrusions on actual movement gcodes
            THEKERNEL->call_event(ON_HALT, nullptr);
            THEKERNEL->streams->printf("Cold extrusion prevented - reset or M999 to re-enable the printer\r\n");
        }
    }
}

void ColdExtrusionPrevention::on_idle(void* argument) {
        THEKERNEL->streams->printf("FLAG_ENABLE:    %d\r\n", read_flag(FLAG_ENABLE));
    if(read_flag(FLAG_ENABLE | FLAG_READ_TEMP)) {
        THEKERNEL->streams->printf("FLAG_ENABLE:    %d\r\n", read_flag(FLAG_ENABLE));
        THEKERNEL->streams->printf("FLAG_READ_TEMP: %d\r\n", read_flag(FLAG_READ_TEMP));

        // TODO: Find out if there are semaphores in Smoothieware.
        // Currently worst case is that this writes the variable directly after
        // the read_temp_tick ISR sets it so that the value is not read the next
        // time. So worst case is a temperature read every 2 ticks.
        set_flag(FLAG_READ_TEMP, false);

        uint8_t temp = get_highest_temperature();

        // Read current temperature and set the FLAG_TEMP_OK accordingly
        set_flag(FLAG_TEMP_OK, temp >= minExtrusionTemperature);
    }
}

void ColdExtrusionPrevention::on_second_tick(void* argument) {
    // We only set the flag to read the temperature upon the next idle event
    // since this is an ISR.
    set_flag(FLAG_READ_TEMP, true);

    THEKERNEL->streams->printf("OnSecondTick\r\n");
}

uint8_t ColdExtrusionPrevention::get_highest_temperature()
{
    void *returned_temp;

    bool temp_ok = PublicData::get_value(temperature_control_checksum, temperatureController, current_temperature_checksum, &returned_temp);
    if (temp_ok) {
        struct pad_temperature temp =  *static_cast<struct pad_temperature *>(returned_temp);

        if(temp.current_temperature > 0xFF) {
            return 0xFF;
        } else if(temp.current_temperature < 0x00) {
            return 0;
        }

        return static_cast<uint8_t>(temp.current_temperature);
    }

    // Something went wrong
    return 0;
}

void ColdExtrusionPrevention::set_flag(uint8_t flag, bool enabled) {
    if(enabled) {
        this->flags |= flag;
    } else {
        this->flags &= ~flag;
    }
}

bool ColdExtrusionPrevention::read_flag(uint8_t flag) {
    return (this->flags & flag);
}

