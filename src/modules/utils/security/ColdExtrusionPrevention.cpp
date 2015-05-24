#include "ColdExtrusionPrevention.h"

#include "libs/Kernel.h"
#include "libs/PublicData.h"
#include "libs/StreamOutputPool.h"
#include "Config.h"
#include "ConfigValue.h"
#include "Gcode.h"
#include "checksumm.h"
#include "PublicDataRequest.h"

#include "SecurityPool.h"
#include "TemperatureControlPool.h"
#include "TemperatureControlPublicAccess.h"

#define designator_checksum         CHECKSUM("designator")
#define min_temp_checksum           CHECKSUM("min_extrusion_temp")

#define extruder_checksum           CHECKSUM("extruder")
#define extruder_enable_checksum    CHECKSUM("extruder_enable")

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
    // We do not remove the module if it is not enabled since the user could
    // decide to enable the module later on.

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
        delete this;
        return;
    }

    minExtrusionTemperature = static_cast<uint8_t>(THEKERNEL->config->value(coldextrusionprevention_checksum, this->identifier, min_temp_checksum)->by_default(0)->as_number());
    if(minExtrusionTemperature == 0) {
        // The minimal extrusion temperature must be set
        delete this;
        return;
    }

    set_flag(FLAG_TEMP_OK, false);

    // For M302
    this->register_for_event(ON_GCODE_RECEIVED);
    // To react on moving gcodes
    this->register_for_event(ON_GCODE_EXECUTE);
    // To read the temperature
    this->register_for_event(ON_IDLE);

    // Read the temperature every second
    this->register_for_event(ON_SECOND_TICK);
}

void ColdExtrusionPrevention::on_gcode_received(void* argument) {
    Gcode* gcode = static_cast<Gcode*>(argument);

    if(gcode->has_m) {
        if(gcode->m == 302) {
            if(gcode->has_letter('P')) {
                // Enable the prevention if the value of the P argument is 0 (or
                // if get_uint returns an error, which has an value of 0).
                // This enables cold extrusion with M302 P1 (disables the
                // prevention) and disables cold extrusion with M302 P0 (enables
                // the prevention).
                // This is compatible to the definition on reprap.org
                set_flag(FLAG_ENABLE, (gcode->get_uint('P', nullptr) == 0));
            }

            THEKERNEL->streams->printf("Cold extrusions are %s.\r\n", read_flag(FLAG_ENABLE) ? "not allowed" : "allowed");
        } else if(gcode->m == 999) {
            set_flag(FLAG_WAS_KILLED, false);
        }
    }
}

void ColdExtrusionPrevention::on_gcode_execute(void* argument) {
    if(!read_flag(FLAG_ENABLE)) {
        return;
    }

    Gcode* gcode = static_cast<Gcode*>(argument);

    // Only do the checks if this is a gcode involving a movement of the
    // extruder.
    if(!gcode->has_g || !gcode->has_letter('E')) {
        return;
    }

    // Now set the flag that the printer should be killed. We don't do this here
    // because of ISR stuff
    if(!read_flag(FLAG_TEMP_OK) && !read_flag(FLAG_WAS_KILLED)) {
        set_flag(FLAG_KILL_NOW, true);
    }
}

void ColdExtrusionPrevention::on_idle(void* argument) {
    if(read_flag(FLAG_KILL_NOW)) {
        // We should kill the printer! Set/Reset all flags to the appropriate
        // state.
        set_flag(FLAG_WAS_KILLED, true);
        set_flag(FLAG_KILL_NOW, false);

        // Now print a message that we've killed the printer!
        THEKERNEL->streams->printf("Cold extrusion prevented, %d°C are required - reset or M999 to re-enable the printer\r\n", minExtrusionTemperature);

        THEKERNEL->call_event(ON_HALT, nullptr);
    }

    if(read_flag(FLAG_ENABLE) && read_flag(FLAG_READ_TEMP)) {
        // TODO: Find out if there are semaphores in Smoothieware.
        // Currently worst case is that this writes the variable directly after
        // the read_temp_tick ISR sets it so that the value is not read the next
        // time. So worst case is a temperature read every 2 ticks.
        set_flag(FLAG_READ_TEMP, false);

        uint8_t currentTemperature = get_highest_temperature();
        bool ok = (currentTemperature >= minExtrusionTemperature);

        if(read_flag(FLAG_TEMP_OK) != ok) {
            // Read current temperature and set the FLAG_TEMP_OK accordingly
            set_flag(FLAG_TEMP_OK, ok);
        }
    }
}

void ColdExtrusionPrevention::on_second_tick(void* argument) {
    // Only read the temperature every 2 seconds
    if(read_flag(FLAG_SECOND_TICK)) {
        // We only set the flag to read the temperature upon the next idle event
        // since this is an ISR.
        set_flag(FLAG_READ_TEMP, true);
        set_flag(FLAG_SECOND_TICK, false);
    } else {
        // Wait for the next tick
        set_flag(FLAG_SECOND_TICK, true);
    }
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

