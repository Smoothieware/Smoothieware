/*
      This file is part of Smoothie (http://smoothieware.org/). The motion control part is heavily based on Grbl (https://github.com/simen/grbl).
      Smoothie is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
      Smoothie is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
      You should have received a copy of the GNU General Public License along with Smoothie. If not, see <http://www.gnu.org/licenses/>.
*/

// TODO : THIS FILE IS LAME, MUST BE MADE MUCH BETTER

#include "libs/Module.h"
#include "libs/Kernel.h"
#include <math.h>
#include "TemperatureControl.h"
#include "TemperatureControlPool.h"
#include "libs/Pin.h"
#include "modules/robot/Conveyor.h"
#include "PublicDataRequest.h"

#include "PublicData.h"
#include "ToolManagerPublicAccess.h"
#include "StreamOutputPool.h"
#include "Config.h"
#include "checksumm.h"
#include "Gcode.h"
#include "SlowTicker.h"
#include "Pauser.h"
#include "ConfigValue.h"
#include "PID_Autotuner.h"

// Temp sensor implementations:
#include "Thermistor.h"
#include "max31855.h"

#include "MRI_Hooks.h"

#define UNDEFINED -1

#define sensor_checksum                    CHECKSUM("sensor")

#define readings_per_second_checksum       CHECKSUM("readings_per_second")
#define max_pwm_checksum                   CHECKSUM("max_pwm")
#define pwm_frequency_checksum             CHECKSUM("pwm_frequency")
#define bang_bang_checksum                 CHECKSUM("bang_bang")
#define hysteresis_checksum                CHECKSUM("hysteresis")
#define heater_pin_checksum                CHECKSUM("heater_pin")

#define get_m_code_checksum                CHECKSUM("get_m_code")
#define set_m_code_checksum                CHECKSUM("set_m_code")
#define set_and_wait_m_code_checksum       CHECKSUM("set_and_wait_m_code")

#define designator_checksum                CHECKSUM("designator")

#define p_factor_checksum                  CHECKSUM("p_factor")
#define i_factor_checksum                  CHECKSUM("i_factor")
#define d_factor_checksum                  CHECKSUM("d_factor")

#define i_max_checksum                     CHECKSUM("i_max")

#define preset1_checksum                   CHECKSUM("preset1")
#define preset2_checksum                   CHECKSUM("preset2")

TemperatureControl::TemperatureControl(uint16_t name, int index)
{
    name_checksum= name;
    pool_index= index;
    waiting= false;
    min_temp_violated= false;
    sensor= nullptr;
    readonly= false;
}

TemperatureControl::~TemperatureControl()
{
    delete sensor;
}

void TemperatureControl::on_module_loaded()
{

    // We start not desiring any temp
    this->target_temperature = UNDEFINED;

    // Settings
    this->load_config();

    // Register for events
    this->register_for_event(ON_GCODE_RECEIVED);
    this->register_for_event(ON_GET_PUBLIC_DATA);

    if(!this->readonly) {
        this->register_for_event(ON_GCODE_EXECUTE);
        this->register_for_event(ON_SECOND_TICK);
        this->register_for_event(ON_MAIN_LOOP);
        this->register_for_event(ON_SET_PUBLIC_DATA);
        this->register_for_event(ON_HALT);
    }
}

void TemperatureControl::on_halt(void *arg)
{
    // turn off heater
    this->o = 0;
    this->heater_pin.set(0);
    this->target_temperature = UNDEFINED;
}

void TemperatureControl::on_main_loop(void *argument)
{
    if (this->min_temp_violated) {
        THEKERNEL->streams->printf("Error: MINTEMP triggered. Check your temperature sensors!\n");
        this->min_temp_violated = false;
    }
}

// Get configuration from the config file
void TemperatureControl::load_config()
{

    // General config
    this->set_m_code          = THEKERNEL->config->value(temperature_control_checksum, this->name_checksum, set_m_code_checksum)->by_default(104)->as_number();
    this->set_and_wait_m_code = THEKERNEL->config->value(temperature_control_checksum, this->name_checksum, set_and_wait_m_code_checksum)->by_default(109)->as_number();
    this->get_m_code          = THEKERNEL->config->value(temperature_control_checksum, this->name_checksum, get_m_code_checksum)->by_default(105)->as_number();
    this->readings_per_second = THEKERNEL->config->value(temperature_control_checksum, this->name_checksum, readings_per_second_checksum)->by_default(20)->as_number();

    this->designator          = THEKERNEL->config->value(temperature_control_checksum, this->name_checksum, designator_checksum)->by_default(string("T"))->as_string();

    // Heater pin
    this->heater_pin.from_string( THEKERNEL->config->value(temperature_control_checksum, this->name_checksum, heater_pin_checksum)->by_default("nc")->as_string());
    if(this->heater_pin.connected()){
        this->readonly= false;
        this->heater_pin.as_output();

    } else {
        this->readonly= true;
    }

    // For backward compatibility, default to a thermistor sensor.
    std::string sensor_type = THEKERNEL->config->value(temperature_control_checksum, this->name_checksum, sensor_checksum)->by_default("thermistor")->as_string();

    // Instantiate correct sensor (TBD: TempSensor factory?)
    delete sensor;
    sensor = nullptr; // In case we fail to create a new sensor.
    if(sensor_type.compare("thermistor") == 0) {
        sensor = new Thermistor();
    } else if(sensor_type.compare("max31855") == 0) {
        sensor = new Max31855();
    } else {
        sensor = new TempSensor(); // A dummy implementation
    }
    sensor->UpdateConfig(temperature_control_checksum, this->name_checksum);

    this->preset1 = THEKERNEL->config->value(temperature_control_checksum, this->name_checksum, preset1_checksum)->by_default(0)->as_number();
    this->preset2 = THEKERNEL->config->value(temperature_control_checksum, this->name_checksum, preset2_checksum)->by_default(0)->as_number();


    // sigma-delta output modulation
    this->o = 0;

    if(!this->readonly) {
        // used to enable bang bang control of heater
        this->use_bangbang = THEKERNEL->config->value(temperature_control_checksum, this->name_checksum, bang_bang_checksum)->by_default(false)->as_bool();
        this->hysteresis = THEKERNEL->config->value(temperature_control_checksum, this->name_checksum, hysteresis_checksum)->by_default(2)->as_number();
        this->heater_pin.max_pwm( THEKERNEL->config->value(temperature_control_checksum, this->name_checksum, max_pwm_checksum)->by_default(255)->as_number() );
        this->heater_pin.set(0);
        set_low_on_debug(heater_pin.port_number, heater_pin.pin);
        // activate SD-DAC timer
        THEKERNEL->slow_ticker->attach( THEKERNEL->config->value(temperature_control_checksum, this->name_checksum, pwm_frequency_checksum)->by_default(2000)->as_number(), &heater_pin, &Pwm::on_tick);
    }


    // reading tick
    THEKERNEL->slow_ticker->attach( this->readings_per_second, this, &TemperatureControl::thermistor_read_tick );
    this->PIDdt = 1.0 / this->readings_per_second;

    // PID
    setPIDp( THEKERNEL->config->value(temperature_control_checksum, this->name_checksum, p_factor_checksum)->by_default(10 )->as_number() );
    setPIDi( THEKERNEL->config->value(temperature_control_checksum, this->name_checksum, i_factor_checksum)->by_default(0.3f)->as_number() );
    setPIDd( THEKERNEL->config->value(temperature_control_checksum, this->name_checksum, d_factor_checksum)->by_default(200)->as_number() );

    if(!this->readonly) {
        // set to the same as max_pwm by default
        this->i_max = THEKERNEL->config->value(temperature_control_checksum, this->name_checksum, i_max_checksum   )->by_default(this->heater_pin.max_pwm())->as_number();
    }

    this->iTerm = 0.0;
    this->lastInput = -1.0;
    this->last_reading = 0.0;
}

void TemperatureControl::on_gcode_received(void *argument)
{
    Gcode *gcode = static_cast<Gcode *>(argument);
    if (gcode->has_m) {

        if( gcode->m == this->get_m_code ) {
            char buf[32]; // should be big enough for any status
            int n = snprintf(buf, sizeof(buf), "%s:%3.1f /%3.1f @%d ", this->designator.c_str(), this->get_temperature(), ((target_temperature == UNDEFINED) ? 0.0 : target_temperature), this->o);
            gcode->txt_after_ok.append(buf, n);
            gcode->mark_as_taken();
            return;
        }

        // readonly sensors don't handle the rest
        if(this->readonly) return;

        if (gcode->m == 301) {
            gcode->mark_as_taken();
            if (gcode->has_letter('S') && (gcode->get_value('S') == this->pool_index)) {
                if (gcode->has_letter('P'))
                    setPIDp( gcode->get_value('P') );
                if (gcode->has_letter('I'))
                    setPIDi( gcode->get_value('I') );
                if (gcode->has_letter('D'))
                    setPIDd( gcode->get_value('D') );
                if (gcode->has_letter('X'))
                    this->i_max    = gcode->get_value('X');
            }
            //gcode->stream->printf("%s(S%d): Pf:%g If:%g Df:%g X(I_max):%g Pv:%g Iv:%g Dv:%g O:%d\n", this->designator.c_str(), this->pool_index, this->p_factor, this->i_factor/this->PIDdt, this->d_factor*this->PIDdt, this->i_max, this->p, this->i, this->d, o);
            gcode->stream->printf("%s(S%d): Pf:%g If:%g Df:%g X(I_max):%g O:%d\n", this->designator.c_str(), this->pool_index, this->p_factor, this->i_factor / this->PIDdt, this->d_factor * this->PIDdt, this->i_max, o);

        } else if (gcode->m == 500 || gcode->m == 503) { // M500 saves some volatile settings to config override file, M503 just prints the settings
            gcode->stream->printf(";PID settings:\nM301 S%d P%1.4f I%1.4f D%1.4f\n", this->pool_index, this->p_factor, this->i_factor / this->PIDdt, this->d_factor * this->PIDdt);
            gcode->mark_as_taken();

        } else if( ( gcode->m == this->set_m_code || gcode->m == this->set_and_wait_m_code ) && gcode->has_letter('S')) {
            // this only gets handled if it is not controlle dby the tool manager or is active in the toolmanager
            this->active = true;

            // this is safe as old configs as well as single extruder configs the toolmanager will not be running so will return false
            // this will also ignore anything that the tool manager is not controlling and return false, otherwise it returns the active tool
            void *returned_data;
            bool ok = PublicData::get_value( tool_manager_checksum, is_active_tool_checksum, this->name_checksum, &returned_data );
            if (ok) {
                uint16_t active_tool_name =  *static_cast<uint16_t *>(returned_data);
                this->active = (active_tool_name == this->name_checksum);
            }

            if(this->active) {
                // Attach gcodes to the last block for on_gcode_execute
                THEKERNEL->conveyor->append_gcode(gcode);

                // push an empty block if we have to wait, so the Planner can get things right, and we can prevent subsequent non-move gcodes from executing
                if (gcode->m == this->set_and_wait_m_code) {
                    // ensure that no subsequent gcodes get executed with our M109 or similar
                    THEKERNEL->conveyor->queue_head_block();
                }
            }
        }
    }
}

void TemperatureControl::on_gcode_execute(void *argument)
{
    Gcode *gcode = static_cast<Gcode *>(argument);
    if( gcode->has_m) {
        if (((gcode->m == this->set_m_code) || (gcode->m == this->set_and_wait_m_code))
            && gcode->has_letter('S') && this->active) {
            float v = gcode->get_value('S');

            if (v == 0.0) {
                this->target_temperature = UNDEFINED;
                this->heater_pin.set((this->o = 0));
            } else {
                this->set_desired_temperature(v);

                if( gcode->m == this->set_and_wait_m_code && !this->waiting) {
                    THEKERNEL->pauser->take();
                    this->waiting = true;
                }
            }
        }
    }
}

void TemperatureControl::on_get_public_data(void *argument)
{
    PublicDataRequest *pdr = static_cast<PublicDataRequest *>(argument);

    if(!pdr->starts_with(temperature_control_checksum)) return;

    if(pdr->second_element_is(pool_index_checksum)) {
        // asking for our instance pointer if we have this pool_index
        if(pdr->third_element_is(this->pool_index)) {
            static void *return_data;
            return_data = this;
            pdr->set_data_ptr(&return_data);
            pdr->set_taken();
        }
        return;

    }else if(!pdr->second_element_is(this->name_checksum)) return;

    // ok this is targeted at us, so send back the requested data
    if(pdr->third_element_is(current_temperature_checksum)) {
        this->public_data_return.current_temperature = this->get_temperature();
        this->public_data_return.target_temperature = (target_temperature == UNDEFINED) ? 0 : this->target_temperature;
        this->public_data_return.pwm = this->o;
        this->public_data_return.designator= this->designator;
        pdr->set_data_ptr(&this->public_data_return);
        pdr->set_taken();
    }

}

void TemperatureControl::on_set_public_data(void *argument)
{
    PublicDataRequest *pdr = static_cast<PublicDataRequest *>(argument);

    if(!pdr->starts_with(temperature_control_checksum)) return;

    if(!pdr->second_element_is(this->name_checksum)) return;

    // ok this is targeted at us, so set the temp
    float t = *static_cast<float *>(pdr->get_data_ptr());
    this->set_desired_temperature(t);
    pdr->set_taken();
}

void TemperatureControl::set_desired_temperature(float desired_temperature)
{
    if (desired_temperature == 1.0)
        desired_temperature = preset1;
    else if (desired_temperature == 2.0)
        desired_temperature = preset2;

    target_temperature = desired_temperature;
    if (desired_temperature == 0.0)
        heater_pin.set((this->o = 0));
}

float TemperatureControl::get_temperature()
{
    return last_reading;
}

uint32_t TemperatureControl::thermistor_read_tick(uint32_t dummy)
{
    float temperature = sensor->get_temperature();
    if(this->readonly) {
        last_reading = temperature;
        return 0;
    }

    if (target_temperature > 0) {
        if (isinf(temperature)) {
            this->min_temp_violated = true;
            target_temperature = UNDEFINED;
            heater_pin.set((this->o = 0));
        } else {
            pid_process(temperature);
            if ((temperature > target_temperature) && waiting) {
                THEKERNEL->pauser->release();
                waiting = false;
            }
        }
    } else {
        heater_pin.set((this->o = 0));
    }
    last_reading = temperature;
    return 0;
}

/**
 * Based on https://github.com/br3ttb/Arduino-PID-Library
 */
void TemperatureControl::pid_process(float temperature)
{
    if(use_bangbang) {
        // bang bang is very simple, if temp is < target - hysteresis turn on full else if  temp is > target + hysteresis turn heater off
        // good for relays
        if(temperature > (target_temperature + hysteresis) && this->o > 0) {
            heater_pin.set(false);
            this->o = 0; // for display purposes only

        } else if(temperature < (target_temperature - hysteresis) && this->o <= 0) {
            if(heater_pin.max_pwm() >= 255) {
                // turn on full
                this->heater_pin.set(true);
                this->o = 255; // for display purposes only
            } else {
                // only to whatever max pwm is configured
                this->heater_pin.pwm(heater_pin.max_pwm());
                this->o = heater_pin.max_pwm(); // for display purposes only
            }
        }
        return;
    }

    // regular PID control
    float error = target_temperature - temperature;
    this->iTerm += (error * this->i_factor);
    if (this->iTerm > this->i_max) this->iTerm = this->i_max;
    else if (this->iTerm < 0.0) this->iTerm = 0.0;

    if(this->lastInput < 0.0) this->lastInput = temperature; // set first time
    float d = (temperature - this->lastInput);

    // calculate the PID output
    // TODO does this need to be scaled by max_pwm/256? I think not as p_factor already does that
    this->o = (this->p_factor * error) + this->iTerm - (this->d_factor * d);

    if (this->o >= heater_pin.max_pwm())
        this->o = heater_pin.max_pwm();
    else if (this->o < 0)
        this->o = 0;

    this->heater_pin.pwm(this->o);
    this->lastInput = temperature;
}

void TemperatureControl::on_second_tick(void *argument)
{
    if (waiting)
        THEKERNEL->streams->printf("%s:%3.1f /%3.1f @%d\n", designator.c_str(), get_temperature(), ((target_temperature == UNDEFINED) ? 0.0 : target_temperature), o);
}

void TemperatureControl::setPIDp(float p)
{
    this->p_factor = p;
}

void TemperatureControl::setPIDi(float i)
{
    this->i_factor = i * this->PIDdt;
}

void TemperatureControl::setPIDd(float d)
{
    this->d_factor = d / this->PIDdt;
}
