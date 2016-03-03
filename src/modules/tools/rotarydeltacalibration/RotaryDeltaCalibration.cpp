#include "RotaryDeltaCalibration.h"
#include "EndstopsPublicAccess.h"
#include "Kernel.h"
#include "Robot.h"
#include "Config.h"
#include "checksumm.h"
#include "ConfigValue.h"
#include "StreamOutput.h"
#include "PublicDataRequest.h"
#include "PublicData.h"
#include "Gcode.h"
#include "StepperMotor.h"

#define rotarydelta_checksum CHECKSUM("rotary_delta_calibration")
#define enable_checksum CHECKSUM("enable")

void RotaryDeltaCalibration::on_module_loaded()
{
    // if the module is disabled -> do nothing
    if(!THEKERNEL->config->value( rotarydelta_checksum, enable_checksum )->by_default(false)->as_bool()) {
        // as this module is not needed free up the resource
        delete this;
        return;
    }

    // register event-handlers
    register_for_event(ON_GCODE_RECEIVED);
}

float *RotaryDeltaCalibration::get_homing_offset()
{
    float *theta_offset; // points to theta offset in Endstop module
    bool ok = PublicData::get_value( endstops_checksum, home_offset_checksum, &theta_offset );
    return ok ? theta_offset : nullptr;
}

void RotaryDeltaCalibration::on_gcode_received(void *argument)
{
    Gcode *gcode = static_cast<Gcode *>(argument);

    if( gcode->has_m) {
        switch( gcode->m ) {
            case 206: {
                float *theta_offset= get_homing_offset(); // points to theta offset in Endstop module
                if (theta_offset == nullptr) {
                    gcode->stream->printf("error:no endstop module found\n");
                    return;
                }

                // set theta offset, set directly in the Endstop module (bad practice really)
                if (gcode->has_letter('A')) theta_offset[0] = gcode->get_value('A');
                if (gcode->has_letter('B')) theta_offset[1] = gcode->get_value('B');
                if (gcode->has_letter('C')) theta_offset[2] = gcode->get_value('C');

                gcode->stream->printf("Theta offset set: A %8.5f B %8.5f C %8.5f\n", theta_offset[0], theta_offset[1], theta_offset[2]);

                break;
            }

            case 306: {
                // for a rotary delta M306 calibrates the homing angle
                // by doing M306 A-56.17 it will calculate the M206 A value (the theta offset for actuator A) based on the difference
                // between what it thinks is the current angle and what the current angle actually is specified by A (ditto for B and C)

                ActuatorCoordinates current_angle;
                // get the current angle for each actuator, NOTE we only deal with  ABC so if there are more than 3 actuators this will probably go wonky
                for (size_t i = 0; i < THEKERNEL->robot->actuators.size(); i++) {
                    current_angle[i]= THEKERNEL->robot->actuators[i]->get_current_position();
                }

                if (gcode->has_letter('L') && gcode->get_value('L') != 0) {
                    // specifying L1 it will take the last probe position (set by G30 or G38.x ) and set the home offset based on that
                    // this allows the use of G30 to find the angle tool
                    uint8_t ok;
                    std::tie(current_angle[0], current_angle[1], current_angle[2], ok) = THEKERNEL->robot->get_last_probe_position();
                    if(ok == 0) {
                        gcode->stream->printf("error:Nothing set as probe failed or not run\n");
                        return;
                    }
                }

                float *theta_offset= get_homing_offset(); // points to theta offset in Endstop module
                if (theta_offset == nullptr) {
                    gcode->stream->printf("error:no endstop module found\n");
                    return;
                }

                int cnt= 0;

                //figure out what home_offset needs to be to correct the homing_position
                if (gcode->has_letter('A')) {
                    float a = gcode->get_value('A'); // what actual angle is
                    theta_offset[0] -= (current_angle[0] - a);
                    current_angle[0]= a;
                    cnt++;
                }
                if (gcode->has_letter('B')) {
                    float b = gcode->get_value('B');
                    theta_offset[1] -= (current_angle[1] - b);
                    current_angle[1]= b;
                    cnt++;
                }
                if (gcode->has_letter('C')) {
                    float c = gcode->get_value('C');
                    theta_offset[2] -= (current_angle[2] - c);
                    current_angle[2]= c;
                    cnt++;
                }

                // reset the actuator positions (and machine position accordingly)
                // But only if all three actuators have been specified at the same time
                if(cnt == 3 || (gcode->has_letter('R') && gcode->get_value('R') != 0)) {
                    THEKERNEL->robot->reset_actuator_position(current_angle);
                    gcode->stream->printf("NOTE: actuator position reset\n");
                }else{
                    gcode->stream->printf("NOTE: actuator position NOT reset\n");
                }

                gcode->stream->printf("Theta Offset: A %8.5f B %8.5f C %8.5f\n", theta_offset[0], theta_offset[1], theta_offset[2]);
            }
        }
    }
}
