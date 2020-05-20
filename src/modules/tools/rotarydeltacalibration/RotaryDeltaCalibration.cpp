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

bool RotaryDeltaCalibration::get_homing_offset(float *theta_offset)
{
    bool ok = PublicData::get_value( endstops_checksum, home_offset_checksum, theta_offset );
    return ok;
}

void RotaryDeltaCalibration::on_gcode_received(void *argument)
{
    Gcode *gcode = static_cast<Gcode *>(argument);

    if( gcode->has_m) {
        switch( gcode->m ) {
            case 206: {
                float theta_offset[3];
                if(!get_homing_offset(theta_offset)) {
                    gcode->stream->printf("error:no endstop module found\n");
                    return;
                }

                // set theta offset
                if (gcode->has_letter('X')) theta_offset[0] = gcode->get_value('X');
                if (gcode->has_letter('Y')) theta_offset[1] = gcode->get_value('Y');
                if (gcode->has_letter('Z')) theta_offset[2] = gcode->get_value('Z');

                PublicData::set_value( endstops_checksum, home_offset_checksum, theta_offset );

                gcode->stream->printf("Theta offset set: X %8.5f Y %8.5f Z %8.5f\n", theta_offset[0], theta_offset[1], theta_offset[2]);

                break;
            }

            case 306: {
                // for a rotary delta M306 calibrates the homing angle
                // by doing M306 X-56.17 it will calculate the M206 X value (the theta offset for actuator X) based on the difference
                // between what it thinks is the current angle and what the current angle actually is specified by X (ditto for Y and Z)

                ActuatorCoordinates current_angle;
                // get the current angle for each actuator, relies on being left where probe triggered (G30.1)
                // NOTE we only deal with XYZ so if there are more than 3 actuators this will probably go wonky
                for (size_t i = 0; i < 3; i++) {
                    current_angle[i]= THEROBOT->actuators[i]->get_current_position();
                }

                if (gcode->has_letter('L') && gcode->get_value('L') != 0) {
                    // specifying L1 it will take the last probe position (set by G30) which is degrees moved
                    // we convert that to actual postion of the actuator when it triggered.
                    // this allows the use of G30 to find the angle tool and M306 L1 X-37.1234 to set X
                    uint8_t ok;
                    float d;
                    // only Z is set by G30, but all will be the same amount moved
                    std::tie(std::ignore, std::ignore, d, ok) = THEROBOT->get_last_probe_position();
                    if(ok == 0) {
                        gcode->stream->printf("error:Nothing set as probe failed or not run\n");
                        return;
                    }
                    // presuming we returned to original position we need to change current angle by the amount moved
                    current_angle[0] += d;
                    current_angle[1] += d;
                    current_angle[2] += d;
                }

                float theta_offset[3];
                if(!get_homing_offset(theta_offset)) {
                    gcode->stream->printf("error:no endstop module found\n");
                    return;
                }

                int cnt= 0;

                // figure out what home_offset needs to be to correct the homing_position
                if (gcode->has_letter('X')) {
                    float a = gcode->get_value('X'); // what actual angle is
                    theta_offset[0] += (a - current_angle[0]);
                    current_angle[0]= a;
                    cnt++;
                }
                if (gcode->has_letter('Y')) {
                    float b = gcode->get_value('Y');
                    theta_offset[1] += (b - current_angle[1]);
                    current_angle[1]= b;
                    cnt++;
                }
                if (gcode->has_letter('Z')) {
                    float c = gcode->get_value('Z');
                    theta_offset[2] += (c - current_angle[2]);
                    current_angle[2]= c;
                    cnt++;
                }

                PublicData::set_value( endstops_checksum, home_offset_checksum, theta_offset );

                // reset the actuator positions (and machine position accordingly)
                // But only if all three actuators have been specified at the same time
                if(cnt == 3 || (gcode->has_letter('R') && gcode->get_value('R') != 0)) {
                    THEROBOT->reset_actuator_position(current_angle);
                    gcode->stream->printf("NOTE: actuator position reset\n");
                }else{
                    gcode->stream->printf("NOTE: actuator position NOT reset\n");
                }

                gcode->stream->printf("Theta Offset: X %8.5f Y %8.5f Z %8.5f\n", theta_offset[0], theta_offset[1], theta_offset[2]);
            }
        }
    }
}
