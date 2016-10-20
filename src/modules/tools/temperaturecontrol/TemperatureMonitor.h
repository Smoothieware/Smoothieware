/*
 * TemperatureMonitor.h
 *
 *  Created on: Oct 19, 2016
 *      Author: mmoore
 */

#ifndef SRC_MODULES_TOOLS_TEMPERATURECONTROL_TEMPERATUREMONITOR_H_
#define SRC_MODULES_TOOLS_TEMPERATURECONTROL_TEMPERATUREMONITOR_H_

#include <stdint.h>
#include <string>

using std::string;


class TemperatureMonitor
{
public:
    TemperatureMonitor(uint8_t range, uint16_t timeout);
    virtual ~TemperatureMonitor();

    void set_target_temperature(float target, float last_target, float current_temp);
    void on_second_tick(float target_temp, float current_temp, string control_designator);

private:
    enum RUNAWAY_TYPE {
        NOT_HEATING              = 0,
        HEATING_UP               = 1,
        COOLING_DOWN             = 2,
        MAINTAINING_TEMPERATURE  = 3,
    };


    struct {
        RUNAWAY_TYPE state       :2; // max value 3.
        uint16_t heating_timer   :9; // max value 511 (08:31).
        uint16_t heating_timeout :9; // max value 511 (08:31).
        uint8_t range            :6; // max value 63.
        uint8_t last_temperature :6; // max value 63 (we only track the lowest 6-bits of the integer temperature value).
    };
};

#endif /* SRC_MODULES_TOOLS_TEMPERATURECONTROL_TEMPERATUREMONITOR_H_ */
