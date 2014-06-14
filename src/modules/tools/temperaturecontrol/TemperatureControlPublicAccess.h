#ifndef __TEMPERATURECONTROLPUBLICACCESS_H
#define __TEMPERATURECONTROLPUBLICACCESS_H

#include "checksumm.h"

#include <string>

// addresses used for public data access
#define temperature_control_checksum      CHECKSUM("temperature_control")
#define hotend_checksum                   CHECKSUM("hotend")
#define bed_checksum                      CHECKSUM("bed")
#define current_temperature_checksum      CHECKSUM("current_temperature")
#define target_temperature_checksum       CHECKSUM("target_temperature")
#define temperature_pwm_checksum          CHECKSUM("temperature_pwm")
#define pool_index_checksum               CHECKSUM("pool_index")

struct pad_temperature {
    float current_temperature;
    float target_temperature;
    int pwm;
    std::string designator;
};
#endif
