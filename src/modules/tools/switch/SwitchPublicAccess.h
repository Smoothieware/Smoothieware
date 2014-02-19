#ifndef __SWITCHPUBLICACCESS_H
#define __SWITCHPUBLICACCESS_H

// addresses used for public data access
#define fan_checksum                        CHECKSUM("fan")
#define state_checksum                      CHECKSUM("state")
#define value_checksum                      CHECKSUM("value")
/*
#define temperature_control_checksum      CHECKSUM("temperature_control")
#define hotend_checksum                   CHECKSUM("hotend")
#define bed_checksum                      CHECKSUM("bed")
#define current_temperature_checksum      CHECKSUM("current_temperature")
#define target_temperature_checksum       CHECKSUM("target_temperature")
#define temperature_pwm_checksum          CHECKSUM("temperature_pwm")
*/

struct pad_switch {
    int name;
    bool state;
    float value;
};
#endif // __SWITCHPUBLICACCESS_H
