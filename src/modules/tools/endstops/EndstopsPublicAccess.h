#ifndef __ENDSTOPSPUBLICACCESS_H_
#define __ENDSTOPSPUBLICACCESS_H_

// addresses used for public data access
#define endstops_checksum    CHECKSUM("endstop")
#define trim_checksum        CHECKSUM("trim")
#define home_offset_checksum CHECKSUM("home_offset")
#define saved_position_checksum CHECKSUM("saved_position")
#define get_homing_status_checksum CHECKSUM("homing_status")
#define get_homed_status_checksum CHECKSUM("homed_status")

#endif
