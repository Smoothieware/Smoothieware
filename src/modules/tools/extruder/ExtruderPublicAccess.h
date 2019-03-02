#pragma once

#define extruder_checksum                    CHECKSUM("extruder")
#define save_state_checksum                  CHECKSUM("save_state")
#define restore_state_checksum               CHECKSUM("restore_state")
#define target_checksum                      CHECKSUM("target")

using pad_extruder_t = struct pad_extruder {
    float steps_per_mm;
    float filament_diameter;
    float flow_rate;
    float accleration;
    float retract_length;
    float current_position;
};
