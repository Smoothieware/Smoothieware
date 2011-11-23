#ifndef EXTURDER_MODULE_H
#define EXTRUDER_MODULE_H

#include "mbed.h"
#include "libs/Module.h"
#include "libs/Kernel.h"
#include "modules/robot/Block.h"

#define microseconds_per_step_pulse_ckeckusm 42333
#define extruder_module_enable_checksum      6183

class Extruder : public Module{
    public:
        Extruder(PinName stppin, PinName dirpin);
        void on_module_loaded();
        void on_config_reload(void* argument);
        void on_gcode_execute(void* argument);
        void on_block_begin(void* argument);
        void on_block_end(void* argument);
        void on_speed_change(void* argument);
        void acceleration_tick();
        void stepping_tick();

        DigitalOut      step_pin;                     // Step pin for the stepper driver
        DigitalOut      dir_pin;                      // Dir pin for the stepper driver
        double          start_position;               // Start point ( in steps ) for the current move
        double          target_position;              // End point ( in steps ) for the current move
        double          current_position;             // Current point ( in steps ) for the current move, incremented every time a step is outputed
        Ticker          acceleration_ticker;          // Ticker responsible with updating the speed ( acceleration management ). Uses Timer3
        Block*          current_block;                // Current block we are stepping, same as Stepper's one
        int             microseconds_per_step_pulse;  // Pulse duration for step pulses

        bool            solo_mode;
        double          travel_ratio;
        double          travel_distance;
        bool            absolute_mode;

        int             direction;
};

#endif
