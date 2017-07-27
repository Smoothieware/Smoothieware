/*
      this file is part of smoothie (http://smoothieware.org/). the motion control part is heavily based on grbl (https://github.com/simen/grbl).
      smoothie is free software: you can redistribute it and/or modify it under the terms of the gnu general public license as published by the free software foundation, either version 3 of the license, or (at your option) any later version.
      smoothie is distributed in the hope that it will be useful, but without any warranty; without even the implied warranty of merchantability or fitness for a particular purpose. see the gnu general public license for more details.
      you should have received a copy of the gnu general public license along with smoothie. if not, see <http://www.gnu.org/licenses/>.
*/

#pragma once

#include <memory>

#include "libs/Pin.h"
#include "Gcode.h"

#define readpin_checksum CHECKSUM("readpin")

class ReadPin : public Module {
public:
    ReadPin();
    ReadPin(uint16_t name);
    
    void on_module_loaded() override;

    // This is not virtual?
    void on_config_reload(void* argument);
    
    void on_gcode_received(void* argument) override;
    
    uint32_t pin_tick(uint32_t dummy);

private:
    uint16_t name_checksum;
    Pin my_pin;
    std::unique_ptr<Gcode> my_gcode;

    // Pin state, set in ISR
    volatile bool state = false;
};
