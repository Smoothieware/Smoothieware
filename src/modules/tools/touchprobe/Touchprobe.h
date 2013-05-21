/*
      This file is part of Smoothie (http://smoothieware.org/). The motion control part is heavily based on Grbl (https://github.com/simen/grbl).
      Smoothie is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
      Smoothie is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
      You should have received a copy of the GNU General Public License along with Smoothie. If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef TOUCHPROBE_H_
#define TOUCHPROBE_H_

#include "libs/Module.h"
#include "modules/robot/Conveyor.h"
#include "libs/Kernel.h"
#include "modules/communication/utils/Gcode.h"
#include "libs/StepperMotor.h"
#include "libs/Pin.h"

#define touchprobe_enable_checksum           CHECKSUM("touchprobe_enable")
#define touchprobe_pin_checksum              CHECKSUM("touchprobe_pin")
#define touchprobe_debounce_count_checksum   CHECKSUM("touchprobe_debounce_count")

class Touchprobe: public Module {
    private:
        void wait_for_touch(int remaining_steps[]);
    public:
        void on_module_loaded();
        void on_config_reload(void* argument);
        void on_gcode_received(void* argument);

        StepperMotor*  steppers[3];
        Pin            pin;
        unsigned int   debounce_count;
        static bool enabled;
};

#endif /* TOUCHPROBE_H_ */
