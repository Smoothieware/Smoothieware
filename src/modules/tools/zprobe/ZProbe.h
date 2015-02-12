/*
      This file is part of Smoothie (http://smoothieware.org/). The motion control part is heavily based on Grbl (https://github.com/simen/grbl).
      Smoothie is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
      Smoothie is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
      You should have received a copy of the GNU General Public License along with Smoothie. If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef ZPROBE_H_
#define ZPROBE_H_

#include "Module.h"
#include "Pin.h"

#include <vector>

// defined here as they are used in multiple files
#define zprobe_checksum            CHECKSUM("zprobe")
#define leveling_strategy_checksum CHECKSUM("leveling-strategy")

class StepperMotor;
class Gcode;
class StreamOutput;
class LevelingStrategy;

class ZProbe: public Module
{

public:
    void on_module_loaded();
    void on_config_reload(void *argument);
    void on_gcode_received(void *argument);
    void acceleration_tick(void);

    bool wait_for_probe(int& steps);
    bool run_probe(int& steps, bool fast= false);
    bool return_probe(int steps);
    bool doProbeAt(int &steps, float x, float y);
    float probeDistance(float x, float y);

    void coordinated_move(float x, float y, float z, float feedrate, bool relative=false);
    void home();

    bool getProbeStatus() { return this->pin.get(); }
    float getSlowFeedrate() { return slow_feedrate; }
    float getFastFeedrate() { return fast_feedrate; }
    float getProbeHeight() { return probe_height; }
    float zsteps_to_mm(float steps);

    unsigned int getDebounceCount() { return debounce_count; }
    void setDebounceCount(unsigned int i) { if(i > 0 && i < 10000) debounce_count = i; }

    void setFastFeedrate(float f) { if(f > 0 && f < 200) fast_feedrate = f; }
    void setSlowFeedrate(float f) { if(f > 0 && f < 200) slow_feedrate = f; }

    void setDecelerateOnTrigger(bool t);
    bool getDecelerateOnTrigger() { return decelerate_on_trigger; }
    unsigned int getStepsAtDecelEnd() { return steps_at_decel_end; }
    
    void clearHasExceededRunout() { has_exceeded_runout = false; }
    bool getHasExceededRunout() { return has_exceeded_runout; }
    

private:
    void accelerate(int c);
    void decelerate(int c);
    bool accelerating;
    unsigned int steps_at_decel_end;	// Holds # of steps at the moment decelerate() stops the motors

    volatile float current_feedrate;
    float slow_feedrate;
    float fast_feedrate;
    float probe_height;
    float max_z;
    bool decelerate_on_trigger;
    float decelerate_runout;
    uint32_t runout_steps;
    bool has_exceeded_runout;
    volatile struct {
        volatile bool running:1;
        bool is_delta:1;
    };

    Pin pin;
    uint8_t debounce_count;
    std::vector<LevelingStrategy*> strategies;
};

#endif /* ZPROBE_H_ */
