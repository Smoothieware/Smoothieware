/*
      This file is part of Smoothie (http://smoothieware.org/). The motion control part is heavily based on Grbl (https://github.com/simen/grbl).
      Smoothie is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
      Smoothie is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
      You should have received a copy of the GNU General Public License along with Smoothie. If not, see <http://www.gnu.org/licenses/>.
*/

#pragma once

#include <array>

#ifndef MAX_ROBOT_ACTUATORS
    #ifdef CNC
    #define MAX_ROBOT_ACTUATORS 3
    #else
    // includes 2 extruders
    #define MAX_ROBOT_ACTUATORS 5
    #endif
#endif

#if MAX_ROBOT_ACTUATORS < 3 || MAX_ROBOT_ACTUATORS > 6
#error "MAX_ROBOT_ACTUATORS must be >= 3 and <= 6"
#endif

#ifndef N_PRIMARY_AXIS
    // This may chnage and include ABC
    #define N_PRIMARY_AXIS 3
#endif

#if N_PRIMARY_AXIS < 3 || N_PRIMARY_AXIS > MAX_ROBOT_ACTUATORS
#error "N_PRIMARY_AXIS must be >= 3 and <= MAX_ROBOT_ACTUATORS"
#endif

// Keep MAX_ROBOT_ACTUATORS as small as practical it impacts block size and therefore free memory.
const size_t k_max_actuators = MAX_ROBOT_ACTUATORS;
typedef struct std::array<float, k_max_actuators> ActuatorCoordinates;
