/*
      This file is part of Smoothie (http://smoothieware.org/). The motion control part is heavily based on Grbl (https://github.com/simen/grbl).
      Smoothie is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
      Smoothie is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
      You should have received a copy of the GNU General Public License along with Smoothie. If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef ACTUATOR_COORDINATES_H
#define ACTUATOR_COORDINATES_H
#include <array>

#ifndef MAX_ROBOT_ACTUATORS
#define MAX_ROBOT_ACTUATORS 3
#endif

//The subset in use is determined by the arm solution's get_actuator_count().
//Keep MAX_ROBOT_ACTUATORS as small as practical it impacts block size and therefore free memory.
const size_t k_max_actuators = MAX_ROBOT_ACTUATORS;
typedef struct std::array<float, k_max_actuators> ActuatorCoordinates;

#endif
