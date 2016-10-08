
/*
This file is part of Smoothie (http://smoothieware.org/). The motion control part is heavily based on Grbl (https://github.com/simen/grbl).
Smoothie is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
Smoothie is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
You should have received a copy of the GNU General Public License along with Smoothie. If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef JOYSTICKPUBLICACCESS_H
#define JOYSTICKPUBLICACCESS_H

//addresses used for public data access
#define joystick_checksum              CHECKSUM("joystick")
#define value_checksum                 CHECKSUM("value")

struct PAD_joystick {
    int name_checksum; //checksum of the module name
    unsigned int raw; //raw ADC voltage reading
    float position; //position of the joystick axis (from -1 to 1)
};

#endif // JOYSTICKPUBLICACCESS_H
