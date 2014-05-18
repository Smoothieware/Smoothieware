/*
nuts_bolts.h - cartesian robot controller.
Part of Grbl

Copyright (c) 2009-2011 Simen Svale Skogsrud

Grbl is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

Grbl is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with Grbl. If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef nuts_bolts_h
#define nuts_bolts_h

#define X_AXIS 0
#define Y_AXIS 1
#define Z_AXIS 2

#define ALPHA_STEPPER 0
#define BETA_STEPPER 1
#define GAMMA_STEPPER 2

#define clear_vector(a) memset(a, 0, sizeof(a))
#define clear_vector_float(a) memset(a, 0, sizeof(a))

#define confine(value, min, max) (((value) < (min))?(min):(((value) > (max))?(max):(value)))

#define dd(...) LPC_GPIO2->FIODIR = 0xffff; LPC_GPIO2->FIOCLR = 0xffff; LPC_GPIO2->FIOSET = __VA_ARGS__


#endif

