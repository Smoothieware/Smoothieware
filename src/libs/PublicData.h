/*
      This file is part of Smoothie (http://smoothieware.org/). The motion control part is heavily based on Grbl (https://github.com/simen/grbl).
      Smoothie is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
      Smoothie is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
      You should have received a copy of the GNU General Public License along with Smoothie. If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef PUBLICDATA_H
#define PUBLICDATA_H

#include "libs/Kernel.h"

class PublicData : public Module {
	public:
		void get_value(uint16_t csa, void **data) { get_value(csa, 0, 0, data); }
		void get_value(uint16_t csa, uint16_t csb, void **data) { get_value(csa, csb, 0, data); }
		void get_value(uint16_t cs[3], void **data) { get_value(cs[0], cs[1], cs[2], data); };
		void get_value(uint16_t csa, uint16_t csb, uint16_t csc, void **data);

		void set_value(uint16_t csa, void *data) { set_value(csa, 0, 0, data); }
		void set_value(uint16_t csa, uint16_t csb, void *data) { set_value(csa, csb, 0, data); }
		void set_value(uint16_t cs[3], void *data) { set_value(cs[0], cs[1], cs[2], data); }
		void set_value(uint16_t csa, uint16_t csb, uint16_t csc, void *data);
};

#endif