/*
      This file is part of Smoothie (http://smoothieware.org/). The motion control part is heavily based on Grbl (https://github.com/simen/grbl).
      Smoothie is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
      Smoothie is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
      You should have received a copy of the GNU General Public License along with Smoothie. If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef PUBLICDATAREQUEST_H
#define PUBLICDATAREQUEST_H

#include "libs/Kernel.h"

class PublicDataRequest {
	public:
 		// targets.. the module(s) that needs to respond
 		static const uint16_t pdr_temperature_control_checksum=    CHECKSUM("temperature_control");
 		static const uint16_t pdr_robot_checksum=                  CHECKSUM("robot");
 
 		// sub targets.. the sub target in a module
 		static const uint16_t pdr_hotend_checksum=                 CHECKSUM("hotend");
 		static const uint16_t pdr_bed_checksum=                    CHECKSUM("bed");
 
 
 		// values.. The values to retrieve from the target
 		static const uint16_t pdr_current_temperature_checksum=    CHECKSUM("current_temperature");
 		static const uint16_t pdr_target_temperature_checksum=     CHECKSUM("target_temperature");
 		static const uint16_t pdr_temperature_pwm_checksum=        CHECKSUM("temperature_pwm");
 		static const uint16_t pdr_speed_override_percent_checksum= CHECKSUM("speed_override_percent");


	public:
		PublicDataRequest(uint16_t addrcs1){ target[0]= addrcs1; target[1]= 0; target[2]= 0; data_taken= false; data= NULL; }
		PublicDataRequest(uint16_t addrcs1, uint16_t addrcs2){ target[0]= addrcs1; target[1]= addrcs2; target[2]= 0; data_taken= false; data= NULL; }
		PublicDataRequest(uint16_t addrcs1, uint16_t addrcs2, uint16_t addrcs3){ target[0]= addrcs1; target[1]= addrcs2; target[2]= addrcs3; data_taken= false; data= NULL; }

		virtual ~PublicDataRequest() { data= NULL; }
		
		bool starts_with(uint16_t addr) const { return addr == target[0]; }
		bool second_element_is(uint16_t addr) const { return addr == target[1]; }
		bool third_element_is(uint16_t addr) const { return addr == target[2]; }

		bool is_taken() const { return data_taken; }
		void set_taken() { this->data_taken= true; }
		
		void set_data_ptr(void *d) { data= d; }
		void* get_data_ptr(void) { return data; }

	private:
		uint16_t target[3];
		void* data;
		bool data_taken;
};

#endif