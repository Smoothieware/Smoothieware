/*
      This file is part of Smoothie (http://smoothieware.org/). The motion control part is heavily based on Grbl (https://github.com/simen/grbl).
      Smoothie is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
      Smoothie is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
      You should have received a copy of the GNU General Public License along with Smoothie. If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef PUBLICDATAREQUEST_H
#define PUBLICDATAREQUEST_H

class PublicDataRequest {
    public:
        PublicDataRequest(uint16_t addrcs1){ target[0]= addrcs1; target[1]= 0; target[2]= 0; data_taken= false; data= NULL; returned_data= true; }
        PublicDataRequest(uint16_t addrcs1, uint16_t addrcs2){ target[0]= addrcs1; target[1]= addrcs2; target[2]= 0; data_taken= false; data= NULL; returned_data= true; }
        PublicDataRequest(uint16_t addrcs1, uint16_t addrcs2, uint16_t addrcs3){ target[0]= addrcs1; target[1]= addrcs2; target[2]= addrcs3; data_taken= false; data= NULL; returned_data= true; }

        virtual ~PublicDataRequest() { data= nullptr; }

        bool starts_with(uint16_t addr) const { return addr == this->target[0]; }
        bool second_element_is(uint16_t addr) const { return addr == this->target[1]; }
        bool third_element_is(uint16_t addr) const { return addr == this->target[2]; }

        bool is_taken() const { return this->data_taken; }
        void set_taken() { this->data_taken= true; }
        bool has_returned_data() const { return this->returned_data; }
        void set_data_ptr(void *d, bool flag= true) { this->data= d; returned_data= flag; }
        void* get_data_ptr(void) const { return this->data; }

    private:
        uint16_t target[3];
        void* data;
        struct {
            bool data_taken:1;
            bool returned_data:1; // this is set if the callee returns the data, it is false if the caller supplied the storage for the return
        };
};

#endif
