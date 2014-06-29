/*
      This file is part of Smoothie (http://smoothieware.org/). The motion control part is heavily based on Grbl (https://github.com/simen/grbl).
      Smoothie is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
      Smoothie is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
      You should have received a copy of the GNU General Public License along with Smoothie. If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef PUBLICDATA_H
#define PUBLICDATA_H

class PublicData {
    public:
        static bool get_value(uint16_t csa, void **data) { return get_value(csa, 0, 0, data); }
        static bool get_value(uint16_t csa, uint16_t csb, void **data) { return get_value(csa, csb, 0, data); }
        static bool get_value(uint16_t cs[3], void **data) { return get_value(cs[0], cs[1], cs[2], data); };
        static bool get_value(uint16_t csa, uint16_t csb, uint16_t csc, void **data);

        static bool set_value(uint16_t csa, void *data) { return set_value(csa, 0, 0, data); }
        static bool set_value(uint16_t csa, uint16_t csb, void *data) { return set_value(csa, csb, 0, data); }
        static bool set_value(uint16_t cs[3], void *data) { return set_value(cs[0], cs[1], cs[2], data); }
        static bool set_value(uint16_t csa, uint16_t csb, uint16_t csc, void *data);
};

#endif
