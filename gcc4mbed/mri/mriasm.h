/* Copyright 2011 Adam Green (http://mbed.org/users/AdamGreen/)

   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU Lesser General Public License as published
   by the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU Lesser General Public License for more details.

   You should have received a copy of the GNU Lesser General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.   
*/
/* Monitor for Remote Inspection ARM assembler routine header file. */
#ifndef _MRIASM_H_
#define _MRIASM_H_

extern "C" int MriDisableMbed(void);
extern "C" unsigned int MriGetIPSR(void);

#endif /* _MRIASM_H_ */
