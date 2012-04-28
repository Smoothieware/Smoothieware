/* Copyright 2012 Adam Green (http://mbed.org/users/AdamGreen/)

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
/* Monitor for Remote Inspection private header file. */
#ifndef _MRIPRIV_H_
#define _MRIPRIV_H_

extern "C" int __MriSemihostWrite(int file, char *ptr, int len);
extern "C" int __MriSemihostRead(int file, char *ptr, int len);

#endif /* _MRIPRIV_H_ */
