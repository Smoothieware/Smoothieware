/*  
      This file is part of Smoothie (http://smoothieware.org/). The motion control part is heavily based on Grbl (https://github.com/simen/grbl).
      Smoothie is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
      Smoothie is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
      You should have received a copy of the GNU General Public License along with Smoothie. If not, see <http://www.gnu.org/licenses/>. 
*/
#ifndef LCDBANG_H
#define LCDBANG_H
#include "mbed.h" // mbed.h lib
#include "I2CBang.h"

void lcdbang_writenibble(I2C i2c, char c, bool command = false){
    const int addr = 0x40;
    char cmd[2];
    c <<= 4;
    c &= 0xF0;
    c |= 0x01;
    if(!command) c |= 0x02;

    cmd[0] = 0x0C;
    cmd[1] = c;
    i2c.write(addr, cmd, 2);
    cmd[1] = c | 0x08;
    i2c.write(addr, cmd, 2);
    cmd[1] = c;
    i2c.write(addr, cmd, 2);
//    wait_ms(1);
}

void lcdbang_write(I2C i2c, char c, bool command = false){
    if(command){
        lcdbang_writenibble(i2c, c, command);
    }else{
        lcdbang_writenibble(i2c, c >> 4, command);
        lcdbang_writenibble(i2c, c, command);
    }
}

void lcdbang_init(I2C i2c){
    const int addr = 0x40;
    char cmd[2];
    cmd[0] = 0x1C;
    cmd[1] = 0x00;
    i2c.write(addr, cmd, 2);

    lcdbang_write(i2c, 0x3, true);
    wait_ms(50);
    lcdbang_write(i2c, 0x3, true);
    wait_ms(10);
    lcdbang_write(i2c, 0x3, true);
    wait_ms(10);
    lcdbang_write(i2c, 0x2, true);
    wait_ms(1);

    lcdbang_write(i2c, 0x2, true);
    lcdbang_write(i2c, 0x8, true);
    wait_ms(1);

    lcdbang_write(i2c, 0x0, true);
    lcdbang_write(i2c, 0x8, true);
    wait_ms(1);

    lcdbang_write(i2c, 0x0, true);
    lcdbang_write(i2c, 0x1, true);
    wait_ms(1);

    lcdbang_write(i2c, 0x0, true);
    lcdbang_write(i2c, 0x6, true);
    wait_ms(1);

    lcdbang_write(i2c, 0x0, true);
    lcdbang_write(i2c, 0x2, true);
    wait_ms(1);

    lcdbang_write(i2c, 0x0, true);
    lcdbang_write(i2c, 0xC, true);
    wait_ms(1);
}

void lcdbang_print(I2C i2c, const char* msg){
    for(int i=0;msg[i];i++){
        lcdbang_write(i2c, msg[i]);
    }
}

void lcdbang_contrast(I2C i2c, int contrast){
    // set dac pins as output and set dac
    i2cbang_init(i2c);
    i2cbang_start(i2c);
    i2cbang_write(i2c, 0xC0);
    i2cbang_write(i2c, 0x60);
    i2cbang_write(i2c, contrast >> 8);
    i2cbang_write(i2c, (contrast << 8) & 0xF0 );
    i2cbang_stop(i2c);
}

#endif // LCDBANG_H

