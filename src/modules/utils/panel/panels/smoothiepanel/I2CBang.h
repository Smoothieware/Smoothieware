/*  
      This file is part of Smoothie (http://smoothieware.org/). The motion control part is heavily based on Grbl (https://github.com/simen/grbl).
      Smoothie is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
      Smoothie is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
      You should have received a copy of the GNU General Public License along with Smoothie. If not, see <http://www.gnu.org/licenses/>. 
*/
#ifndef I2CBANG_H
#define I2CBANG_H
#include "mbed.h" // mbed.h lib

void i2cbang_init(I2C i2c){
    const int addr = 0x40;
    char cmd[2];
    cmd[0] = 0x1B;
    cmd[1] = 0x3F;
    i2c.write(addr, cmd, 2);
    cmd[0] = 0x0B;
    cmd[1] = 0xC0;
    i2c.write(addr, cmd, 2);
    wait_ms(1);
}

void i2cbang_start(I2C i2c){
    const int addr = 0x40;
    char cmd[2];
    cmd[0] = 0x0B;
    cmd[1] = 0xBF;
    i2c.write(addr, cmd, 2);
    wait_ms(1);
    cmd[1] = 0x3F;
    i2c.write(addr, cmd, 2);
    wait_ms(1);
}

void i2cbang_stop(I2C i2c){
    const int addr = 0x40;
    char cmd[2];
    cmd[0] = 0x0B;
    cmd[1] = 0xBF;
    i2c.write(addr, cmd, 2);
    wait_ms(1);
    cmd[1] = 0xFF;
    i2c.write(addr, cmd, 2);
    wait_ms(1);
}

void i2cbang_writebit(I2C i2c, bool bit){
    const int addr = 0x40;
    char cmd[2];
    cmd[0] = 0x0B;
    if(bit){
        cmd[1] = 0x7F;
    }else{
        cmd[1] = 0x3F;
    }
    i2c.write(addr, cmd, 2);
    wait_ms(1);

    if(bit){
        cmd[1] = 0xFF;
    }else{
        cmd[1] = 0xBF;
    }
    i2c.write(addr, cmd, 2);
    wait_ms(1);

    if(bit){
        cmd[1] = 0x7F;
    }else{
        cmd[1] = 0x3F;
    }
    i2c.write(addr, cmd, 2);
    wait_ms(1);

    if(bit){
        cmd[1] = 0x3F;
        i2c.write(addr, cmd, 2);
    }
    wait_ms(1);
}

char i2cbang_readbit(I2C i2c){
    const int addr = 0x40;
    char cmd[2];
    char res;
    cmd[0] = 0x0B;
    cmd[1] = 0x7F;
    i2c.write(addr, cmd, 2);
    wait_ms(1);

    cmd[1] = 0xFF;
    i2c.write(addr, cmd, 2);
    wait_ms(1);

    cmd[0] = 0x03;
    i2c.write(addr, cmd, 1, false);
    i2c.read(addr, cmd, 1);
    res = cmd[0];
    wait_ms(1);

    cmd[0] = 0x0B;
    cmd[1] = 0x7F;
    i2c.write(addr, cmd, 2);
    wait_ms(1);

//    cmd[1] = 0x3F;
//    i2c.write(addr, cmd, 2);
//    wait_ms(1);

    //res = (~res) & 0x40;
    return res;
}

int i2cbang_write(I2C i2c, char c){
    for (int i=0;i<8;i++){
        i2cbang_writebit(i2c, (c&0x80) > 0);
        c<<=1;
    }

    return i2cbang_readbit(i2c);
/*
    const int addr = 0x40;
    char cmd[2];
    char d = 0x00;
    //data
    for (int i=7;i>=0;i--){
        i2c.write(0x3F | d);
        d = ((c>>i)&1)<<6;
        i2c.write(0x3F | d);
        i2c.write(0xBF | d);
    }
    //ack
    i2c.write(0x3F | d);
    i2c.write(0xBF);
    i2c.stop();
    cmd[0] = 0x1B;
    cmd[1] = 0x7F;
    i2c.write(addr, cmd, 2);
    cmd[0] = 0x03;
    i2c.write(addr, cmd, 1, false);
    i2c.start();
    i2c.write(addr | 0x01);
    cmd[1] = i2c.read(false);
//    int res = (~cmd[1]) & 0x40;
    int res = cmd[1];
    i2c.stop();
    cmd[0] = 0x1B;
    cmd[1] = 0x3F;
    i2c.write(addr, cmd, 2);
    cmd[0] = 0x0B;
    cmd[1] = 0xBF;
    i2c.write(addr, cmd, 2, false);
    return res;
*/
}

char i2cbang_read(I2C i2c, bool ack){
    char res = 0;
    for(int i=0;i<8;i++){
        res<<=1;
        res |= i2cbang_readbit(i2c);
    }

    if(ack){
        i2cbang_writebit(i2c, 0);
    }else{
        i2cbang_writebit(i2c, 1);
    }

    wait_ms(1);

    return res;
}

#endif // I2CBANG_H

