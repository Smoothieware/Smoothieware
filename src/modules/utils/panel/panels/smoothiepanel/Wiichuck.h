/*  
      This file is part of Smoothie (http://smoothieware.org/). The motion control part is heavily based on Grbl (https://github.com/simen/grbl).
      Smoothie is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
      Smoothie is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
      You should have received a copy of the GNU General Public License along with Smoothie. If not, see <http://www.gnu.org/licenses/>. 
*/
#ifndef WIICHUCK_H
#define WIICHUCK_H
#include "mbed.h" // mbed.h lib

using namespace std;
#include <vector>
#include <string>
#include <cstdio>
#include <cstdarg>

#define DEVICE_NUNCHUCK             0x0000
#define DEVICE_CLASSIC              0x0101
#define DEVICE_INACT_WMP            0x0005
#define DEVICE_WMP                  0x0405
#define DEVICE_WMP_NUNCHUCK         0x0505
#define DEVICE_WMP_CLASSIC          0x0705

#define WIICHUCK                    0xA4

class Wiichuck {
public:
    Wiichuck(PinName sda, PinName scl, int frequency = 10000){
        this->i2c_mine = true;

        this->i2c = new mbed::I2C(sda, scl);   
        this->i2c->frequency(frequency);
    }

    Wiichuck(I2C* i2c){
        this->i2c_mine = false;

        this->i2c = i2c;
    }

    ~Wiichuck(){
        if(this->i2c_mine)
            delete this->i2c;
    }

    void init_device(){
        char cmd[2];
        data_ready = false;

        cmd[0] = 0xF0; // first two writes are a magic init sequence
        cmd[1] = 0x55;
        this->i2c->write(WIICHUCK, cmd, 2);
        wait_ms(10);
        cmd[0] = 0xFB;
        cmd[1] = 0x00;
        this->i2c->write(WIICHUCK, cmd, 2);
        wait_ms(10);
        cmd[0] = 0xFA; // read out the device type
        this->i2c->write(WIICHUCK, cmd, 1, false);
        int res = this->i2c->read(WIICHUCK, data_buf, 6);
        cmd[0] = 0x00; // request first sensor readings
        this->i2c->write(WIICHUCK, cmd, 1);
        if(res == 0 && data_buf[2] == 0xA4){
            device_type = (data_buf[4] << 8) + data_buf[5];
        }else{
            device_type = -1;
        }
    }

    void poll_device() {
        data_ready = false;
        if(device_type < 0) {
            init_device();
            wait_ms(100);
        }
        // if there is a connected device read it and if it responds right parse the data
        if(device_type >= 0 && read_device() == 0) {
            switch(device_type) {
            case DEVICE_NUNCHUCK:       parse_nunchuck(); break;
            case DEVICE_CLASSIC:        parse_classic(); break;
            case DEVICE_INACT_WMP:      init_wmp(); break;
            case DEVICE_WMP:            parse_wmp(); break;
            default:
                break;
            }
        }
    }

    int read_device() {
        char cmd = 0x00;
        int res = this->i2c->read(WIICHUCK, data_buf, 6); // read sensors
        this->i2c->write(WIICHUCK, &cmd, 1); // request next sensor readings
        return res;
    }

    void parse_nunchuck() {
        SX = data_buf[0];
        SY = data_buf[1];
        AX = (data_buf[2] << 2) + ((data_buf[5] >> 2) & 0x03);
        AY = (data_buf[3] << 2) + ((data_buf[5] >> 4) & 0x03);
        AZ = (data_buf[5] << 2) + ((data_buf[5] >> 6) & 0x03);
        BC = (data_buf[5] >> 1) & 0x01;
        BZ = data_buf[5] & 0x01;
        data_ready = true;
    }

    void parse_classic() {
        LX = data_buf[0] << 2;
        LY = data_buf[1] << 2;
        RX = (data_buf[0] & 0xC0) + ((data_buf[1] & 0xC0) >> 2) + ((data_buf[2] & 0x80) >> 4);
        RY = data_buf[2] << 3;
        LT = ((data_buf[2] & 0x60) << 1) + ((data_buf[3] & 0xE0) >> 2);
        RT = data_buf[3] << 3;
        BDU = data_buf[5] & 0x01;
        BDD = (data_buf[4] >> 6) & 0x01;
        BDL = (data_buf[5] >> 1) & 0x01;
        BDR = (data_buf[4] >> 7) & 0x01;
        BLT = (data_buf[4] >> 5) & 0x01;
        BRT = (data_buf[4] >> 1) & 0x01;
        BH  = (data_buf[4] >> 3) & 0x01;
        BP  = (data_buf[4] >> 2) & 0x01;
        BM  = (data_buf[4] >> 4) & 0x01;
        BA  = (data_buf[5] >> 4) & 0x01;
        BB  = (data_buf[5] >> 6) & 0x01;
        BX  = (data_buf[5] >> 3) & 0x01;
        BY  = (data_buf[5] >> 5) & 0x01;
        BZL = (data_buf[5] >> 7) & 0x01;
        BZR = (data_buf[5] >> 2) & 0x01;
        data_ready = true;
    }

    void init_wmp() {
    }

    void activate_wmp() {
    }

    void deactivate_wmp() {
    }

    void parse_wmp() {
    }

    char* get_raw() {
        return data_buf;
    }

    // nunchuck input state variables
    char SX,SY;               // 8-bit joystick
    short AX,AY,AZ;           // 10-bit accelerometer
    bool BC,BZ;               // buttons

    // classic input state variables
    char LX,LY,RX,RY,LT,RT;   // 6-bit left joystick, 5-bit right joystick and triggers
    bool BDU,BDD,BDL,BDR;     // d-pad buttons
    bool BLT,BRT;             // digital click of triggers
    bool BH,BP,BM;            // home, plus, minus buttons
    bool BA,BB,BX,BY,BZL,BZR; // buttons

    bool i2c_mine;
    bool data_ready = false;
    char data_buf[6];
    int device_type = -1;
    mbed::I2C* i2c;
};


#endif // WIICHUCK_H

