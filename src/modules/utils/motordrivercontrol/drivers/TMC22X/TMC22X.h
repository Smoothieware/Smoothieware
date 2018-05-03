/*
 modified from...
 TMC26X.cpp - - TMC26X Stepper library for Wiring/Arduino

 based on the stepper library by Tom Igoe, et. al.

 Copyright (c) 2011, Interactive Matter, Marcus Nowotny

 Permission is hereby granted, free of charge, to any person obtaining a copy
 of this software and associated documentation files (the "Software"), to deal
 in the Software without restriction, including without limitation the rights
 to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 copies of the Software, and to permit persons to whom the Software is
 furnished to do so, subject to the following conditions:

 The above copyright notice and this permission notice shall be included in
 all copies or substantial portions of the Software.

 THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 THE SOFTWARE.

 */


// ensure this library description is only included once
#pragma once

#include <functional>
#include <map>
#include <bitset>

class StreamOutput;

/*!
 * \class TMC22X
 * \brief Class representing a TMC22X stepper driver
 */
class TMC22X
{
public:
    /*!
     * \brief creates a new represenatation of a stepper motor connected to a TMC22X stepper driver
     *
     * This is the main constructor. If in doubt use this. You must provide all parameters as described below.
     *
     * \param uart send function
     *
     * By default the Constant Off Time chopper is used, see TCM262Stepper.setConstantOffTimeChopper() for details.
     * This should work on most motors (YMMV). You may want to configure and use the Spread Cycle Chopper, see  setSpreadCycleChopper().
     *
     * By default a microstepping of 1/32th is used to provide a smooth motor run, while still giving a good progression per step.
     * You can select a different stepping with setMicrosteps() to aa different value.
     * \sa start(), setMicrosteps()
     */
    TMC22X(std::function<int(uint8_t *b, int cnt, uint8_t *r)> uart, char designator);

    /*!
     * \brief configures the TMC22X stepper driver. Before you called this function the stepper driver is in nonfunctional mode.
     *
     * \param rms_current the maximum current to privide to the motor in mA (!). A value of 200 would send up to 200mA to the motor
     * \param resistor the current sense resistor in milli Ohm, defaults to ,15 Ohm ( or 150 milli Ohm) as in the TMC260 Arduino Shield

     * This routine configures the TMC22X stepper driver for the given values via SPI.
     * Most member functions are non functional if the driver has not been started.
     * Therefore it is best to call this in your Arduino setup() function.
     */
    void init(uint16_t cs);

    /*!
     * \brief Set the number of microsteps in 2^i values (rounded) up to 256
     *
     * This method set's the number of microsteps per step in 2^i interval.
     * This means you can select 1, 2, 4, 16, 32, 64, 128 or 256 as valid microsteps.
     * If you give any other value it will be rounded to the next smaller number (3 would give a microstepping of 2).
     * You can always check the current microstepping with getMicrosteps().
     */

private:

    // UART sender
    inline bool send2208(uint8_t reg, uint32_t datagram);
    std::function<int(uint8_t *b, int cnt, uint8_t *r)> uart;

    unsigned int resistor{50}; // current sense resitor value in milliohm

    //driver control register copies to easily set & modify the registers
    unsigned long driver_control_register_value;
    unsigned long chopper_config_register;
    unsigned long cool_step_register_value;
    unsigned long stall_guard2_current_register_value;
    unsigned long driver_configuration_register_value;
    //the driver status result
    unsigned long driver_status_result;

    //status values
    int microsteps; //the current number of micro steps

    std::bitset<8> error_reported;

    // only beeded for the tuning app report
    struct {
        int8_t blank_time:8;
        int8_t constant_off_time:5; //we need to remember this value in order to enable and disable the motor
        int8_t h_start:4;
        int8_t h_end:4;
        int8_t h_decrement:3;
        bool cool_step_enabled:1; //we need to remember this to configure the coolstep if it si enabled
        bool started:1; //if the stepper has been started yet
    };

    uint8_t cool_step_lower_threshold; // we need to remember the threshold to enable and disable the CoolStep feature
    char designator;

};

