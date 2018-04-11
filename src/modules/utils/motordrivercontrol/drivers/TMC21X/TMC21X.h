/*
 Modified from TMC26X.h

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
 * \class TMC21X
 * \brief Class representing a TMC21X stepper driver
 */
class TMC21X
{
public:
    /*!
     * \brief creates a new represenatation of a stepper motor connected to a TMC21X stepper driver
     *
     * This is the main constructor. If in doubt use this. You must provide all parameters as described below.
     *
     * \param spi send function
     *
     * By default the Constant Off Time chopper is used, see TCM262Stepper.setConstantOffTimeChopper() for details.
     * This should work on most motors (YMMV). You may want to configure and use the Spread Cycle Chopper, see  setSpreadCycleChopper().
     *
     * By default a microstepping of 1/32th is used to provide a smooth motor run, while still giving a good progression per step.
     * You can select a different stepping with setMicrosteps() to aa different value.
     * \sa start(), setMicrosteps()
     */
    TMC21X(std::function<int(uint8_t *b, int cnt, uint8_t *r)> spi, char designator);

    /*!
     * \brief configures the TMC21X stepper driver. Before you called this function the stepper driver is in nonfunctional mode.
     *
     * \param rms_current the maximum current to privide to the motor in mA (!). A value of 200 would send up to 200mA to the motor
     * \param resistor the current sense resistor in milli Ohm, defaults to ,15 Ohm ( or 150 milli Ohm) as in the TMC260 Arduino Shield

     * This routine configures the TMC26X stepper driver for the given values via SPI.
     * Most member functions are non functional if the driver has not been started.
     * Therefore it is best to call this in your Arduino setup() function.
     */
    void init(uint16_t cs);


private:
    // SPI sender
    inline void send2130(uint8_t reg, uint32_t datagram);
    //inline void send2130(unsigned long datagram);
    std::function<int(uint8_t *b, int cnt, uint8_t *r)> spi;

    unsigned int resistor{50}; // current sense resitor value in milliohm
    char designator;

};

