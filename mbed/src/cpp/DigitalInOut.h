/* mbed Microcontroller Library
 * Copyright (c) 2006-2013 ARM Limited
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#ifndef MBED_DIGITALINOUT_H
#define MBED_DIGITALINOUT_H

#include "platform.h"

#include "gpio_api.h"

namespace mbed {

/** A digital input/output, used for setting or reading a bi-directional pin
 */
class DigitalInOut {

public:
    /** Create a DigitalInOut connected to the specified pin
     *
     *  @param pin DigitalInOut pin to connect to
     */
    DigitalInOut(PinName pin) {
        gpio_init(&gpio, pin, PIN_INPUT);
    }

    /** Set the output, specified as 0 or 1 (int)
     *
     *  @param value An integer specifying the pin output value,
     *      0 for logical 0, 1 (or any other non-zero value) for logical 1
     */
    void write(int value) {
        gpio_write(&gpio, value);
    }

    /** Return the output setting, represented as 0 or 1 (int)
     *
     *  @returns
     *    an integer representing the output setting of the pin if it is an output,
     *    or read the input if set as an input
     */
    int read() {
        return gpio_read(&gpio);
    }

    /** Set as an output
     */
    void output() {
        gpio_dir(&gpio, PIN_OUTPUT);
    }

    /** Set as an input
     */
    void input() {
        gpio_dir(&gpio, PIN_INPUT);
    }

    /** Set the input pin mode
     *
     *  @param mode PullUp, PullDown, PullNone, OpenDrain
     */
    void mode(PinMode pull) {
        gpio_mode(&gpio, pull);
    }

#ifdef MBED_OPERATORS
    /** A shorthand for write()
     */
    DigitalInOut& operator= (int value) {
        write(value);
        return *this;
    }

    DigitalInOut& operator= (DigitalInOut& rhs) {
        write(rhs.read());
        return *this;
    }

    /** A shorthand for read()
     */
    operator int() {
        return read();
    }
#endif

protected:
    gpio_t gpio;
};

} // namespace mbed

#endif
