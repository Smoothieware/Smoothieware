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
#ifndef MBED_DIGITALIN_H
#define MBED_DIGITALIN_H

#include "platform.h"

#include "gpio_api.h"

namespace mbed {

/** A digital input, used for reading the state of a pin
 *
 * Example:
 * @code
 * // Flash an LED while a DigitalIn is true
 *
 * #include "mbed.h"
 *
 * DigitalIn enable(p5);
 * DigitalOut led(LED1);
 *
 * int main() {
 *     while(1) {
 *         if(enable) {
 *             led = !led;
 *         }
 *         wait(0.25);
 *     }
 * }
 * @endcode
 */
class DigitalIn {

public:
    /** Create a DigitalIn connected to the specified pin
     *
     *  @param pin DigitalIn pin to connect to
     *  @param name (optional) A string to identify the object
     */
    DigitalIn(PinName pin) {
        gpio_init(&gpio, pin, PIN_INPUT);
    }

    /** Read the input, represented as 0 or 1 (int)
     *
     *  @returns
     *    An integer representing the state of the input pin,
     *    0 for logical 0, 1 for logical 1
     */
    int read() {
        return gpio_read(&gpio);
    }

    /** Set the input pin mode
     *
     *  @param mode PullUp, PullDown, PullNone, OpenDrain
     */
    void mode(PinMode pull) {
        gpio_mode(&gpio, pull);
    }

#ifdef MBED_OPERATORS
    /** An operator shorthand for read()
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
