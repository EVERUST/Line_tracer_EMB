/* mbed SRF05 Ultrasonic Rangefiner Library
 * Copyright (c) 2007-2010, cstyles, sford
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#ifndef MBED_SRF05_H
#define MBED_SRF05_H

#include "mbed.h"

/** Library for the SRF05 Ultrasonic range finder
 *
 * Example:
 * @code
 * // Print measured distance
 *
 * #include "mbed.h"
 * #include "SRF05.h"
 *
 * SRF05 srf(p9,p10);
 *
 * int main() {
 *     while(1) {
 *         printf("Measured : %.1f\n", srf.read());
 *         wait(0.2);
 *     }
 * }
 * @endcode
 */
class SRF05 {
public:

    /** Create a SRF05 object, connected to the specified pins
     *
     * @param trigger DigitalOut to the SRF05 trigger
     * @param echo InterruptIn to measure the return pulse
     */
    SRF05(PinName trigger, PinName echo);
    
    /** A non-blocking function that will return the last measurement
     *
     * @returns floating point representation of distance in cm
     */
    float read();

    /** A short hand way of using the read function */
    operator float();
    
private :
    DigitalOut _trigger;
    InterruptIn _echo;
    Timer _timer;
    Ticker _ticker;
    void _rising (void);
    void _falling (void);
    void _startRange (void);
    float _dist;
};

#endif
