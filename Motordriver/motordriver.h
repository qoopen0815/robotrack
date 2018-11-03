/*motor driver libary modified from the following libary,
*  
* mbed simple H-bridge motor controller
* Copyright (c) 2007-2010, sford
* 
* by Christopher Hasler.
* 
* from sford's libary,
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
 
#ifndef MBED_MOTOR_H
#define MBED_MOTOR_H
 
#include "mbed.h"
 
/** Interface to control a standard DC motor 
* with an H-bridge using a PwmOut and 2 DigitalOuts
*/
class Motor {
    public:
 
/** Create a motor control interface    
*
* @param pwm A PwmOut pin, driving the H-bridge enable line to control the speed
* @param fwd A DigitalOut, set high when the motor should go forward
* @param rev A DigitalOut, set high when the motor should go backwards
* @param set if the motor driver is able to do braking 0 false 1 true.
*/
        Motor(PinName pwm, PinName fwd, PinName rev, int brakeable);
  
/** Set the speed of the motor 
* 
* @param speed The speed of the motor as a normalised value between -1.0 and 1.0.
* @return the applied speed to the motor after checking to ensure motor doesn't switch from forward to reverse without stopping.
*/
        float speed(float speed);
        
/** Set the the motor to coast
* 
* @param void 
* @return motor coasts until another instruction is recived.
*/        
  
        void coast(void);

/** Set the motor to dynamicaly brake
* 
* @param float 0 - 1.0 provides some control over how hard the motor brakes. 
* @return duty applied to motor driver. -1 is error, motor driver can't brake.
*/

        float stop(float duty);
/** return the current state of the motor
*
* @param void
* @return state of motor, -1 to 1 is speed, -2 is braking, 2 is coasting. -3 is error. 
*/
        float state(void);
        
    protected:
        PwmOut _pwm;
        DigitalOut _fwd;
        DigitalOut _rev;
        int Brakeable; // cna the motor driver break
        int sign; //prevents throwing the motor from full foward to full reverse and stuff melting.
 
};





#endif
