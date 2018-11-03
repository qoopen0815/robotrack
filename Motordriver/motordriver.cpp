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

#include "motordriver.h"

#include "mbed.h"

Motor::Motor(PinName pwm, PinName fwd, PinName rev, int brakeable):
        _pwm(pwm), _fwd(fwd), _rev(rev) {

    // Set initial condition of PWM
    _pwm.period(0.001);
    _pwm = 0;

    // Initial condition of output enables
    _fwd = 0;
    _rev = 0;

    //set if the motor dirver is capable of braking. (addition)
    Brakeable= brakeable;
    sign = 0;//i.e nothing.
}

float Motor::speed(float speed) {
    float temp = 0;
    if (sign == 0) {
        _fwd = (speed > 0.0);
        _rev = (speed < 0.0);
        temp = abs(speed);
        _pwm = temp;
    } else if (sign == 1) {
        if (speed < 0) {
            _fwd = (speed > 0.0);
            _rev = (speed < 0.0);
            _pwm = 0;
            temp = 0;
       } else {
            _fwd = (speed > 0.0);
            _rev = (speed < 0.0);
            temp = abs(speed);
            _pwm = temp;
        }
    } else if (sign == -1) {
        if (speed > 0) {
            _fwd = (speed > 0.0);
            _rev = (speed < 0.0);
            _pwm = 0;
            temp = 0;
        } else {
            _fwd = (speed > 0.0);
            _rev = (speed < 0.0);
            temp = abs(speed);
            _pwm = temp;
        }
    }
    if (speed > 0)
        sign = 1;
    else if (speed < 0) {
        sign = -1;
    } else if (speed == 0) {
        sign = 0;
    }
    return temp;
}
//  (additions)
void Motor::coast(void) {
    _fwd = 0;
    _rev = 0;
    _pwm = 0;
    sign = 0;
}

float Motor::stop(float duty) {
    if (Brakeable == 1) {
        _fwd = 1;
        _rev = 1;
        _pwm = duty;
        sign = 0;
        return duty;
    } else
        Motor::coast();
        return -1;
}

float Motor::state(void) {
    if ((_fwd == _rev) && (_pwm > 0)) {
        return -2;//braking
    } else if (_pwm == 0) {
        return 2;//coasting
    } else if ((_fwd == 0) && (_rev == 1)) {
        return -(_pwm);//reversing
    }  else if ((_fwd == 1) && (_rev == 0)) {
        return _pwm;//fowards
    } else
        return -3;//error
}

/*
 test code, this demonstrates working motor drivers.

Motor A(p22, p6, p5, 1); // pwm, fwd, rev, can break
Motor B(p21, p7, p8, 1); // pwm, fwd, rev, can break
int main() {
    for (float s=-1.0; s < 1.0 ; s += 0.01) {
       A.speed(s);
       B.speed(s);
       wait(0.02);
    }
    A.stop();
    B.stop();
    wait(1);
    A.coast();
    B.coast();
}
*/