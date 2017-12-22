/******************************************************************************
 * The MIT License (MIT)
 *
 * Copyright (c) 2015 Blob Robotics
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
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE 
 * SOFTWARE.
 * 
 * \file       pid.cpp
 * \brief      converter from holonomic ideal motion to non-holonomic motion
 * \author     adrian jimenez-gonzalez (blob.robots@gmail.com)
 * \copyright  the MIT License Copyright (c) 2017 Blob Robots.
 *
 ******************************************************************************/

#include "blob/math.h"

blob::PIDcontrol::update(real_t error, real_t dt, bool saturate) {

  if(_imax==0) {
    /* proportional term */
    _p = _kp*error;
    
    /* integral term */
    _i = _i + _s*dt;

    /* derivative+filter term */
    _g = _g + _d*dt;
    _d = _nd*(_kd*error - _g);

    /* PID output before saturation */
    _u = _p + _i + _d;

     /* anti wind-up (clamping)*/
    if(((_p+_i)*_ki*error) > 0 && (_u > _umax || _u < -_umax || saturate == true)) {

      _s = 0;

    } else {

      _s = _ki*error;
    }

    /* return saturated PID output */
    _u = blob::math::constrained(_u,-_umax, _umax);		

    return _u;
  } else { // i saturation instead of entire u saturation

    if((_p + _i + _ki*error*dt)*(_ki*error) < 0 || saturate == false)
    {
      _p = blob::math::constrained(_kp*error,-_umax, _umax);
      _i = blob::math::constrained(_i+_ki*error*dt,-_imax, _imax);
    }
    /* derivative+filter term */
    _g = _g + _d*dt;
    _d = _nd*(_kd*error - _g);

    _u = _p +_i +_d;
  }
  
  return _u;
}


