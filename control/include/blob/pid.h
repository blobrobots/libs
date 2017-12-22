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
 * \file       pid.h
 * \brief      interface for PID (Proportional Integral Derivative) controller
 * \author     adrian jimenez-gonzalez (blob.robots@gmail.com)
 * \copyright  the MIT License Copyright (c) 2017 Blob Robots.
 *
 ******************************************************************************/

#ifndef B_PID_H
#define B_PID_H

namespace blob {
/**
 * Implements PID controller with anti-windup based on clamping
 */
class PIDControl
{  
  public:
    
    PIDControl ();
    /**
     * Configures PID gains and antiwindup limits.
     * \param kp  proportional gain
     * \param ki  integral gain
     * \param kd  derivative gain
     * \param nd  derivative filter parameter
     * \param umax  maximum control action allowed (clamping anti-windup)
     * \param imax  maximum integral term allowed (if different to zero)
     * \return  true if successful, false otherwise
     */
    bool config(kp,ki,kd,nd,umax,imax);
    /**
     * Updates PID, including internal terms and control action.
     * \param error  difference between reference and measurement
     * \param dt  time lapse from last update
     * \param saturate  force saturation when true
     * \return  resulting control action
     */
    real_t update(real_t error, real_t dt, bool saturate);

  protected:
    real_t _kp;   /**< Proportional gain */
    real_t _kd;   //! Derivative gain
    real_t _ki;   //! Integral gain
    real_t _nd;   //! Derivative component filter constant
	  real_t _p;    //! Previous proportional term 
    real_t _i;    //! Previous integral term 
    real_t _d;    //! Previous derivative term  
    real_t _g;		//! Derivative filter gain 
    real_t _s;		//! Anti-windup integral term 
    real_t _umax; //! Control output saturation limit 
    real_t _imax; //! Integral saturation limit 
    real_t _u; //! Control action
}

#endif /* B_PID_H */

