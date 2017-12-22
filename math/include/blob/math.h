/******************************************************************************
 * The MIT License (MIT)
 *
 * Copyright (c) 2017 Blob Robotics
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
 * \file       math.h
 * \brief      blob libraries specific math operations and constants
 * \author     adrian jimenez-gonzalez (blob.robots@gmail.com)
 * \copyright  the MIT License Copyright (c) 2017 Blob Robots.
 *
 ******************************************************************************/

#ifndef B_MATH_H
#define B_MATH_H

#include <blob/types.h>

#if defined(__AVR_ATmega32U4__)
  #include "Arduino.h"
#endif // defined(__AVR_ATmega32U4__)

#if defined(__linux__)
  #include <math.h>
  #include <algorithm>
#endif //defined(__linux__)

namespace blob {

const real_t pi (3.14159265359); /**< Pi constant */

class math
{

  public:
    /**
     * Rotate vector
     * \return true if real_t is equivalent to double; false if single float 
     */
    static bool using_double() {return (sizeof(real_t)==sizeof(double));} 
    /**
     * Absolute value of real number
     * \param a  real number to provide the absolute value from
     * \return absolute value of input real number
     */ 
    static real_t rabs (const real_t & x) 
    {
#if defined(__linux__)
      return using_double()?fabsf(x):fabs(x);
#elif defined(__AVR_ATmega32U4__)
      return fabs(x);
#endif //if defined(__linux__)
    }
    /**
     * Square root of real number
     * \param x  real number to calculate the square root of
     * \return square root of input real number
     */ 
    static real_t sqrtr (const real_t & x) 
    {
#if defined(__linux__)
      return using_double()?sqrt(x):sqrtf(x); 
#elif defined(__AVR_ATmega32U4__)
      return sqrt(x);
#endif //if defined(__linux__)
    }
    /**
     * Cosine of real number
     * \param x  real number to calculate the cosine of
     * \return cosine of input real number
     */ 
    static real_t cos (const real_t & x) {
      return using_double()? cos(x) : cosf(x);
    }
    /**
     * Sine of real number
     * \param x  real number to calculate the Sine of
     * \return sine of input real number
     */
    static real_t sin (const real_t & x) {
      return using_double()? sin(x) : sinf(x);
    }
    /**
     * Tangent of real number
     * \param x  real number to calculate the tangent of
     * \return tangent of input real number
     */
    static real_t tan (const real_t & x) {
      return using_double()? tan(x) : tanf(x);
    }
    /**
     * Arcsine of real number
     * \param x  real number to calculate the arcsine of
     * \return arcsine of input real number
     */
    static real_t asin (const real_t & x) {
      return using_double()? asin(x) : asinf(x);
    }
    /**
     * Arccosine of real number
     * \param x  real number to calculate the arccosine of
     * \return arccosine of input real number
     */
    static real_t acos (const real_t & x) {
      return using_double()? acos(x) : acosf(x);
    }
    /**
     * Arctangent of real number
     * \param x  real number to calculate the arctangent of
     * \return arctangent of input real number
     */
    static real_t atan (const real_t & x) {
      return using_double()? atan(x) : atanf(x);
    }
    /**
     * Two argument arctangent of real number
     * \param y  first argument (y-axis)
     * \param x  second argument (x-axis)
     * \return two argument arctangent of input real number
     */
    static real_t atan2 (const real_t & y, const real_t& x) {
      return using_double()? atan2(y, x) : atan2f(y, x);
    }
    /**
     * Sign of a number
     * \param x  number to calculate the sign of
     * \return 1 or -1 depending if the number is positive or negative
     */
    template <class T> static T sign (const T & x) {return (x<0)? -1:1;}
    /**
     * Minimum of two numbers
     * \param a  first number to compare
     * \param b  second number to compare
     * \return the minimum of the two numbers
     */
    template <class T> static const T & minimum (const T & a, const T & b) {
#if defined(__AVR_ATmega32U4__)
      return min(a,b);
#endif // defined(__AVR_ATmega32U4__)
#if defined(__linux__)
      return std::min(a,b);
#endif //defined(__linux__)
    }
    /**
     * Maximum of two numbers
     * \param a  first number to compare
     * \param b  second number to compare
     * \return the maximum of the two numbers
     */
    template <class T> static const T & maximum (const T & a, const T & b) {
#if defined(__AVR_ATmega32U4__)
      return max(a,b);
#endif // defined(__AVR_ATmega32U4__)
#if defined(__linux__)
      return std::max(a,b);
#endif //defined(__linux__)
    }
    /**
     * Value constrained within an upper and a lower limit
     * \param min  lower limit
     * \param max  upper limit
     * \return the minimum of the two numbers
     */
    template <class T> static const T & constrained (const T & x, const T & min,
                                                                  const T & max) 
    {
#if defined(__AVR_ATmega32U4__)
      return constrain(x, min, max);
#endif // defined(__AVR_ATmega32U4__)
#if defined(__linux__)
      if (x < min) return min; if (x > max) return max; return x;
#endif //defined(__linux__)
    }
};

}
#endif // B_MATH_H
