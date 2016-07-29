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
 * \file       rate_limiter.h
 * \brief      generic filte interface
 * \author     adrian jimenez-gonzalez (blob.robots@gmail.com)
 * \copyright  the MIT License Copyright (c) 2015 Blob Robots.
 *
 ******************************************************************************/

#ifndef B_RATE_LIMITER_H
#define B_RATE_LIMITER_H

#include <blob/types.h>

namespace blob {

/**
 * Rate limiter filter.
 */
class RateLimiter : public Filter
{
  public:

    RateLimiter(const real_t& rate) {_rate=rate;}
    /**
     * Adds a sample to filter and returns current filter output.
     * \param sample  new signal sample
     * \param dt      time lapse in seconds
     */
    virtual real_t update (const real_t& sample, const real_t& dt=0)
    {
      if(dt!=0)
        _output += blob::math::constrained(sample-_output, -_rate*dt, _rate*dt);
      else
        _output += blob::math::constrained(sample-_output, -_rate, _rate);
      return _output;
    }
    
  protected: 
    real_t _rate; /**< rate limit */
};
}

#endif // B_ESTIMATOR_H 
