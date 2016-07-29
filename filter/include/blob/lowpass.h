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
 * \file       lowpass.h
 * \brief      exponential lowpass filter interface
 * \author     adrian jimenez-gonzalez (blob.robots@gmail.com)
 * \copyright  the MIT License Copyright (c) 2015 Blob Robots.
 *
 ******************************************************************************/

#ifndef B_LOWPASS_H
#define B_LOWPASS_H

#include <blob/types.h>

namespace blob {

/**
 * Exponential lowpass filter.
 */
class LowPassF : public Filter
{
  public:
    /**
     * Initializes filter factor.
     * \param factor  filtering factor [0,1]
     */
    LowPassF(const real_t& factor) {_factor = factor;}
    /**
     * Adds a sample to filter and returns current filter output.
     * \param sample  new signal sample
     * \param dt      time lapse in seconds (optional)
     */
    virtual real_t update (const real_t& sample, const real_t& dt=0) {
      return (_output = _output*_factor+(1-_factor)*sample); 
    }
    
  protected:
    real_t _factor; /**< filtering factor */
};
}

#endif // B_ESTIMATOR_H 
