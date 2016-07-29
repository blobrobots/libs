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
 * \file       cf.h
 * \brief      interface for generic complimentary filter
 * \author     adrian jimenez-gonzalez (blob.robots@gmail.com)
 * \copyright  the MIT License Copyright (c) 2015 Blob Robots.
 *
 ******************************************************************************/

#ifndef B_CF_H
#define B_CF_H

#include <blob/estimator.h>
#include <blob/matrix.h>

#if !defined(BLOB_CF_MAX_N)
 #define BLOB_CF_MAX_N BLOB_ESTIMATOR_MAX_STATE_LENGTH
#endif

#if !defined(BLOB_CF_MAX_M)
 #define BLOB_CF_MAX_M BLOB_ESTIMATOR_MAX_MEASUREMENT_LENGTH
#endif

#if (BLOB_CF_MAX_N > BLOB_CF_MAX_M)
 #define BLOB_CF_MAX_LENGTH BLOB_CF_MAX_N
#else
 #define BLOB_CF_MAX_LENGTH BLOB_CF_MAX_M 
#endif

namespace blob {

/**
 * Implements generic Unscented Kalman Filter.
 */
class CF : public Estimator
{
  public:
    /**
     * Initializes filter parameters and state vector.
     * \param n      number of states
     * \param init_x state vector inital value
     */
    CF (uint8_t n=0, real_t* init_x=NULL);

    /**
     * Applies f function to provide an error corrected prediction of system state.
     * \param function pointer to f function to be applied during prediction
     * \param dt       time lapse
     * \param l        control input vector length
     * \param u        control input vector
     * \param ki       integral error correction gain vector
     * \return         true if successful, false otherwise
     * \sa sigmas(), ut(), update()
     */
    virtual bool predict (estimator_function_t function, const real_t& dt, 
                          const uint8_t& l, real_t* u, real_t* ki);

    /**
     * Applies h function to update system error with sensor measurement.
     * \param function pointer to function to calculate error on sensor update
     * \param dt       time lapse
     * \param m        sensor measurement vector length
     * \param z        sensor measurement vector
     * \param kp       expected measurement error correction gain vector
     * \return         true if successful, false otherwise
     * \sa sigmas(), ut(), predict()
     */
    virtual bool update  (estimator_function_t function, const real_t& dt, 
                          const uint8_t& m, real_t* z, real_t* kp);
    /**
     * Outputs internal and state information from filter to standard output.
     */
    virtual void print   ();

  protected:
    uint8_t _l;                         /**< error vector length */
    real_t _error[BLOB_CF_MAX_LENGTH]; /**< error vector */
};

}

#endif // B_CF_H 
