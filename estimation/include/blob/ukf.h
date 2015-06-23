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
 * \file       ukf.h
 * \brief      interface for generic UKF
 * \author     adrian jimenez-gonzalez (blob.robotics@gmail.com)
 * \copyright  the MIT License Copyright (c) 2015 Blob Robotics.
 *
 ******************************************************************************/

#ifndef B_UKF_H
#define B_UKF_H

#include <blob/estimator.h>
#include <blob/matrix.h>

#if !defined(BLOB_UKF_MAX_N)
 #define BLOB_UKF_MAX_N BLOB_ESTIMATOR_MAX_STATE_LENGTH
#endif

#if !defined(BLOB_UKF_MAX_M)
 #define BLOB_UKF_MAX_M BLOB_ESTIMATOR_MAX_MEASUREMENT_LENGTH
#endif

#if (BLOB_UKF_MAX_N > BLOB_UKF_MAX_M)
 #define BLOB_UKF_MAX_LENGTH BLOB_UKF_MAX_N
#else
 #define BLOB_UKF_MAX_LENGTH BLOB_UKF_MAX_M 
#endif

namespace blob {

/**
 * Implements generic Unscented Kalman Filter.
 */

class UKF : public Estimator
{
  public:
    /**
     * Initializes number of states, and xstate vector.
     */
    UKF (uint8_t n=0, real_t* init_x=NULL);

    /**
     * Applies f function to provide a model based prediction of system state.
     * \param function pointer to f function to be applied during prediction
     * \param args     pointer to f function argument structure
     * \param r        state model noise covariance matrix
     * \sa sigmas(), ut(), update()
     */
    virtual bool predict (estimator_function_t function, void* args, real_t* r);

    /**
     * Applies h function to update system state with sensor measurement.
     * \param function pointer to h function to be applied during sensor update
     * \param args     pointer to h function argument structure
     * \param m   sensor measurement vector length
     * \param z   sensor measurement vector
     * \param q   sensor measurement noise covariance matrix
     * \sa sigmas(), ut(), predict()
     */
    virtual bool update  (estimator_function_t function, void* args, 
                          uint8_t m, real_t* z, real_t* q);
    /**
     * Outputs internal and state information from filter to standard output.
     */
    virtual void print   ();

  protected:

    /**
     * Calculates sigma points from state vector and covariance matrix. 
     * \param x   state vector
     * \param P   state covariance matrix
     * \param X   state sigma points
     */
    bool sigmas  (Matrix& x, Matrix& P, Matrix& X);

    /**
     * Performs unscented transformation applying function and covariance to 
     * sigma points.
     * \param function pointer to function to be applied during transform
     * \param args     pointer to function argument structure
     * \param X   state sigma points
     * \param R   process covariance matrix
     * \param u   transformed state vector
     * \param Pu  transformed covariance matrix
     * \param U   transformed sigma points
     * \param Us  transformed std. dev. sigma points
     * \sa sigmas()
     */
    bool ut (estimator_function_t function, void* args, Matrix& X, Matrix& R, 
             Matrix& u, Matrix& Pu, Matrix& U, Matrix& Us);
    
    real_t _alpha;                  /**< alpha tunable parameter. */
    real_t _ki;                     /**< ki tunable parameter.    */
    real_t _beta;                   /**< beta tunable parameter.  */
    real_t _lambda;                 /**< lambda factor.           */
    real_t _c;                      /**< c scaling factor.        */
    real_t _wm[2*BLOB_UKF_MAX_N+1]; /**< weights for means.       */
    real_t _wc[2*BLOB_UKF_MAX_N+1]; /**< weights for covariance.  */ 

    bool _updated; /**< indicates if state has already been updated with a 
                        sensor measurement. */

    real_t _P[BLOB_UKF_MAX_N*BLOB_UKF_MAX_N]; /**< covariance matrix. */

    real_t _X [((2*BLOB_UKF_MAX_N+1)*BLOB_UKF_MAX_N)]; /**< state unscented 
                                                            transformation. */
    real_t _Xs[((2*BLOB_UKF_MAX_N+1)*BLOB_UKF_MAX_N)]; /**< std. dev. unscented 
                                                            transformation.  */ 
};

}

#endif // B_UKF_H 
