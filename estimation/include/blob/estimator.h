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
 * \file       estimator.h
 * \brief      interface for generic estimation algorithm
 * \author     adrian jimenez-gonzalez (blob.robots@gmail.com)
 * \copyright  the MIT License Copyright (c) 2015 Blob Robots.
 *
 ******************************************************************************/

#ifndef B_ESTIMATOR_H
#define B_ESTIMATOR_H

#include <blob/types.h>

#if defined(__linux__)
  #include <iostream>
  #include <cstring>
#endif

#if !defined(BLOB_ESTIMATOR_MAX_STATE_LENGTH)
 #define BLOB_ESTIMATOR_MAX_STATE_LENGTH 20
#endif

#if !defined(BLOB_ESTIMATOR_MAX_MEASUREMENT_LENGTH)
 #define BLOB_ESTIMATOR_MAX_MEASUREMENT_LENGTH 10
#endif

#if (BLOB_ESTIMATOR_MAX_STATE_LENGTH > BLOB_ESTIMATOR_MAX_MEASUREMENT_LENGTH)
 #define BLOB_ESTIMATOR_MAX_LENGTH BLOB_ESTIMATOR_MAX_STATE_LENGTH
#else
 #define BLOB_ESTIMATOR_MAX_LENGTH BLOB_ESTIMATOR_MAX_MEASUREMENT_LENGTH 
#endif

namespace blob {
/**
 * Defines function to predict and update estimations 
 */
typedef void (*estimator_function_t)(real_t *state, void *args, real_t *result);

/**
 * Implements generic estimation algorithm.
 */
class Estimator
{
  public:
    /**
     * Initializes filter parameters and state vector.
     * \param n      number of states
     * \param init_x state vector inital value
     */
    Estimator (uint8_t n_states=0, real_t * init_state=NULL)
    {
      _n = n_states;
      if(init_state && _n)
        memcpy(_x, init_state, _n*sizeof(real_t));    
    }

    /**
     * Applies function to provide a model based estimation of system state.
     * \param predictFunction  pointer to function to predict state
     * \param args             pointer to prediction function argument structure
     * \param importance       state model importance factor
     * \sa update()
     */
    virtual bool predict (estimator_function_t predictFunction, void *args, 
                          real_t *importance) {return false;};
    
    /**
     * Applies function to update system state with sensor measurement.
     * \param updateFunction pointer to function to update state with sensors
     * \param args           pointer to update function argument structure
     * \param n_measurements sensor measurement vector length
     * \param measurements   sensor measurement vector
     * \param importance     sensor importance factor
     * \return               true if successful, false otherwise
     * \sa predict()
     */
    virtual bool update  (estimator_function_t updateFunction, void *args, 
                          uint8_t n_measurements, real_t *measurements, 
                          real_t *importance) {return false;};
    /**
     * Outputs internal and state information from algorithm to standard output.
     */    
    virtual void print   () {
#if defined(__DEBUG__)&defined(__linux__)
      std::cerr << "blob::Estimator interface (pure virtual)" << std::endl;
#endif
      return;
    }

    /**
     * Provides number of states.
     * \return state vector length
     * \sa getState()
     */
    uint8_t  getNumStates () {return _n;}
    /**
     * Provides pointer to state vector.
     * \return pointer to state vector
     * \sa getNumStates()
     */
    real_t * getState     () {return _x;}
    /**
     * Provides copy of state vector.
     * \param pointer to destination vector
     * \sa getNumStates()
     */
    void     getState (real_t * state) {memcpy(state, _x, _n*sizeof(real_t));}
    /**
     * Provides indexed element of state vector.
     * \param i state element index to retrieve
     * \return  state vector element in ith position
     * \sa getNumStates()
     */
    real_t   getState (uint8_t i) {return _x[i];}

  protected: 
    real_t _n;                                  /**< state vector length */
    real_t _x[BLOB_ESTIMATOR_MAX_STATE_LENGTH]; /**< state vector */
};
}

#endif // B_ESTIMATOR_H 
