/********* blob robotics 2014 *********
 *  title: ukf.h
 *  brief: interface for generic 
 *         estimation algorithm
 * author: adrian jimenez-gonzalez
 * e-mail: blob.robotics@gmail.com
 **************************************/
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

typedef void (*estimator_function_t)(real_t *state, void *args, real_t *result);

class Estimator
{
  public:
    
    Estimator (uint8_t n_states=0, real_t * init_state=NULL) {

      _n = n_states;
      if(init_state && _n)
        memcpy(_x, init_state, _n*sizeof(real_t));

        print();
    
    }

    virtual bool predict (estimator_function_t predictFunction, void *args, 
                          real_t *importance) {return false;};
    
    virtual bool update  (estimator_function_t updateFunction, void *args, 
                          uint8_t n_measurements, real_t *measurements, 
                          real_t *importance) {return false;};
    
    virtual void print   () {
#if defined(__DEBUG__)&defined(__linux__)
      std::cerr << "blob::Estimator interface (pure virtual)" << std::endl;
#endif
      return;
    }

    uint8_t  getNumStates () {return _n;}
    real_t * getState     () {return _x;}
    void     getState (real_t * state) {memcpy(state, _x, _n*sizeof(real_t));}
    real_t   getState (uint8_t index) {return _x[index];}

  protected: 
    real_t _n;       // number of states    
    real_t _x[BLOB_ESTIMATOR_MAX_STATE_LENGTH];
};
}

#endif // B_ESTIMATOR_H 
