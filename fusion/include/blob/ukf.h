/********* blob robotics 2014 *********
 *  title: ukf.h
 *  brief: interface for generic UKF
 * author: adrian jimenez-gonzalez
 * e-mail: blob.robotics@gmail.com
 **************************************/
#ifndef B_UKF_H
#define B_UKF_H

#include <blob/matrix.h>

#if !defined(BLOB_UKF_MAX_STATE_LENGTH)
 #define BLOB_UKF_MAX_STATE_LENGTH 20
#endif

#if !defined(BLOB_UKF_MAX_Z_LENGTH)
 #define BLOB_UKF_MAX_Z_LENGTH 10
#endif

#if (BLOB_UKF_MAX_STATE_LENGTH > BLOB_UKF_MAX_Z_LENGTH)
 #define BLOB_UKF_MAX_LENGTH BLOB_UKF_MAX_STATE_LENGTH
#else
 #define BLOB_UKF_MAX_LENGTH BLOB_UKF_MAX_Z_LENGTH 
#endif

namespace blob {

typedef void (*ukf_function)(real_t *x, void *args, real_t * res);

class UKF
{
  public:
    UKF (uint8_t states=0, real_t * x_init=NULL);
   
    bool predict (ukf_function function, void *args, real_t *r);
    bool update  (ukf_function function, void *args, const uint8_t m, real_t *z, real_t *q);
    void print   ();

    uint8_t getNumStates () {return _n;}
    real_t * getState     () {return _x;}
    void    getState (real_t * state) {memcpy(state, _x, _n*sizeof(real_t));}
    real_t   getState (uint8_t index) {return _x[index];}

  private:

    bool ut (ukf_function function, void *args, Matrix & X, const Matrix & R, Matrix & x_u, Matrix & P_u, Matrix & X_u, Matrix & Xs_u);
    
    bool sigmas  (Matrix & x, Matrix &P, Matrix & X);

    real_t _n;       // number of states
    real_t _alpha;   // tunable
    real_t _ki;      // tunable
    real_t _beta;    // tunable
    real_t _lambda;  // factor
    real_t _c;       // scaling factor
    real_t _wm[2*BLOB_UKF_MAX_STATE_LENGTH+1]; // weights for means
    real_t _wc[2*BLOB_UKF_MAX_STATE_LENGTH+1]; // weights for covariance

    bool _updated;

    real_t _x[BLOB_UKF_MAX_STATE_LENGTH];
    real_t _P[BLOB_UKF_MAX_STATE_LENGTH*BLOB_UKF_MAX_STATE_LENGTH];

    real_t _X [((2*BLOB_UKF_MAX_STATE_LENGTH+1)*BLOB_UKF_MAX_STATE_LENGTH)]; 
    real_t _Xs[((2*BLOB_UKF_MAX_STATE_LENGTH+1)*BLOB_UKF_MAX_STATE_LENGTH)];
};
}

#endif // B_UKF_H 