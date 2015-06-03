/********* blob robotics 2014 *********
 *  title: ukf.h
 *  brief: interface for generic UKF
 * author: adrian jimenez-gonzalez
 * e-mail: blob.robotics@gmail.com
 **************************************/
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

class UKF : public Estimator
{
  public:

    UKF (uint8_t n=0, real_t * init_x=NULL);

    virtual bool predict (estimator_function_t function, void *args, real_t *r);
    virtual bool update  (estimator_function_t function, void *args, 
                          uint8_t m, real_t *z, real_t *q);
    virtual void print   ();

  protected:

    bool ut (estimator_function_t function, void *args, Matrix & X, Matrix & R, 
             Matrix & u, Matrix & Pu, Matrix & U, Matrix & Us);
    
    bool sigmas  (Matrix & x, Matrix &P, Matrix & X);

    real_t _alpha;   // tunable
    real_t _ki;      // tunable
    real_t _beta;    // tunable
    real_t _lambda;  // factor
    real_t _c;       // scaling factor
    real_t _wm[2*BLOB_UKF_MAX_N+1]; // weights for means
    real_t _wc[2*BLOB_UKF_MAX_N+1]; // weights for covariance

    bool _updated;

    real_t _P[BLOB_UKF_MAX_N*BLOB_UKF_MAX_N]; // covariance matrix

    real_t _X [((2*BLOB_UKF_MAX_N+1)*BLOB_UKF_MAX_N)]; // unscented transformation of state
    real_t _Xs[((2*BLOB_UKF_MAX_N+1)*BLOB_UKF_MAX_N)]; // unscented transformation of std. dev.
};
}

#endif // B_UKF_H 
