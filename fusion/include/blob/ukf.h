/********* blob robotics 2014 *********
 *  title: ukf.h
 *  brief: interface for generic UKF
 * author: adrian jimenez-gonzalez
 * e-mail: blob.robotics@gmail.com
 **************************************/
#ifndef B_UKF_H
#define B_UKF_H

#include <blob/types.h>

// auxiliary functions
#define ukf_length(x)   sizeof(x)/sizeof(x[0])
// variable initialization functions
#define ukf_x_alloc(l)  float x[l]
#define ukf_X_alloc(l)  float X[((2*l+1)*l)]
#define ukf_P_alloc(l)  float P[(l*l)]
#define ukf_Xs_alloc(l) float Xs[((2*l+1)*l)]

#define ukf_alloc(l)   float x[l], X[((2*l+1)*l)], P[(l*l)], Xs[((2*l+1)*l)], wm[(2*l+1)],wc[(2*l+1)],aux[(l*l)]

namespace blob {

typedef void (*ukf_f)(float *x, float *u, const float dt);
typedef void (*ukf_h)(float *x, float *z);

class UKF
{
  public:
    UKF (uint8_t states=0, float *x=NULL, float *P=NULL, float *X=NULL, float *Xs=NULL, float *aux=NULL);
   
    void predict ();
    void update  ();
    void ut      ();
    void sigmas  ();

    uint8_t getNumStates ();
    float *getState ();
    void getState (float * state);
    void getState (uint8_t index);

  private:

    Matrix _x, _P, _X, _Xs, _wm, _wc, _Aux; // internal variables
    float _n;                     // number of states
    float _alpha;                 // tunable
    float _ki;                    // tunable
    float _beta;                  // tunable
    float _lambda;                // factor
    float _c;                     // scaling factor
    Matrix _Wm;                   // weights for means
    Matrix _Wc;                   // weights for covariance
};
}

#endif // B_UKF_H 