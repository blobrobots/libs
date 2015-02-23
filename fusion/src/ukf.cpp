/********* blob robotics 2014 *********
 *  title: ukf.cpp
 *  brief: implemention of generic UKF
 * author: adrian jimenez-gonzalez
 * e-mail: blob.robotics@gmail.com
 **************************************/
#include "blob/ukf.h"

blob::UKF::UKF (uint8_t states, float *x, float *P, float *X, float *Xs) :  _x(states,1,x), 
                                                                            _P(states,states,P), 
                                                                            _X(states,2*states+1,X), 
                                                                            _Xs(2*states+1,2*states+1,Xs)
{

}

void blob::UKF::sigmas  ()
{
  L = numel(x);
  A = c*chol(P)';
  Y = x(:,ones(1,L));
  X = [x Y+A Y-A];
}