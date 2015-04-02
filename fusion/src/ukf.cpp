/********* blob robotics 2014 *********
 *  title: ukf.cpp
 *  brief: implemention of generic UKF
 * author: adrian jimenez-gonzalez
 * e-mail: blob.robotics@gmail.com
 **************************************/
#include "blob/ukf.h"

#if defined(__linux__)
  #include <math.h>
  #include <iostream>
#endif

blob::UKF::UKF (uint8_t states, real_t *init_x)
{
  _n = states;
  
  memcpy(_x, init_x, _n*sizeof(real_t));
  blob::Matrix P(_n, _n, _P);
  P.eye();
  memset(_X, 0, sizeof(_X));
  memset(_Xs, 0, sizeof(_Xs));
  
  _alpha = 0.001;                          // tunable
  _ki = 0;                                // tunable
  _beta = 2;                              // tunable
  _lambda = _alpha*_alpha*(_n + _ki) - _n;  // factor
  _c = _n + _lambda;                        // factor
  _wc[0] = _wm[0] = _lambda/_c; 
  
  for(int i = 1; i<2*_n+1; i++)
    _wc[i] = _wm[i] = 0.5/_c;                 // weights for means
  
  _wc[0] = _wc[0]+(1-_alpha*_alpha+_beta); // weights for covariance
  _c = sqrtf(_c);

  _updated = false;

}

bool blob::UKF::sigmas (blob::Matrix &x, blob::Matrix &P, blob::Matrix &X)
{
  bool retval = true;
  real_t aux[BLOB_UKF_MAX_STATE_LENGTH*BLOB_UKF_MAX_STATE_LENGTH];
  blob::Matrix Aux(_n,_n, aux);

  retval &= Aux.copy(P);
  retval &= Aux.cholesky();
  retval &= Aux.scale(_c);
  
  for(int i=0; i<X.ncols(); i++)
    retval &= X.copy(x,0,i);
  
  retval &= X.add(Aux,0,1);
  retval &= X.substract(Aux,0,_n+1);

#if defined(__DEBUG__) & defined(__linux__)
  if(retval == false)
    std::cerr << "UKF::sigmas() error" << std::endl;
#endif

  return retval;
}

bool blob::UKF::ut (ukf_function function, void *args, blob::Matrix & X, const blob::Matrix & R, 
                    blob::Matrix & x_u, blob::Matrix & P_u, blob::Matrix & X_u, blob::Matrix & Xs_u)
{
  bool retval = true;
  real_t aux [(2*BLOB_UKF_MAX_STATE_LENGTH+1)*BLOB_UKF_MAX_LENGTH];

  int l = x_u.nrows();

  blob::Matrix x (_n, 1, aux);
  blob::Matrix y (l, 1, &(aux[(int)_n]));

  blob::Matrix wc(2*_n+1,1,_wc);
  
  x_u.zero();
  
  for(int k=0; k<2*_n+1; k++)
  {
    for(int i=0; i<_n; i++)
      x[i] = X(i,k);

    function(x.data(), args, y.data());

    for(int i=0; i<l; i++)
      X_u(i,k) = y[i];

    retval &= y.scale(_wm[k]);

    retval &= x_u.add(y);
    
  }

  retval &= Xs_u.copy(X_u);
  for(int i=0; i<X_u.ncols();i++)
    retval &= Xs_u.substract(x_u, 0, i);
  
  blob::Matrix Aux(l, 2*_n+1, aux);

  retval &= blob::Matrix::multiplyDiag(Xs_u, wc, Aux);
  
  retval &= Xs_u.transpose();
  
  retval &= blob::Matrix::multiply(Aux,Xs_u,P_u);
  retval &= P_u.add(R);

  retval &= Xs_u.transpose(); // avoid with a copy
  
#if defined(__DEBUG__) & defined(__linux__)
  if(retval == false)
    std::cerr << "UKF::ut() error" << std::endl;
#endif

  return retval;
}

bool blob::UKF::predict (ukf_function function, void *args, real_t *r)
{
  bool retval = true;

  blob::Matrix R(_n,_n,r);
  
  blob::Matrix x(_n,1,_x);
  blob::Matrix P(_n,_n,_P);
  blob::Matrix X(_n,2*_n+1,_X);
  blob::Matrix Xs(_n,2*_n+1,_Xs);

  // calculate sigma points around x
  retval &= sigmas(x, P, X);
  // unscented transformation of state
  retval &= ut(function, args, X, R, x, P, X, Xs);

  if(retval == true)
    _updated = false;  
#if defined(__DEBUG__) & defined(__linux__)
  else
    std::cerr << "UKF::predict() error" << std::endl;
#endif

  return retval;
  
}

bool blob::UKF::update  (ukf_function function, void *args, const uint8_t m, real_t *z_, real_t *q)
{
  bool retval = true;

  real_t z1_  [BLOB_UKF_MAX_Z_LENGTH];
  real_t P1_  [BLOB_UKF_MAX_Z_LENGTH*BLOB_UKF_MAX_Z_LENGTH];
  real_t Z1_  [(2*BLOB_UKF_MAX_Z_LENGTH+1)*BLOB_UKF_MAX_Z_LENGTH];
  real_t Z1s_ [(2*BLOB_UKF_MAX_Z_LENGTH+1)*BLOB_UKF_MAX_Z_LENGTH];

  blob::Matrix Q(m,m,q);

  blob::Matrix z(m,1,z_);
  blob::Matrix z1(m,1,z1_);
  blob::Matrix P2(m,m,P1_);
  blob::Matrix Z1(m,2*_n+1,Z1_);
  blob::Matrix Z1s(m,2*_n+1,Z1s_);

  blob::Matrix x(_n,1,_x);
  blob::Matrix P(_n,_n,_P);
  blob::Matrix X(_n,2*_n+1,_X);
  blob::Matrix Xs(_n,2*_n+1,_Xs);
  blob::Matrix wc(2*_n+1,1,_wc);
  
  if(_updated)
  {    
    // if already updated, re-calculate sigma points around x
    retval &= sigmas(x,P,X);
    // if already updated, re-calculate deviation of X
    retval &= Xs.copy(X);
    for(int i=0; i<X.ncols(); i++)
      retval &= Xs.substract(x,0,i);
  }

  // unscented transformation of measurments
  ut(function, args, X, Q, z1, P2, Z1, Z1s);

  real_t auxb[(2*BLOB_UKF_MAX_STATE_LENGTH+1)*BLOB_UKF_MAX_LENGTH];
  real_t p12 [(2*BLOB_UKF_MAX_STATE_LENGTH+1)*BLOB_UKF_MAX_LENGTH];
  real_t   k [BLOB_UKF_MAX_STATE_LENGTH*BLOB_UKF_MAX_Z_LENGTH];
  
  blob::Matrix P12 (_n,m,p12);
  blob::Matrix K (_n,m,k);
  blob::Matrix aux (_n,2*_n+1,auxb);

  // P!2: transformed cross-covariance
  retval &= blob::Matrix::multiplyDiag(Xs, wc, aux);
  retval &= Z1s.transpose();

  retval &= blob::Matrix::multiply(aux, Z1s, P12);
 
  retval &= P2.inverse();
  retval &= blob::Matrix::multiply(P12, P2, K);
  
  //state update
  aux.refurbish(_n,1);
  retval &= z.substract(z1);
  retval &= blob::Matrix::multiply(K, z, aux);       
  retval &= x.add(aux);

  // covariance update
  aux.refurbish(_n,_n);

  retval &= P12.transpose();
  retval &= blob::Matrix::multiply(K, P12, aux);
  retval &= P.substract(aux);

  if(retval == true)
    _updated = true;  
#if defined(__DEBUG__) & defined(__linux__)
  else
    std::cerr << "UKF::update() error" << std::endl;
#endif

  return retval;
}

void blob::UKF::print  ()
{
  blob::Matrix x(_n,1,_x);
  blob::Matrix P(_n,_n,_P);
  blob::Matrix X(_n,2*_n+1,_X);
  blob::Matrix Xs(_n,2*_n+1,_Xs);
  
#if defined(__linux__)
  std::cout << "UKF::x = " << std::endl;
#endif
  x.print();
#if defined(__linux__)
  std::cout << std::endl << "UKF::P = " << std::endl;
#endif
  P.print();
#if defined(__linux__)
  std::cout << std::endl << "UKF::X = " << std::endl;
#endif
  X.print();
#if defined(__linux__)
  std::cout << std::endl << "UKF::Xs = " << std::endl;
#endif
  Xs.print();
#if defined(__linux__)
  std::cout << std::endl;
#endif

}
