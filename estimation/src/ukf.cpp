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
 * \file       ukf.cpp
 * \brief      implemention of generic UKF
 * \author     adrian jimenez-gonzalez (blob.robots@gmail.com)
 * \copyright  the MIT License Copyright (c) 2015 Blob Robots.
 *
 ******************************************************************************/

#include <blob/ukf.h>
#include <blob/math.h>

blob::UKF::UKF (uint8_t n, real_t *init_x, real_t alpha, real_t beta, real_t ki) : Estimator (n, init_x)
{

  blob::Matrix P(_n, _n, _P);
  P.eye();
  memset(_X, 0, sizeof(_X));
  memset(_Xs, 0, sizeof(_Xs));

  _alpha = alpha;                          // tunable
  _ki = ki;                                // tunable
  _beta = beta;                            // tunable
  _lambda = _alpha*_alpha*(_n + _ki) - _n; // factor
  _c = _n + _lambda;                       // factor
  _wc[0] = _wm[0] = _lambda/_c; 
  
  for(int i = 1; i<2*_n+1; i++)
    _wc[i] = _wm[i] = 0.5/_c;              // weights for means

  _wc[0] = _wc[0]+(1-_alpha*_alpha+_beta); // weights for covariance

  _c = blob::math::sqrt(_c);
#if defined(__DEBUG__) & defined(__linux__)
        std::cout << "[test] - created UKF " << _n << std::endl;
#endif
  _updated = false;
}

bool blob::UKF::sigmas (blob::Matrix &x, blob::Matrix &P, blob::Matrix &X)
{
  bool retval = true;
  real_t aux[BLOB_UKF_MAX_N*BLOB_UKF_MAX_N];
  blob::Matrix Aux(_n,_n, aux);

  // A = c*chol(P)';
  retval &= Aux.copy(P);
  retval &= Aux.cholesky();
  retval &= Aux.scale(_c);

  // Y = x(:,ones(1,L));
  for(int i=0; i<X.ncols(); i++)
    retval &= X.copy(x,0,i);

  // X = [x Y+A Y-A];
  retval &= X.add(Aux,0,1);
  retval &= X.substract(Aux,0,_n+1);

#if defined(__DEBUG__) & defined(__linux__)
  if(retval == false)
    std::cerr << "UKF::sigmas() error" << std::endl;
#endif

  return retval;
}

bool blob::UKF::ut (estimator_function_t function, void *args, blob::Matrix & X, blob::Matrix & R, 
                    blob::Matrix & u, blob::Matrix & Pu, blob::Matrix & U, blob::Matrix & Us)
{
  bool retval = true;
  real_t aux [(2*BLOB_UKF_MAX_N+1)*BLOB_UKF_MAX_LENGTH];

  int l = u.nrows();

  blob::Matrix in (_n, 1, aux);
  blob::Matrix out (l, 1, &(aux[(int)_n]));

  blob::Matrix wc(2*_n+1,1,_wc);
  
  u.zero();
  
  for(int k=0; k<2*_n+1; k++)
  {
    // Y(:,i) = func(X(:,i),fargs);
    for(int i=0; i<_n; i++)
      in[i] = X(i,k);

    function(in.data(), args, out.data());

    for(int i=0; i<l; i++)
      U(i,k) = out[i];

    // y = y + Wm(i)*Y(:,i);  
    retval &= out.scale(_wm[k]);

    retval &= u.add(out);
    
  }

  // Ys = Y - y(:,ones(1,N));
  retval &= Us.copy(U);
  for(int i=0; i<U.ncols();i++)
    retval &= Us.substract(u, 0, i);

  // P = Ys*diag(Wc)*Ys' + R;
  blob::Matrix Aux(l, 2*_n+1, aux);

  retval &= blob::Matrix::multiplyDiag(Us, wc, Aux);
  
  retval &= Us.transpose();
  
  retval &= blob::Matrix::multiply(Aux,Us,Pu);
  retval &= Pu.add(R);

  retval &= Us.transpose(); // can be avoided with a copy
  
#if defined(__DEBUG__) & defined(__linux__)
  if(retval == false)
    std::cerr << "UKF::ut() error" << std::endl;
#endif

  return retval;
}

bool blob::UKF::predict (estimator_function_t function, void *args, real_t *r)
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

bool blob::UKF::update  (estimator_function_t function, void *args, const uint8_t m, real_t *z_, real_t *q)
{
  bool retval = true;

  real_t z1_  [BLOB_UKF_MAX_M];
  real_t Pz_  [BLOB_UKF_MAX_M*BLOB_UKF_MAX_M];
  real_t Z1_  [(2*BLOB_UKF_MAX_M+1)*BLOB_UKF_MAX_M];
  real_t Z1s_ [(2*BLOB_UKF_MAX_M+1)*BLOB_UKF_MAX_M];

  blob::Matrix Q(m,m,q);

  blob::Matrix z(m,1,z_);
  blob::Matrix z1(m,1,z1_);
  blob::Matrix Pz(m,m,Pz_);
  blob::Matrix Z1(m,2*_n+1,Z1_);
  blob::Matrix Z1s(m,2*_n+1,Z1s_);

  blob::Matrix x(_n,1,_x);
  blob::Matrix P(_n,_n,_P);
  blob::Matrix X(_n,2*_n+1,_X);
  blob::Matrix Xs(_n,2*_n+1,_Xs);
  blob::Matrix wc(2*_n+1,1,_wc);
  
  if(_updated) // if already updated at least once,
  {    
    // re-calculate sigma points around x
    retval &= sigmas(x,P,X);
    // re-calculate deviation of X
    retval &= Xs.copy(X);
    for(int i=0; i<X.ncols(); i++)
      retval &= Xs.substract(x,0,i);
  }

  // unscented transformation of measurments
  ut(function, args, X, Q, z1, Pz, Z1, Z1s);

  real_t auxb[(2*BLOB_UKF_MAX_N+1)*BLOB_UKF_MAX_LENGTH];
  real_t pxz [(2*BLOB_UKF_MAX_N+1)*BLOB_UKF_MAX_LENGTH];
  real_t   k [BLOB_UKF_MAX_N*BLOB_UKF_MAX_M];
  
  blob::Matrix Pxz (_n,m,pxz);
  blob::Matrix K (_n,m,k);
  blob::Matrix aux (_n,2*_n+1,auxb);

  // transformed cross-covariance: Pxz = X1s*diag(Wc)*Z1s'
  retval &= blob::Matrix::multiplyDiag(Xs, wc, aux);
  retval &= Z1s.transpose();
  retval &= blob::Matrix::multiply(aux, Z1s, Pxz);
  
  // K = Pxz/Pz; 
  retval &= blob::Matrix::divide(Pxz, Pz, K);
  
  // update state: x = x + K*(z - z1)
  aux.refurbish(_n,1);
  retval &= z.substract(z1);
  retval &= blob::Matrix::multiply(K, z, aux);       
  retval &= x.add(aux);

  aux.refurbish(_n,_n);

  // update covariance: P = P - K*Pxz'  
  retval &= Pxz.transpose();
  retval &= blob::Matrix::multiply(K, Pxz, aux);
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

