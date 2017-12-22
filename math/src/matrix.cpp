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
 * \file       matrix.h
 * \brief      implementation of matrix object and operations
 * \author     adrian jimenez-gonzalez (blob.robots@gmail.com)
 * \copyright  the MIT License Copyright (c) 2017 Blob Robots.
 *
 ******************************************************************************/
#include "blob/matrix.h"
#include "blob/math.h"

#if defined(__linux__)
 #include <iostream>
#endif

blob::MatrixR::MatrixR (uint8_t rows, uint8_t cols, real_t *data) : 
                                              Matrix<real_t>(rows,cols,data) {};

bool blob::MatrixR::cholesky (bool zero)
{
  bool retval = true;

  if(_nrows == _ncols)
  {
    real_t t;
    uint8_t n = _nrows;
    int i=0,j=0,k=0;
    for(i=0 ; i<n && retval == true; i++) 
    {
      if(i > 0) 
      {
        for(j=i; j<n; j++) 
        {
          t = 0;
          for(k=0; k<i; k++)
            t += _data[j*n + k]*_data[i*n + k];
          _data[j*n+i] -= t;
        }
      }
      if(_data[i*n + i] <= 0) 
      {
#if defined(__DEBUG__) & defined(__linux__)
        std::cerr << "MatrixR::cholesky() error: Matrix is not positive definite"
                  << std::endl;
#endif
        retval = false;
      }
      else
      {
        t = 1/blob::math::sqrtr(_data[i*n + i]);
        for(j = i ; j < n ; j++)
          _data[j*n + i] *= t;
      }
    }
    if(zero)
    {
      for(int i=n-1; i>0; i--)
      {
        for(int j=i-1; j>=0; j--)
        {
          _data[j*n+i]=0;
        }
      }
    }
  }
  else
  {
#if defined(__DEBUG__) & defined(__linux__)
    std::cerr << "MatrixR::cholesky() error: Matrix is not square" << std::endl;
#endif
    retval = false;
  }
  return retval;
}

bool blob::MatrixR::cholupdate (MatrixR & v, int sign)
{
  bool retval = false;
  uint8_t n = _nrows;

  if(_nrows != _ncols)
  {
#if defined(__DEBUG__) & defined(__linux__)
    std::cerr << "MatrixR::cholupdate() error: Matrix is not square"<< std::endl;
#endif
    return false;
  }

  if((v.length() != n)||((v.nrows() != 1)&&(v.ncols() != 1)))
  {
#if defined(__DEBUG__) & defined(__linux__)
    std::cerr << "MatrixR::cholupdate() error: vector not 1x" << (int)n 
              << std::endl;
#endif
    return false;
  }
  
  if((sign != -1)&&(sign != 1))
  {
#if defined(__DEBUG__) & defined(__linux__)
    std::cerr << "Matrix::cholupdate() error: sign not unitary s=" << sign 
              << std::endl;
#endif
    return false;
  }

  for (int i=0; i<n; i++)
  {
    real_t sr = _data[i*n+i]*_data[i*n+i] + (real_t)sign*v[i]*v[i];

    if(_data[i*n+i] == 0.0)
    {
#if defined(__DEBUG__) & defined(__linux__)
      std::cerr << "MatrixR::cholupdate() error: Matrix with zero diagonal "
                << "element " << i << std::endl;
#endif
      return false;
    }

    if(sr < 0.0)
    {
#if defined(__DEBUG__) & defined(__linux__)
      std::cerr << "Matrix::cholupdate() error: Result is not positive definite"
                << " sqrt(neg)" << std::endl;
#endif
      return false;
    }
    real_t r =  blob::math::sqrtr(_data[i*n+i]*_data[i*n+i] + 
                                                        (real_t)sign*v[i]*v[i]);
    real_t c = r/_data[i*n+i];
    real_t s = v[i]/_data[i*n+i];
    _data[i*n+i] = r;

    if(c == 0.0)
    {
#if defined(__DEBUG__) & defined(__linux__)
      std::cerr << "MatrixR::cholupdate() error: Result is not positive definite" 
                << std::endl;
#endif
      return false;
    }

    for(int j=i+1; j<n; j++)
    {
      _data[j*n+i] = (_data[j*n+i] + sign*s*v[j])/c;
      v[j] = c*v[j] - s*_data[j*n+i];
    }
  }
  return true;
}

bool blob::MatrixR::cholrestore (bool zero)
{
  bool retval = false;

  if(_nrows == _ncols)
  {
    if(zero == false) // original matrix stored in upper triangle
    {
      uint8_t n = _nrows;
      for(int i=0; i<n; i++)
      {
        real_t t = 0.0;
        for(int j=0; j<i+1; j++)
        {
          t += _data[i*n+j]*_data[i*n+j];
          // assumes original matrix stored in upper triangle
          _data[i*n+j] = (i==j)? t:_data[j*n+i]; 
        }
      }
      retval = true;
    }
    else
    { // TODO
#if defined(__DEBUG__) & defined(__linux__)
      std::cerr << "MatrixR::cholrestore() error: not implemented for zero=true" 
                << std::endl;
#endif
    }
  }
#if defined(__DEBUG__) & defined(__linux__)
  else
    std::cerr << "MatrixR::cholinverse() error: Matrix is not square" 
              << std::endl;
#endif
  return retval;
}

bool blob::MatrixR::cholinverse ()
{
  bool retval = false;

  if(_nrows == _ncols)
  {
    uint8_t n = _nrows;
    for(int i=0; i<n; i++)
    {
      _data[i*n + i] = 1/_data[i*n + i];
      for(int j=i+1; j<n; j++)
      {
        real_t t = 0.0;
        for(int k=i; k<j; k++)
          t -= _data[j*n + k]*_data[k*n + i];
        _data[j*n + i] = t/_data[j*n + j];
      }
    }
    retval = true;
  }
#if defined(__DEBUG__) & defined(__linux__)
  else
    std::cerr <<"MatrixR::cholinverse() error: Matrix is not square"<< std::endl;
#endif
  return retval;
}

bool blob::MatrixR::lu() 
{
   if(_nrows!=_ncols)
  {
#if defined(__DEBUG__) & defined(__linux__)
    std::cerr << "MatrixR::lu() error: Matrix is not square" << std::endl;
#endif
    return false;
  }

  for(int k=0; k<_ncols-1; k++)
  {
    for(int j=k+1; j<_ncols; j++) 
    {
      _data[j*_ncols+k]=_data[j*_ncols+k]/_data[k*_ncols+k];
      for(int l=k+1;l<_ncols;l++)
        _data[j*_ncols+l]=_data[j*_ncols+l]-_data[j*_ncols+k]*_data[k*_ncols+l];
    }
  }
}

bool blob::MatrixR::lurestore() 
{
   if(_nrows!=_ncols)
  {
#if defined(__DEBUG__) & defined(__linux__)
    std::cerr << "MatrixR::lurestore() error: Matrix is not square" << std::endl;
#endif
    return false;
  }
  for(int i=_nrows-1; i>0; i--)
  {
    for(int j=_ncols-1; j>=0; j--)   
    {
      if(i>j)
        _data[i*_ncols+j]*=_data[j*_ncols+j];
      for(int k=0;k<i&&k<j;k++)
        _data[i*_ncols+j]+=_data[i*_ncols+k]*_data[k*_ncols+j];
#if defined(__DEBUG__) & defined(__linux__)
      print();
      std::cout << std::endl;
      std::cout << i << "," << j << std::endl;
      std::cout << std::endl;
#endif
    }
  }
}

bool blob::MatrixR::inverse ()
{
  bool retval = false;
  
  if((cholesky(false) == true) && (cholinverse()== true))
  {
  // reconstruct inverse of A: inv(A) = inv(L)'*inv(L)
    uint8_t n = _nrows;
    for(int i=0; i<n; i++)
    {
      int ii = n-i-1;
      for(int j=0; j<=i; j++)
      {
        int jj = n-j-1;
        real_t t = 0.0;
        for(int k=0; k<=ii; k++)
        {
          int kk = n-k-1;
          t += _data[kk*n + i]*_data[kk*n + j];
        }
        _data[i*n + j] = t;
      }
    }
    //de-triangularization
    for(int i=n-1; i>0; i--)
    {
      for(int j=i-1; j>=0; j--)
      {
        _data[j*n+i] = _data[i*n+j];
      }
    }
    retval = true;
  }
  return retval;
}

bool blob::MatrixR::forcePositive ()
{
  bool retval = false;

  if(_nrows == _ncols)
  {
    real_t min = blob::math::rabs(_data[0]);
    real_t max = min;
    for (int i = 0; i<_nrows; i++)
    {
      real_t mm = blob::math::rabs(_data[i*_ncols + i]);
      if (mm < min) min = mm;
      if (mm > max) max = mm;
    }
    real_t epsilon = min/max;

    if(epsilon < 1.0)
    { 
#if defined(__DEBUG__) & defined(__linux__)
      std::cout << "MatrixR::forcePositive() epsilon=" << epsilon << std::endl;
#endif
      for (int i = 0; i<_nrows; i++)
        _data[i*_ncols + i] += epsilon;
    }

    retval = true;
  }
#if defined(__DEBUG__) & defined(__linux__)
  else
    std::cerr << "MatrixR::forcePositive() error: Matrix is not square"
              << std::endl;
#endif

  return retval;
}

bool blob::MatrixR::simmetrize ()
{
  bool retval = false;

  if(_nrows == _ncols)
  {
    for (int i = 0; i<_nrows; i++)
    {
      for (int j = 0; j<i; j++)
      {
        real_t t = (_data[i*_ncols + j]+_data[j*_ncols + i])/2;
        _data[i*_ncols + j] = _data[j*_ncols + i] = t;
      }
    }
    retval = true;
  }
#if defined(__DEBUG__) & defined(__linux__)
  else
    std::cerr<< "MatrixR::simmetrize() error: Matrix is not square" << std::endl;
#endif

  return retval;
}

bool blob::MatrixR::divide (const blob::MatrixR & A, blob::MatrixR & B, 
                                                   blob::MatrixR & R)
{
  bool retval = false;
  uint8_t n = B.nrows();
  uint8_t m = A.nrows();

  if((R.nrows() == m)&&
     (R.ncols() == n)&&
      B.cholesky(false) == true)
  {
    for(int c = 0; c<m; c++)
    {
      // forward solve Ly = I(c)
      for(int i = 0; i<n; i++)
      {
        R(c,i) = A(c,i);
        for(int j=0; j<i; j++)
          R(c,i) -= B(i,j)*R(c,j);
        R(c,i) /= B(i,i);
      }

      // backward solve L'x = y
      for(int i=n-1; i>=0; i--)
      {
        for (int j=i+1; j<n; j++)
          R(c,i) -= B(j,i)*R(c,j);
        R(c,i) /= B(i,i);
      }
    }
    // restore A from L
    B.cholrestore(false);
    retval = true;
  }
  return retval;
}

bool blob::MatrixR::cholesky (const blob::MatrixR & A, blob::MatrixR & L)
{
  bool retval = true;
  
  real_t t;
  uint8_t n = A.nrows();
  L.zero();
  if((A.nrows() == A.ncols()) &&
     (A.nrows() == L.nrows()) && 
     (A.ncols() == L.ncols()))
  {
    for (int i = 0; i < n; i++)
    {
      for (int j = 0; j < (i+1); j++)
      {
        real_t s = 0;
        for (int k = 0; k < j; k++)
        {
          s += L[i*n + k]*L[j*n + k];
        }
        if(i==j && (A[i*n + i] - s) <=0)
        {
#if defined(__DEBUG__) & defined(__linux__)
          std::cerr << "MatrixR::cholesky() error: Matrix is not positive"
                    << " definite" << std::endl;
#endif
          return false;
        }
        else
        {
          L[i*n + j] = (i == j)?
          blob::math::sqrtr(A[i*n + i] - s) : (1/L[j*n + j]*(A[i*n + j] - s));
        } 
      }
    }
  }
  else
  {
#if defined(__DEBUG__) & defined(__linux__)
    std::cerr << "MatrixR::cholesky() error: Matrix is not square" << std::endl;
#endif
    retval = false;
  }
  return retval;
}

/*
bool blob::Matrix::inverse (const blob::Matrix &A, blob::Matrix &R)
{
  return (R.copy(A) && R.inverse());
}
*/
// TODO: https://rosettacode.org/wiki/LU_decomposition
bool blob::MatrixR::inverse (blob::MatrixR & A, blob::MatrixR & R)
{
  bool retval = false;

  if((R.nrows() == R.ncols())&&
     (R.nrows() == A.nrows())&& 
     (R.ncols() == R.ncols())&&
      A.cholesky(false) == true) // A=L
  {
    uint8_t n = R.nrows();

    for(int c = 0; c<n; c++)
    {
      // forward solve Ly = I(c)
      for(int i = 0; i<n; i++)
      {
        R(c,i) = (c==i)? 1:0;
        for(int j=0; j<i; j++)
          R(c,i) -= A(i,j)*R(c,j);
        R(c,i) /= A(i,i);
      }

      // backward solve L'x = y
      for(int i=n-1; i>=0; i--)
      {
        for (int j=i+1; j<n; j++)
          R(c,i) -= A(j,i)*R(c,j);
        R(c,i) /= A(i,i);
      }
    }
    // restore A from L
    A.cholrestore(false);
    retval = true;
  }
  return retval;
}

bool blob::MatrixR::cholinverse (const blob::MatrixR & L, blob::MatrixR & R)
{
  bool retval = false;

  if((L.nrows() == L.ncols())&&
     (R.nrows() == L.nrows())&& 
     (R.ncols() == L.ncols()))
  {
    uint8_t n = L.nrows();
    R.zero();
    for(int i=0; i<n; i++)
    {
      R[i*n + i] = 1/L[i*n + i];
      for(int j=i+1; j<n; j++)
      {
        real_t t = 0.0;
        for(int k=i; k<j; k++)
          t -= L[j*n + k]*L[k*n + i];
        R[j*n + i] = t/L[j*n + j];
      }
    }
    retval = true;
  }
#if defined(__DEBUG__) & defined(__linux__)
  else
    std::cerr << "Matrix::cholinverse() error: " << (int)L.nrows() << "==" 
                                                 << (int)L.ncols() << "?" 
                                                 << (int)R.nrows() << "==" 
                                                 << (int)L.nrows() << "?" 
                                                 << (int)R.ncols() << "==" 
                                                 << (int)L.ncols() << "?" 
                                                 << std::endl;
#endif
  return retval;
}

bool blob::MatrixR::ldl (const blob::MatrixR & A, blob::MatrixR & L, 
                                                  blob::MatrixR & d)
{

  if((A.nrows()!=A.ncols())||(L.nrows()!=L.ncols())||(A.ncols()!=L.nrows()))
  {
#if defined(__DEBUG__) & defined(__linux__)
    std::cerr << "Matrix::ldl() error: Matrix is not square" << std::endl;
#endif
    return false;
  }
  if(((d.nrows() != 1)&&(d.ncols() != 1))||(d.length() != A.ncols()))
  {
#if defined(__DEBUG__) & defined(__linux__)
    std::cerr << "Matrix::ldl() error: Matrix is not square" << std::endl;
#endif
    return false;
  }

  uint8_t n = A.nrows();
  int i,j,k;
  L.zero();
  for(j=0; j<n; j++)
  {
    L(j,j) = 1.0;
    real_t t = A(j,j);
    for(k=1; k<j; k++)
      t -= d[k]*L(j,k)*L(j,k);
    d[j] = t;
    
    for(i=j+1; i<n; i++)
    {
      L(j,i) = 0.0;
      t = A(i,j);
      for (k=1; k<j;k++)
        t -= d[k]*L(i,k)*L(j,k);
      L(i,j) = t/d[j];
    }
  }
}

bool blob::MatrixR::qr (const MatrixR & A, MatrixR &Q, MatrixR & R)
{
  uint8_t m = A.nrows();
  uint8_t n = A.ncols();
  int i=0,j=0,k=0;
  
  if(A.length()>MATRIX_MAX_LENGTH)
  {
#if defined(__DEBUG__) & defined(__linux__)
    std::cerr << "Matrix::qr() error: Matrix A length is greater than " 
              << (int)MATRIX_MAX_LENGTH << std::endl;
#endif
    return false;
  }

  if((Q.nrows() != m)||(Q.ncols() != m))
  {
#if defined(__DEBUG__) & defined(__linux__)
    std::cerr << "Matrix::qr() error: Matrix Q is not square with m=" << (int)m 
              << std::endl;
#endif
    return false;
  }

  if((R.nrows() != m)||(R.ncols() != n))
  {
#if defined(__DEBUG__) & defined(__linux__)
    std::cerr << "Matrix::qr() error: Matrix R has not the same size as A"
              << std::endl;
#endif
    return false;
  }
 
  real_t h[MATRIX_MAX_LENGTH];
  real_t aux[MATRIX_MAX_LENGTH];
  
  Matrix H(m,m,h);
  Matrix Aux(m,n,aux);

  Q.eye();
  R.copy(A);

  for (i=0; i<n && i<(m-1); i++)
  {
    H.eye();

    real_t sign = (R(i,i) < 0)? -1:1;
        
    real_t nrm = 0.0;
    for(k=i; k<m; k++)
      nrm += R(k,i)*R(k,i);
    nrm = blob::math::sqrtr(nrm);
        
    real_t d = (R(i,i) + sign*nrm);
    real_t t = 1;
    for(k=i+1; k<m; k++)
      t += (R(k,i)/d)*(R(k,i)/d);
    t=2/t;
  
    for(j=i; j<m; j++)
    {
      for(k=i; k<m; k++)
      {
        real_t vj = (j==i)? 1.0 : R(j,i)/d;
        real_t vk = (k==i)? 1.0 : R(k,i)/d;
        H(j,k) = H(j,k) - vj*vk*t;
      }
    }
    Aux.refurbish(m,m);
    multiply(Q,H,Aux);
    Q.copy(Aux);

    Aux.refurbish(m,n);
    multiply(H,R,Aux);
    R.copy(Aux);
  }

  return true;
}
// https://rosettacode.org/wiki/LU_decomposition 
// TODO: Check why partial pivoting, applied but A=LU instead of PA=LU and result differs from octave and rosettacode 
bool blob::MatrixR::lu (const MatrixR & A, MatrixR & L, MatrixR & U, MatrixR & P)
{
  if((A.nrows()!=A.ncols())||(L.nrows()!=L.ncols())||(U.ncols()!=U.nrows()))
  {
#if defined(__DEBUG__) & defined(__linux__)
    std::cerr << "Matrix::lu() error: Matrix are not square" << std::endl;
#endif
    return false;
  }
  if((A.nrows()!=L.ncols())||(A.nrows()!=U.ncols()))
  {
#if defined(__DEBUG__) & defined(__linux__)
    std::cerr << "Matrix::lu() error: Matrix sizes are not equal" << std::endl;
#endif
    return false;
  }

  U.copy(A); L.eye(); P.eye();

  for(int k=0; k<U.ncols()-1; k++)
  {
    // select next row to permute (partial pivoting)
/*    real_t max=blob::math::rabs(U(k,k));
    uint8_t imax=k;
    for(int i=k+1; i<A.nrows(); i++) 
    { 
      real_t next=blob::math::rabs(U(i,k));
      if(next>max)
      {
        max=next;
        imax=i;
      }
    }
    if(imax!=k)
    {
      U.permuteRows(k,imax,U.ncols()-k,k);
      L.permuteRows(k,imax,k,0);
      P.permuteRows(k,imax);
    }
*/    for(int j=k+1; j<A.ncols(); j++) 
    {
      L(j,k)=U(j,k)/U(k,k);
      for(int l=k;l<A.ncols();l++)
        U(j,l)=U(j,l)-L(j,k)*U(k,l);
    }
  }
}

