/********* blob robotics 2014 *********
 *  title: matrix.cpp
 *  brief: implemention of matrix ops
 * author: adrian jimenez-gonzalez
 * e-mail: blob.robotics@gmail.com
 **************************************/
#include "blob/matrix.h"

#if defined(__linux__)
#include <iostream>
#include <math.h>
#endif

real_t & blob::Matrix::operator()(const uint8_t row, const uint8_t col)
{
  if(_data && (row<_nrows) && (col<_ncols))
    return _data[_ncols*row + col];
  else
    return _data[0];
}

const real_t & blob::Matrix::operator()(const uint8_t row, const uint8_t col) const
{
  if(_data && (row<_nrows) && (col<_ncols))
    return _data[_ncols*row + col];
  else
    return _data[0];
}

real_t & blob::Matrix::operator[](int i)
{
  if(_data && (i<_nrows*_ncols))
    return _data[i];
  else
    return _data[0];
}

const real_t & blob::Matrix::operator[](int i) const 
{
  if(_data && (i<_nrows*_ncols))
    return _data[i];
  else
    return _data[0];
}

blob::Matrix & blob::Matrix::operator+=(const blob::Matrix &A)
{
  this->add(A); 
}

blob::Matrix & blob::Matrix::operator-=(const blob::Matrix &A)
{
  this->substract(A);
}

blob::Matrix & blob::Matrix::operator*=(const real_t &n)
{
  this->scale(n);
}

bool blob::Matrix::copy (const blob::Matrix &A, uint8_t startrow, uint8_t startcol)
{
  bool retval = false;
  if((_nrows >= startrow+A.nrows())&&(_ncols >= startcol+A.ncols()))
  {
    for (int i = startrow; i<startrow+A.nrows(); i++)
    {
       memcpy(&(_data[i*_ncols + startcol]), &A(i,0), A.ncols()*sizeof(real_t));
    }
    retval = true;
  }
#if defined(__DEBUG__) & defined(__linux__)
  else
    std::cerr << "Matrix::copy() error: " << (int)_nrows << ">=" << (int)(A.nrows()+startrow) << "?" 
                                          << (int)_ncols << ">=" << (int)(A.ncols()+startcol) << "?" 
                                          << std::endl;
#endif

  return retval;
}

bool blob::Matrix::add (const blob::Matrix &A, uint8_t startrow, uint8_t startcol)
{
  bool retval = false;
  if((_nrows >= startrow+A.nrows())&&(_ncols >= startcol+A.ncols()))
  {
    for (int i = startrow; i<startrow+A.nrows(); i++)
    {
      for (int j = startcol; j<startcol+A.ncols(); j++)
      {
        _data[i*_ncols + j] += A(i-startrow,j-startcol);
      }
    }
    retval = true;
  }
#if defined(__DEBUG__) & defined(__linux__)
  else
    std::cerr << "Matrix::add() error: "  << (int)_nrows << ">=" << (int)(A.nrows()+startrow) << "?" 
                                          << (int)_ncols << ">=" << (int)(A.ncols()+startcol) << "?" 
                                          << std::endl;
#endif

  return retval;
}

bool blob::Matrix::substract  (const blob::Matrix &A, uint8_t startrow, uint8_t startcol)
{
  bool retval = false;
  if((_nrows >= startrow+A.nrows())&&(_ncols >= startcol+A.ncols()))
  {
    for (int i = startrow; i<startrow+A.nrows(); i++)
    {
      for (int j = startcol; j<startcol+A.ncols(); j++)
      {
        _data[i*_ncols + j] -= A(i-startrow,j-startcol);
      }
    }
    retval = true;
  }
#if defined(__DEBUG__) & defined(__linux__)
  else
    std::cerr << "Matrix::substract() error: "  << (int)_nrows << ">=" << (int)(A.nrows()+startrow) << "?" 
                                                << (int)_ncols << ">=" << (int)(A.ncols()+startcol) << "?" 
                                                << std::endl;
#endif

  return retval;
}

bool blob::Matrix::scale (const real_t &n)
{
  bool retval = false;
  
  if(_data)
  {
    for (int i = 0; i < _nrows*_ncols; i++) 
    {
      _data[i] = n*_data[i];
    }
    retval = true;
  }
  return true;
}

bool blob::Matrix::transpose ()
{
  int start, next, i;
  real_t tmp;
 
  for (start = 0; start <= _ncols*_nrows - 1; start++) {
    next = start;
    i = 0;
    do {  
      i++;
      next = (next % _nrows)*_ncols + next/_nrows;
    } while (next > start);
    
    if (next < start || i == 1) continue;
 
    tmp = _data[next = start];
    do {
      i = (next % _nrows)*_ncols + next/_nrows;
      _data[next] = (i == start) ? tmp : _data[i];
      next = i;
    } while (next > start);
  }

  i=_ncols;
  _ncols = _nrows;
  _nrows = i;

  return true;
}

bool blob::Matrix::cholesky (bool zero)
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
        std::cerr << "Matrix::cholesky() error: Matrix is not positive definite" << std::endl;
 #endif
        retval = false;
      }
      else
      {
        t = 1/sqrtf(_data[i*n + i]);
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
    std::cerr << "Matrix::cholesky() error: Matrix is not square" << std::endl;
#endif
    retval = false;
  }
  return retval;
}

bool blob::Matrix::inverseLow ()
{
  // Inverse assuming Lower triangular matrix
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
    std::cerr << "Matrix::inverseLow() error: Matrix is not square" << std::endl;
#endif
  return retval;
}

bool blob::Matrix::inverse ()
{
  bool retval = false;
  
  if((cholesky(false) == true) && 
     (inverseLow()== true))
  {
  //Reconstruct inverse of A: inv(L)'*inv(L)
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

bool blob::Matrix::eye()
{
  bool retval = false;
  if(_ncols == _nrows)
  {
    this->zero();
    for (int i=0; i<_nrows; i++)
    {
      _data[i*_ncols + i] = 1;
    }
    retval = true;
  }
#if defined(__DEBUG__) & defined(__linux__)
  else
    std::cerr << "Matrix::inverseLow() error: Matrix is not square" << std::endl;
#endif
  
  return retval;
}

/*bool blob::Matrix::copy (blob::Matrix  &Dest, const blob::Matrix &Orig, uint8_t startrow, uint8_t startcol)
{
  bool retval = false;
  if((Dest.nrows() >= startrow+Orig.nrows())&&(Dest.ncols() >= startcol+Orig.ncols()))
  {
    for (int i = startrow; i<startrow+Orig.nrows(); i++)
    {
       memcpy(&Dest(i,startcol), &Orig(i,0), Orig.ncols()*sizeof(real_t));
    }
    retval = true;
  }
#if defined(__DEBUG__) & defined(__linux__)
  else
    std::cerr << "Matrix::copy() error: " << (int)Dest.nrows() << ">=" << (int)(Orig.nrows()+startrow) << "?" 
                                          << (int)Dest.ncols() << ">=" << (int)(Orig.ncols()+startcol) << "?" 
                                          << std::endl;
#endif

  return retval;
}
*/
bool blob::Matrix::add (const blob::Matrix  &A, const blob::Matrix &B, blob::Matrix &R)
{
  bool retval = false;
  if((A.nrows() == B.nrows()) && (A.ncols() == B.ncols()) &&
     (A.nrows() == R.nrows()) && (A.ncols() == R.ncols()))
  {
    for (int i = 0; i < R.length(); i++) 
    {
      R[i] = A[i]+B[i];
    }
    retval = true;
  }
#if defined(__DEBUG__) & defined(__linux__)
  else
    std::cerr << "Matrix::add() error: "  << (int)A.nrows() << "==" << (int)B.nrows() << "?" 
                                          << (int)A.ncols() << "==" << (int)B.ncols() << "?" 
                                          << (int)A.nrows() << "==" << (int)R.nrows() << "?" 
                                          << (int)A.ncols() << "==" << (int)R.ncols() << "?" 
                                          << std::endl;
#endif
  return retval;
}

bool blob::Matrix::substract (const blob::Matrix &A, const blob::Matrix &B, blob::Matrix &R)
{
  bool retval = false;
  if((A.nrows() == B.nrows()) && (A.ncols() == B.ncols()) &&
     (A.nrows() == R.nrows()) && (A.ncols() == R.ncols()))
  {
    for (int i = 0; i < R.length(); i++) 
    {
      R[i] = A[i]-B[i];
    }
    retval = true;
  }
#if defined(__DEBUG__) & defined(__linux__)
  else
    std::cerr << "Matrix::substract() error: " << (int)A.nrows() << "==" << (int)B.nrows() << "?" 
                                               << (int)A.ncols() << "==" << (int)B.ncols() << "?" 
                                               << (int)A.nrows() << "==" << (int)R.nrows() << "?" 
                                               << (int)A.ncols() << "==" << (int)R.ncols() << "?" 
                                               << std::endl;
#endif

  return retval;
}

bool blob::Matrix::scale (const real_t &n, const blob::Matrix &A, blob::Matrix &R)
{
  bool retval = false;
  if((A.nrows() == R.nrows()) && (A.ncols() == R.ncols()))
  {
    for (int i = 0; i < R.length(); i++) 
    {
      R[i] = n*A[i];
    }
    retval = true;
  }
#if defined(__DEBUG__) & defined(__linux__)
  else
    std::cerr << "Matrix::scale() error: " << (int)A.nrows() << "==" << (int)R.nrows() << "?" 
                                           << (int)A.ncols() << "==" << (int)R.ncols() << "?" 
                                           << std::endl;
#endif

  return retval;
}

bool blob::Matrix::multiply (const blob::Matrix &A, const blob::Matrix &B, blob::Matrix &R)
{
  bool retval = false;
  if((A.ncols() == B.nrows()) &&
     (R.nrows() == A.nrows()) && 
     (R.ncols() == B.ncols()))
  {
    for (int i = 0; i < R.nrows(); i++)
    {
      for (int j = 0; j < R.ncols(); j++)
      {
        uint8_t index = i*R.ncols() + j;
        R[index] = 0;
        for (int k = 0; k < A.ncols(); k++)
        {
          R[index] += A[i*A.ncols()+k]*B[k*B.ncols()+j]; 
        }
      }
    }
    retval = true;
  }
#if defined(__DEBUG__) & defined(__linux__)
  else
    std::cerr << "Matrix::multiply() error: " << (int)A.ncols() << "==" << (int)B.nrows() << "?" 
                                              << (int)R.nrows() << "==" << (int)A.nrows() << "?" 
                                              << (int)R.ncols() << "==" << (int)B.ncols() << "?" 
                                              << std::endl;
#endif

  return retval;
}

bool blob::Matrix::multiplyDiag (const Matrix &A, const Matrix &D, Matrix &R)
{
  bool retval = false;
  
  // M*D
  if (((D.nrows() == A.ncols())&&(D.ncols() == 1))||
      ((D.ncols() == A.ncols())&&(D.nrows() == 1))&&
      ((R.ncols() == A.ncols())&&(R.nrows() == D.length())))
  {
    for (int i = 0; i < R.nrows(); i++)
    {
      for (int j = 0; j < R.ncols(); j++)
      {
        R(i,j) = A(i,j)*D[j];
      }
    }
    retval = true;
  }
  // D*M
  else if (((D.nrows() == A.nrows())&&(D.ncols() == 1))||
           ((D.ncols() == A.nrows())&&(D.nrows() == 1))&&
           ((R.nrows() == A.nrows())&&(R.ncols() == D.length())))
  {
    for (int i = 0; i < R.nrows(); i++)
    {
      for (int j = 0; j < R.ncols(); j++)
      {
        R(i,j) = A(i,j)*D[i];
      }
    }
    retval = true;
  }
#if defined(__DEBUG__) & defined(__linux__)
  else
    std::cerr << "Matrix::multiplyDiag() error: " << (int)A.ncols() << "/" << (int)A.nrows() << "==" 
                                                  << (int)R.ncols() << "/" << (int)R.nrows() << "==" 
                                                  << (int)D.nrows() << "/" << (int)D.ncols() << "?" 
                                                  << (int)D.nrows() << "/" << (int)D.ncols() << "== 1?" 
                                                  << std::endl;
#endif

  return retval;
}

bool blob::Matrix::transpose (const blob::Matrix &A, blob::Matrix &R)
{
  bool retval = false;
  if((A.nrows() == R.ncols()) && (A.ncols() == R.nrows()))
  {
    for (int i = 0; i < A.nrows(); i++)
    {
      for (int j = 0; j < A.ncols(); j++) 
      {
        R[j*A.nrows() + i] = A[i*A.ncols() + j];
      }
    }
    retval = true;
  }
#if defined(__DEBUG__) & defined(__linux__)
  else
    std::cerr << "Matrix::transpose() error: " << (int)A.nrows() << "==" << (int)R.ncols() << "?" 
                                               << (int)A.ncols() << "==" << (int)R.nrows() << "?" 
                                               << std::endl;
#endif

  return retval;
}

bool blob::Matrix::cholesky (const blob::Matrix &A, blob::Matrix &L)
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
          std::cerr << "Matrix::cholesky() error: Matrix is not positive definite" << std::endl;
#endif
          return false;
        }
        else
        {
          L[i*n + j] = (i == j)?
          sqrtf(A[i*n + i] - s) : (1/L[j*n + j]*(A[i*n + j] - s));
        } 
      }
    }
#if defined(__DEBUG__) & defined(__linux__)
        std::cerr << "Matrix::cholesky() error: Matrix is not square" << std::endl;
#endif
    return retval;
  }
}

bool blob::Matrix::inverseLow (const blob::Matrix &L, blob::Matrix &R)
{
  // Inverse assuming Lower triangular matrix
  bool retval = false;

  if((L.nrows() == L.ncols())&&
     (R.nrows() == L.nrows()) && 
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
    std::cerr << "Matrix::inverseLow() error: " << (int)L.nrows() << "==" << (int)L.ncols() << "?" 
                                                << (int)R.nrows() << "==" << (int)L.nrows() << "?" 
                                                << (int)R.ncols() << "==" << (int)L.ncols() << "?" 
                                                << std::endl;
#endif

  return retval;
}

bool blob::Matrix::inverse (const blob::Matrix &A, blob::Matrix &R)
{
  bool retval = false;
  
  if((cholesky(A,R)  == true) && 
     (R.inverseLow() == true))
  {
    //Reconstruct inverse of R: inv(L)'*inv(L)
    uint8_t n = R.nrows();
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
          t += R[kk*n + i]*R[kk*n + j];
        }
        R[i*n + j] = t;
      }
    }
    // de-triangularization
    for(int i=n-1; i>0; i--)
    {
      for(int j=i-1; j>=0; j--)
      {
        R[j*n+i] = R[i*n+j];
      }
    }
    retval = true;
  }
  return retval;
}

void blob::Matrix::print ()
{
  for(int i=0; i<_nrows; i++)
  {
    for(int j=0; j<_ncols; j++)
#if defined(__linux__)  
      std::cout << " " << _data[i*_ncols + j];
    std::cout << std::endl;
#elif defined(__AVR_ATmega32U4__)
    if(Serial)
    {
      Serial.print(" ");
      Serial.print(_data[i*_ncols + j]);
    }
    Serial.println("");
#endif
  }
}