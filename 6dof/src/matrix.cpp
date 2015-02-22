/********* blob robotics 2014 *********
 *  title: matrix.cpp
 *  brief: implemention of matrix ops
 * author: adrian jimenez-gonzalez
 * e-mail: blob.robotics@gmail.com
 **************************************/
#include "blob/matrix.h"

#define BLOB_MATRIX_MAX_ROWS 50
#define BLOB_MATRIX_MAX_COLS 50

#if defined(__linux__)
#include <iostream>
#include <math.h>
#endif

float & blob::Matrix::operator()(const uint8_t row, const uint8_t col)
{
  if(_data && (row<_nrows) && (col<_ncols))
    return _data[_ncols*row + col];
  else
    return _data[0];
}

const float & blob::Matrix::operator()(const uint8_t row, const uint8_t col) const
{
  if(_data && (row<_nrows) && (col<_ncols))
    return _data[_ncols*row + col];
  else
    return _data[0];
}

float & blob::Matrix::operator[](int i)
{
  if(_data && (i<_nrows*_ncols))
    return _data[i];
  else
    return _data[0];
}

const float & blob::Matrix::operator[](int i) const 
{
  if(_data && (i<_nrows*_ncols))
    return _data[i];
  else
    return _data[0];
}

blob::Matrix & blob::Matrix::operator+=(blob::Matrix &A)
{
  this->add(A); 
}

blob::Matrix & blob::Matrix::operator-=(blob::Matrix &A)
{
  this->substract(A);
}

blob::Matrix & blob::Matrix::operator*=(const float &n)
{
  this->scale(n);
}

blob::Matrix & blob::Matrix::operator*=(blob::Matrix &A)
{
  this->multiply(A);
}

bool blob::Matrix::copy        (blob::Matrix &A)
{
  if(_data && A.data() && (A.nrows()==_nrows) && (A.ncols()==_ncols))
  {
    memcpy(_data, A.data(), A.length()*sizeof(float));
  }
  #if defined(__DEBUG__) & defined(__linux__)
  else
    std::cerr << "Matrix is not square" << std::endl;
#endif
}

bool blob::Matrix::add        (blob::Matrix &A)
{
  bool retval = false;

  if(_data && A.data() &&
    (A.nrows()==_nrows) && (A.ncols()==_ncols))
  {
    for (int i = 0; i < _nrows*_ncols; i++) 
    {
      _data[i] += A[i];
    }
    retval = true;
  }
  return retval;
}

bool blob::Matrix::substract  (blob::Matrix &A)
{
  bool retval = false;
  
  if(_data && A.data() &&
    (A.nrows()==_nrows) && (A.ncols()==_ncols))
  {
    for (int i = 0; i < _nrows*_ncols; i++) 
    {
      _data[i] -= A[i];
    }
    retval = true;
  }
  return retval;
}
bool blob::Matrix::scale (const float &n)
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

bool blob::Matrix::multiply   (blob::Matrix &A)
{
  bool retval = false;
  
  if((_nrows == A.nrows()) && (_ncols == A.ncols()))
  {
    float r[BLOB_MATRIX_MAX_COLS*BLOB_MATRIX_MAX_ROWS];
    Matrix R(_nrows, _ncols,r);
  
    R.zero();

    for (int i = 0; i < A.nrows(); i++)
    {
      for (int j = 0; j < A.ncols(); j++)
      {
        for (int k = 0; k < A.ncols(); k++)
        {
          R(i,j) += _data[i*_ncols+k]*A(k,j); 
        }
      }
    }
    memcpy(_data,R.data(),R.length()*sizeof(float));
    retval = true;
  }
  return retval;
}

bool blob::Matrix::transpose  ()
{
  bool retval = false;
  
  float r[BLOB_MATRIX_MAX_COLS*BLOB_MATRIX_MAX_ROWS];
  Matrix R(_nrows, _ncols, r);
  
  R.zero();

  for (int i = 0; i < _nrows; i++)
  {
    for (int j = 0; j < _ncols; j++) 
    {
      R(j,i) = _data[i*_ncols + j];
    }
  }
  memcpy(_data, R.data(), R.length()*sizeof(float));
  retval = true;

  return retval;
}

bool blob::Matrix::cholesky (bool zero)
{
  bool retval = true;

  if(_nrows == _ncols)
  {
    float t;
    uint8_t n = _nrows;
    int i=0,j=0,k=0;
    for(i=0 ; i<n && retval == true; i++) 
    {
      if(i > 0) 
      {
        for(j=i; j<n; j++) 
        {
          t = 0.f;
          for(k=0; k<i; k++)
            t += _data[j*n + k]*_data[i*n + k];
          _data[j*n+i] -= t;
        }
      }
      if(_data[i*n + i] <= 0.f) 
      {
#if defined(__DEBUG__) & defined(__linux__)
        std::cerr << "Matrix is not positive definite" << std::endl;
 #endif
        retval = false;
      }
      else
      {
        t = 1.f/sqrtf(_data[i*n + i]);
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
    std::cerr << "Matrix is not square" << std::endl;
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
      _data[i*n + i] = 1.f/_data[i*n + i];
      for(int j=i+1; j<n; j++)
      {
        float t = 0.0;
        for(int k=i; k<j; k++)
          t -= _data[j*n + k]*_data[k*n + i];
        _data[j*n + i] = t/_data[j*n + j];
      }
    }
    retval = true;
  }
  return retval;
}

bool blob::Matrix::inverse ()
{
  bool retval = false;
  
  if((cholesky() == true) && 
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
        float t = 0.0;
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

bool blob::Matrix::fillRows(blob::Matrix &A, uint8_t startrow, uint8_t nrows)
{
  bool retval = false;
  if((A.ncols() == _ncols)&&(A.nrows() >= nrows)&&(_nrows >= startrow+nrows))
  {
    memcpy(&(_data[_ncols*startrow]), A.data(), nrows*_ncols*sizeof(float));
  }
#if defined(__DEBUG__)&&defined(__linux__)
  else
    std::cerr << "Matrix::fillRows() error: "<< (int)A.ncols() << "==" << (int)(_ncols) << "? "
                                             << (int)A.nrows() << ">=" << (int)nrows << "? "
                                             << (int)_nrows << ">=" << (int)(nrows+startrow) << "?" 
                                             << std::endl;
#endif

  return retval;
}

bool blob::Matrix::fillCols(blob::Matrix &A, uint8_t startcol, uint8_t ncols)
{
  bool retval = false;
  if((A.nrows() == _nrows)&&(A.ncols() >= ncols)&&(_ncols >= startcol+ncols))
  {
    for (int i = 0; i<_nrows; i++)
    {
       memcpy(&(_data[i*_ncols + startcol]), &(A.data()[i*A.ncols()]), ncols*sizeof(float));
    }
    retval = true;
  }
#if defined(__DEBUG__) & defined(__linux__)
  else
    std::cerr << "Matrix::fillCols() error: "<< (int)A.nrows() << "==" << (int)_nrows << "? " 
                                             << (int)A.ncols() << ">=" << (int)ncols << "? "
                                             << (int)_ncols << ">=" << (int)(ncols+startcol) << "?" 
                                             << std::endl;
#endif //defined(__DEBUG__) & defined(__linux__)
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
      _data[i*_ncols + i] = 1.f;
    }
    return true;
  }
}

bool blob::Matrix::copy (blob::Matrix  &Dest, blob::Matrix &Orig)
{
  bool retval = false;
  if(Dest.data() && Orig.data() &&
    (Dest.nrows() == Orig.nrows()) && (Dest.ncols() == Orig.ncols()))
  {
    memcpy(Dest.data(),Orig.data(),Orig.length()*sizeof(float));
    retval = true;
  }
  return retval;
}    

bool blob::Matrix::add (blob::Matrix  &A, blob::Matrix &B, blob::Matrix &R)
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
  return retval;
}

bool blob::Matrix::substract (blob::Matrix &A, blob::Matrix &B, blob::Matrix &R)
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
  return retval;
}

bool blob::Matrix::scale (const float &n, blob::Matrix &A, blob::Matrix &R)
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
  return retval;
}


bool blob::Matrix::multiply (blob::Matrix &A, blob::Matrix &B, blob::Matrix &R)
{
  bool retval = false;
  if((A.nrows() == B.ncols()) && (A.ncols() == B.nrows()) &&
     (R.nrows() == A.nrows()) && (R.ncols() == B.ncols()))
  {
    for (int i = 0; i < A.nrows(); i++)
    {
      for (int j = 0; j < B.ncols(); j++)
      {
        uint8_t index = i*R.ncols() + j;
        R[index] = 0.f;
        for (int k = 0; k < A.ncols(); k++)
        {
          R[index] += A[i*A.ncols()+k]*B[k*B.ncols()+j]; 
        }
      }
    }
    retval = true;
  }
  return retval;
}

bool blob::Matrix::transpose (blob::Matrix &A, blob::Matrix &R)
{
  bool retval = false;
  if((A.nrows() == R.nrows()) && (A.ncols() == R.ncols()))
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
  return retval;
}

bool blob::Matrix::cholesky (blob::Matrix &A, blob::Matrix &L)
{
  bool retval = true;
  
  float t;
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
        float s = 0;
        for (int k = 0; k < j; k++)
        {
          s += L[i*n + k]*L[j*n + k];
        }
        if(i==j && (A[i*n + i] - s) <=0)
        {
          return false;
        }
        else
        {
          L[i*n + j] = (i == j)?
          sqrtf(A[i*n + i] - s) : (1.f/L[j*n + j]*(A[i*n + j] - s));
        } 
      }
    }
    return retval;
  }
}

bool blob::Matrix::inverseLow (blob::Matrix &L, blob::Matrix &R)
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
      R[i*n + i] = 1.f/L[i*n + i];
      for(int j=i+1; j<n; j++)
      {
        float t = 0.0;
        for(int k=i; k<j; k++)
          t -= L[j*n + k]*L[k*n + i];
        R[j*n + i] = t/L[j*n + j];
      }
    }
    retval = true;
  }
  return retval;
}

bool blob::Matrix::inverse (blob::Matrix &A, blob::Matrix &R)
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
        float t = 0.0;
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