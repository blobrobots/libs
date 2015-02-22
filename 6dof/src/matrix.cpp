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

bool blob::Matrix::get (uint8_t row, uint8_t col, float &val) 
{
  float retval = false;
  if((_data)&&(row<_nrows)&&(col<_ncols)) 
  { 
    val = _data[_ncols*row + col]; 
    retval = true;
  } 
  return retval;
}

bool blob::Matrix::set (uint8_t row, uint8_t col, float val) 
{
  bool retval = false;
  if((_data)&&(row<_nrows)&&(col<_ncols)) 
  { 
    _data[_ncols*row + col] = val; 
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

bool blob::Matrix::scale (const float &n)
{
  for (int i = 0; i < _nrows*_ncols; i++) 
  {
    _data[i] = n*_data[i];
  }
  return true;
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
 
/*bool blob::Matrix::inverse (blob::Matrix &A)
{
  // Backward solve Ux = y, with: U=R[i][j], x[i]=R[i][k],  y[i] = s[i] = (i==k) R[i][i]:0
  // http://arxiv.org/pdf/1111.4144.pdf
  // http://en.wikipedia.org/wiki/Triangular_matrix#Forward_and_back_substitution
  // FIXME: Probably needs transpose
  if((A.nrows() == A.nrows()) && (A.ncols() == A.ncols()) &&
     (blob::Matrix::cholesky(A) == true))
  {
    for(int k=0; k<A.ncols(); k++)
    {
      for (int i=A.nrows()-1; i>=0; i--)
      {
        //A[i*A.ncols()+k] = (i==k)? 1.f/A[i*A.ncols()+k] : 0; // x[i] = y[i]
        A[k*A.ncols()+i] = (i==k)? 1.f/A[k*A.ncols()+i] : 0; // x[i] = y[i]

        for (int j=i+1; j < A.nrows(); j++)
        {
          //A[i*A.ncols()+k] -= A[i*A.ncols()+j]*A[j*A.ncols()+k]; // x[i] = x[i] - U[i, j]*x[j];
          A[k*A.ncols()+i] -= A[j*A.ncols()+i]*A[k*A.ncols()+j]; // x[i] = x[i] - U[i, j]*x[j];
        }
        //A[i*A.ncols()+k] /= A[i*A.ncols()+i]; // x[i] /= U[i,i];
        A[k*A.ncols()+i] /= A[i*A.ncols()+i]; // x[i] /= U[i,i]; 
      }
    }
  }
}
*/

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
  if((A.nrows() == L.nrows()) && (A.ncols() == L.ncols()))
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


bool blob::Matrix::cholesky (blob::Matrix &L, bool zero)
{
  bool retval = true;

  if(L.nrows() == L.ncols())
  {
    float t;
    uint8_t n = L.nrows();
    int i=0,j=0,k=0;
    for(i=0 ; i<n && retval == true; i++) 
    {
      if(i > 0) 
      {
        for(j=i; j<n; j++) 
        {
          t = 0.f;
          for(k=0; k<i; k++)
            t += L[j*n + k] * L[i*n + k];
          L[j*n+i] -= t;
        }
      }
      if(L[i * n+i] <= 0.f) 
      {
#if defined(__DEBUG__) & defined(__linux__)
        std::cerr << "Matrix is not positive definite" << std::endl;
 #endif
        retval = false;
      }
      else
      {
        t = 1.f/sqrtf(L[i*n + i]);
        for(j = i ; j < n ; j++)
          L[j*n + i] *= t;
      }
    }
    if(zero)
    {
      for(int i=n-1; i>0; i--)
      {
        for(int j=i-1; j>=0; j--)
        {
          L[j*n+i]=0;
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

bool blob::Matrix::inverseLow (blob::Matrix &L)
{
  // Inverse of Lower triangular matrix
  bool retval = false;

  if(L.nrows() == L.ncols())
  {
    uint8_t n = L.nrows();
    for(int i=0; i<n; i++)
    {
      L[i*n + i] = 1.f/L[i*n + i];
      for(int j=i+1; j<n; j++)
      {
        float t = 0.0;
        for(int k=i; k<j; k++)
          t -= L[j*n + k]*L[k*n + i];
        L[j*n + i] = t/L[j*n + j];
      }
    }
    retval = true;
  }
  return retval;
}

bool blob::Matrix::inverse (blob::Matrix &A)
{
  // Reconstruct inverse of A: inv(L)'*inv(L)
  bool retval = false;
  
  if((A.nrows() == A.ncols()) && 
     (blob::Matrix::cholesky(A) == true) && 
     (blob::Matrix::inverseLow(A)))
  {
    uint8_t n = A.nrows();
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
          t += A[kk*n + i]*A[kk*n + j];
        }
        A[i*n + j] = t;
      }
    }
    // de-triangularization
    for(int i=n-1; i>0; i--)
    {
      for(int j=i-1; j>=0; j--)
      {
        A[j*n+i]=A[i*n+j];
      }
    }
    retval = true;
  }
  return retval;
}
