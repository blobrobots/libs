/******************************************************************************
 * The MIT License (MIT)
 *
 * Copyright (c) 2017 Blob Robotics
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
 * \brief      interface for matrix object and operations
 * \author     adrian jimenez-gonzalez (blob.robots@gmail.com)
 * \copyright  the MIT License Copyright (c) 2017 Blob Robots.
 *
 ******************************************************************************/
 
#ifndef B_MATRIX_H
#define B_MATRIX_H

#include <blob/types.h>
#include <blob/math.h>

#if defined(__linux__)  
#include <string.h>
#include <iostream>
#endif
// FIXME: revise "virtual" and "const"
//#define Matrix_MaxLength (50*50)

namespace blob {

const int MATRIX_MAX_LENGTH (2500); // 

/**
 * Implements generic Matrix object and operations.
 */
template <typename T> class Matrix
{
  public:
    /**
     * Initializes matrix from already allocated array.
     * \param rows   matrix number of rows
     * \param cols   matrix number of columns.
     * \param data   array with matriz elements allocation with the following 
     *               distribution: [row0 row1 row2 ... rowN].
     */
    Matrix (uint8_t rows=0, uint8_t cols=0, T * data=NULL)
    { 
      _nrows=rows; 
      _ncols=cols; 
      _data=data; 
    }
    /**
     * Provides matrix number of rows.
     * \return matrix number of rows.
     */    
    uint8_t nrows () const { return _nrows; }
    /**
     * Provides matrix number of columns.
     * \return matrix number of columns.
     */
    uint8_t ncols () const { return _ncols; }
    /**
     * Provides matrix number of elements.
     * \return matrix number of elements.
     */    
    uint16_t length () const { return (uint16_t)_ncols*_nrows; }
    /**
     * Provides pointer to matrix data array.
     * \return pointer to matrix data array.
     */     
    T * data () const { return _data; } 
    /**
     * Changes matrix size and if necessary data array.
     * \param rows   matrix number of rows.
     * \param cols   matrix number of columns.
     * \param data   array with matriz elements allocation with the following 
     *               distribution: [row0 row1 row2 ... rowN].
     */
    void refurbish (uint8_t rows, uint8_t cols, T *data = NULL)
    { 
      _nrows=rows; 
      _ncols=cols; 
      if(data) 
        _data=data; 
    }
    /**
     * Makes zero all elements of matrix.
     * \return  true if successful, false otherwise.
     */ 
    bool zero ()
    {
      memset(_data,0,sizeof(T)*_nrows*_ncols); 
      return true;
    }
    /**
     * Makes 1.0 all elements of matrix.
     * \return  true if successful, false otherwise.
     */ 
    bool ones ()
    {
      for(int i=0;i<length();i++)
        _data[i]=1;
    }
    /**
     * Makes identity matrix if matrix is square (nrows=ncols). Identity matrix 
     * elements are: 1 if row=col 0 otherwise.
     * \return  true if successful, false otherwise.
     */
    bool eye ()
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
        std::cerr << "Matrix::cholinverse() error: Matrix is not square" 
                  << std::endl;
    #endif
      
      return retval;
    }
    /**
     * Fills elements from matrix M into this.
     * \param M  matrix to copy from.
     * \return  true if successful, false otherwise.
     */   
    bool copy (const blob::Matrix<T> & M)
    {
      bool retval = false;
      
      if (M.nrows()==_nrows && M.ncols()==_ncols)
      {
        memcpy(this->data(), M.data(), sizeof(T)*this->length());
        retval = true;
      }
      return retval;
    }
    /**
     * Fills elements from matrix M into this.
     * \param M      matrix to copy elements from.
     * \param row0   first row to substitute in this matrix.
     * \param col0   first column to substitute in this matrix.
     * \param row0m  first row to copy from matrix M.
     * \param col0m  first column to copy from matrix M.
     * \param nrows  number of rows to copy.
     * \param ncols  number of columns to copy.
     * \return  true if successful, false otherwise.
     */   
    bool copy (const blob::Matrix<T> & M, uint8_t nrows, uint8_t ncols,
                                          uint8_t row0=0, uint8_t col0=0,
                                          uint8_t row0m=0, uint8_t col0m=0)
    {
      bool retval = false;
      
      if ((row0m+nrows)<=M.nrows()    && (col0m+ncols)<=M.ncols() && 
          (row0+nrows)<=this->nrows() && (col0+ncols)<=this->ncols())
      {
        for (int i=0; i<nrows; i++)
        {
          // _data[this->ncols()*(row0+i) + col0]
          // M(row0m+i,col0m) &M[M.ncols()*(row0m+i) + col0m]
           memcpy(&(this->_data[this->ncols()*(row0+i) + col0]), &M(row0m+i,col0m), ncols*sizeof(T));
        }
        retval = true;
      }
    #if defined(__DEBUG__) & defined(__linux__)
      else
        std::cerr << "Matrix::copy() error: " 
                  << (int)_nrows << ">=" << (int)(nrows+row0) << "?" 
                  << (int)_ncols << ">=" << (int)(ncols+col0) << "?" 
                  << (int)M.nrows() << ">=" << (int)(nrows+row0m) << "?" 
                  << (int)M.ncols() << ">=" << (int)(ncols+col0m) << "?" 
                  << std::endl;
    #endif
      return retval;
    }

    /**
     * Provides element in given row and col.
     * \param row row the element is in.
     * \param col column the element is in.
     * \return  value of matrix element in given row and col.
     */ 
    T & operator () (const uint8_t row, const uint8_t col)
    {
      if(_data && (row<_nrows) && (col<_ncols))
        return _data[_ncols*row + col];
      else
        return _data[0];
    }
    /**
     * Provides element in given row and col.
     * \param row row the element is in.
     * \param col column the element is in.
     * \return  value of matrix element in given row and col.
     */ 
    const T & operator () (const uint8_t row, const uint8_t col) const
    {
      if(_data && (row<_nrows) && (col<_ncols))
        return _data[_ncols*row + col];
      else
        return _data[0];
    }
    /**
     * Provides element in given array position.
     * \param i column the element is in.
     * \return  value of matrix element in given row and col.
     */ 
    T & operator () (int i)
    {
      if(_data && (i<_nrows*_ncols))
        return _data[i];
      else
        return _data[0];
    }
    /**
     * Provides element in given array position.
     * \param i column the element is in.
     * \return  value of matrix element in given row and col.
     */ 
    const T & operator () (int i) const
    {
      if(_data && (i<_nrows*_ncols))
        return _data[i];
      else
        return _data[0];
    }
    /**
     * Provides element in given array position.
     * \param i column the element is in.
     * \return  value of matrix element in given row and col.
     */ 
    T & operator [] (int i)
    {
      if(_data && (i<_nrows*_ncols))
        return _data[i];
      else
        return _data[0];
    }
    /**
     * Provides element in given array position.
     * \param i column the element is in.
     * \return  value of matrix element in given row and col.
     */ 
    const T & operator [] (int i) const
    {
      if(_data && (i<_nrows*_ncols))
        return _data[i];
      else
        return _data[0];
    }
    /**
     * Adds this matrix with other matrix and stores the result in this matrix.
     * \param M  matrix to add.
     * \return  this matrix after adding matrix M.
     */ 
    Matrix<T> & operator += (const Matrix<T> & M) { this->add(M); }
    /**
     * Substracts a matrix to this matrix and stores the result in this matrix.
     * \param M  matrix to substract.
     * \return  this matrix after substracting matrix M.
     */
    Matrix<T> & operator -= (const Matrix<T> & M) { this->substract(M); }
    /**
     * Scales this matrix.
     * \param n  Scalar to scale this matrix.
     * \return  this matrix after scaling.
     */
    Matrix<T> & operator *= (const real_t & n) { this->scale(n); }
   /**
     * Scales this matrix (division).
     * \param n  Scalar to scale this matrix (division).
     * \return  this matrix after dividing.
     */
    Matrix<T> & operator /= (const real_t & n) { this->scale(1/n); }
    /**
     * Elementwise multiplies this matrix with other matrix and stores the 
     * result in this matrix.
     * \param M  Matrix to elementwise multiply with.
     * \return  this matrix elementwise multiplied.
     */
    Matrix<T> & operator &= (const Matrix<T> &M) { this->multiplyElem(M); }
    /**
     * Equality condition operator. Matrix are equal if all elements are equal.
     * \param M Matrix to compare with.
     * \return  true if Matrix are equal, false otherwise.
     */ 
    bool operator == (const Matrix<T> & M) const
    {
      for(int i=0;i<length();i++)
        if(_data[i]!=M[i])
          return false;
      return true;
    }
    /**
     * Inequality condition operator. Matrix are equal if all elements are equal
     * \param M Matrix to compare with.
     * \return  true if Matrix are not equal, false otherwise.
     */ 
    bool operator != (const Matrix<T> &M) const
    {
      for(int i=0;i<length();i++)
        if(_data[i]!=M[i])
          return true;
      return false;
    }
    /**
     * Checks if any matrix elements are NaN (Not a Number)
     * \return  true if a Matrix element is NaN, false otherwise.
     */ 
    bool isNan (void) const
    {
      for(int i=0;i<length();i++)
        if(isnan(_data[i]))
          return true;
      return false;
    }
    /**
     * Checks if any matrix elements are infinite
     * \return  true if a Matrix element is infinite, false otherwise.
     */ 
    bool isInf (void) const
    {
      for(int i=0;i<length();i++)
        if(isinf(_data[i]))
          return true;
        return false;
    }
    /**
     * Checks if all matrix elements are zero
     * \return  true if all Matrix elements are zero, false otherwise.
     */ 
    bool isZero (void) const
    {
      for(int i=0;i<length();i++)
        if(_data[i]!=0)
          return false;
      return true;
    }
    /**
     * Adds matrix M elements to this matrix.
     * \param M      matrix to add elements from.
     * \return  true if successful, false otherwise.
     */       
    bool add (const Matrix<T> & M)
    {
      return add(M,_nrows,_ncols,0,0);
    }
    /**
     * Adds matrix M elements to this matrix.
     * \param M      matrix to add elements from.
     * \param nrows  number of rows to sum.
     * \param ncols  number of columns to sum.
     * \param row0   first row to substitute in this matrix.
     * \param col0   first column to substitute in this matrix.
     * \return  true if successful, false otherwise.
     */       
    bool add (const Matrix<T> & M, uint8_t nrows, uint8_t ncols,
                                           uint8_t row0=0, uint8_t col0=0)
    {
      bool retval = false;
      if ((row0+nrows)<=M.nrows()     && (col0+ncols)<=M.ncols() && 
          (row0+nrows)<=this->nrows() && (col0+ncols)<=this->ncols())
      {
        retval = true;
        for (int i=row0; i<row0+nrows; i++)
          for (int j=col0; j<col0+ncols; j++)
            _data[i*_ncols + j] += M(i,j);
      }
    #if defined(__DEBUG__) & defined(__linux__)
      else
        std::cerr << "Matrix::add() error: "  
                  << (int)_nrows << ">=" << (int)(M.nrows()+row0) << "?" 
                  << (int)_ncols << ">=" << (int)(M.ncols()+col0) << "?" 
                  << std::endl;
    #endif
      return retval;
    }
    /**
     * Substracts matrix M elements from this matrix.
     * \param M  matrix to substract elements from.
     * \return  true if successful, false otherwise.
     */   
    bool substract (const Matrix<T> & M)
    {
      return substract(M,_nrows,_ncols,0,0);
    }    
    /**
     * Substracts matrix M elements from this matrix.
     * \param M      matrix to substract elements from.
     * \param nrows  number of rows to substract.
     * \param ncols  number of columns to substract.
     * \param row0   first row to substitute in this matrix.
     * \param col0   first column to substitute in this matrix.
     * \return  true if successful, false otherwise.
     */   
    bool substract (const Matrix<T> & M, uint8_t nrows, uint8_t ncols,
                                                uint8_t row0=0, uint8_t col0=0)
    {
      bool retval = false;

      if ((row0+nrows)<=M.nrows()     && (col0+ncols)<=M.ncols() && 
          (row0+nrows)<=this->nrows() && (col0+ncols)<=this->ncols())
      {
        retval = true;
        for (int i=row0; i<row0+nrows; i++)
          for (int j=col0; j<col0+ncols; j++)
            _data[i*_ncols + j] -= M(i,j);
      }
    #if defined(__DEBUG__) & defined(__linux__)
      else
        std::cerr << "Matrix::substract() error: "  
                  << (int)_nrows << ">=" << (int)(M.nrows()+row0) << "?" 
                  << (int)_ncols << ">=" << (int)(M.ncols()+col0) << "?" 
                  << std::endl;
    #endif

      return retval;
    }    
    /**
     * Multiplies matrix A and B and stores the result into this matrix.
     * \param A  left matrix to multiply.
     * \param B  right matrix to multiply.
     * \return  true if successful, false otherwise.
     */
    bool multiply (const Matrix<T> & A, const Matrix<T> & B)
    {
      bool retval = false;

      if((A.ncols() == B.nrows()) &&
         (this->nrows() == A.nrows()) && 
         (this->ncols() == B.ncols()))
      {
        for (int i = 0; i < this->nrows(); i++)
        {
          for (int j = 0; j < this->ncols(); j++)
          {
            uint8_t index = i*this->ncols() + j;
            _data[index] = 0;
            for (int k = 0; k < A.ncols(); k++)
            {
              _data[index] += A[i*A.ncols()+k]*B[k*B.ncols()+j]; 
            }
          }
        }
        retval = true;
      }
    #if defined(__DEBUG__) & defined(__linux__)
      else
        std::cerr << "Matrix::multiply() error: " 
                  << (int)A.ncols() << "==" << (int)B.nrows() << "?" 
                  << (int)this->nrows() << "==" << (int)A.nrows() << "?" 
                  << (int)this->ncols() << "==" << (int)B.ncols() << "?" 
                  << std::endl;
    #endif
      return retval;
    }
    /**
     * Multiplies this matrix with diagonal elements in unidimensional matrix D 
     * (row matrix or column matrix).
     * \param d  unidimensional matrix with diagonal elements.
     * \return  true if successful, false otherwise.
     */
    bool multiplyDiag (const Matrix<T> & d)
    {
      bool retval = false;
      
      if (((d.nrows()==_ncols)&&(d.ncols()==1)) || // this*d
          ((d.ncols()==_ncols)&&(d.nrows()==1)) && (_nrows==d.length()))
      {
        for (int i=0; i<_nrows; i++)
          for (int j=0; j<_ncols; j++)
            _data[_ncols*i + j] *= d[j];

        retval = true;
      }
      else if (((d.nrows()==_nrows)&&(d.ncols()==1)) || // D*this
               ((d.ncols()==_nrows)&&(d.nrows()==1)) && (_ncols==d.length()))
      {
        for (int i=0; i<_nrows; i++)
          for (int j=0; j<_ncols; j++)
            _data[_ncols*i + j] *= d[i];

        retval = true;
      }
    #if defined(__DEBUG__) & defined(__linux__)
      else
        std::cerr << "Matrix::multiplyDiag() error: " << (int)_ncols << "/" 
                  << (int)_nrows << "==" << (int)d.nrows() << "/" 
                  << (int)d.ncols() << "?" << (int)d.nrows() << "/" 
                  << (int)d.ncols() << "== 1?" << std::endl;
    #endif

      return retval;
    }
    /**
     * Elementwise multiplies this matrix with matrix M.
     * \param M  matrix to elementwise multply with.
     * \return  true if successful, false otherwise.
     */
    bool multiplyElem (const Matrix<T> & M)
    {
      bool retval = false;

      if ((M.nrows()==_nrows)&&(M.ncols()==_ncols))
      {
        for (int i=0; i < _nrows; i++)
          for (int j=0; j < _ncols; j++)
            _data[_ncols*i+j] *= M[j];
        retval = true;
      }

      return retval;
    }
    /**
     * Scales this matrix.
     * \param n  Scalar to scale this matrix.
     * \return  true if successful, false otherwise.
     */
    bool scale (const T & n)
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
    /**
     * Transposes this matrix.
     * \return  true if successful, false otherwise.
     */
    bool transpose ()
    {
      int start, next, i;
      T tmp;
     
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
    /**
     * Squared Euclidean norm of the matrix
     * \return  squared Euclidean norm of the matrix 
     */
    T squareNorm () const
    {
      T ret=0;
      for(int i=0;i<this->length();i++)
        ret+=_data[i]*_data[i];
      return ret;
    }
    /**
     * Euclidean norm of the matrix
     * \return  Euclidean norm of the matrix
     */
    real_t norm () const { return math::sqrtr(this->squareNorm()); }
    /**
     * Normalizes (divides by norm) the matrix so that its norm is 1.0
     * \return  true if successful, false otherwise.
     */
    bool normalize ()
    {
      real_t norm=this->norm();
      for(int i=0;i<this->length();i++)
        _data[i]/=norm;
      return true;
    }
    /** 
     * Permutes two rows of this matrix
     * \param row1  row to permute 
     * \param row2  row to permute
     * \param ncols  columns to permute (e.g. if all remaining elements in the  
     *               row are zero)  
     * \param col0  column to start permutation at (e.g. if all previous 
     *              elements are zero)      
     * \return  true if successful, false otherwise.
     */
    bool permuteRows (uint8_t row1, uint8_t row2, uint8_t ncols, uint8_t col0=0) 
                                                                  
    {
      if(row1>=_nrows || row2>=_nrows || (col0+ncols)>_ncols)
        return false;

      T aux=0;
        
      for(int i=col0;i<col0+ncols;i++)
      {
        aux=_data[_ncols*row1+i];
        _data[_ncols*row1+i]=_data[_ncols*row2+i];
        _data[_ncols*row2+i]=aux;
      }
      return true;
    }
    /** 
     * Permutes two rows of this matrix
     * \param row1  row to permute 
     * \param row2  row to permute
     * \return  true if successful, false otherwise.
     */
    bool permuteRows(uint8_t row1, uint8_t row2)
    {
      return permuteRows(row1,row2,_ncols,0);
    }
    /** 
     * Permutes two columns of this matrix
     * \param col1  column to permute 
     * \param col2  column to permute
     * \param nrows  rows to permute (e.g. if all remaining elements in the  
     *               column are zero)  
     * \param row0  row to start permutation at (e.g. if all previous elements
     *              are zero)      
     * \return  true if successful, false otherwise.
     */
    bool permuteCols (uint8_t col1, uint8_t col2, uint8_t nrows, uint8_t row0=0) 
                                                                  
    {
      if(col1>=_ncols || col2>=_ncols || (row0+nrows)>_nrows)
        return false;

      T aux=0;
        
      for(int i=row0;i<row0+nrows;i++)
      {
        aux=_data[_ncols*i+col1];
        _data[_ncols*i+col1]=_data[_ncols*i+col2];
        _data[_ncols*i+col2]=aux;
      }
      return true;
    }
    /** 
     * Permutes two columns of this matrix
     * \param col1  column to permute 
     * \param col2  column to permute
     * \return  true if successful, false otherwise.
     */
    bool permuteCols(uint8_t col1, uint8_t col2)
    {
      return permuteCols(col1,col2,_nrows,0);
    }
    /**
     * Shows the matrix elements in standard output
     */
    void print ()
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
    /**
     * Adds matrix M elements to this matrix.
     * \param A matrix to sum.
     * \param B matrix to sum.
     * \param R resulting matrix.
     * \return  true if successful, false otherwise.
     */       
    static bool add (const Matrix<T> & A, const Matrix<T> & B, Matrix<T> & R)
    {
      return (R.copy(A) && R.add(B));
    }
    /**
     * Adds matrix M elements to this matrix.
     * \param A      matrix to sum.
     * \param B      matrix to sum.
     * \param R      resulting matrix.
     * \param nrows  number of rows to sum.
     * \param ncols  number of columns to sum.
     * \param row0   first row to substitute in this matrix.
     * \param col0   first column to substitute in this matrix.
     * \return  true if successful, false otherwise.
     */       
    static bool add (const Matrix<T> & A, const Matrix<T> & B, Matrix<T> & R,
                                                uint8_t nrows, uint8_t ncols,  
                                                uint8_t row0=0, uint8_t col0=0)
    {
      return (R.copy(A) && R.add(B,nrows,ncols,row0,col0));
    }
    /**
     * Substracts matrix M elements from this matrix.
     * \param A      initial matrix.
     * \param B      matrix to substract.
     * \param R      resulting matrix.
     * \return  true if successful, false otherwise.
     */       
    static bool substract(const Matrix<T> & A,const Matrix<T> & B,Matrix<T> & R)
    {
      return (R.copy(A) && R.substract(B));
    }
    /**
     * Substracts matrix M elements from this matrix.
     * \param A      initial matrix.
     * \param B      matrix to substract.
     * \param R      resulting matrix.
     * \param nrows  number of rows to substract.
     * \param ncols  number of columns to substract.
     * \param row0   first row to substitute in this matrix.
     * \param col0   first column to substitute in this matrix.
     * \return  true if successful, false otherwise.
     */       
    static bool substract(const Matrix<T> & A,const Matrix<T> & B,Matrix<T> & R,
                                              uint8_t nrows, uint8_t ncols,
                                              uint8_t row0=0, uint8_t col0=0)
    {
      return (R.copy(A) && R.substract(B,nrows,ncols,row0,col0));
    }
    /**
     * Scales this matrix.
     * \param n  Scalar to scale this matrix.
     * \param A  initial matrix.
     * \param R  resulting matrix.
     * \return  true if successful, false otherwise.
     */
    static bool scale (const T & n, const Matrix<T> & A, Matrix<T> & R)
    {
      return (R.copy(A) && R.scale(n));
    }
    /**
     * Multiplies matrix A and B and stores the result into this matrix.
     * \param A  left matrix to multiply.
     * \param B  right matrix to multiply.
     * \param R  resulting matrix.
     * \return  true if successful, false otherwise.
     */
    static bool multiply (const Matrix<T> & A, const Matrix<T> & B, 
                                                                 Matrix<T> & R)
    {
      return R.multiply(A,B);
    }
    /**
     * Multiplies this matrix with diagonal elements in unidimensional matrix D 
     * (row matrix or column matrix).
     * \param A  initial matrix.
     * \param d  unidimensional matrix with diagonal elements.
     * \param R  resulting matrix.
     * \return  true if successful, false otherwise.
     */    
    static bool multiplyDiag (const Matrix<T> & A, const Matrix<T> & d,
                                                                 Matrix<T> & R)
    {
      return (R.copy(A) && R.multiplyDiag(d));
    }
    /**
     * Elementwise multiplies this matrix with matrix M.
     * \param A  left matrix to elementwise multiply.
     * \param B  right matrix to elementwise multiply.
     * \param R  resulting matrix.
     * \return  true if successful, false otherwise.
     */
    static bool multiplyElem (const Matrix<T> & A, const Matrix<T> & B, 
                                                                 Matrix<T> & R)
    {
      return (R.copy(A) && R.multiplyElem(B));
    }
    /**
     * Transposes this matrix.
     * \param A  initial matrix to transpose.
     * \param R  resulting matrix.
     * \return  true if successful, false otherwise.
     */
    static bool transpose (const Matrix<T> & A, Matrix<T> & R)
    {
      return (R.copy(A) && R.transpose());
    }
    
  protected:
    uint8_t _nrows; /**< matrix number of rows */
    uint8_t _ncols; /**< matrix number of columns */
    T * _data;      /**< pointer to matrix element array */
};

/**
 * Implements real number Matrix object and operations.
 */
class MatrixR : public Matrix<real_t>
{
  public:
    /**
     * Initializes matrix from already allocated array.
     * \param rows   matrix number of rows
     * \param cols   matrix number of columns.
     * \param data   array with matriz elements allocation with the following 
     *               distribution: [row0 row1 row2 ... rowN].
     */
    MatrixR (uint8_t rows = 0, uint8_t cols = 0, real_t *data = NULL);
    /**
     * Calculates this matrix Cholesky decomposition, resulting in a triangular 
     * matrix. https://en.wikipedia.org/wiki/Cholesky_decomposition
     * \param zero  if true, lower triangle converted to zeros
     * \return  true if successful, false otherwise.
     */
    bool cholesky (bool zero=true);
    /**
     * Updates a cholesky factor matrix with a vector and returns the upper 
     * triangular Cholesky factor of A + x*x', where x is a column vector of 
     * appropriate length. cholupdate uses only the diagonal and upper triangle 
     * of R. https://es.mathworks.com/help/matlab/ref/cholupdate.html
     * \param zero  if true, lower triangle converted to zeros
     * \return  true if successful, false otherwise.
     */
    bool cholupdate (MatrixR & v, int sign);
    /**
     * Restores original matrix from its Cholesky decomposition.
     * https://en.wikipedia.org/wiki/Cholesky_decomposition
     * \param zero  if true, lower triangle converted to zeros
     * \return  true if successful, false otherwise.
     */
    bool cholrestore (bool zero=true);
    /**
     * Inverses matrix lower triangular matrix
     * https://en.wikipedia.org/wiki/Triangular_matrix
     * \return  true if successful, false otherwise.
     */
    bool cholinverse ();
    /**
     * Performs LU factorization
     * \return  true if successful, false otherwise.
     */
    bool lu ();
    /**
     * Restores original matrix from LU factorization
     * \return  true if successful, false otherwise.
     */
    bool lurestore ();
    /**
     * Inverses matrix based on Cholesky decomposition
     * \return  true if successful, false otherwise.
     */
    bool inverse ();
    /**
     * Forces matrix definite positiveness.
     * \return  true if successful, false otherwise.
     */ 
    bool forcePositive ();
    /**
     * Forces matrix simmetry.
     * \return  true if successful, false otherwise.
     */ 
    bool simmetrize ();
    /**
     * Divides matrix A by B and stores the result into matrixR.
     * \param A  Dividend matrix.
     * \param B  Divisor matrix.
     * \param R  Resulting matrix
     * \return  true if successful, false otherwise.
     */
    static bool divide (const MatrixR & A, MatrixR & B, MatrixR & R);
    /**
     * Calculates this matrix Cholesky decomposition, resulting in a triangular 
     * matrix. https://en.wikipedia.org/wiki/Cholesky_decomposition
     * \param A  original matrix
     * \param L  resulting cholesky decomposed lower triangular matrix
     * \return  true if successful, false otherwise.
     */
    static bool cholesky (const MatrixR & A, MatrixR & L);
    /**
     * Inverses matrix based on Cholesky decomposition
     * \param A  original matrix
     * \param R  resulting inverse matrix inv(A)
     * \return  true if successful, false otherwise.
     */
    static bool inverse (MatrixR & A, MatrixR & R);
    /**
     * Inverses matrix lower triangular matrix
     * https://en.wikipedia.org/wiki/Triangular_matrix
     * \param L  lower triangle Cholesky factor matrix
     * \param R  resulting inverse matrix inv(L)
     * \return  true if successful, false otherwise.
     */
    static bool cholinverse (const MatrixR & L, MatrixR & R);
    /**
     * Calculates this matrix LDL decomposition, resulting in a triangular 
     * matrix and diagolar elements vector.
     * https://es.mathworks.com/help/dsp/ref/ldlfactorization.html
     * \param A  original matrix
     * \param L  resulting cholesky decomposed lower triangular matrix
     * \return  true if successful, false otherwise.
     */
    static bool ldl (const MatrixR & A, MatrixR & L, MatrixR & d);
    /**
     * Matrix QR decomposition. https://en.wikipedia.org/wiki/QR_decomposition
     * \param A  original matrix A=Q*R
     * \param Q  resulting mxm orthogonal matrix or unitary matrix
     * \param R  resulting mxn upper triangular matrix
     * \return  true if successful, false otherwise.
     */
    static bool qr (const MatrixR & A, MatrixR & Q, MatrixR & R);
    /**
     * Matrix LU decomposition.
     * \param A  original matrix P*A=L*U
     * \param L  resulting lower triangular matrix
     * \param U  resulting upper triangular matrix
     * \param P  resulting pivot matrix
     * \return  true if successful, false otherwise.
     */
    static bool lu (const MatrixR & A, MatrixR & L, MatrixR & U, MatrixR & P);
};

}

#endif // B_MATRIX_H 
