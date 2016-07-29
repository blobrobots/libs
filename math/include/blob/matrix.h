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
 * \brief      interface for matrix object and operations
 * \author     adrian jimenez-gonzalez (blob.robots@gmail.com)
 * \copyright  the MIT License Copyright (c) 2015 Blob Robots.
 *
 ******************************************************************************/

#ifndef B_MATRIX_H
#define B_MATRIX_H

#include <blob/types.h>

#if defined(__linux__)  
#include <string.h>
#endif

namespace blob {

/**
 * Implements generic Matrix object and operations.
 */
class Matrix
{
  public:
    /**
     * Initializes matrix from already allocated array.
     * \param rows   matrix number of rows
     * \param cols   matrix number of columns.
     * \param data   array with matriz elements allocation with the following 
     *               distribution: [row0 row1 row2 ... rowN].
     */
    Matrix (uint8_t rows = 0, uint8_t cols = 0, real_t *data = NULL) { _nrows=rows; _ncols=cols; _data=data; }

    /**
     * Changes matrix size and if necessary data array.
     * \param rows   matrix number of rows.
     * \param cols   matrix number of columns.
     * \param data   array with matriz elements allocation with the following 
     *               distribution: [row0 row1 row2 ... rowN].
     */
    void refurbish (uint8_t rows, uint8_t cols, real_t *data = NULL) { _nrows=rows; _ncols=cols; if(data) _data=data; }

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
    real_t *data () {return _data;};
    
    /**
     * Copies n rows from matrix A into this.
     * \param A     matrix to copy rows from.
     * \param row   first row to copy into this.
     * \param nrows number of rows to copy.
     * \return  true if successful, false otherwise.
     */  
    bool fillRows (Matrix &A, uint8_t row=0, uint8_t nrows=1);

    /**
     * Copies n columns from matrix A into this.
     * \param A     matrix to copy columns from.
     * \param col   first column to copy into this.
     * \param ncols number of columns to copy.
     * \return  true if successful, false otherwise.
     */   
    bool fillCols (Matrix &A, uint8_t col=0, uint8_t ncols=1);
    
    /**
     * Makes zero all elements of matrix.
     * \return  true if successful, false otherwise.
     */ 
    bool zero () {memset(_data,0,sizeof(real_t)*_nrows*_ncols); return true;}

    /**
     * Makes identity matrix if matrix is square (nrows=ncols). Identity matrix 
     * elements are: 1 if row=col 0 otherwise.
     * \return  true if successful, false otherwise.
     */ 
    bool eye  ();

    /**
     * Forces matrix simmetry.
     * \return  true if successful, false otherwise.
     */ 
    bool simmetrize    ();
    
    /**
     * Forces matrix definite positiveness.
     * \return  true if successful, false otherwise.
     */ 
    bool forcePositive ();

    /**
     * Provides element in given row and col.
     * \param row  row the element is in.
     * \param col  column the element is in.
     * \return     value of matrix element in given row and col.
     */ 
    real_t & operator()(const uint8_t row, const uint8_t col);

    /**
     * Provides element in given row and col.
     * \param row row the element is in.
     * \param col column the element is in.
     * \return  value of matrix element in given row and col.
     */ 
    const real_t & operator()(const uint8_t row, const uint8_t col) const;

    /**
     * Provides element in given array position.
     * \param i column the element is in.
     * \return  value of matrix element in given row and col.
     */ 
    real_t & operator[](int i);
    const real_t & operator[](int i) const;
    Matrix & operator+=(const Matrix &A);
    Matrix & operator-=(const Matrix &A);
    Matrix & operator*=(const real_t &n);

    bool copy         (const Matrix &A, uint8_t startrow=0, uint8_t startcol=0);
    bool add          (const Matrix &A, uint8_t startrow=0, uint8_t startcol=0);
    bool substract    (const Matrix &A, uint8_t startrow=0, uint8_t startcol=0);
    bool scale        (const real_t &n);
    bool transpose    ();
    bool inverse      ();
    bool cholesky     (bool zero=true);
    bool cholupdate   (Matrix &v, int sign);
    bool cholrestore  (bool zero=true);
    bool cholinverse  ();
    real_t norm ();
    
//    static bool copy         (Matrix &Dest, const Matrix &Orig, uint8_t startrow=0, uint8_t startcol=0); // copy Orig into Dest
    static bool add          (const Matrix &A, const Matrix &B, Matrix &R);
    static bool substract    (const Matrix &A, const Matrix &B, Matrix &R);
    static bool multiply     (const Matrix &A, const Matrix &B, Matrix &R);
    static bool multiplyDiag (const Matrix &A, const  Matrix &B, Matrix &R);
    static bool scale        (const real_t &n, const Matrix &A, Matrix &R);
    static bool transpose    (const Matrix &A, Matrix &R);
    static bool divide       (const Matrix &A, Matrix &B, Matrix &R);
    static bool cholesky     (const Matrix &A, Matrix &L);
    static bool ldl          (const Matrix &A, Matrix &L, Matrix &d);
    static bool inverse      (Matrix &A, Matrix &R);
    static bool cholinverse  (const Matrix &L, Matrix &R);
    static bool qr           (const Matrix &A, Matrix &Q, Matrix &R);
    
    void print ();
    
  private:
    uint8_t _nrows;
    uint8_t _ncols;
    real_t *_data;

};

}

#endif // B_MATRIX_H 
