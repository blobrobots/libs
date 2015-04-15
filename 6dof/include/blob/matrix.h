/********* blob robotics 2014 *********
 *  title: matrix.h
 *  brief: interface for matrix ops.
 * author: adrian jimenez-gonzalez
 * e-mail: blob.robotics@gmail.com
 **************************************/
#ifndef B_MATRIX_H
#define B_MATRIX_H

#include <blob/types.h>

#if defined(__linux__)  
#include <string.h>
#endif

namespace blob {

class Matrix
{
  public:
    Matrix (uint8_t rows = 0, uint8_t cols = 0, real_t *data = NULL) { _nrows=rows; _ncols=cols; _data=data; }
    void refurbish (uint8_t rows = 0, uint8_t cols = 0, real_t *data = NULL) { _nrows=rows; _ncols=cols; if(data) _data=data; }
    
    uint8_t nrows () const { return _nrows; }
    uint8_t ncols () const { return _ncols; }
    uint16_t length () const { return (uint16_t)_ncols*_nrows; }
 
    real_t *data () {return _data;};
    
    bool fillRows (Matrix &A, uint8_t row=0, uint8_t nrows=1);
    bool fillCols (Matrix &A, uint8_t col=0, uint8_t ncols=1);
    
    bool zero () {memset(_data,0,sizeof(real_t)*_nrows*_ncols); return true;}
    bool eye  ();

    real_t & operator()(const uint8_t row, const uint8_t col);
    const real_t & operator()(const uint8_t row, const uint8_t col) const;
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
    
//    static bool copy         (Matrix &Dest, const Matrix &Orig, uint8_t startrow=0, uint8_t startcol=0); // copy Orig into Dest
    static bool add          (const Matrix &A, const Matrix &B, Matrix &R);
    static bool substract    (const Matrix &A, const Matrix &B, Matrix &R);
    static bool multiply     (const Matrix &A, const Matrix &B, Matrix &R);
    static bool multiplyDiag (const Matrix &A, const  Matrix &B, Matrix &R);
    static bool scale        (const real_t &n, const Matrix &A, Matrix &R);
    static bool transpose    (const Matrix &A, Matrix &R);
    static bool divide       (const Matrix &A, Matrix &B, Matrix &R);
    static bool cholesky     (const Matrix &A, Matrix &L);
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
