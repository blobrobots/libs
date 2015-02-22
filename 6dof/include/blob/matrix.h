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
    Matrix (uint8_t rows = 0, uint8_t cols = 0, float *data = NULL) { _nrows=rows; _ncols=cols; _data=data; }
    uint8_t nrows () { return _nrows; }
    uint8_t ncols () { return _ncols; }
    uint16_t length () { return (uint16_t)_ncols*_nrows; }

    bool get (uint8_t row, uint8_t col, float &val);
    bool set (uint8_t row, uint8_t col, float val);
    
    float *data () {return _data;};
    
    bool fillRows (Matrix &A, uint8_t row=0, uint8_t nrows=1);
    bool fillCols (Matrix &A, uint8_t col=0, uint8_t ncols=1);
    bool zero () {memset(_data,0,sizeof(float)*_nrows*_ncols); return true;}
    bool eye  ();

    float & operator[](int i) {return _data[i];}

    const float & operator[](int i) const {return _data[i];}
    
    bool scale (const float &n);
    
    static bool add        (Matrix &A, Matrix &B, Matrix &R);
    static bool substract  (Matrix &A, Matrix &B, Matrix &R);
    static bool multiply   (Matrix &A, Matrix &B, Matrix &R);
    static bool transpose  (Matrix &A, Matrix &R);
    static bool inverse    (Matrix &A);
    static bool inverseLow (Matrix &L);
    static bool cholesky   (Matrix &A, Matrix &L);
    static bool cholesky   (Matrix &A, bool zero=true);

    void print ();
    
  private:
    uint8_t _nrows;
    uint8_t _ncols;
    float *_data;
};

}

#endif // B_MATRIX_H 
