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
 * \file       vector.h
 * \brief      interface for generic vector (column matrix) and 3d vector
 * \author     adrian jimenez-gonzalez (blob.robots@gmail.com)
 * \copyright  the MIT License Copyright (c) 2017 Blob Robots.
 *
 ******************************************************************************/

#ifndef B_VECTOR_H
#define B_VECTOR_H

#include <blob/types.h>
#include <blob/matrix.h>

namespace blob {

/**
 * Implements generic vector object and operations.
 */
template <typename T> 
class Vector : public Matrix<T>
{
  public:
    /**
     * Initializes vector (column matrix) from already allocated array.
     * \param n  vector number of elements
     * \param data  array with vector elements allocation
     */
    Vector<T> (uint8_t n, T * data=NULL) : Matrix<T> (n,1,data) {};
    /**
     * Dot product operator
     * \param v   vector to calculate dot product with
     * \return  dot product between this vector and v 
     */
    T operator *(const Vector<T> & v) const //FIXME: exception if lengths differ
    {
      if(this->length()!=v.length())
        return 0;
      T sum=0;
      for(int i=0;i<this->length();i++)
        sum+=this->_data[i]*v[i];  
      return sum;
    }
    /**
     * Provides the angle between this vector and another vector
     * \param v  vector to calculate angle with
     * \return  angle between vectors 
     */
    real_t angle(const Vector<T> & v) const
    {
      return math::acos(((*this)*v)/(this->norm()*v.norm()));
    }
    /**
     * Projects this vector onto v
     * \param v  vector to project onto
     */
    void project(const Vector<T> & v)
    {
      if(this->length()!=v.length())
        return;                                     
      T n=((*this)*v)/(v*v);
      for(int i=0;i<this->length();i++)
        *this[i]=v[i]*n;
    }
    /**
     * Checks if vector remains as a column matrix or has been transposed
     * \return true if transposeed (row matrix), false otherwise
     */
    bool isTransposed()
    {
      return(this->cols()>1);
    }

};

/**
 * Implements real vector object and operations.
 */
template <typename T> 
class Vector3 : public Vector<T>
{
  public:
    /*
     * Constructor
     */
    Vector3<T> () : Vector<T>(3,_vector) {};
    /*
     * Constructor
     * \param data  vector to copy construct from
     */
    Vector3<T> (const Vector3<T> & v) : Vector<T> (3,_vector) 
    {
      memcpy(_vector,v.data(),sizeof(T)*this->length());
    }
    /*
     * Constructor
     * \param data  array to copy construct from
     */
    Vector3<T> (T * data) : Vector<T>(3,_vector) 
    {
      memcpy(_vector,data,sizeof(T)*this->length());
    }
    /*
     * Constructor
     * \param x0  first element value
     * \param y0  second element value
     * \param z0  third element value
     */
    Vector3<T> (T x0, T y0, T z0) : Vector<T> (3,_vector)
    {
      *this[0]=x0;
      *this[1]=y0;
      *this[2]=z0;
    }
    /*
     * Get first element (x) value 
     * \return  first element (x) value
     */
    T x () {return *this[0];}
    /*
     * Get second element (y) value 
     * \return  second element (y) value
     */
    T y () {return *this[1];}
    /*
     * Get first element (z) value 
     * \return  first element (z) value
     */
    T z () {return *this[2];}
    /*
     * Set first element (x) value
     * \param value  first element (x) value
     */
    void x (T value) {*this[0]=value;}
    /*
     * Set second element (y) value
     * \param value  second element (y) value
     */
    void y (T value) {*this[1]=value;}
    /*
     * Set first element (z) value
     * \param value  first element (z) value
     */
    void z (T value) {*this[2]=value;}
    /**
     * Negative operator
     * \return  Negative vector
     */
    Vector3<T> operator - (void) const
    {
      return blob::Vector3<T> (-*this[0],-*this[1],-*this[2]);
    }
    /**
     * Addition operator
     * \param v  Vector to add
     * \return addition resulting vector
     */
    Vector3<T> operator + (const Vector3<T> & v) const
    {
      return blob::Vector3<T> (*this[0]+v[0], *this[1]+v[1], *this[2]+v[2]);
    }
    /**
     * Subtraction operator
     * \param v  Vector to substract
     * \return subtraction resulting vector
     */
    Vector3<T> operator - (const Vector3<T> & v) const
    {
      return blob::Vector3<T> (*this[0]-v[0], *this[1]-v[1], *this[2]-v[2]);
    }
    /**
     * Uniform scaling operator
     * \param n  Vector to substract
     * \return Resulting scaled vector
     */
    Vector3<T> operator * (const T & n)
    {
      return Vector3<T>(*this[0]*n, *this[1]*n, *this[2]*n);
    }
    /**
     * Uniform scaling operator
     * \param n  Vector to substract
     * \return Resulting scaled vector
     */
    Vector3<T> operator / (const T & n)
    {
      return Vector3<T>(*this[0]/n, *this[1]/n, *this[2]/n);
    }
    /**
     * Elementwise multiplication operator
     * \param v  Vector to elementwise multiply with
     * \return elementwise multiplication resulting vector
     */
    Vector3<T> operator & (const Vector3<T> &v) const
    {
      return blob::Vector3<T>(*this[0]*v[0], *this[1]*v[1], *this[2]*v[2]);
    }
    /**
     * Cross product operator
     * \param v  Vector to perform cross product with
     * \return cross product
     */
    Vector3<T> operator % (const Vector3<T> &v) const
    {
      return Vector3<T>(*this[1]*v[2] - *this[2]*v[1], 
                        *this[2]*v[0] - *this[0]*v[2], 
                        *this[0]*v[1] - *this[1]*v[0]);
    }
    /**
     * Normalized vector
     * \return normalized copy of the vector
     */
    Vector3<T> normalized ()
    { // TODO: check
      //Vector3<T> copy(*this);
      //return copy.normalize();
      return Vector3<T>(*this).normalize();
    }
    /**
     * Provides this vector projection onto v 
     * \return this vector projected onto v
     */
    Vector3<T> projected (const Vector3<T> &v) const
    {
      return v*(*this*v)/(v*v);
    }
    /**
     * Reflects the vector about another vector v
     * \param v  vector to project onto
     */ 
    void reflect (const Vector3<T> & v)
    {
      Vector3<T> orig(*this);
      project(v);
      *this = *this*2 - orig;
    }
    /**
     * Rotate vector
     * \param rot Rotation matrix
     */
    void rotate (T rot) // FIXME: Rotation3d rot
    {
      return; // TODO
    }
    /**
     * Rotate vector
     * \param rx  rotation angle around X axis in radians
     * \param ry  rotation angle around Y axis in radians
     * \param rz  rotation angle around Z axis in radians 
     */
    void rotate (real_t rx, real_t ry, real_t rz)  // FIXME: Rotation3d rot
    {
      return; // TODO
    }

  protected:

    T _vector[3]; /**< 3-element array */
};

class Vector3R : public Vector3<real_t> {};

}

#endif // B_VECTOR_H
