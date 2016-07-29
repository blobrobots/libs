/********* blob robotics 2014 *********
 *  title: math.h
 *  brief: Math functions and constants
 * author: adrian jimenez-gonzalez
 * e-mail: blob.robotics@gmail.com
 **************************************/

#ifndef B_MATH_H
#define B_MATH_H

#include <blob/types.h>
#include <blob/matrix.h>

namespace blob {

template <typename T>
class Vector3d
{

  public:
    T x, y, z;

    // trivial ctor
    Vector3d<T>() {
        x = y = z = 0;
    }

    // setting ctor
    Vector3d<T>(const T x0, const T y0, const T z0) : x(x0), y(y0), z(z0) {
    }

    // function call operator
    void operator ()(const T x0, const T y0, const T z0)
    {
        x= x0; y= y0; z= z0;
    }

    // test for equality
    bool operator ==(const Vector3d<T> &v) const
    {
      return (x==v.x && y==v.y && z==v.z);
    }
    // test for inequality
    bool operator !=(const Vector3d<T> &v) const
    {
      return (x!=v.x && y!=v.y && z!=v.z);
    }
    // negation
    Vector3d<T> operator -(void) const
    {
      return blob::Vector3d<T>(-x,-y,-z);
    }
    
    // addition
    Vector3d<T> operator +(const Vector3d<T> &v) const
    {
      return blob::Vector3d<T>(x+v.x, y+v.y, z+v.z);
    }
    // subtraction
    Vector3d<T> operator -(const Vector3d<T> &v) const
    {
      return blob::Vector3d<T>(x-v.x, y-v.y, z-v.z);
    }
    // uniform scaling
    Vector3d<T> operator *(const T num) const
    {
      return blob::Vector3d<T>(x*num, y*num, z*num);
    }
    // uniform scaling
    Vector3d<T> operator  /(const T num) const
    {
      return Vector3d<T>(x/num, y/num, z/num);
    }
    
    // addition
    Vector3d<T> &operator +=(const Vector3d<T> &v)
    {
      x += v.x; y += v.y; z += v.z;
      return *this;
    }
    // subtraction
    Vector3d<T> &operator -=(const Vector3d<T> &v)
    {
      x -= v.x; y -= v.y; z -= v.z;
      return *this;
    }
    // uniform scaling
    Vector3d<T> &operator *=(const T num)
    {
      x*=num; y*=num; z*=num;
      return *this;
    }
    
    // elementwise multiplication
    Vector3d<T> &operator &=(const Vector3d<T> &v)
    {
      x*=v.x; y*=v.y; z*=v.z;
      return *this;
    }

    // uniform scaling
    Vector3d<T> &operator /=(const T num)
    {
      x /= num; y /= num; z /= num;
      return *this;
    }
    
    // cast to float
    operator Vector3d<float>() 
    {
      return Vector3d<float> ((float)x, (float)y, (float)z);
    }

    // allow a Vector3d to be used as an array, 0 indexed
    T & operator[](int i) {
        T *_v = &x;
        return _v[i];
    }

    const T & operator[](int i) const {
        const T *_v = &x;
        return _v[i];
    }

    // dot product
    T operator *(const Vector3d<T> &v) const
    {
      return x*v.x + y*v.y + z*v.z;
    }
    
    // elementwise multiplication
    Vector3d<T> operator &(const Vector3d<T> &v) const
    {
      return Vector3d<T>(x*v.x, y*v.y, z*v.z);
    }

    // cross product
    Vector3d<T> operator %(const Vector3d<T> &v) const
    {
      return Vector3d<T>(y*v.z - z*v.y, z*v.x - x*v.z, x*v.y - y*v.x);
    }

    // computes the angle between this vector and another vector
    float angle(const Vector3d<T> &v2) const
    {
      return acos(((*this)*v2) / (this->length()*v2.length()));
    }

    // check if any elements are NAN
    bool is_nan(void) const
    {
      return isnan(x) || isnan(y) || isnan(z);
    }
    // check if any elements are infinity
    bool is_inf(void) const
    {
      return isinf(x) || isinf(y) || isinf(z);
    }

    // check if all elements are zero
    bool is_zero(void) const { return x == 0 && y == 0 && z == 0; }

    // rotate
    void rotate(T rx, T ry, T rz)
    {
      return; // TODO
    }

    // gets the length of this vector squared
    T  length_squared() const
    {
        return (T)(length()*length());
    }

    // gets the length of this vector
    float length(void) const
    {
      return sqrt(x*x+y*y+z*z);
    }

    // normalizes this vector
    void normalize()
    {
        *this /= length();
    }

    // zero the vector
    void zero()
    {
        x = y = z = 0;
    }

    // returns the normalized version of this vector
    Vector3d<T> normalized() const
    {
        return *this/length();
    }

    // reflects this vector about n
    void  reflect(const Vector3d<T> &n)
    {
        Vector3d<T>        orig(*this);
        project(n);
        *this = *this*2 - orig;
    }

    // projects this vector onto v
    void project(const Vector3d<T> &v)
    {
        *this= v * (*this * v)/(v*v);
    }

    // returns this vector projected onto v
    Vector3d<T> projected(const Vector3d<T> &v) const
    {
        return v * (*this * v)/(v*v);
    }
};
    // uniform scaling
    template <typename T>
    Vector3d<T> operator *(const T num, Vector3d<T> &r)
    {
      return Vector3d<T>(r.x*num, r.y*num, r.z*num);
    }
}
#endif // B_BMATHD_H