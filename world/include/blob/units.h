/********* blob robotics 2014 *********
 *  title: units.h
 *  brief: Unit converter
 * author: adrian jimenez-gonzalez
 * e-mail: blob.robotics@gmail.com
 **************************************/

#ifndef B_UNITS_H
#define B_UNITS_H

#include "blob/math.h"

namespace blob {

class Units
{
  public:
    template <class T> static const T degToRad (const T& deg) { return (T)deg*pi/180.f;}
    template <class T> static const T knotToKmh (const T& knot) { return knot;} // FIXME
    template <class T> static const T kmhToMps (const T& kmh) { return kmh;} // FIXME
};

}

#endif // B_UNITS_H
