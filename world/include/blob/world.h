/********* blob robotics 2014 *********
 *  title: physics.h
 *  brief: Physics methods and constants
 * author: adrian jimenez-gonzalez
 * e-mail: blob.robotics@gmail.com
 **************************************/

#ifndef B_PHYSICS_H
#define B_PHYSICS_H

namespace blob {

const float gravity (9.80665f); // m/s^2
const float earth_radius (6371000.f); // m

class RefSys
{ 
  public:
    enum {FLU, FRD, ENU, NED, LLA, NUM_SYSREF};
  
    RefSys(const uint8_t & sys=FLU) {_system=sys;}

    uint8_t referenceSystemGet () {return _system;}
    
    virtual bool to (const uint8_t & sys) {_system = sys; return true;}

    virtual bool to (const std::string & sys) {_system = fromString(sys); 
                                               return true;}
    
    static std::string toString (const RefSys & rs)
    static RefSys fromString (const std::string & rs);

  protected:
    uint8_t _system;
    static const char *strSys[NUM_SYSREF] = {"FLU","FRD","ENU","NED","LLA"};
};


template <typename T>
class Position3d : public Vector3d<T>, public RefSys
{

  public:
    Position3d (const uint8_t & sys=RefSys::FLU);

};

}

#endif // B_PHYSICS_H

