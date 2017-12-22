/********* blob robotics 2014 *********
 *  title: world.h
 *  brief: Physics methods and constants
 * author: adrian jimenez-gonzalez
 * e-mail: blob.robotics@gmail.com
 **************************************/

#ifndef B_WORLD_H
#define B_WORLD_H

namespace blob {

const real_t gravity (9.80665);     //! m/s^2
const real_t earthRadius (6371000); //! m
/*
 * Reference system
 */
class RefSys
{
  public:
   
    enum {FLU, FRD, ENU, NED, LLA, ECEF, NUM_REFSYS};
  
    RefSys(const uint8_t & sys=FLU) {_system=sys;}

    uint8_t getType () {return _system;}
    
    virtual bool toSystem (const uint8_t & sys) {_system = sys; return true;}

    virtual bool toSystem (const std::string & sys) {_system = fromString(sys); 
                                               return true;}
    
    static std::string toString (const RefSys & rs)
    static RefSys fromString (const std::string & rs);

  protected:
    RefSys * _parent;
    Vector3d <real_t> origin;
    Vector3d <real_t> rotation;
 
    uint8_t _system;
    static const char *strSys[NUM_SYSREF] = {"FLU","FRD","ENU","NED","LLA","ECEF"};
};


template <typename T>
class Vector3d : public Vector3d<T>, public RefSys
{
  public:
    Pos3d (const uint8_t & sys=RefSys::FLU);
};

template <typename T>
class Pos6dof : public RefSys
{

  public:
    Pos6dof (const uint8_t & sys=RefSys::FLU);

};

template <typename T>
class Vel6dof : public Vector3d<T>, public RefSys
{

  public:
    Vel3d (const uint8_t & sys=RefSys::FLU);

};

template <typename T>
class Acc6dof : public Vector3d<T>, public RefSys
{
  public:
    Acc6dof (const uint8_t & sys=RefSys::FLU);

  Vector3d<T> lin;  
  Vector3d<T> ang;

};

}

#endif // B_WORLD_H

