/********* blob robotics 2014 *********
 *  title: b6dof3d.h
 *  brief: 6dof classes and functions
 * author: adrian jimenez-gonzalez
 * e-mail: blob.robotics@gmail.com
 **************************************/

#ifndef B_6DOF_H
#define B_6DOF_H

namespace blob {
  
enum RefSystem { FLU, FRD, ENU, NED };

template <typename T>
class Position3d : public Vector3d<T>
{

public:
   Position3d(RefSystem rs = FLU) {refSystem = rs;}
   void toFLU(); // Rotate position to FLU system
   void toFRD(); // Rotate position to FRD system
   void toENU(); // Rotate position to ENU system
   void toNED(); // Rotate position to NED system

   Position3d inFLU(); // Return position in FLU system
   Position3d inFRD(); // Return position in FLU system
   Position3d inENU(); // Return position in FLU system
   Position3d inNED(); // Return position in FLU system

   RefSystem refSystem;

};

template <typename T>
class Pose3d
{
   Vector3d<T> position;
   Vector3d<T> orientation;

public:
   Pose3d(RefSystem rs = FLU) {refSystem = rs;}
   void toFLU(); // Rotate position to FLU system
   void toFRD(); // Rotate position to FRD system
   void toENU(); // Rotate position to ENU system
   void toNED(); // Rotate position to NED system

   Pose3d inFLU(); // Return position in FLU system
   Pose3d inFRD(); // Return position in FLU system
   Pose3d inENU(); // Return position in FLU system
   Pose3d inNED(); // Return position in FLU system

   RefSystem refSystem;

};

}
#endif // B_6DOF_H
