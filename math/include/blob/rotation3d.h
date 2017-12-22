/********* blob robotics 2014 *********
 *  title: units.h
 *  brief: Unit converter
 * author: adrian jimenez-gonzalez
 * e-mail: blob.robotics@gmail.com
 **************************************/

#ifndef B_UNITS_H
#define B_UNITS_H

#include "blob/matrix.h"

namespace blob {

/**
 * Implements 3-dimensional rotation
 */
class Rotation3d : public Matrix {

public:
  /*
   * Constructor
   */  
  Rotation3d () : Matrix (3,3,_data) {}
  /*
   * Constructor from other rotation object
   * \param Rotation  
   */  
  Rotation3d (const Rotation3d& rotation);
  /*
   * Constructor from a rotation matrix object
   * \param Matrix  
   */  
  Rotation3d (const Matrix& matrix);

  /*
   * Constructor from a rotation matrix object
   * \param Matrix  
   */  
  Rotation3d (const Vector3d & euler) {
    _nrows=3; _ncols=3;    

    real_t cR = Math.cos(e[0]),  sR = Math.sin(e[0]),
           cP = Math.cos(e[1]),  sP = Math.sin(e[1]),
           cY = Math.cos(e[2]),  sY = Math.sin(e[2]);

    _data[0] = cP*cY;          _data[1] = cP*sY;          _data[2] = -sP;
    _data[3] = sR*sP*cY-cR*sY; _data[4] = sR*sP*sY+cR*cY; _data[5] = sR*cP;
    _data[6] = cR*sP*cY+sR*sY; _data[7] = cR*sP*sY-sR*cY; _data[8] = cR*cP;
  }

  const Matrix & matrix();


  const Vector<real_t>& euler();

  bool rotate (const Rotation3d& rotation) {
    real_t data[9];
    Matrix current(3,3,data);
    current.copy(*this);   
    multiply(rotation,current);
  }

  bool derotate (const Rotation3d& rotation) {
    return rotate(rotation.transpose());
  }

  bool getEuler(Vector3d<real_t>) {
  }
  
  //FIXME: Pending quaternions
protected:

  /*
   * Provides rotation matrix equivalent to given euler angles
   * \param e  euler angles array (roll, pitch, yaw)
   * \return  rotation matrix equivalent to given euler angles
   */  
  void fromEuler (Vector3d<real_t> euler, );

  /*
   * Provides euler angles equivalent to given rotation matrix
   * \param mat  rotation matrix
   * \return  euler angles array (roll, pitch, yaw) equivalent to given rotation matrix
   */
  void matToEuler (Vector3d<real_t> & euler);
  

  real_t _data[] = {1,0,0,
                    0,1,0,
                    0,0,1};  
  
  private double [] mat2euler (double [][] mat) {
    double [] e = { Math.atan2( mat[1][2],mat[2][2]),
                    Math.atan2(-mat[0][2],Math.sqrt(mat[0][0]*mat[0][0]+mat[0][1]*mat[0][1])),
                    Math.atan2( mat[0][1],mat[0][0]) };
    return e;
  }
  
  /**
   * Constructor from euler angles
   * @param e  euler angles array
   */
  public Rotation3d(double [] e) {
    matrix=euler2mat(e);
  }
  
  /**
   * Provides matrix reference
   * @return matrix reference
   */
  public double [][] matrix() {
    return matrix;
  };
  
  /**
   * Provides transposed rotation matrix
   * @return  transposed Matrix 
   */
  public double [][] matrixT() {
    return transpose(matrix);
  };
    
  /**
   * Provides rotation represented as euler angles 
   * @return  euler angles array (roll, pitch, yaw)
   */
  public double [] euler() {
    return mat2euler(matrix);
  };
  
  /**
   * Provides transposed rotation represented as euler angles 
   * @return  transposed rotation euler angles array (roll, pitch, yaw)
   */
  public double [] eulerT() {
    return mat2euler(transpose(matrix));
  };
  
  /**
   * Combines this rotation with a given rotation represented as rotation matrix
   * @param  rotation represented by rotation matrix
   * @return  true if successful, false otherwise
   */
  public boolean rotate(double [][] rotation) {
    if(rotation==null)
    	return false;
    matrix = multiply(rotation,matrix);
    return true;
  }

  /**
   * Combines this rotation with a given rotation represented as euler angles
   * @param e  euler angles array (roll, pitch, yaw)
   * @return  true if successful, false otherwise 
   */
  public boolean rotate(double [] e) {
    if(e==null)
      return false;
    rotate(euler2mat(e));
    
    return true;
  }

  /**
   * Combines this rotation with the inverse rotation (multiply by transposed)
   * represented by given euler angles
   * @param rotation  rotation matrix representing the inverse rotation to the 
   *                  one to be applied
   * @return  true if successful, false otherwise  
   */
  public boolean derotate(double [][] rotation) {
    return rotate(transpose(rotation));
  }
  
  /**
   * Combines this rotation with the inverse rotation (multiply by transposed) 
   * of a given rotation matrix
   * @param e  euler angles representing the inverse rotation to the one to be 
   *           applied
   * @return  true if successful, false otherwise  
   */
  public boolean derotate(double [] e) {
    return rotate(transpose(euler2mat(e)));
  }
  
  /*
   * Provides rotation matrix equivalent to given euler angles
   * @param e  euler angles array (roll, pitch, yaw)
   * @return  rotation matrix equivalent to given euler angles
   */
  private double [][] euler2mat (double [] e) {
    double cR = Math.cos(e[0]),  sR = Math.sin(e[0]),
           cP = Math.cos(e[1]),  sP = Math.sin(e[1]),
           cY = Math.cos(e[2]),  sY = Math.sin(e[2]);
    double [][] mat = {{cP*cY,          cP*sY,         -sP   },
                       {sR*sP*cY-cR*sY, sR*sP*sY+cR*cY, sR*cP},
                       {cR*sP*cY+sR*sY, cR*sP*sY-sR*cY, cR*cP}};
   
    return mat;
  }
  
  /*
   * Provides euler angles equivalent to given rotation matrix
   * @param mat  rotation matrix
   * @return  euler angles array (roll, pitch, yaw) equivalent to given rotation matrix
   */
  private double [] mat2euler (double [][] mat) {
    double [] e = { Math.atan2( mat[1][2],mat[2][2]),
                    Math.atan2(-mat[0][2],Math.sqrt(mat[0][0]*mat[0][0]+mat[0][1]*mat[0][1])),
                    Math.atan2( mat[0][1],mat[0][0]) };
    return e;
  }

  /*
   * Calculates transposed matrix from given 3x3 matrix
   * @param R  matrix to be transposed
   * @return  transposed matrix
   */
  private double [][] transpose (double [][]R) {
    double [][] Rt={{0,0,0},{0,0,0},{0,0,0}};
  
    for (int i=0; i<3; i++)
      for (int j=0; j<3 ; j++)
        Rt[i][j] = R[j][i];
    return Rt;
  }
  
  /*
   * Multiplies two 3x3 matrices
   * @param A  left-side matrix 
   * @param B  right-side matrix 
   * @return  matrix resulting from A*B
   */
  private double [][] multiply (double [][] A, double [][] B)
  {
    double [][] R={{0,0,0},{0,0,0},{0,0,0}};
    
    for(int i=0; i<3; i++)
      for(int j=0; j<3; j++)
        for(int k=0;k<3;k++)
          R[i][j] += A[i][k]*B[k][j];
    
    return R;
  }
}

}

#endif // B_UNITS_H
