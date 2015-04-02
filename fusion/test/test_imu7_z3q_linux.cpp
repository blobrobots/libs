/********* blob robotics 2014 *********
 *  title: test_linux.cpp
 *  brief: test for ukf library (linux)
 * author: adrian jimenez-gonzalez
 * e-mail: blob.robotics@gmail.com
 /*************************************/
#include <iostream>
#include <fstream>
#include <math.h>

#include "blob/ukf.h"

#define N   7   // Number of states

#define T 0.01
#define Tacc 0.02
#define Tmag 0.05

#define qq   0
#define qbg  0.0001
#define racc 0.1
#define rmag 0.25

#define qq_T_2   (qq*qq*T*T)
#define qbg_T_2  (qq*qq*T*T)
#define racc_T_2 (racc*racc*Tacc*Tacc)
#define rmag_T_2 (rmag*rmag*Tmag*Tmag)

typedef struct {
  real_t u[3];
  real_t dt;
} fargs_t;

typedef struct {
  real_t dt;
} hargs_t;

void f(real_t *x, void *args, real_t * res)
{
  // x = [q0, q1, q2, q3, gbx, gby, gbz]
  // u = [gx, gy, gz]

  fargs_t *fargs = (fargs_t *)args;
  
  real_t q0 = x[0], q1 = x[1], q2 = x[2], q3 = x[3], gbx = x[4], gby = x[5], gbz = x[6];

  real_t gx = fargs->u[0] - gbx; 
  real_t gy = fargs->u[1] - gby; 
  real_t gz = fargs->u[2] - gbz;

  // predict new state (FRD)

  res[0] = q0 + (-q1*gx - q2*gy - q3*gz)*fargs->dt/2;
  res[1] = q1 + ( q0*gx + q3*gy - q2*gz)*fargs->dt/2;
  res[2] = q2 + (-q3*gx + q0*gy + q1*gz)*fargs->dt/2;
  res[3] = q3 + ( q2*gx - q1*gy + q0*gz)*fargs->dt/2;
  res[4] = gbx;
  res[5] = gby;
  res[6] = gbz;

  // re-normalize quaternion
  real_t qnorm = sqrt(res[0]*res[0] + res[1]*res[1] + res[2]*res[2] + res[3]*res[3]);
  res[0] = res[0]/qnorm;
  res[1] = res[1]/qnorm;
  res[2] = res[2]/qnorm;
  res[3] = res[3]/qnorm;

}

void ha(real_t *x, void *args, real_t * res)
{
  // x = [q0, q1, q2, q3, gbx, gby, gbx]
  // z = [ax, ay, az]

  real_t q0 = x[0], q1 = x[1], q2 = x[2], q3 = x[3];

  // estimated direction of gravity (NED)
  res[0] =  2*(q0*q2 - q1*q3);             // ax
  res[1] = -2*(q0*q1 + q2*q3);             // ay
  res[2] = -q0*q0 + q1*q1 + q2*q2 - q3*q3; // az
}

void hm(real_t *x, void *args, real_t * res)
{
  // x = [q0, q1, q2, q3, gbx, gby, gbx]
  // z = [mx, my, mz]

  real_t q0 = x[0], q1 = x[1], q2 = x[2], q3 = x[3];

  // estimated direction of flux (NED)
  res[0] = q0*q0 + q1*q1 - q2*q2 - q3*q3; // mx
  res[1] = 2*(q1*q2 - q0*q3);             // my
  res[2] = 2*(q0*q2 + q1*q3);             // mz

}

// covariance of process
real_t  q[] = { qq_T_2,  0,   0,   0,    0,    0,    0,      
                 0, qq_T_2,  0,   0,    0,    0,    0,     
                 0,   0, qq_T_2,  0,    0,    0,    0,     
                 0,   0,   0, qq_T_2,   0,    0,    0,    
                 0,   0,   0,   0, qbg_T_2,   0,    0,    
                 0,   0,   0,   0,    0, qbg_T_2,   0,    
                 0,   0,   0,   0,    0,    0, qbg_T_2 };

// covariance of measurement
real_t ra[] = { racc_T_2,    0,     0,
                   0, racc_T_2,    0,
                   0,     0, racc_T_2 };

// covariance of measurement
real_t rm[] = { rmag_T_2,    0,     0, 
                   0, rmag_T_2,    0,
                   0,     0, rmag_T_2 };

int main(int argc, char* argv[])
{
  bool result = true;
  real_t za[3] = {0,  0, -1};
  real_t zm[3] = {0,  0,  0};
  real_t x[N]  = {1,  0,  0,  0,  0,  0,  0};

  if(argc == 3)
  {
    std::ifstream input_file (argv[1]);
    std::ofstream output_file (argv[2]);

    if (input_file.is_open())
    {
      if (output_file.is_open())
      {
        std::string line;
        
        real_t ta = 0, tm = 0;

        blob::UKF ukf(N, x);
        
        fargs_t fargs;

        while ( getline (input_file,line) )
        {
          if((line[0] == '-') || ((line[0] >= '0')&&(line[0] <='9')))
          {
            real_t gx, gy, gz, ax, ay, az, mx, my, mz, anorm, mnorm, roll, pitch, yaw;

            sscanf(line.c_str(),"%lf %lf %lf %lf %lf %lf %lf %lf %lf"
                               , &gx, &gy, &gz, &ax, &ay, &az, &mx, &my, &mz);
            ta += T; 
            tm += T;

            // normalise measurements
            anorm = sqrt(ax*ax + ay*ay + az*az);
            if (anorm > 0)
            {
              ax = ax/anorm;
              ay = ay/anorm;
              az = az/anorm;
            }

            mnorm = sqrt(mx*mx + my*my + mz*mz);
            if (mnorm > 0)
            {
              mx = mx/mnorm;
              my = my/mnorm;
              mz = mz/mnorm;
            }

            fargs.u[0] = gx;
            fargs.u[1] = gy;
            fargs.u[2] = gz;
            fargs.dt = T;

#if defined(__DEBUG__) & defined(__linux__)
            std::cout << "[test] - predicting over u=[" << fargs.u[0] << ", " << fargs.u[1] << 
                         ", " << fargs.u[2] << "], dt=" << fargs.dt << std::endl;
#endif
            result &= ukf.predict(&f, &fargs, q);

            if ((result==true)&&(ta>=Tacc))
            {
              za[0] = ax; za[1] = ay; za[2] = az;
#if defined(__DEBUG__) & defined(__linux__)
            std::cout << "[test] - updating over za=[" << za[0] << ", " << za[1] << 
                         ", " << za[2] << "], dta=" << ta << std::endl;
#endif
              result &= ukf.update (&ha, NULL, 3, za, ra);
              ta = 0;
            }
            if ((result==true)&&(tm >= Tmag))
            {
              zm[0] = mx; zm[1] = my; zm[2] = mz;
#if defined(__DEBUG__) & defined(__linux__)
            std::cout << "[test] - updating over zm=[" << zm[0] << ", " << zm[1] << 
                         ", " << zm[2] << "], dtm=" << tm << std::endl;
#endif
              result &= ukf.update (&hm, NULL, 3, zm, rm);
              tm = 0;
            }

            // re-normalize quaternion
            real_t *q = ukf.getState();
            real_t qnorm = sqrtf(q[0]*q[0] + q[1]*q[1] + q[2]*q[2] + q[3]*q[3]);
            q[0] = q[0]/qnorm;
            q[1] = q[1]/qnorm;
            q[2] = q[2]/qnorm;
            q[3] = q[3]/qnorm;

            roll = atan2(2*(q[0]*q[1] + q[2]*q[3]), 1 - 2*(q[1]*q[1] + q[2]*q[2]));
            pitch = asin(2*(q[0]*q[2] - q[1]*q[3]));
            yaw  = atan2(2*(q[0]*q[3] + q[1]*q[2]), 1 - 2*(q[2]*q[2] + q[3]*q[3]));
            
#if defined(__DEBUG__) & defined(__linux__)
            std::cout << "[test] - got state x=[";
#endif
            for(int i = 0; i < ukf.getNumStates(); i++)
            {
              output_file << ukf.getState(i) << " ";

#if defined(__DEBUG__) & defined(__linux__)
              std::cout << ukf.getState(i) << " ";
#endif
            }

            output_file << roll << " " << pitch << " " << yaw << std::endl; 
#if defined(__DEBUG__) & defined(__linux__)
            std::cout << "] rpy=[" << roll << ", " << pitch << ", " << yaw << "]" << std::endl;;
#endif
          }
          if (result == false)
            return -1;
        }
        input_file.close();
        output_file.close();
      }
      else 
        std::cerr << "[test] - file i/o error: unable to open file " << argv[2] << std::endl;
    }
    else 
      std::cerr << "[test] - file i/o error: unable to open file " << argv[1] << std::endl;
  }
  else
    std::cerr << "[test] - usage: ./test input_file output_file" << std::endl;
  
  return 0;
}
