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
 * \file       test_cf_imu7z3q_linux.cpp
 * \brief      test for cf library to estimate attitude and orientation (linux)
 * \author     adrian jimenez-gonzalez (blob.robots@gmail.com)
 * \copyright  the MIT License Copyright (c) 2015 Blob Robots.
 *
 ******************************************************************************/

#include <iostream>
#include <sstream> 
#include <fstream>
#include <math.h>

#include <blob/math.h>
#include <blob/cf.h>

#define N   7   // Length of state vector
#define L   3   // Length of control input

// sample times
#define T 0.01
#define Tacc 0.02
#define Tmag 0.05

// gains
real_t kp_acc[] = {0.01, 0.01, 0.01};
real_t kp_mag[] = {0.01, 0.01, 0.01};
real_t ki[] = {0.0001, 0.0001, 0.0001};

void f(const real_t& dt, real_t* u, real_t* x, real_t* res)
{
  // x = [q0, q1, q2, q3]
  // u = [gx, gy, gz]

  real_t q0 = x[0], q1 = x[1], q2 = x[2], q3 = x[3];

  real_t gx = u[0]; 
  real_t gy = u[1]; 
  real_t gz = u[2];

  // predict new state (FRD)
  res[0] = q0 + (-q1*gx - q2*gy - q3*gz)*dt/2;
  res[1] = q1 + ( q0*gx + q3*gy - q2*gz)*dt/2;
  res[2] = q2 + (-q3*gx + q0*gy + q1*gz)*dt/2;
  res[3] = q3 + ( q2*gx - q1*gy + q0*gz)*dt/2;

  // re-normalize quaternion
  real_t qnorm = blob::math::sqrt(res[0]*res[0] + res[1]*res[1] + res[2]*res[2] + res[3]*res[3]);
  res[0] = res[0]/qnorm;
  res[1] = res[1]/qnorm;
  res[2] = res[2]/qnorm;
  res[3] = res[3]/qnorm;

}

void ha(const real_t& dt, real_t* arg, real_t* x, real_t* res)
{
  // x = [q0, q1, q2, q3]
  // z = [ax, ay, az]
  // no args

  real_t q0 = x[0], q1 = x[1], q2 = x[2], q3 = x[3];

  // estimated direction of gravity (NED)
  res[0] =  2*(q0*q2 - q1*q3);             // ax
  res[1] = -2*(q0*q1 + q2*q3);             // ay
  res[2] = -q0*q0 + q1*q1 + q2*q2 - q3*q3; // az
}

void hm(const real_t& dt, real_t* arg, real_t* x, real_t* res)
{
  // x = [q0, q1, q2, q3]
  // z = [mx, my, mz]
  // no args

  real_t q0 = x[0], q1 = x[1], q2 = x[2], q3 = x[3];

  // estimated direction of flux (NED)
  res[0] = q0*q0 + q1*q1 - q2*q2 - q3*q3; // mx
  res[1] = 2*(q1*q2 - q0*q3);             // my
  res[2] = 2*(q0*q2 + q1*q3);             // mz

}

int main(int argc, char* argv[])
{
  bool result = true;

  real_t  x[N] = {1, 0, 0, 0, 0, 0, 0};
  real_t  u[L] = {0, 0, 0};    
  real_t za[3] = {0, 0,-1};
  real_t zm[3] = {0, 0, 0};
  
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

        blob::CF cf(N, x);

        while ( getline (input_file,line) )
        {
          if((line[0] == '-') || ((line[0] >= '0')&&(line[0] <='9')))
          {
            real_t gx, gy, gz, ax, ay, az, mx, my, mz, anorm, mnorm, roll, pitch, yaw;

            std::stringstream lineinput(line);
            lineinput >> gx >> gy >> gz >> ax >> ay >> az  >> mx >> my >> mz;
            
            //if(sizeof(real_t) == sizeof(double)) // FIXME: change to stringstream >> 
            //  sscanf(line.c_str(),"%lf %lf %lf %lf %lf %lf %lf %lf %lf"
            //                    , &gx, &gy, &gz, &ax, &ay, &az, &mx, &my, &mz);
            //else
            //  sscanf(line.c_str(),"%f %f %f %f %f %f %f %f %f"
            //                    , &gx, &gy, &gz, &ax, &ay, &az, &mx, &my, &mz);
              
            ta += T; 
            tm += T;

            // normalise measurements
            anorm = blob::math::sqrt(ax*ax + ay*ay + az*az);
            if (anorm > 0)
            {
              ax = ax/anorm;
              ay = ay/anorm;
              az = az/anorm;
            }

            mnorm = blob::math::sqrt(mx*mx + my*my + mz*mz);
            if (mnorm > 0)
            {
              mx = mx/mnorm;
              my = my/mnorm;
              mz = mz/mnorm;
            }

            u[0] = gx;
            u[1] = gy;
            u[2] = gz;
            
#if defined(__DEBUG__) & defined(__linux__)
            std::cout << "[test] - predicting over u=[" << u[0] << ", " << u[1] << 
                         ", " << u[2] << "], dt=" << T << std::endl;
#endif
            result &= cf.predict(&f, T, L, u, ki);

            if ((result==true)&&(ta>=Tacc))
            {
              za[0] = ax; za[1] = ay; za[2] = az;
#if defined(__DEBUG__) & defined(__linux__)
            std::cout << "[test] - updating over za=[" << za[0] << ", " << za[1] << 
                         ", " << za[2] << "], dta=" << ta << std::endl;
#endif
              result &= cf.update (&ha, ta, 3, za, kp_acc);
              ta = 0;
            }
            if ((result==true)&&(tm >= Tmag))
            {
              zm[0] = mx; zm[1] = my; zm[2] = mz;
#if defined(__DEBUG__) & defined(__linux__)
            std::cout << "[test] - updating over zm=[" << zm[0] << ", " << zm[1] << 
                         ", " << zm[2] << "], dtm=" << tm << std::endl;
#endif
              result &= cf.update (&hm, tm, 3, zm, kp_mag);
              tm = 0;
            }

            // re-normalize quaternion
            real_t *q = cf.getState();
            real_t qnorm = blob::math::sqrt(q[0]*q[0] + q[1]*q[1] + q[2]*q[2] + q[3]*q[3]);
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
            for(int i = 0; i < cf.getNumStates(); i++)
            {
              output_file << cf.getState(i) << " ";

#if defined(__DEBUG__) & defined(__linux__)
              std::cout << cf.getState(i) << " ";
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
