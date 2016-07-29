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
 * \file       cf.cpp
 * \brief      implemention of generic complimentary filter
 * \author     adrian jimenez-gonzalez (blob.robots@gmail.com)
 * \copyright  the MIT License Copyright (c) 2015 Blob Robots.
 *
 ******************************************************************************/

#include <blob/cf.h>
#include <blob/math.h>

blob::CF::CF (uint8_t n, real_t *init_x) : Estimator (n, init_x)
{
  _l = 0;
  memset(_error,0,sizeof(_error));
}

bool blob::CF::predict (estimator_function_t function, const real_t& dt, 
                        const uint8_t& lu, real_t* u, real_t* ki)
{
  bool retval = true;

  // error vector includes instantaneous error and accumulated error = 2*u_length
  _l = 2*lu; // error vector length is twice control input length 

  // correct control input with gained error and bias (integral error)
  for(int i =0; i<_l/2; i++)
  {
    u[i] += _error[i] + ki[i]*_error[i+_l/2];
  }
  
  // predict state given compensated control input
  function(dt,u,_x,_x); 
  
  // reset error
  memset(_error,0,sizeof(real_t)*_l/2);
  
  return retval;
}

bool blob::CF::update  (estimator_function_t function, const real_t& dt, 
                        const uint8_t& lz, real_t* z, real_t* kp)
{
  bool retval = true;

  real_t e [BLOB_CF_MAX_LENGTH];

  function(dt,NULL,z,e);

  for(int i=0; i<_l/2; i++) 
  {
    _error[i] += kp[i]*e[i]/dt;
    _error[i+_l/2] += e[i];
  }

  return retval;
}

void blob::CF::print  ()
{
  blob::Matrix x(_n,1,_x);
  blob::Matrix e(_l,1,_error);
  
#if defined(__linux__)
  std::cout << "CF::x = " << std::endl;
#endif
  x.print();
#if defined(__linux__)
  std::cout << std::endl << "CF::error = " << std::endl;
#endif
  e.print();
#if defined(__linux__)
  std::cout << std::endl;
#endif
}

