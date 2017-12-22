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
 * \file       test_vector_linux.cpp
 * \brief      tests for generic vector (column matrix) and 3d vector in linux
 * \author     adrian jimenez-gonzalez (blob.robots@gmail.com)
 * \copyright  the MIT License Copyright (c) 2017 Blob Robots.
 *
 ******************************************************************************/

#include <iostream>
#include "blob/vector.h"

bool test00_eye()
{
  std::cout << "test00_eye()" << std::endl;

  real_t r[] = { 10.f, 10.f, 10.f, 10.f, 10.f};

  blob::Vector<real_t> R(5,r);
//  R.eye();
  R.print();
  std::cout << std::endl;
  return true;
}

bool test01_add()
{
  std::cout << "test01_add()" << std::endl;

  real_t a[] = { 1.f, 1.f, 1.f, 1.f, 1.f};

  real_t b[] = { 2.f, 2.f, 2.f, 2.f, 2.f};
  
  real_t r[] = { 0.f, 0.f, 0.f, 0.f, 0.f};
  
  blob::Vector<real_t> A(5,a);
  blob::Vector<real_t> B(5,b);
  blob::Vector<real_t> R(5,r);

  A.print();
  std::cout << " + " << std::endl;
  B.print();
  std::cout << " = " << std::endl;
  blob::Vector<real_t>::add(A,B,R);
  R.print();
  std::cout << std::endl;
    
  return true;
}

bool test02_copy()
{
  std::cout << "test02_copy()" << std::endl;

  real_t a[] = { 1.f, 1.f, 1.f, 1.f, 1.f};

  real_t b[] = { 2.f, 2.f, 2.f, 2.f, 2.f};

  blob::Vector<real_t> A(5,a);
  blob::Vector<real_t> B(5,b);

  A.print();
  std::cout << "copy" << std::endl;
  B.print();
  A.copy(B);
  std::cout << "A\n=" << std::endl;
  A.print();
  std::cout << std::endl;
  
  return true;
}

bool test03_scale()
{
  std::cout << "test03_scale()" << std::endl;

  real_t r[] = { 10.f, 10.f, 10.f, 10.f, 10.f};
  blob::Vector<real_t> R(5,r);
  R.print();
  std::cout << "*\n0.01\n=" << std::endl;
  R.scale(0.01f);
  R.print();
  std::cout << std::endl;
  return true;
}

bool test04_substract()
{
  std::cout << "test04_substract()" << std::endl;

  real_t a[] = { 1.f, 1.f, 1.f, 1.f, 1.f};

  real_t b[] = { 2.f, 2.f, 2.f, 2.f, 2.f};

  real_t r[] = { 0.f, 0.f, 0.f, 0.f, 0.f};
  
  blob::Vector<real_t> A(5,a);
  blob::Vector<real_t> B(5,b);
  blob::Vector<real_t> R(5,r);

  A.print();
  std::cout << " - " << std::endl;
  B.print();
  std::cout << " = " << std::endl;
  blob::Vector<real_t>::substract(A,B,R);
  R.print();
  std::cout << std::endl;

  return true;
}

int main(int argc, char* argv[])
{

  test00_eye ();
  test01_add ();
  test02_copy();
  test03_scale();
  test04_substract();

  return 0;
}

