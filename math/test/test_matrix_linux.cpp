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
 * \file       test_matrix_linux.cpp
 * \brief      tests for real numbers matrix in linux
 * \author     adrian jimenez-gonzalez (blob.robots@gmail.com)
 * \copyright  the MIT License Copyright (c) 2017 Blob Robots.
 *
 ******************************************************************************/
#include <iostream>

#include "blob/matrix.h"

bool test00_eye()
{
  std::cout << "test00_eye()" << std::endl;

  real_t r[] = { 10.f, 10.f, 10.f, 10.f, 10.f,
                10.f, 10.f, 10.f, 10.f, 10.f,
                10.f, 10.f, 10.f, 10.f, 10.f,
                10.f, 10.f, 10.f, 10.f, 10.f,
                10.f, 10.f, 10.f, 10.f, 10.f };
  blob::MatrixR R(5,5,r);
  R.eye();
  R.print();
  std::cout << std::endl;
  return true;
}

bool test01_add()
{
  std::cout << "test01_add()" << std::endl;

  real_t a[] = { 1.f, 1.f, 1.f, 1.f, 1.f,
                1.f, 1.f, 1.f, 1.f, 1.f,
                1.f, 1.f, 1.f, 1.f, 1.f,
                1.f, 1.f, 1.f, 1.f, 1.f,
                1.f, 1.f, 1.f, 1.f, 1.f };

  real_t b[] = { 2.f, 2.f, 2.f, 2.f, 2.f,
                2.f, 2.f, 2.f, 2.f, 2.f,
                2.f, 2.f, 2.f, 2.f, 2.f,
                2.f, 2.f, 2.f, 2.f, 2.f,
                2.f, 2.f, 2.f, 2.f, 2.f };
  
  real_t c[] = { 3.f, 3.f,
                3.f, 3.f,
                3.f, 3.f,
                3.f, 3.f,
                3.f, 3.f };

  real_t d[] = { 4.f, 4.f, 4.f, 4.f, 4.f,
                4.f, 4.f, 4.f, 4.f, 4.f };

  real_t r[] = { 0.f, 0.f, 0.f, 0.f, 0.f,
                0.f, 0.f, 0.f, 0.f, 0.f,
                0.f, 0.f, 0.f, 0.f, 0.f,
                0.f, 0.f, 0.f, 0.f, 0.f,
                0.f, 0.f, 0.f, 0.f, 0.f };
  
  blob::MatrixR A(5,5,a);
  blob::MatrixR B(5,5,b);
  blob::MatrixR C(5,2,c);
  blob::MatrixR D(2,5,d);
  blob::MatrixR R(5,5,r);

  A.print();
  std::cout << " + " << std::endl;
  B.print();
  std::cout << " = " << std::endl;
  blob::MatrixR::add(A,B,R);
  R.print();
  std::cout << std::endl;
  
  std::cout << "A.add(C,3,2,2,0) = " << std::endl;
  A.add(C,3,2,2,0);
  A.print();
  std::cout << std::endl;
  
  std::cout << "A.add(D,2,3,0,2) = " << std::endl;
  A.add(D,2,3,0,2);
  A.print();
  std::cout << std::endl;
  
  return true;
}

bool test02_copy()
{
  std::cout << "test02_copy()" << std::endl;

  real_t a[] = { 1.f, 1.f, 1.f, 1.f, 1.f,
                1.f, 1.f, 1.f, 1.f, 1.f,
                1.f, 1.f, 1.f, 1.f, 1.f,
                1.f, 1.f, 1.f, 1.f, 1.f,
                1.f, 1.f, 1.f, 1.f, 1.f };

  real_t b[] = { 2.f, 2.f, 2.f, 2.f, 2.f,
                2.f, 2.f, 2.f, 2.f, 2.f,
                2.f, 2.f, 2.f, 2.f, 2.f,
                2.f, 2.f, 2.f, 2.f, 2.f,
                2.f, 2.f, 2.f, 2.f, 2.f };

  real_t c[] = { 3.f, 3.f,
                3.f, 3.f,
                3.f, 3.f,
                3.f, 3.f,
                3.f, 3.f };

  real_t d[] = { 4.f, 4.f, 4.f, 4.f, 4.f,
                4.f, 4.f, 4.f, 4.f, 4.f };

  blob::MatrixR A(5,5,a);
  blob::MatrixR B(5,5,b);
  blob::MatrixR C(5,2,c);
  blob::MatrixR D(2,5,d);

  std::cout << "A.copy(B)" << std::endl;
  A.copy(B);
  A.print();
  std::cout << std::endl;
  std::cout << "A.copy(C,3,2,2,0)" << std::endl;
  A.copy(C,3,2,2,0);
  A.print();
  std::cout << std::endl;
  std::cout << "A.copy(D,2,3,0,2)" << std::endl;
  A.copy(D,2,3,0,2);
  A.print();
  std::cout << std::endl;
  
  return true;
}

bool test03_scale()
{
  real_t r[] = { 10.f, 10.f, 10.f, 10.f, 10.f,
                10.f, 10.f, 10.f, 10.f, 10.f,
                10.f, 10.f, 10.f, 10.f, 10.f,
                10.f, 10.f, 10.f, 10.f, 10.f,
                10.f, 10.f, 10.f, 10.f, 10.f };
  blob::MatrixR R(5,5,r);
  R.scale(0.01f);
  R.print();
  std::cout << std::endl;
  return true;
}

bool test04_substract()
{
  real_t a[] = { 1.f, 1.f, 1.f, 1.f, 1.f,
                1.f, 1.f, 1.f, 1.f, 1.f,
                1.f, 1.f, 1.f, 1.f, 1.f,
                1.f, 1.f, 1.f, 1.f, 1.f,
                1.f, 1.f, 1.f, 1.f, 1.f };

  real_t b[] = { 2.f, 2.f, 2.f, 2.f, 2.f,
                2.f, 2.f, 2.f, 2.f, 2.f,
                2.f, 2.f, 2.f, 2.f, 2.f,
                2.f, 2.f, 2.f, 2.f, 2.f,
                2.f, 2.f, 2.f, 2.f, 2.f };

  real_t c[] = { 3.f, 3.f,
                3.f, 3.f,
                3.f, 3.f,
                3.f, 3.f,
                3.f, 3.f };

  real_t d[] = { 4.f, 4.f, 4.f, 4.f, 4.f,
                4.f, 4.f, 4.f, 4.f, 4.f };

  real_t r[] = { 0.f, 0.f, 0.f, 0.f, 0.f,
                0.f, 0.f, 0.f, 0.f, 0.f,
                0.f, 0.f, 0.f, 0.f, 0.f,
                0.f, 0.f, 0.f, 0.f, 0.f,
                0.f, 0.f, 0.f, 0.f, 0.f };
  
  blob::MatrixR A(5,5,a);
  blob::MatrixR B(5,5,b);
  blob::MatrixR C(5,2,c);
  blob::MatrixR D(2,5,d);
  blob::MatrixR R(5,5,r);

  A.print();
  std::cout << " - " << std::endl;
  B.print();
  std::cout << " = " << std::endl;
  blob::MatrixR::substract(A,B,R);
  R.print();
  std::cout << std::endl;

  std::cout << "A.substract(C,3,2,2,0) = " << std::endl;
  A.substract(C,3,2,2,0);
  A.print();
  std::cout << std::endl;
  
  std::cout << "A.substract(D,2,3,0,2) = " << std::endl;
  A.substract(D,2,3,0,2);
  A.print();
  std::cout << std::endl;

  return true;
}

bool test05_multiply()
{
  real_t a[] = { 1.5f, 1.5f, 1.5f, 1.5f, 1.5f,
                1.5f, 1.5f, 1.5f, 1.5f, 1.5f,
                1.5f, 1.5f, 1.5f, 1.5f, 1.5f,
                1.5f, 1.5f, 1.5f, 1.5f, 1.5f,
                1.5f, 1.5f, 1.5f, 1.5f, 1.5f };

  real_t b[] = { 2.f, 2.f, 2.f, 2.f, 2.f,
                2.f, 2.f, 2.f, 2.f, 2.f,
                2.f, 2.f, 2.f, 2.f, 2.f,
                2.f, 2.f, 2.f, 2.f, 2.f,
                2.f, 2.f, 2.f, 2.f, 2.f };

  real_t c[] = { 3.f, 3.f,
                3.f, 3.f,
                3.f, 3.f,
                3.f, 3.f,
                3.f, 3.f };

  real_t d[] = { 4.f, 4.f, 4.f, 4.f, 4.f,
                4.f, 4.f, 4.f, 4.f, 4.f };

  real_t r[] = { 0.f, 0.f, 0.f, 0.f, 0.f,
                0.f, 0.f, 0.f, 0.f, 0.f,
                0.f, 0.f, 0.f, 0.f, 0.f,
                0.f, 0.f, 0.f, 0.f, 0.f,
                0.f, 0.f, 0.f, 0.f, 0.f };
  
  blob::MatrixR A(5,5,a);
  blob::MatrixR B(5,5,b);
  blob::MatrixR C(5,2,c);
  blob::MatrixR D(2,5,d);
  blob::MatrixR R(5,5,r);

  std::cout << "test05_multiply" << std::endl << std::endl;

  A.print();
  std::cout << " * " << std::endl;
  B.print();
  std::cout << " = " << std::endl;
  if(blob::MatrixR::multiply(A,B,R))
    R.print();
  else
    std::cout << "NaN" << std::endl;  
  std::cout << std::endl;

  C.print();
  std::cout << " * " << std::endl;
  D.print();
  std::cout << " = " << std::endl;
  if(blob::MatrixR::multiply(C,D,R))
    R.print();
  else
    std::cout << "NaN" << std::endl;  
  std::cout << std::endl;

  R.refurbish(2,2);
  D.print();
  std::cout << " * " << std::endl;
  C.print();
  std::cout << " = " << std::endl;
  if(blob::MatrixR::multiply(D,C,R))
    R.print();
  else
    std::cout << "NaN" << std::endl;  

  std::cout << std::endl;

  R.refurbish(5,2);
  A.print();
  std::cout << " * " << std::endl;
  C.print();
  std::cout << " = " << std::endl;
  if(blob::MatrixR::multiply(A,C,R))
    R.print();
  else
    std::cout << "NaN" << std::endl;  
  std::cout << std::endl;

  R.refurbish(2,5);
  D.print();
  std::cout << " * " << std::endl;
  B.print();
  std::cout << " = " << std::endl;
  if(blob::MatrixR::multiply(D,B,R))
    R.print();
  else
    std::cout << "NaN" << std::endl;  
  std::cout << std::endl;

  R.refurbish(5,5);
  A.print();
  std::cout << " & " << std::endl;
  B.print();
  std::cout << " = " << std::endl;
  blob::MatrixR::multiplyElem(A,B,R);
  R.print();
}

bool test06_transpose()
{
  real_t a[] = { 1.f, 2.f, 3.f, 4.f, 5.f,
                1.f, 2.f, 3.f, 4.f, 5.f,
                1.f, 2.f, 3.f, 4.f, 5.f,
                1.f, 2.f, 3.f, 4.f, 5.f,
                1.f, 2.f, 3.f, 4.f, 5.f };

  real_t c[] = { 1.f, 1.f, 1.f,
                2.f, 2.f, 2.f,
                3.f, 3.f, 3.f,
                4.f, 4.f, 4.f,
                5.f, 5.f, 5.f };

  real_t d[] = { 1.f, 2.f, 3.f, 4.f, 5.f,
                1.f, 2.f, 3.f, 4.f, 5.f,
                1.f, 2.f, 3.f, 4.f, 5.f };

  real_t r[] = { 0.f, 0.f, 0.f, 0.f, 0.f,
                0.f, 0.f, 0.f, 0.f, 0.f,
                0.f, 0.f, 0.f, 0.f, 0.f,
                0.f, 0.f, 0.f, 0.f, 0.f,
                0.f, 0.f, 0.f, 0.f, 0.f };
  
  blob::MatrixR A(5,5,a);
  blob::MatrixR C(5,3,c);
  blob::MatrixR D(3,5,d);
  
  blob::MatrixR R(5,5,r);
  
  std::cout << "test06_transpose" << std::endl << std::endl;
  
  A.print();
  std::cout << "T" << std::endl;
  std::cout << " = " << std::endl;
  
  if(blob::MatrixR::transpose(A,R))
    R.print();
  else
    std::cout << "NaN" << std::endl;  
  
  std::cout << std::endl;
  
  A.print();
  std::cout << "T" << std::endl;
  std::cout << " = " << std::endl;
  if(A.transpose())
    A.print();
  else
    std::cout << "NaN" << std::endl;  
  
  std::cout << std::endl;

  C.print();
  std::cout << "T" << std::endl;
  std::cout << " = " << std::endl;
  C.transpose();
  C.print();
  std::cout << std::endl;
  
  C.print();
  std::cout << "T" << std::endl;
  std::cout << " = " << std::endl;
  R.refurbish(5,3,r);
  blob::MatrixR::transpose(C,R);
  R.print();
  std::cout << std::endl;

  D.print();
  std::cout << "T" << std::endl;
  std::cout << " = " << std::endl;
  D.transpose();
  D.print();
  std::cout << std::endl;
  
  D.print();
  std::cout << "T" << std::endl;
  std::cout << " = " << std::endl;
  R.refurbish(3,5,r);
  blob::MatrixR::transpose(D,R);
  R.print();
  std::cout << std::endl;

  return true;
}

bool test07_cholesky()
{
  real_t a[] = { 1.f, 2.f, 3.f, 4.f, 5.f,
                1.f, 2.f, 3.f, 4.f, 5.f,
                1.f, 2.f, 3.f, 4.f, 5.f,
                1.f, 2.f, 3.f, 4.f, 5.f,
                1.f, 2.f, 3.f, 4.f, 5.f };

  real_t l[] = { 1.f, 0.f, 0.f, 0.f, 0.f,
                2.f, 2.f, 0.f, 0.f, 0.f,
                3.f, 3.f, 3.f, 0.f, 0.f,
                4.f, 4.f, 4.f, 4.f, 0.f,
                5.f, 5.f, 5.f, 5.f, 5.f };
  
  real_t lt[] = { 0.f, 0.f, 0.f, 0.f, 0.f,
                 0.f, 0.f, 0.f, 0.f, 0.f,
                 0.f, 0.f, 0.f, 0.f, 0.f,
                 0.f, 0.f, 0.f, 0.f, 0.f,
                 0.f, 0.f, 0.f, 0.f, 0.f };
  
  real_t r[] = { 0.f, 0.f, 0.f, 0.f, 0.f,
                0.f, 0.f, 0.f, 0.f, 0.f,
                0.f, 0.f, 0.f, 0.f, 0.f,
                0.f, 0.f, 0.f, 0.f, 0.f,
                0.f, 0.f, 0.f, 0.f, 0.f };
  
  blob::MatrixR A(5,5,a);
  blob::MatrixR L(5,5,l);
  blob::MatrixR Lt(5,5,lt);
  blob::MatrixR R(5,5,r);

  std::cout << "test07_cholesky" << std::endl << std::endl;

  L.print();
  std::cout << " * " << std::endl; 
  blob::MatrixR::transpose(L,Lt);
  Lt.print();
  std::cout << " = " << std::endl; 
  blob::MatrixR::multiply(L,Lt,A);
  A.print();
  std::cout << " => " << std::endl; 
  std::cout << "chol(" << std::endl;
  A.print();
  std::cout << ") = " << std::endl; 
  A.cholesky(true);
  A.print();
  std::cout << std::endl;

  blob::MatrixR::multiply(L,Lt,A);
  std::cout << "chol(" << std::endl;
  A.print();
  std::cout << ") = " << std::endl; 
  A.cholesky(false);
  A.print();
  std::cout << " => " << std::endl; 
  std::cout << "A=" << std::endl;
  A.cholrestore(false);
  A.print();
  std::cout << std::endl;
  std::cout << std::endl;
  return true;
}

bool test08_inverse()
{
  real_t a[] = { 1.f, 2.f, 3.f, 4.f, 5.f,
                2.f, 8.f, 12.f, 16.f, 20.f,
                3.f, 12.f, 27.f, 36.f, 45.f,
                4.f, 16.f, 36.f, 64.f, 80.f,
                5.f, 20.f, 45.f, 80.f, 125.f };

  real_t a1[] = { 0.f, 0.f, 0.f, 0.f, 0.f,
                0.f, 0.f, 0.f, 0.f, 0.f,
                0.f, 0.f, 0.f, 0.f, 0.f,
                0.f, 0.f, 0.f, 0.f, 0.f,
                0.f, 0.f, 0.f, 0.f, 0.f };

  real_t b[] = { 0.958254f,0.585388f, 0.995586f, 0.870752f, 0.427295f,
                 0.540637f, 0.149850f, 0.096668f, 0.478074f, 0.900839f, 
                 0.942919f, 0.383296f, 0.733441f, 0.675273f, 0.783696f, 
                 0.552495f, 0.875449f, 0.837438f, 0.926825f, 0.291345f,
                 0.189773f, 0.787383f, 0.436059f, 0.854803f, 0.881570f};

  real_t r[] = { 0.f, 0.f, 0.f, 0.f, 0.f,
                0.f, 0.f, 0.f, 0.f, 0.f,
                0.f, 0.f, 0.f, 0.f, 0.f,
                0.f, 0.f, 0.f, 0.f, 0.f,
                0.f, 0.f, 0.f, 0.f, 0.f };
  
  blob::MatrixR A(5,5,a);
  blob::MatrixR A1(5,5,a1);
  blob::MatrixR B(5,5,b);
  blob::MatrixR R(5,5,r);

  std::cout << "test08_inverse" << std::endl << std::endl;
  
  std::cout << "inv(" << std::endl;
  A.print();
  std::cout << ") = " << std::endl;
  A1.copy(A);
  A1.inverse();
  A1.print();
  std::cout << " => " << std::endl; 
  A.print();
  std::cout << " * " << std::endl; 
  A1.print();
  std::cout << " = " << std::endl; 
  blob::MatrixR::multiply(A,A1,R);
  R.print();
  std::cout << std::endl;

  A1.zero();
  std::cout << "inv(" << std::endl;
  A.print();
  std::cout << ") = " << std::endl;
  blob::MatrixR::inverse(A,A1);
  A1.print();
  std::cout << " => " << std::endl; 
  A.print();
  std::cout << " * " << std::endl; 
  A1.print();
  std::cout << " = " << std::endl; 
  blob::MatrixR::multiply(A,A1,R);
  R.print();
  std::cout << std::endl;

  std::cout << "inv(" << std::endl;
  B.print();
  std::cout << ") = " << std::endl;
  blob::MatrixR::inverse(B,A1);
  A1.print();
  std::cout << " => " << std::endl; 
  B.print();
  std::cout << " * " << std::endl; 
  A1.print();
  std::cout << " = " << std::endl; 
  blob::MatrixR::multiply(B,A1,R);
  R.print();
  std::cout << std::endl;
  return true;
}

bool test09_divide()
{
  
  real_t a[] = { 1.f, 1.f, 1.f, 1.f, 1.f,
                 2.f, 2.f, 2.f, 2.f, 2.f,
                 3.f, 3.f, 3.f, 3.f, 3.f,
                 4.f, 4.f, 4.f, 4.f, 4.f,
                 5.f, 5.f, 5.f, 5.f, 5.f };

  real_t b[] = { 1.f, 2.f, 3.f, 4.f, 5.f,
                2.f, 8.f, 12.f, 16.f, 20.f,
                3.f, 12.f, 27.f, 36.f, 45.f,
                4.f, 16.f, 36.f, 64.f, 80.f,
                5.f, 20.f, 45.f, 80.f, 125.f };

  real_t r[] = { 0.f, 0.f, 0.f, 0.f, 0.f,
                 0.f, 0.f, 0.f, 0.f, 0.f,
                 0.f, 0.f, 0.f, 0.f, 0.f,
                 0.f, 0.f, 0.f, 0.f, 0.f,
                 0.f, 0.f, 0.f, 0.f, 0.f };
  
  blob::MatrixR A(5,5,a);
  blob::MatrixR B(5,5,b);
  blob::MatrixR R(5,5,r);

  std::cout << "test09_divide" << std::endl << std::endl;
  
  A.print();
  std::cout << " / " << std::endl;
  B.print();
  std::cout << " = " << std::endl;
  blob::MatrixR::divide(A,B,R);
  R.print();
  std::cout << " = " << std::endl;
  B.inverse();
  blob::MatrixR::multiply(A,B,R);
  R.print();

  std::cout << std::endl << std::endl;
  A.refurbish(1,5);
  R.refurbish(1,5);
  R.zero();
  B.inverse();
  A.print();
  std::cout << " / " << std::endl;
  B.print();
  std::cout << " = " << std::endl;
  blob::MatrixR::divide(A,B,R);
  R.print();
  std::cout << " = " << std::endl;
  B.inverse();
  blob::MatrixR::multiply(A,B,R);
  R.print();
  
  std::cout << std::endl << std::endl;
  return true;
}

bool test10_diag()
{
  real_t a[] = { 3.f, 3.f,
                3.f, 3.f,
                3.f, 3.f,
                3.f, 3.f,
                3.f, 3.f };

  real_t b[] = { 4.f, 4.f, 4.f, 4.f,
                4.f, 4.f, 4.f, 4.f,
                4.f, 4.f, 4.f, 4.f };

  real_t d1[] = {2.f, 3.f};
  real_t d2[] = {2.f, 3.f, 4.f, 5.f};
  real_t d3[] = {2.f, 3.f, 4.f, 5.f, 6.f};


  real_t r[] = { 0.f, 0.f, 0.f, 0.f, 0.f,
                0.f, 0.f, 0.f, 0.f, 0.f,
                0.f, 0.f, 0.f, 0.f, 0.f,
                0.f, 0.f, 0.f, 0.f, 0.f,
                0.f, 0.f, 0.f, 0.f, 0.f };
  
  blob::MatrixR A(5,2,a);
  blob::MatrixR B(3,4,b);
  blob::MatrixR D1(1,2,d1);
  blob::MatrixR D2(4,1,d2);
  blob::MatrixR D3(1,5,d3);
  blob::MatrixR R(5,2,r);

  A.print();
  std::cout << " diag " << std::endl;
  D1.print();
  std::cout << " = " << std::endl;
  if(blob::MatrixR::multiplyDiag(A,D1,R))
    R.print();
  else
    std::cout << "NaN" << std::endl;  
  std::cout << std::endl;
    
  A.print();
  std::cout << " diag " << std::endl;
  D2.print();
  std::cout << " = " << std::endl;
  if(blob::MatrixR::multiplyDiag(A,D2,R))
    R.print();
  else
    std::cout << "NaN" << std::endl;  
  std::cout << std::endl;
  
  A.print();
  std::cout << " diag " << std::endl;
  D3.print();
  std::cout << " = " << std::endl;
  if(blob::MatrixR::multiplyDiag(A,D3,R))
    R.print();
  else
    std::cout << "NaN" << std::endl;  
  std::cout << std::endl;
  
  R.refurbish(3,4);

  B.print();
  std::cout << " diag " << std::endl;
  D1.print();
  std::cout << " = " << std::endl;
  if(blob::MatrixR::multiplyDiag(B,D1,R))
    R.print();
  else
    std::cout << "NaN" << std::endl;  
  std::cout << std::endl;
    
  B.print();
  std::cout << " diag " << std::endl;
  D2.print();
  std::cout << " = " << std::endl;
  if(blob::MatrixR::multiplyDiag(B,D2,R))
    R.print();
  else
    std::cout << "NaN" << std::endl;  
  std::cout << std::endl;
  
  B.print();
  std::cout << " diag " << std::endl;
  D3.print();
  std::cout << " = " << std::endl;
  if(blob::MatrixR::multiplyDiag(B,D3,R))
    R.print();
  else
    std::cout << "NaN" << std::endl;  
  std::cout << std::endl;
  
  return true;
}

bool test11_cholupdate()
{
  real_t ve[] = {1.f, 2.f, 3.f, 4.f, 5.f};

  real_t l[] = { 1.f, 0.f, 0.f, 0.f, 0.f,
                2.f, 2.f, 0.f, 0.f, 0.f,
                3.f, 3.f, 3.f, 0.f, 0.f,
                4.f, 4.f, 4.f, 4.f, 0.f,
                5.f, 5.f, 5.f, 5.f, 5.f };
  
  real_t r[] = { 0.f, 0.f, 0.f, 0.f, 0.f,
                 0.f, 0.f, 0.f, 0.f, 0.f,
                 0.f, 0.f, 0.f, 0.f, 0.f,
                 0.f, 0.f, 0.f, 0.f, 0.f,
                 0.f, 0.f, 0.f, 0.f, 0.f };
  
  blob::MatrixR L(5,5,l);
  blob::MatrixR v(1,5,ve);
  blob::MatrixR R(5,5,r);

  std::cout << "test11_cholupdate" << std::endl << std::endl;

  L.print();
  std::cout << " updated with " << std::endl; 
  v.print();
  std::cout << " = " << std::endl; 
  L.cholupdate(v,1);
  L.print();
  std::cout << std::endl;
  std::cout << std::endl;
  return true;
}

bool test12_qr()
{
  real_t a[] = { 2.f, -51.f,   4.f,
                  6.f, 167.f, -68.f,
                 -4.f,  24.f, -41.f,
                 -1.f,   1.f,   0.f,
                  2.f,   0.f,   3.f};

  real_t q[] = { 1.f, 0.f, 0.f, 0.f, 0.f,
                2.f, 2.f, 0.f, 0.f, 0.f,
                3.f, 3.f, 3.f, 0.f, 0.f,
                4.f, 4.f, 4.f, 4.f, 0.f,
                5.f, 5.f, 5.f, 5.f, 5.f };
  
  real_t r[] = { 0.f, 0.f, 0.f,
                 0.f, 0.f, 0.f,
                 0.f, 0.f, 0.f,
                 0.f, 0.f, 0.f,
                 0.f, 0.f, 0.f };
  
  blob::MatrixR A(5,3,a);
  blob::MatrixR Q(5,5,q);
  blob::MatrixR R(5,3,r);

  std::cout << "test12_qr" << std::endl << std::endl;

  blob::MatrixR::qr(A,Q,R);
  std::cout << " QR(" << std::endl; 
  A.print();
  std::cout << " ) => " << std::endl; 
  std::cout << " Q = " << std::endl; 
  Q.print();
  std::cout << " , " <<std::endl;
  std::cout << " R = " << std::endl;
  R.print();
  std::cout << std::endl;
  std::cout << std::endl;
}

bool test13_permute()
{
  
  real_t a[] = { 1.f, 1.f, 1.f, 1.f, 1.f,
                 2.f, 2.f, 2.f, 2.f, 2.f,
                 3.f, 3.f, 3.f, 3.f, 3.f,
                 4.f, 4.f, 4.f, 4.f, 4.f,
                 5.f, 5.f, 5.f, 5.f, 5.f };

  std::cout << "test13_permute" << std::endl << std::endl;
  blob::MatrixR A(5,5,a);
  std::cout << " A = " << std::endl << std::endl;
  A.print();
  std::cout << " Permute rows (A,1,4)" << std::endl; 
  A.permuteRows(1,4);
  std::cout << " A \n =" << std::endl; 
  A.print();
  std::cout << " Permute rows (A,2,3,3,2)" << std::endl; 
  A.permuteRows(2,3,3,2);
  std::cout << " A \n =" << std::endl; 
  A.print();
  std::cout << std::endl; 
}

bool test14_lu()
{
  real_t a[] = { 2.f, -51.f,  4.f,  3.f,  2.f,
                 6.f, 167.f,-68.f,-10.f, 0.5f, 
                -4.f,  24.f,-41.f, 44.f,-0.9f,
                -1.f,   1.f,  0.f, 50.f,-44.f,
                 2.f,   0.f,  3.f,  1.f,  1.f};

  real_t u[] = { 1.f, 0.f, 0.f, 0.f, 0.f,
                 2.f, 2.f, 0.f, 0.f, 0.f,
                 3.f, 3.f, 3.f, 0.f, 0.f,
                 4.f, 4.f, 4.f, 4.f, 0.f,
                 5.f, 5.f, 5.f, 5.f, 5.f };
  
  real_t l[] = { 0.f, 0.f, 0.f, 0.f, 0.f,
                 0.f, 0.f, 0.f, 0.f, 0.f,
                 0.f, 0.f, 0.f, 0.f, 0.f,
                 0.f, 0.f, 0.f, 0.f, 0.f,
                 0.f, 0.f, 0.f, 0.f, 0.f };

  real_t p[] = { 0.f, 0.f, 0.f, 0.f, 0.f,
                 0.f, 0.f, 0.f, 0.f, 0.f,
                 0.f, 0.f, 0.f, 0.f, 0.f,
                 0.f, 0.f, 0.f, 0.f, 0.f,
                 0.f, 0.f, 0.f, 0.f, 0.f };  

  real_t r[] = { 0.f, 0.f, 0.f, 0.f, 0.f,
                 0.f, 0.f, 0.f, 0.f, 0.f,
                 0.f, 0.f, 0.f, 0.f, 0.f,
                 0.f, 0.f, 0.f, 0.f, 0.f,
                 0.f, 0.f, 0.f, 0.f, 0.f };  
  
  blob::MatrixR A(5,5,a);
  blob::MatrixR L(5,5,l);
  blob::MatrixR U(5,5,u);
  blob::MatrixR P(5,5,p);
  blob::MatrixR R(5,5,r);

  std::cout << "test14_lu" << std::endl << std::endl;

  blob::MatrixR::lu(A,L,U,P);
  std::cout << " PA = LU =>" << std::endl; 
  std::cout << " P = " << std::endl; 
  P.print();
  std::cout << " A = " << std::endl; 
  A.print();
  std::cout << "  = " << std::endl; 
  std::cout << " L = " << std::endl; 
  L.print();
  std::cout << " , " << std::endl;
  std::cout << " U = " << std::endl;
  U.print();
  std::cout << std::endl;
  std::cout << "  => " << std::endl; 
  //R.multiply(P,A);
  //R.print();
  A.lu();
  A.print();  
  std::cout << "  = " << std::endl; 
  R.multiply(L,U);  
  R.print();
  std::cout << std::endl;
  std::cout << "  = " << std::endl; 
  A.lurestore();
  A.print();  
}

bool test15_lu()
{
  real_t a[] = { 11,    9,   24,    2,
                  1,    5,    2,    6,
                  3,   17,   18,    1,
                  2,    5,    7,    1 };

  real_t u[] = { 1.f, 0.f, 0.f, 0.f, 
                 2.f, 2.f, 0.f, 0.f, 
                 3.f, 3.f, 3.f, 0.f, 
                 4.f, 4.f, 4.f, 4.f };
  
  real_t l[] = { 0.f, 0.f, 0.f, 0.f,
                 0.f, 0.f, 0.f, 0.f,
                 0.f, 0.f, 0.f, 0.f,
                 0.f, 0.f, 0.f, 0.f };

  real_t p[] = { 0.f, 0.f, 0.f, 0.f,
                 0.f, 0.f, 0.f, 0.f,
                 0.f, 0.f, 0.f, 0.f,
                 0.f, 0.f, 0.f, 0.f };

  real_t r[] = { 0.f, 0.f, 0.f, 0.f,
                 0.f, 0.f, 0.f, 0.f,
                 0.f, 0.f, 0.f, 0.f,
                 0.f, 0.f, 0.f, 0.f };

  blob::MatrixR A(4,4,a);
  blob::MatrixR L(4,4,l);
  blob::MatrixR U(4,4,u);
  blob::MatrixR P(4,4,p);
  blob::MatrixR R(4,4,r);

  std::cout << "test15_lu" << std::endl << std::endl;

  blob::MatrixR::lu(A,L,U,P);
  std::cout << " PA = LU =>" << std::endl; 
  std::cout << " P = " << std::endl; 
  P.print();
  std::cout << " A = " << std::endl; 
  A.print();
  std::cout << "  = " << std::endl; 
  std::cout << " L = " << std::endl; 
  L.print();
  std::cout << " , " << std::endl;
  std::cout << " U = " << std::endl;
  U.print();
  std::cout << std::endl;
  std::cout << "  => " << std::endl; 
  R.multiply(P,A);
  R.print();
  std::cout << "  = " << std::endl; 
  R.multiply(L,U);  
  R.print();
  std::cout << std::endl;
}

bool test16_lu()
{
  real_t a[] = { 3.f,   2.f,   1.1,
                 6.f,   2.f,   1.f,
                 1.f,   4.f,   2.f };

  real_t u[] = { 1.f, 0.f, 0.f,  
                 2.f, 2.f, 0.f,  
                 3.f, 3.f, 3.f };
  
  real_t l[] = { 0.f, 0.f, 0.f, 
                 0.f, 0.f, 0.f, 
                 0.f, 0.f, 0.f };

  real_t p[] = { 0.f, 0.f, 0.f, 
                 0.f, 0.f, 0.f, 
                 0.f, 0.f, 0.f };

  real_t r[] = { 0.f, 0.f, 0.f, 
                 0.f, 0.f, 0.f, 
                 0.f, 0.f, 0.f };

  blob::MatrixR A(3,3,a);
  blob::MatrixR L(3,3,l);
  blob::MatrixR U(3,3,u);
  blob::MatrixR P(3,3,p);
  blob::MatrixR R(3,3,r);
  
  std::cout << "test16_lu" << std::endl << std::endl;

  blob::MatrixR::lu(A,L,U);
  std::cout << " PA = LU =>" << std::endl; 
  std::cout << " P = " << std::endl; 
  P.print();
  std::cout << " A = " << std::endl; 
  A.print();
  std::cout << "  = " << std::endl; 
  std::cout << " L = " << std::endl; 
  L.print();
  std::cout << " , " << std::endl;
  std::cout << " U = " << std::endl;
  U.print();
  std::cout << std::endl;
  std::cout << "  => " << std::endl; 
  //R.multiply(P,A);
  //R.print();
  blob::MatrixR::lu(A,P);
  P.print();  
  std::cout << "  = " << std::endl; 
  R.multiply(L,U);  
  R.print();
  std::cout << std::endl;
  std::cout << "  = " << std::endl; 
  P.lurestore();
  P.print();

  std::cout << std::endl;
  std::cout << " A.inv()" << std::endl;
  std::cout << "  = " << std::endl; 
  //blob::MatrixR::inverse(A,R,false);
  R.copy(A);
  R.inverse();  
  R.print();
  std::cout << std::endl;
  
  std::cout << " A*A.inv()" << std::endl;
  blob::MatrixR::multiply(A,R,P);
  std::cout << "  = " << std::endl; 
  P.print();
  
}

int main(int argc, char* argv[])
{

  test00_eye ();
  test01_add ();
  test02_copy();
  test03_scale();
  test04_substract();
  test05_multiply();
  
  test06_transpose();
  test07_cholesky();
  test08_inverse();
  test09_divide();
  test10_diag();
  test11_cholupdate();
  test12_qr();
  test13_permute();

  test14_lu();
  test15_lu();
  test16_lu();
  
  return 0;
}
