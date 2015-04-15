/********* blob robotics 2014 *********
 *  title: test_linux.cpp
 *  brief: test for matrix ops (linux)
 * author: adrian jimenez-gonzalez
 * e-mail: blob.robotics@gmail.com
 /*************************************/
#include <iostream>
#include "blob/matrix.h"

bool test00_eye()
{
  real_t r[] = { 10.f, 10.f, 10.f, 10.f, 10.f,
                10.f, 10.f, 10.f, 10.f, 10.f,
                10.f, 10.f, 10.f, 10.f, 10.f,
                10.f, 10.f, 10.f, 10.f, 10.f,
                10.f, 10.f, 10.f, 10.f, 10.f };
  blob::Matrix R(5,5,r);
  R.eye();
  R.print();
  std::cout << std::endl;
  return true;
}

bool test01_add()
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
  
  blob::Matrix A(5,5,a);
  blob::Matrix B(5,5,b);
  blob::Matrix C(5,2,c);
  blob::Matrix D(2,5,d);
  blob::Matrix R(5,5,r);

  A.print();
  std::cout << " + " << std::endl;
  B.print();
  std::cout << " = " << std::endl;
  blob::Matrix::add(A,B,R);
  R.print();
  std::cout << std::endl;
  
  std::cout << "A.add(C,0,3) = " << std::endl;
  A.add(C,0,3);
  A.print();
  std::cout << std::endl;
  
  std::cout << "A.add(D,3,0) = " << std::endl;
  A.add(D,3,0);
  A.print();
  std::cout << std::endl;
  
  return true;
}

bool test02_copy()
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

  blob::Matrix A(5,5,a);
  blob::Matrix B(5,5,b);
  blob::Matrix C(5,2,c);
  blob::Matrix D(2,5,d);

  std::cout << "A.copy(B,0,4)" << std::endl;
  A.copy(B,0,4);
  A.print();
  std::cout << std::endl;
  std::cout << "A.copy(C,0,3)" << std::endl;
  A.copy(C,0,3);
  A.print();
  std::cout << std::endl;
  std::cout << "A.copy(D,3,0)" << std::endl;
  A.copy(D,3,0);
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
  blob::Matrix R(5,5,r);
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
  
  blob::Matrix A(5,5,a);
  blob::Matrix B(5,5,b);
  blob::Matrix C(5,2,c);
  blob::Matrix D(2,5,d);
  blob::Matrix R(5,5,r);

  A.print();
  std::cout << " - " << std::endl;
  B.print();
  std::cout << " = " << std::endl;
  blob::Matrix::substract(A,B,R);
  R.print();
  std::cout << std::endl;

  std::cout << "A.substract(C,0,3) = " << std::endl;
  A.substract(C,0,3);
  A.print();
  std::cout << std::endl;
  
  std::cout << "A.substract(D,3,0) = " << std::endl;
  A.substract(D,3,0);
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
  
  blob::Matrix A(5,5,a);
  blob::Matrix B(5,5,b);
  blob::Matrix C(5,2,c);
  blob::Matrix D(2,5,d);
  blob::Matrix R(5,5,r);

  std::cout << "test05_multiply" << std::endl << std::endl;

  A.print();
  std::cout << " * " << std::endl;
  B.print();
  std::cout << " = " << std::endl;
  if(blob::Matrix::multiply(A,B,R))
    R.print();
  else
    std::cout << "NaN" << std::endl;  
  std::cout << std::endl;

  C.print();
  std::cout << " * " << std::endl;
  D.print();
  std::cout << " = " << std::endl;
  if(blob::Matrix::multiply(C,D,R))
    R.print();
  else
    std::cout << "NaN" << std::endl;  
  std::cout << std::endl;

  R.refurbish(2,2);
  D.print();
  std::cout << " * " << std::endl;
  C.print();
  std::cout << " = " << std::endl;
  if(blob::Matrix::multiply(D,C,R))
    R.print();
  else
    std::cout << "NaN" << std::endl;  

  std::cout << std::endl;

  R.refurbish(5,2);
  A.print();
  std::cout << " * " << std::endl;
  C.print();
  std::cout << " = " << std::endl;
  if(blob::Matrix::multiply(A,C,R))
    R.print();
  else
    std::cout << "NaN" << std::endl;  
  std::cout << std::endl;

  R.refurbish(2,5);
  D.print();
  std::cout << " * " << std::endl;
  B.print();
  std::cout << " = " << std::endl;
  if(blob::Matrix::multiply(D,B,R))
    R.print();
  else
    std::cout << "NaN" << std::endl;  
  std::cout << std::endl;

  return true;
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
  
  blob::Matrix A(5,5,a);
  blob::Matrix C(5,3,c);
  blob::Matrix D(3,5,d);
  
  blob::Matrix R(5,5,r);
  
  std::cout << "test06_transpose" << std::endl << std::endl;
  
  A.print();
  std::cout << "T" << std::endl;
  std::cout << " = " << std::endl;
  
  if(blob::Matrix::transpose(A,R))
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
  blob::Matrix::transpose(C,R);
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
  blob::Matrix::transpose(D,R);
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
  
  blob::Matrix A(5,5,a);
  blob::Matrix L(5,5,l);
  blob::Matrix Lt(5,5,lt);
  blob::Matrix R(5,5,r);

  std::cout << "test07_cholesky" << std::endl << std::endl;

  L.print();
  std::cout << " * " << std::endl; 
  blob::Matrix::transpose(L,Lt);
  Lt.print();
  std::cout << " = " << std::endl; 
  blob::Matrix::multiply(L,Lt,A);
  A.print();
  std::cout << " => " << std::endl; 
  std::cout << "chol(" << std::endl;
  A.print();
  std::cout << ") = " << std::endl; 
  A.cholesky(true);
  A.print();
  std::cout << std::endl;

  blob::Matrix::multiply(L,Lt,A);
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

  real_t r[] = { 0.f, 0.f, 0.f, 0.f, 0.f,
                0.f, 0.f, 0.f, 0.f, 0.f,
                0.f, 0.f, 0.f, 0.f, 0.f,
                0.f, 0.f, 0.f, 0.f, 0.f,
                0.f, 0.f, 0.f, 0.f, 0.f };
  
  blob::Matrix A(5,5,a);
  blob::Matrix A1(5,5,a1);
  blob::Matrix R(5,5,r);

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
  blob::Matrix::multiply(A,A1,R);
  R.print();
  std::cout << std::endl;

  A1.zero();
  std::cout << "inv(" << std::endl;
  A.print();
  std::cout << ") = " << std::endl;
  blob::Matrix::inverse(A,A1);
  A1.print();
  std::cout << " => " << std::endl; 
  A.print();
  std::cout << " * " << std::endl; 
  A1.print();
  std::cout << " = " << std::endl; 
  blob::Matrix::multiply(A,A1,R);
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
  
  blob::Matrix A(5,5,a);
  blob::Matrix B(5,5,b);
  blob::Matrix R(5,5,r);

  std::cout << "test09_divide" << std::endl << std::endl;
  
  A.print();
  std::cout << " / " << std::endl;
  B.print();
  std::cout << " = " << std::endl;
  blob::Matrix::divide(A,B,R);
  R.print();
  std::cout << " = " << std::endl;
  B.inverse();
  blob::Matrix::multiply(A,B,R);
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
  blob::Matrix::divide(A,B,R);
  R.print();
  std::cout << " = " << std::endl;
  B.inverse();
  blob::Matrix::multiply(A,B,R);
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
  
  blob::Matrix A(5,2,a);
  blob::Matrix B(3,4,b);
  blob::Matrix D1(1,2,d1);
  blob::Matrix D2(4,1,d2);
  blob::Matrix D3(1,5,d3);
  blob::Matrix R(5,2,r);

  A.print();
  std::cout << " diag " << std::endl;
  D1.print();
  std::cout << " = " << std::endl;
  if(blob::Matrix::multiplyDiag(A,D1,R))
    R.print();
  else
    std::cout << "NaN" << std::endl;  
  std::cout << std::endl;
    
  A.print();
  std::cout << " diag " << std::endl;
  D2.print();
  std::cout << " = " << std::endl;
  if(blob::Matrix::multiplyDiag(A,D2,R))
    R.print();
  else
    std::cout << "NaN" << std::endl;  
  std::cout << std::endl;
  
  A.print();
  std::cout << " diag " << std::endl;
  D3.print();
  std::cout << " = " << std::endl;
  if(blob::Matrix::multiplyDiag(A,D3,R))
    R.print();
  else
    std::cout << "NaN" << std::endl;  
  std::cout << std::endl;
  
  R.refurbish(3,4);

  B.print();
  std::cout << " diag " << std::endl;
  D1.print();
  std::cout << " = " << std::endl;
  if(blob::Matrix::multiplyDiag(B,D1,R))
    R.print();
  else
    std::cout << "NaN" << std::endl;  
  std::cout << std::endl;
    
  B.print();
  std::cout << " diag " << std::endl;
  D2.print();
  std::cout << " = " << std::endl;
  if(blob::Matrix::multiplyDiag(B,D2,R))
    R.print();
  else
    std::cout << "NaN" << std::endl;  
  std::cout << std::endl;
  
  B.print();
  std::cout << " diag " << std::endl;
  D3.print();
  std::cout << " = " << std::endl;
  if(blob::Matrix::multiplyDiag(B,D3,R))
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
  
  blob::Matrix L(5,5,l);
  blob::Matrix v(1,5,ve);
  blob::Matrix R(5,5,r);

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
  
  blob::Matrix A(5,3,a);
  blob::Matrix Q(5,5,q);
  blob::Matrix R(5,3,r);

  std::cout << "test12_qr" << std::endl << std::endl;

  blob::Matrix::qr(A,Q,R);
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

  return 0;
}
