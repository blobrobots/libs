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
  float r[] = { 10.f, 10.f, 10.f, 10.f, 10.f,
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
  float a[] = { 1.f, 1.f, 1.f, 1.f, 1.f,
                1.f, 1.f, 1.f, 1.f, 1.f,
                1.f, 1.f, 1.f, 1.f, 1.f,
                1.f, 1.f, 1.f, 1.f, 1.f,
                1.f, 1.f, 1.f, 1.f, 1.f };

  float b[] = { 2.f, 2.f, 2.f, 2.f, 2.f,
                2.f, 2.f, 2.f, 2.f, 2.f,
                2.f, 2.f, 2.f, 2.f, 2.f,
                2.f, 2.f, 2.f, 2.f, 2.f,
                2.f, 2.f, 2.f, 2.f, 2.f };

  float r[] = { 0.f, 0.f, 0.f, 0.f, 0.f,
                0.f, 0.f, 0.f, 0.f, 0.f,
                0.f, 0.f, 0.f, 0.f, 0.f,
                0.f, 0.f, 0.f, 0.f, 0.f,
                0.f, 0.f, 0.f, 0.f, 0.f };
  
  blob::Matrix A(5,5,a);
  blob::Matrix B(5,5,b);
  blob::Matrix R(5,5,r);

  A.print();
  std::cout << " + " << std::endl;
  B.print();
  std::cout << " = " << std::endl;
  blob::Matrix::add(A,B,R);
  R.print();
  std::cout << std::endl;
  return true;
}

bool test02_fill()
{
  float a[] = { 1.f, 1.f, 1.f, 1.f, 1.f,
                1.f, 1.f, 1.f, 1.f, 1.f,
                1.f, 1.f, 1.f, 1.f, 1.f,
                1.f, 1.f, 1.f, 1.f, 1.f,
                1.f, 1.f, 1.f, 1.f, 1.f };

  float b[] = { 2.f, 2.f, 2.f, 2.f, 2.f,
                2.f, 2.f, 2.f, 2.f, 2.f,
                2.f, 2.f, 2.f, 2.f, 2.f,
                2.f, 2.f, 2.f, 2.f, 2.f,
                2.f, 2.f, 2.f, 2.f, 2.f };

  float c[] = { 3.f, 3.f,
                3.f, 3.f,
                3.f, 3.f,
                3.f, 3.f,
                3.f, 3.f };

  float d[] = { 4.f, 4.f, 4.f, 4.f, 4.f,
                4.f, 4.f, 4.f, 4.f, 4.f };

  blob::Matrix A(5,5,a);
  blob::Matrix B(5,5,b);
  blob::Matrix C(5,2,c);
  blob::Matrix D(2,5,d);

  A.fillCols(B,4,2);
  A.print();
  std::cout << std::endl;
  A.fillCols(C,3,2);
  A.print();
  std::cout << std::endl;
  A.fillRows(D,3,2);
  A.print();
  std::cout << std::endl;
  
  return true;
}

bool test03_scale()
{
  float r[] = { 10.f, 10.f, 10.f, 10.f, 10.f,
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
  float a[] = { 1.f, 1.f, 1.f, 1.f, 1.f,
                1.f, 1.f, 1.f, 1.f, 1.f,
                1.f, 1.f, 1.f, 1.f, 1.f,
                1.f, 1.f, 1.f, 1.f, 1.f,
                1.f, 1.f, 1.f, 1.f, 1.f };

  float b[] = { 2.f, 2.f, 2.f, 2.f, 2.f,
                2.f, 2.f, 2.f, 2.f, 2.f,
                2.f, 2.f, 2.f, 2.f, 2.f,
                2.f, 2.f, 2.f, 2.f, 2.f,
                2.f, 2.f, 2.f, 2.f, 2.f };

  float r[] = { 0.f, 0.f, 0.f, 0.f, 0.f,
                0.f, 0.f, 0.f, 0.f, 0.f,
                0.f, 0.f, 0.f, 0.f, 0.f,
                0.f, 0.f, 0.f, 0.f, 0.f,
                0.f, 0.f, 0.f, 0.f, 0.f };
  
  blob::Matrix A(5,5,a);
  blob::Matrix B(5,5,b);
  blob::Matrix R(5,5,r);

  A.print();
  std::cout << " - " << std::endl;
  B.print();
  std::cout << " = " << std::endl;
  blob::Matrix::substract(A,B,R);
  R.print();
  std::cout << std::endl;
  return true;
}

bool test05_multiply()
{
  float a[] = { 1.5f, 1.5f, 1.5f, 1.5f, 1.5f,
                1.5f, 1.5f, 1.5f, 1.5f, 1.5f,
                1.5f, 1.5f, 1.5f, 1.5f, 1.5f,
                1.5f, 1.5f, 1.5f, 1.5f, 1.5f,
                1.5f, 1.5f, 1.5f, 1.5f, 1.5f };

  float b[] = { 2.f, 2.f, 2.f, 2.f, 2.f,
                2.f, 2.f, 2.f, 2.f, 2.f,
                2.f, 2.f, 2.f, 2.f, 2.f,
                2.f, 2.f, 2.f, 2.f, 2.f,
                2.f, 2.f, 2.f, 2.f, 2.f };

  float r[] = { 0.f, 0.f, 0.f, 0.f, 0.f,
                0.f, 0.f, 0.f, 0.f, 0.f,
                0.f, 0.f, 0.f, 0.f, 0.f,
                0.f, 0.f, 0.f, 0.f, 0.f,
                0.f, 0.f, 0.f, 0.f, 0.f };
  
  blob::Matrix A(5,5,a);
  blob::Matrix B(5,5,b);
  blob::Matrix R(5,5,r);

  A.print();
  std::cout << " * " << std::endl;
  B.print();
  std::cout << " = " << std::endl;
  blob::Matrix::multiply(A,B,R);
  R.print();
  std::cout << std::endl;
  return true;
}

bool test06_transpose()
{
  float a[] = { 1.f, 2.f, 3.f, 4.f, 5.f,
                1.f, 2.f, 3.f, 4.f, 5.f,
                1.f, 2.f, 3.f, 4.f, 5.f,
                1.f, 2.f, 3.f, 4.f, 5.f,
                1.f, 2.f, 3.f, 4.f, 5.f };

  float r[] = { 0.f, 0.f, 0.f, 0.f, 0.f,
                0.f, 0.f, 0.f, 0.f, 0.f,
                0.f, 0.f, 0.f, 0.f, 0.f,
                0.f, 0.f, 0.f, 0.f, 0.f,
                0.f, 0.f, 0.f, 0.f, 0.f };
  
  blob::Matrix A(5,5,a);
  blob::Matrix R(5,5,r);

  A.print();
  std::cout << "T" << std::endl;
  std::cout << " = " << std::endl;
  blob::Matrix::transpose(A,R);
  R.print();
  std::cout << std::endl;
  return true;
}

bool test07_cholesky()
{
  float a[] = { 1.f, 2.f, 3.f, 4.f, 5.f,
                1.f, 2.f, 3.f, 4.f, 5.f,
                1.f, 2.f, 3.f, 4.f, 5.f,
                1.f, 2.f, 3.f, 4.f, 5.f,
                1.f, 2.f, 3.f, 4.f, 5.f };

  float l[] = { 1.f, 0.f, 0.f, 0.f, 0.f,
                2.f, 2.f, 0.f, 0.f, 0.f,
                3.f, 3.f, 3.f, 0.f, 0.f,
                4.f, 4.f, 4.f, 4.f, 0.f,
                5.f, 5.f, 5.f, 5.f, 5.f };
  
  float lt[] = { 0.f, 0.f, 0.f, 0.f, 0.f,
                 0.f, 0.f, 0.f, 0.f, 0.f,
                 0.f, 0.f, 0.f, 0.f, 0.f,
                 0.f, 0.f, 0.f, 0.f, 0.f,
                 0.f, 0.f, 0.f, 0.f, 0.f };
  
  float r[] = { 0.f, 0.f, 0.f, 0.f, 0.f,
                0.f, 0.f, 0.f, 0.f, 0.f,
                0.f, 0.f, 0.f, 0.f, 0.f,
                0.f, 0.f, 0.f, 0.f, 0.f,
                0.f, 0.f, 0.f, 0.f, 0.f };
  
  blob::Matrix A(5,5,a);
  blob::Matrix L(5,5,l);
  blob::Matrix Lt(5,5,lt);
  blob::Matrix R(5,5,r);

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
  blob::Matrix::cholesky(A);
  A.print();
  std::cout << std::endl;
  return true;
}

bool test08_inverse()
{
  float a[] = { 1.f, 2.f, 3.f, 4.f, 5.f,
                2.f, 8.f, 12.f, 16.f, 20.f,
                3.f, 12.f, 27.f, 36.f, 45.f,
                4.f, 16.f, 36.f, 64.f, 80.f,
                5.f, 20.f, 45.f, 80.f, 125.f };

  float a1[] = { 1.f, 2.f, 3.f, 4.f, 5.f,
                 2.f, 8.f, 12.f, 16.f, 20.f,
                 3.f, 12.f, 27.f, 36.f, 45.f,
                 4.f, 16.f, 36.f, 64.f, 80.f,
                 5.f, 20.f, 45.f, 80.f, 125.f };

  float r[] = { 0.f, 0.f, 0.f, 0.f, 0.f,
                0.f, 0.f, 0.f, 0.f, 0.f,
                0.f, 0.f, 0.f, 0.f, 0.f,
                0.f, 0.f, 0.f, 0.f, 0.f,
                0.f, 0.f, 0.f, 0.f, 0.f };
  
  blob::Matrix A(5,5,a);
  blob::Matrix A1(5,5,a1);
  blob::Matrix R(5,5,r);


  std::cout << "inv(" << std::endl;
  A1.print();
  std::cout << ") = " << std::endl;
  blob::Matrix::inverse(A1);
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

int main(int argc, char* argv[])
{

  test00_eye ();
  test01_add ();
  test02_fill();
  test03_scale();
  test04_substract();
  test05_multiply();
  test06_transpose();
  test07_cholesky();
  test08_inverse();
  return 0;
}
