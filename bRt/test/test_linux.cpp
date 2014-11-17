/********* blob robotics 2014 *********
 *  title: test.cpp
 *  brief: test for comms library
 * author: adrian jimenez-gonzalez
 * e-mail: blob.robotics@gmail.com
 /*************************************/
#include <iostream>
#include "bTask.h"

class TestTask : public blob::Task 
{
  public:
    void init () { 
      std::cout << "task init" << std::endl;
    }
    void update () {
      int i = 0;
      std::cout << "index: " << i++ << "dtime: " << getTimeLapse() << std::endl;
    }
};

int main(int argc, char* argv[])
{
  TestTask test;

  test.init();

  while (true)
  {
    test.loop(10);
  }

  return 0;
}
