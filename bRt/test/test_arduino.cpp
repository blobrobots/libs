/********* blob robotics 2014 *********
 *  title: test_arduino.cpp
 *  brief: test for comms library
 * author: adrian jimenez-gonzalez
 * e-mail: blob.robotics@gmail.com
 /*************************************/

#include "Arduino.h"
#include "bTask.h"

class TestTask : public blob::Task 
{
  public:
    void init () { 
      Serial.println("task init");
    }
    void update () {
      static int i = 0;
      Serial.print("index: "); Serial.print(i++);
      Serial.print(" dtime: "); Serial.println(this->getTimeLapse());
    }
};

TestTask test;

void setup()
{  
  Serial.begin(115200);
  while (!Serial);

  test.init();
}

void loop()
{
  test.loop(10);
}
