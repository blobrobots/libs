/********* blob robotics 2014 *********
 *  title: test_arduino.cpp
 *  brief: test for comms library (arduino)
 * author: adrian jimenez-gonzalez
 * e-mail: blob.robotics@gmail.com
 /*************************************/

#include "Arduino.h"
#include "blob/matrix.h"

void setup()
{  
  Serial.begin(115200);
  while (!Serial);
}

void loop()
{
  delay(5000);
}
