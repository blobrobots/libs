/********* blob robotics 2014 *********
 *  title: task.cpp
 *  brief: implemention of generic task
 * author: adrian jimenez-gonzalez
 * e-mail: blob.robotics@gmail.com
 **************************************/
#include "blob/task.h"

#if defined(__linux__)
  #include <unistd.h>
#endif //defined(__linux__)

uint8_t blob::Task::_number = 0;

blob::Task::Task () 
{
  if(_number < 255) {
    _id = _number; 
    _number++; 
  }
  _ready = false;
  _index = 0;
  _waitStart = 0;
}
    
bool  blob::Task::isReady  () {return _ready;}
uint8_t  blob::Task::getId    () {return _id;}
uint16_t blob::Task::getIndex () {return _index;}
uint32_t blob::Task::getTimeLapse () {return _dt;}
    
bool blob::Task::loop (uint32_t ms) 
{ 
  bool retval = false;
  if(waited(ms)) {
    update(); 
    retval = true;
  }
  return retval;
}

uint8_t blob::Task::getNumberOfTasks() {return _number;}
      
bool blob::Task::waited (uint32_t ms) 
{
  bool retval = false;
  
  if (ms) 
  {
#if defined(__AVR_ATmega32U4__)
    uint32_t now = millis(); // ms FIXME: add linux support
#elif defined(__linux__)
    uint32_t now = 0; // ms FIXME: add linux support
#endif
    if (_waitStart == 0)
    {
      _waitStart = now; 
    } 
    else 
    {
      uint32_t dt = now -_waitStart; // FIXME: add linux support
      if (dt >= ms) 
      {
        _dt = dt;
        _waitStart = now; 
        retval = true;
      }
    }
  } 
  else 
  { 
    retval = true;
  }
  return retval; 
}

void blob::Task::delayMs (uint32_t ms) 
{
#if defined(__AVR_ATmega32U4__)
  delay(ms);
#endif // defined(__AVR_ATmega32U4__)
#if defined(__linux__)
  usleep(1000*ms);
#endif // defined(__linux__)
}


