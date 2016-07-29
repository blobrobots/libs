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
 * \file       task.cpp
 * \brief      implemention of generic task
 * \author     adrian jimenez-gonzalez (blob.robots@gmail.com)
 * \copyright  the MIT License Copyright (c) 2015 Blob Robots.
 *
 ******************************************************************************/

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

uint8_t blob::Task::getNumberOfTasks() {return _number;}

bool blob::Task::loop (uint32_t ms) 
{ 
  bool retval = false;
  
  if(waited(ms) && update()) 
  {
    _index++;
    _ready = true;
    retval = true;
  }
  else
  {
    _ready = false;
  }
  return retval;
}
      
bool blob::Task::waited (uint32_t ms) 
{
  bool retval = false;
  
  if (ms) 
  {
    uint32_t now = timestampMs();

    if (_waitStart == 0)
    {
      _waitStart = now; 
    } 
    else 
    {
      uint32_t dt = now -_waitStart;
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

uint32_t blob::Task::timestampMs ()
{
#if defined(__AVR_ATmega32U4__)
  return millis();
#elif defined (__linux__)
  struct timeval tv;
  gettimeofday(&tv,NULL);
  return 1000*(uint32_t)(tv.tv_sec + 0.000001*tv.tv_usec);
#endif
}


