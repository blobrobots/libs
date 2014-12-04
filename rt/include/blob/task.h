/********* blob robotics 2014 *********
 *  title: task.h
 *  brief: interface for generic task
 * author: adrian jimenez-gonzalez
 * e-mail: blob.robotics@gmail.com
 **************************************/
#ifndef B_TASK_H
#define B_TASK_H

#include <blob/types.h>

namespace blob {

class Task
{
  public:
    Task ();
    
    bool     isReady  ();
    uint8_t  getId    ();
    uint16_t getIndex ();
    uint32_t getTimeLapse ();
    
    virtual void init   () = 0; //! setup function, to be defined in inherited classes
    virtual void update () = 0; //! loop function, to be defined in inherited classes
    
    bool  loop (uint32_t ms = 0);
    
    static uint8_t getNumberOfTasks ();
  
    static void delayMs (uint32_t ms);
  protected:
    static uint8_t _number;
    bool     _ready;  // to be updated externally when init() is completed correctly
    uint16_t _index; // to be updated externally each time a task completes one update
    uint8_t  _id;
    uint32_t _waitStart;
    uint32_t _dt;
    
    bool waited (uint32_t ms);
};
}

#endif /* B_TASK_H */
