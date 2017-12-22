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
 * \file       task.h
 * \brief      interface for generic task
 * \author     adrian jimenez-gonzalez (blob.robots@gmail.com)
 * \copyright  the MIT License Copyright (c) 2015 Blob Robots.
 *
 ******************************************************************************/

#ifndef B_TASK_H
#define B_TASK_H

#include <blob/types.h>

namespace blob {

/**
 * Implements generic task class.
 */
class Task
{
  public:

    /**
     * Initializes task internal variables.
     */
    Task ();
    
    /**
     * Indicates if task has finished setup and working properly.
     * \return  true if ready, false otherwise
     */
    bool     isReady  ();

    /**
     * Provides task identifier.
     * \return  task identifier
     */
    uint8_t  getId    ();

    /**
     * Provides task loop index: number of loop executed from start.
     * \return  task loop index
     */
    uint16_t getIndex ();

    /**
     * Provides time lapse between last to loop executions.
     * \return  time lapse between last to loop executions
     */
    uint32_t getTimeLapse ();
    
    /**
     * Initializes parameters of task.
     * \return  true if successful, false otherwise
     */
    virtual bool init () {return true;}

    /**
     * Provides time lapse between last to loop executions.
     * \param ms  time lapse between consecutive loop executions
     * \return    true if successful, false otherwise
     */
    bool loop (uint32_t ms = 0);
    
    /**
     * Provides number of tasks currently in execution.
     * \return  number of tasks currently in execution
     */
    static uint8_t getNumberOfTasks ();
  
    /**
     * Stops execution for a specified time in milliseconds.
     * \return  time to stop in milliseconds
     */
    static void delayMs (uint32_t ms);

    /**
     * Provides timestamp in milliseconds.
     * \return  timestamp in milliseconds
     */
    static uint32_t timestampMs ();

  protected:
    /**
     * Function to be executed in loop (to be defined in inherited classes).
     * \return  true if successful, false otherwise
     */
    virtual bool update () = 0;
    /**
     * Checks if a specific time lapse has passed.
     * \param ms  time lapse in ms
     * \return    true if time lapse has passed, false otherwise
     */
    bool waited (uint32_t ms);

    static uint8_t _number; /**< number of tasks currently in execution */
    bool     _ready; /**< flag indicating setup end and task working properly */
    uint16_t _index; /**< number of loops executed from start */
    uint8_t  _id;    /**< task identifier  */
    uint32_t _waitStart; /**< last loop execution timestamp in ms */
    uint32_t _dt;        /**< time lapse between task loop executions in ms */
};
}
#endif /* B_TASK_H */
