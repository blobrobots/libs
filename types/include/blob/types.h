/********* blob robotics 2014 *********
 *  title: types.h
 *  brief: Ttypes for blob environment
 * author: adrian jimenez-gonzalez
 * e-mail: blob.robotics@gmail.com
 **************************************/

#ifndef B_TYPES_H
#define B_TYPES_H


#if defined(__AVR_ATmega32U4__)
  #include "Arduino.h"
#endif

#if defined(__linux__)
  #include <stdio.h>
  #include <stdint.h>
#endif

#ifndef   byte
typedef   unsigned char byte;
#endif

#ifndef   word
typedef   unsigned short word;
#endif

#if !defined(__cplusplus)
	#ifndef   bool
	typedef   unsigned char bool;
	#endif
#endif

#endif // B_TYPES_H
