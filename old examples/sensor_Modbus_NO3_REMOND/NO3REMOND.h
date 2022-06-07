#ifndef NO3REMOND_H
#define NO3REMOND_H

#include "Arduino.h"

/*
 * NO3 meter zero and slope calibration points : 10.00 and 100.00 ppm 
 */
const uint32_t zeroNO3_10_00_Solution=0x41200000;
const uint32_t slopeNO3_100_00_Solution=0x42c80000 ;

/*
 * NO3 meter measure mode constants : ION or mV  :: Result on register 0x0005 "Current Output Value
 */
const uint16_t measureModeION=0x0000;
const uint16_t measureModeMV=0x0001;


#endif
