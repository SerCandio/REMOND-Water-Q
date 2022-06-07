#ifndef phREMOND_H
#define phREMOND_H

#include "Arduino.h"

/*
 * phimeter zero point calibration comstants  : ph=6.86 or ph=7.00 only !!
 */
const uint16_t zeropH_7_Solution=0x0000 ; // Zero point calibration constant for ph=7 solution
const uint16_t zeropH_6_86_Solution=0x0001 ; // Zero point calibration constant for ph=7 solution


/*
 * phimeter slope point calibration comstants  : ph=1.68, ph=4.01 , ph=9.18, ph=10.1, ph=12.45
 */
const uint16_t slopepH_1_68_Solution=0x0000; // Slope point calibration constant for ph=1.68
const uint16_t slopepH_4_01_Solution=0x0001; // Slope point calibration constant for ph=4.01
const uint16_t slopepH_9_18_Solution=0x0002; // Slope point calibration constant for ph=9.18
const uint16_t slopepH_10_1_Solution=0x0003; // Slope point calibration constant for ph=10.1
const uint16_t slopepH_12_45_Solution=0x0004; // Slope point calibration constant for ph=12.45

/*
 * phimeter measure mode constants : PH  , ORP  :: So output current on register 0x0007 will be based on!
 */

const uint16_t measureModePH=0x0000; // Measure Mode pH
const uint16_t measureModeORP=0x0001; // Measure Mode 

#endif
