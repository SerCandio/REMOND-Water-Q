#ifndef TURBIDITY_REMOND_H
#define TURBIDITY_REMOND_H

#include "Arduino.h"

/*
 * TURBIDITY sensor REMOND calibration constansts (floating point)
 * Can be free changed by user........  (se pueden establecer libremente
 * 
 * OBS: Usar unicamete en esta funcion  :   REMOND_TURBIDITY_Set_float_Calibration_Points(.. )
 */
const float turbiditYfirstPointCalib=1.000;
const float turbiditYSecondPointCalib=10.000;
const float turbiditYThirdPointCalib=20.000;
const float turbiditYFourthPointCalib=30.000;

/*
 * TURBIDITY sensor REMOND calibration constansts (Double word)
 * 
 * Nota : Ya predefinidos en la hoja de datos (ver pagina 5)
 * 
 * OBS: Usar unicamete en esta funcion  :   REMOND_TURBIDITY_Set_DWord_Calibration_Points(.. )
 */
const uint32_t turbiditYfirstDWordPointCalib=0x3f800000;   // Representa el punto 1.000 de calibracion
const uint32_t turbiditYSecondDWordPointCalib=0x41200000; // Representa el punto 10.000 de calibracion
const uint32_t turbiditYThirdDWordPointCalib=0x41a00000;  // Representa el punto 20.000 de calibracion
const uint32_t turbiditYFourthDWordPointCalib=0x41f00000; // Representa el punto 30.000 de calibracion


#endif
