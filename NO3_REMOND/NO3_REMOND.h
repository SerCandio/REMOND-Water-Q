#ifndef NO3REMOND_H
#define NO3REMOND_H

#include "Arduino.h"
#include <ModbusMaster.h>

/*
 * NO3 meter zero and slope calibration points : 10.00 and 100.00 ppm 
 * .....................Uso futuro !! ..............................
 */
const uint32_t zeroNO3_10_00_Solution=0x41200000;
const uint32_t slopeNO3_100_00_Solution=0x42c80000 ;


/*
 * Data strcuture for saving calibration value
 * Esta estructura de datos guarda los valores de calibracion del ADC (pH, NO3, DO )
 * For private use only
 * 
 * ............................Futura Implementacion !! .......................
 */
static struct {
  uint16_t measuredADC;
} _NO3calibrationValue;

/*
 * Data structure para guardar los valores adquiridos del ph imetro
 * De uso privado solamente
 */
static struct {
  float measurements;
  float temperature;
  float currentOut;
  uint8_t warnings;
} _NO3currentValues ;

static uint64_t _lastRead5=0;  // For millis ()

/*
 * Function prototypes para obtencion de medidas
 * Acceso publico
 */
float REMOND_NO3_Get_Measurement(ModbusMaster& node);
float REMOND_NO3_Get_Temperature(ModbusMaster& node);
float REMOND_NO3_Get_Current_Output(ModbusMaster& node);
uint8_t REMOND_NO3_Get_Warning(ModbusMaster& node);

/*
 * Funcion que actualiza los valores de medicion del pH
 * Nota: De uso privado solamente
 */
static bool NO3UpdateValues(ModbusMaster& node);

#endif
