#ifndef phREMOND_H
#define phREMOND_H

#include "Arduino.h"
#include <ModbusMaster.h>


/*
 * phimeter zero point calibration comstants  : ph=6.86 or ph=7.00 only !!
 * Acceso Publico
 */
const uint16_t zeropH_7_Solution=0x0000 ; // Zero point calibration constant for ph=7 solution
const uint16_t zeropH_6_86_Solution=0x0001 ; // Zero point calibration constant for ph=7 solution


/*
 * phimeter slope point calibration comstants  : ph=1.68, ph=4.01 , ph=9.18, ph=10.1, ph=12.45
 * Acceso publico
 */
const uint16_t slopepH_1_68_Solution=0x0000; // Slope point calibration constant for ph=1.68
const uint16_t slopepH_4_01_Solution=0x0001; // Slope point calibration constant for ph=4.01
const uint16_t slopepH_9_18_Solution=0x0002; // Slope point calibration constant for ph=9.18
const uint16_t slopepH_10_1_Solution=0x0003; // Slope point calibration constant for ph=10.1
const uint16_t slopepH_12_45_Solution=0x0004; // Slope point calibration constant for ph=12.45

/*
 * phimeter measure mode constants : PH  , ORP  :: So output current on register 0x0005 will be based on!
 * Acceso publico
 */

const uint16_t measureModePH=0x0000; // Measure Mode pH
const uint16_t measureModeORP=0x0001; // Measure Mode 

/*
 * Data strcuture for saving calibration value
 * Esta estructura de datos guarda los valores de calibracion del ADC (pH, NO3, DO )
 * For private use only
 */
static struct {
  uint16_t measuredADC;
} _pHcalibrationValue;


/*
 * Data structure para guardar los valores adquiridos del ph imetro
 * De uso privado solamente
 */
static struct {
  float measurements;
  float temperature;
  float currentOut;
  uint8_t warnings;
} _pHcurrentValues ;


/*
 * Function prototypes para obtencion de medidas
 * Acceso publico
 */
float REMOND_pH_Get_Measurement(ModbusMaster& node);
float REMOND_pH_Get_Temperature(ModbusMaster& node);
float REMOND_pH_Get_Current_Output(ModbusMaster& node);
uint8_t REMOND_pH_Get_Warning(ModbusMaster& node);

/*
 * Function prototypes : Funcion principal de calibracion para el ph-imetro REMOND
 * Acceso publico
 */
void REMOND_pH_Calibration(ModbusMaster& node);


/*
 * Funciones para el pre-seteo de los puntos de calibracion cero o slope
 * Nota : De uso privado solamente
 */
static bool REMOND_ph_Set_Zero_calib(ModbusMaster& node, uint16_t zeroCalibSolution);
static bool REMOND_ph_Set_Slope_calib(ModbusMaster& node,uint16_t slopeCalibSolution);
static bool REMOND_ph_Set_Zero_confirmation(ModbusMaster& node);
static bool REMOND_ph_Set_Slope_confirmation(ModbusMaster& node);


static uint64_t _lastRead3=0;  // For millis ()

/*
 * Funcion para la obtencion del valor del ADC en el registro 0x0066
 * Nota : De uso privado solamente
 */
static uint16_t REMOND_ph_Get_ADC_Value(ModbusMaster& node);

/*
 * Funcion que actualiza los valores de medicion del ADC 
 * Nota: De uso privado solamente
 */
static bool pHcalibrationUpdate(ModbusMaster& node);

/*
 * Funcion que actualiza los valores de medicion del pH
 * Nota: De uso privado solamente
 */
static bool pHUpdateValues(ModbusMaster& node);

#endif
