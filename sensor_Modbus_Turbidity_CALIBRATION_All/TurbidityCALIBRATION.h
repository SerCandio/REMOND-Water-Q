#ifndef TurbidityCALIBRATION_H
#define TurbidityCALIBRATION_H

#include "Arduino.h"
#include <ModbusMaster.h>

/*
 * Turbidity 4 Point calbration constants  : : 4 puntos referenciales de calibracion
 */
const float firstPointCalibSolution=1.000;   // Valor del primer punto de calibracion en ppm
const float secondPointCalibSolution=10.000; // Valor del segundo punto de calibracion en ppm
const float thirdPointCalibSolution=20.000 ; // Valor del tercer punto de calibracion en ppm
const float fourthPointCalibSolution=30.000; // Valor del cuarto punto de calibracion en ppm

/*
 * Data strcuture for saving calibration value :: struct para almacenar el valor de calibracion del ADC
 * For private use only !!
 */
static struct {
  uint16_t measuredADC;
} _turbidityCalibrationValue;

/*
 * Data structure para guardar los valores adquiridos del TURBIDIMETRO  [NTU]
 * De uso privado solamente
 */
static struct {
  float measurements;
} _turbiditYcurrentValues ;

/*
 *  Buffer para funciones sprintf (,,,,)
 */
 
static char Str_Turbidity_frame[30];

static uint64_t _lastReadTurb=0;  // For millis ()

/*
 * Funcion para la obtencion de medidas
 * Acceso publico
 */
float REMOND_Turbidity_Get_Measurement(ModbusMaster& node);

/*
 * Funcion para la calibracion interactiva via consola ...........( PUTTY, TERA-TERM, etc)
 * Acceso publico
 */
void sensor_turbidity_Calibration(ModbusMaster& node);


/*
 * Funcion para el pre-seteo de los 4 primeros puntos de calibracion
 * Acceso privado
 */
static bool REMOND_Turbidity_Set_FirstPoint_calib(ModbusMaster& node, float firstPointCalibSolution);
static bool REMOND_Turbidity_Set_SecondPoint_calib(ModbusMaster& node, float secondPointCalibSolution);
static bool REMOND_Turbidity_Set_ThirdPoint_calib(ModbusMaster& node, float thirdPointCalibSolution);
static bool REMOND_Turbidity_Set_FourthPoint_calib(ModbusMaster& node, float fourthPointCalibSolution);

/*
 * Funciones que escriben los valores de calibracion de ADC ( 0x66) en los registro de ADC (0x22, 0x26, 0x2A,....)
 * Acceso privado
 */
static bool REMOND_Turbidity_Set_FirstADC_value(ModbusMaster& node, float adcValue);
static bool REMOND_Turbidity_Set_SecondADC_value(ModbusMaster& node, float adcValue);
static bool REMOND_Turbidity_Set_ThirdADC_value(ModbusMaster& node, float adcValue);
static bool REMOND_Turbidity_Set_FourthADC_value(ModbusMaster& node, float adcValue);

/*
 * Funcion que retorna el valor de calibracion del ADC
 * Nota: De uso privado solamente
 */
static uint16_t REMOND_Turbidity_Get_ADC_Value(ModbusMaster& node);

/*
 * Funcion que actualiza los valores de medicion del turbidimetro
 * Nota: De uso privado solamente
 */
static bool turbiditYUpdateValues(ModbusMaster& node);

/*
 * Funcion que actualiza los valores de calibracion del ADC
 * Nota: De uso privado solamente
 */
static bool REMOND_Turbidity_Calibration_Update(ModbusMaster& node);

#endif
