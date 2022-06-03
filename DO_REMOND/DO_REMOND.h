#ifndef DO_REMOND_H
#define DO_REMOND_H

#include "Arduino.h"
#include <ModbusMaster.h>

/*
 *  DO meter measure mode constants : mg/L or %  :: Result on register 0x0005 "Current Output Value
 */
const uint16_t measureModePercent=0x0000;
const uint16_t measureModeMgL=0x0001;


/*
 * Data strcuture for saving calibration value
 * Esta estructura de datos guarda los valores de calibracion del ADC 
 * For private use only
 */
static struct {
  uint16_t measuredADC;
} _DOcalibrationValue;


/*
 * Data structure para guardar los valores adquiridos del DO oxigeno disuelto
 * De uso privado solamente
 */
static struct {
  float measurements;
  float temperature;
  float currentOut;
  uint8_t warnings;
} _DOcurrentValues ;

static uint64_t _lastRead4=0;  // For millis ()

/*
 * Function prototypes para obtencion de medidas
 * Acceso publico
 */
float REMOND_DO_Get_Measurement(ModbusMaster& node);
float REMOND_DO_Get_Temperature(ModbusMaster& node);
float REMOND_DO_Get_Current_Output(ModbusMaster& node);
uint8_t REMOND_DO_Get_Warning(ModbusMaster& node);

/*
 * Function prototypes : Funcion principal de calibracion para el DO (disolved oxigen) REMOND
 * Acceso publico
 */
void REMOND_DO_Calibration(ModbusMaster& node, bool twoPoints);

/*
 * Funciones para la confirmacion de los puntos zero y slope
 * De uso privado solamnte
 */
static bool REMOND_DO_Set_Zero_calibration(ModbusMaster& node);
static bool REMOND_DO_Set_Slope_calibration(ModbusMaster& node);


/*
 * Funcion para la obtencion del valor del ADC en el registro 0x0066
 * Nota : De uso privado solamente
 */
static uint16_t REMOND_DO_Get_ADC_Value(ModbusMaster& node);

/*
 * Funcion que actualiza los valores de medicion del ADC 
 * Nota: De uso privado solamente
 */
static bool DOcalibrationUpdate(ModbusMaster& node);

/*
 * Funcion que actualiza los valores de medicion del DO
 * Nota: De uso privado solamente
 */
static bool DOUpdateValues(ModbusMaster& node);

#endif
