#ifndef TURBIDITY_REMOND_H
#define TURBIDITY_REMOND_H

#include "Arduino.h"
#include <ModbusMaster.h>


/*
 * Data structure para guardar los valores adquiridos del turbidimetro
 * De uso privado solamente
 */
static struct {
  float measurements;
  float temperature;
  float currentOut;
  uint8_t warnings;
} _turbiditYcurrentValues ;

static uint64_t _lastRead6=0;  // For millis ()

/*
 * Function prototypes para obtencion de medidas
 * Acceso publico
 */
float REMOND_TURB_Get_Measurement(ModbusMaster& node);
float REMOND_TURB_Get_Temperature(ModbusMaster& node);
float REMOND_TURB_Get_Current_Output(ModbusMaster& node);
uint8_t REMOND_TURB_Get_Warning(ModbusMaster& node);

/*
 * Funcion que actualiza los valores de medicion del turbidimetro
 * Nota: De uso privado solamente
 */
static bool turbiditYUpdateValues(ModbusMaster& node);

#endif
