#ifndef ECREMOND_H
#define ECREMOND_H

#include "Arduino.h"
#include <ModbusMaster.h>

/*
 *  Data structure for saving adquired values
 */
static struct {
  float conductivity;
  float resistivity;
  float temperature;
  float tds;
  float salinity;
} _eCcurrentValues ;

static uint64_t _lastRead=0;  // For millis ()
static uint8_t  _max485ReNegPin, _max485DePin ;   // For MAX485  DE and RE_NEG pins


/*
 * Function prototypes
 */

void REMONDWater_begin(uint8_t  max485ReNegPin,uint8_t max485DePin);
void preTransmission();
void postTransmission();

float REMOND_EC_Get_Conductivity(ModbusMaster& node);
float REMOND_EC_Get_Resistivity(ModbusMaster& node);
float REMOND_EC_Get_Temperature(ModbusMaster& node);
float REMOND_EC_Get_Tds(ModbusMaster& node);
float REMOND_EC_Get_Salinity(ModbusMaster& node);


static bool ECupdateValues(ModbusMaster& node);

#endif
