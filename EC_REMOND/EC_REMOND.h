#ifndef EC_REMOND_H
#define EC_REMOND_H

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

static uint64_t _lastRead2=0;  // For millis ()
/*
 * Function prototypes
 */

float REMOND_EC_Get_Conductivity(ModbusMaster& node);
float REMOND_EC_Get_Resistivity(ModbusMaster& node);
float REMOND_EC_Get_Temperature(ModbusMaster& node);
float REMOND_EC_Get_Tds(ModbusMaster& node);
float REMOND_EC_Get_Salinity(ModbusMaster& node);


static bool ECupdateValues(ModbusMaster& node);

#endif
