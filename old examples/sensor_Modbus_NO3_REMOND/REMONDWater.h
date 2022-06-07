#ifndef REMONDWater_H
#define REMONDWater_H

#include "Arduino.h"
#include <ModbusMaster.h>

/*
 * Baudrate constants
 */
const uint16_t baudrate2400=0x0000; // Baudrate 2400 bps
const uint16_t baudrate4800=0x0001; // Baudrate 4800 bps
const uint16_t baudrate9600=0x0002; // Baudrate 9600 bps
const uint16_t baudrate19200=0x0003; // Baudrate 19200 bps
const uint16_t baudrate38400=0x0004; // Baudrate 38400 bps


/*
 * Warning Flags
*/
const uint16_t measureNormal=0x0000;            //No Warning
const uint16_t temperatureExceedsUpperLimit=0x0003;  // Temperature excceds Upper Limit
const uint16_t temperatureExceedsLowerLimit=0x0004;  // Temperature excceds Lower Limit

/*
 *  Data structure for saving adquired values
 */
static struct {
  float measurements;
  float temperature;
  float currentOut;
  uint8_t warnings;
} _currentValues ;

/*
 * Data strcuture for saving calibration value
 * For provate use only
 */
static struct {
  uint16_t measuredADC;
} _calibrationValue;

static uint64_t _lastRead=0;  // For millis ()
static uint8_t  _max485ReNegPin, _max485DePin ;   // For MAX485  DE and RE_NEG pins

/*
 * Function prototypes
 */

void REMONDWater_begin(uint8_t  max485ReNegPin,uint8_t max485DePin);
void preTransmission();
void postTransmission();


float REMOND_Get_Measurement(ModbusMaster& node);
float REMOND_Get_Temperature(ModbusMaster& node);
float REMOND_Get_Current_ION_MV(ModbusMaster& node);
uint8_t REMOND_Get_Warning(ModbusMaster& node);
uint16_t REMOND_Get_ADC_Value(ModbusMaster& node);

bool REMOND_Set_Zero_calib_NO3(ModbusMaster& node, uint32_t zeroCalibSolution);
bool REMOND_Set_Slope_calib_NO3(ModbusMaster& node,uint32_t slopeCalibSolution);

bool REMOND_Set_Zero_confirmation(ModbusMaster& node);
bool REMOND_Set_Slope_confirmation(ModbusMaster& node);

bool REMOND_Set_Measure_Mode(ModbusMaster& node, uint16_t measureMode);
bool REMOND_Set_Upper_Temperature_Limit(ModbusMaster& node, float upperTemperatureLimit);
bool REMOND_Set_Lower_TemperatureLimit(ModbusMaster& node, float lowerTemperatureLimit);
bool REMOND_Set_Baudrate (ModbusMaster& node, uint16_t baudrate);
bool REMOND_Set_Device_Address (ModbusMaster& node, uint8_t slaveAddressID);
uint8_t REMOND_Read_Device_Address (ModbusMaster& node, bool updateAddr,uint8_t &customID);

void sensor_NO3_Calibration(ModbusMaster& node);         // For calibrating NO3 only !!

static bool updateValues(ModbusMaster& node);
static bool calibrationUpdate(ModbusMaster& node);


#endif
