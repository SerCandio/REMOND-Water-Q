#include <ModbusMaster.h>
#include "REMONDWater.h"
#include "NO3REMOND.h"             // Header con las cosntantes de calibracion

#define MAX485_DE          5
#define MAX485_RE_NEG      18

//#define CALIBRATION_MODE

// Constantes Globales
const uint8_t NO3slaveID=0x03;    //Direccion default NO3

// Variables globales
uint8_t sensorAddr;   //Variable de retorno de sensor ADDR
char Str_Measures[100]; // String para trama de medidas
float upperTemperatureLimit=55.1;    // Limites de temperatura
float lowerTemperatureLimit=5.1;

// Objeto Modbus para el NO3 ppm
ModbusMaster nodeNO3;

// Los demas instrumentos deben ser instanciados tambien con su propio objeto de Modbus
// ..............


void setup() {

  //port 0 debug
  Serial.begin (115200);

  // Setear los pines del MAX485 , 1 solo bus para los sensores
  REMONDWater_begin(MAX485_RE_NEG, MAX485_DE);

  //Communicate with Modbus Slave ID over Serial 2 
  //..............................(TX2, RX2) ESP32  
  Serial2.begin(9600);

  //Modbus node begin "slaveID"
  nodeNO3.begin(NO3slaveID,Serial2);      // Begin Node with ADDR..

 // Callbacks de seteo de modo para el PH ....
  nodeNO3.preTransmission(preTransmission);
  nodeNO3.postTransmission(postTransmission);

  // Set Baudrate on Remond Sensor
  if ( REMOND_Set_Baudrate (nodeNO3,baudrate9600)){ // Set default baudrate 9600
    Serial.println();
    Serial.println(" OK SET BAUDRATE 9600 bps ! ");
  }else{
    Serial.println(" COULD NOT SET BAUDRATE");
  }

// Set Upper Temperature Limit
  if (REMOND_Set_Upper_Temperature_Limit(nodeNO3,upperTemperatureLimit)){
    Serial.print(" OK SET UPPER TEMPERATURE LIMIT TO ");
    Serial.println(upperTemperatureLimit,1);
  }else{
    Serial.println ("COULD NOT SET UPPER LIMIT TEMP !!!");
  }

// Set Lower Temperature Limit
  if (REMOND_Set_Lower_TemperatureLimit(nodeNO3,lowerTemperatureLimit)){
    Serial.print(" OK SET LOWER TEMPERATURE LIMIT TO ");
    Serial.println(lowerTemperatureLimit,1);
  }else{
    Serial.println ("COULD NOT SET LOWER LIMIT TEMP !!!");
  }


// Cambiar direccion a 0x03.........Descomentar si ya no cambia la direccion
/*
if (REMOND_Set_Device_Address (nodeNO3, 0x03)){
  Serial.println(" OK SET SLAVE ADDR TO 0x03");
  while(1);
}else{
  Serial.println("COULD NOT SER DEVICE ADDR");
}
  */

// Read Device Address
 if (REMOND_Read_Device_Address (nodeNO3, true, sensorAddr) !=0){
    Serial.print(" DEVICE ADDRESS: ");
    Serial.println(sensorAddr, HEX);
  }else{
    Serial.println(" COULD NOT READ DEVICE ADDRESS !!!");
  }

  Serial.println();

  // Calibracion ....
  #if defined(CALIBRATION_MODE)
    sensor_NO3_Calibration(nodeNO3); 
  #endif

}

void loop() {
  
  float NO3ppmMeasure=REMOND_Get_Measurement(nodeNO3);
  float tempMeasure= REMOND_Get_Temperature(nodeNO3);
  float currentION_MV=REMOND_Get_Current_ION_MV(nodeNO3);
  uint8_t warningAlarm=REMOND_Get_Warning(nodeNO3);

  // Check if valid daata
  if (isnan(NO3ppmMeasure)){
    Serial.println("Error reading NO3 measurements !!");
  }else if (isnan(tempMeasure)){
    Serial.println("Error reading temperature !!");
  }else if(isnan(currentION_MV)){
    Serial.println("Error reading current output value [ION /MV] !!");
  }else if ( isnan(warningAlarm) ){
    Serial.println("Error reading warning !!");
  }else{
    
    // Print all values to serial console
    sprintf(Str_Measures,"Measures: ppm=%.2f ; Temp=%.2f ; CurrOut=%.2f ; Warning=%u",NO3ppmMeasure,tempMeasure,currentION_MV,warningAlarm );
    Serial.println(Str_Measures);
    
  }
    Serial.println();

  delay(1000);
 
}
