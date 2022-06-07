#include <ModbusMaster.h>
#include "REMONDWater.h"
#include "TURBIDITY_REMOND.h"

#define MAX485_DE          5
#define MAX485_RE_NEG      18

//#define CALIBRATION_MODE        // Descomentar para calibrar

// Constantes Globales
//const uint8_t TURBslaveID=0x01;    //Direccion default 
const uint8_t TURBslaveID=0x04;    

// Variables globales
uint8_t sensorAddr;   //Variable de retorno de sensor ADDR
char Str_Measures[100]; // String para trama de medidas
float upperTemperatureLimit=55.1;    // Limites de temperatura
float lowerTemperatureLimit=5.1;

// Objeto Modbus para el NO3 ppm
ModbusMaster nodeTURB;

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
  nodeTURB.begin(TURBslaveID,Serial2);      // Begin Node with ADDR..

 // Callbacks de seteo de modo para el turbidimetro ....
  nodeTURB.preTransmission(preTransmission);
  nodeTURB.postTransmission(postTransmission);

  // Set Baudrate on Remond Sensor
  if ( REMOND_Set_Baudrate (nodeTURB,baudrate9600)){ // Set default baudrate 9600
    Serial.println();
    Serial.println(" OK SET BAUDRATE 9600 bps ! ");
  }else{
    Serial.println(" COULD NOT SET BAUDRATE");
  }

// Set Upper Temperature Limit
  if (REMOND_Set_Upper_Temperature_Limit(nodeTURB,upperTemperatureLimit)){
    Serial.print(" OK SET UPPER TEMPERATURE LIMIT TO ");
    Serial.println(upperTemperatureLimit,1);
  }else{
    Serial.println ("COULD NOT SET UPPER LIMIT TEMP !!!");
  }

// Set Lower Temperature Limit
  if (REMOND_Set_Lower_TemperatureLimit(nodeTURB,lowerTemperatureLimit)){
    Serial.print(" OK SET LOWER TEMPERATURE LIMIT TO ");
    Serial.println(lowerTemperatureLimit,1);
  }else{
    Serial.println ("COULD NOT SET LOWER LIMIT TEMP !!!");
  }

// Read Device Address
 if (REMOND_Read_Device_Address (nodeTURB, true, sensorAddr) !=0){
    Serial.print(" DEVICE ADDRESS: ");
    Serial.println(sensorAddr, HEX);
  }else{
    Serial.println(" COULD NOT READ DEVICE ADDRESS !!!");
  }

 // Set New Address
 /*
 if (REMOND_Set_Device_Address (nodeTURB, 0x04)){
   Serial.println("OK SET NEW ADDRESS TO 4");
   while(1);
 }else{
  Serial.println(" COULD NOT SET NEW ADDRESS !!!");
 }*/
  Serial.println();

  // Calibracion ....
  #if defined(CALIBRATION_MODE)
    sensor_TURBIDITY_Calibration(nodeTURB);
  #endif

}

void loop() {
  
  float NTU_L_Measure=REMOND_Get_Measurement(nodeTURB);
  float tempMeasure= REMOND_Get_Temperature(nodeTURB);
  float currentOutput=REMOND_Get_Output_Current(nodeTURB);
  uint8_t warningAlarm=REMOND_Get_Warning(nodeTURB);

  // Check if valid daata
  if (isnan(NTU_L_Measure)){
    Serial.println("Error reading DO measurements !!");
  }else if (isnan(tempMeasure)){
    Serial.println("Error reading temperature !!");
  }else if(isnan(currentOutput)){
    Serial.println("Error reading current output value [% /mg/L] !!");
  }else if ( isnan(warningAlarm) ){
    Serial.println("Error reading warning !!");
  }else{
    
    // Print all values to serial console
    sprintf(Str_Measures,"Measures: TURB_NTU=%.2f ; Temp=%.2f ; CurrOut=%.2f ; Warning=%u",NTU_L_Measure,tempMeasure,currentOutput,warningAlarm );
    Serial.println(Str_Measures);
    
  }
    Serial.println();

  delay(1000);
 
}
