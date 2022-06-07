#include <ModbusMaster.h>
#include "REMONDWater.h"
#include "phREMOND.h"             // Header con las cosntantes de calibracion

#define MAX485_DE          5
#define MAX485_RE_NEG      18

//#define CALIBRATION_MODE

// Constantes Globales
const uint8_t pHslaveID=0x02;    //Direccion seteada para el phimetro

// Variables globales
uint8_t sensorAddr;   //Variable de retorno de sensor ADDR
char Str_Measures[100]; // String para trama de medidas
float upperTemperatureLimit=55.1;    // Limites de temperatura
float lowerTemperatureLimit=5.1;

// Objeto Modbus para phimetro
ModbusMaster nodePH;

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
  nodePH.begin(pHslaveID,Serial2);      // Begin Node with ADDR..

 // Callbacks de seteo de modo para el PH ....
  nodePH.preTransmission(preTransmission);
  nodePH.postTransmission(postTransmission);

  // Set Baudrate on Remond Sensor
  if ( REMOND_Set_Baudrate (nodePH,baudrate9600)){ // Set default baudrate 9600
    Serial.println();
    Serial.println(" OK SET BAUDRATE 9600 bps ! ");
  }else{
    Serial.println(" COULD NOT SET BAUDRATE");
  }

// Set Upper Temperature Limit
  if (REMOND_Set_Upper_Temperature_Limit(nodePH,upperTemperatureLimit)){
    Serial.print(" OK SET UPPER TEMPERATURE LIMIT TO ");
    Serial.println(upperTemperatureLimit,1);
  }else{
    Serial.println ("COULD NOT SET UPPER LIMIT TEMP !!!");
  }

// Set Lower Temperature Limit
  if (REMOND_Set_Lower_TemperatureLimit(nodePH,lowerTemperatureLimit)){
    Serial.print(" OK SET LOWER TEMPERATURE LIMIT TO ");
    Serial.println(lowerTemperatureLimit,1);
  }else{
    Serial.println ("COULD NOT SET LOWER LIMIT TEMP !!!");
  }

// Read Device Address
 if (REMOND_Read_Device_Address (nodePH, true, sensorAddr) !=0){
    Serial.print(" DEVICE ADDRESS: ");
    Serial.println(sensorAddr, HEX);
  }else{
    Serial.println(" COULD NOT READ DEVICE ADDRESS !!!");
  }

  Serial.println();

  // Calibracion de phimetro
  #if defined(CALIBRATION_MODE)
    sensor_pH_Calibration(nodePH);
  #endif

}

void loop() {
  
  float phMeasure=REMOND_Get_pH(nodePH);
  float tempMeasure= REMOND_Get_Temperature(nodePH);
  float currentPHoRP=REMOND_Get_Current_PH_ORP(nodePH);
  uint8_t warningAlarm=REMOND_Get_Warning(nodePH);

  // Check if valid daata
  if (isnan(phMeasure)){
    Serial.println("Error reading pH !!");
  }else if (isnan(tempMeasure)){
    Serial.println("Error reading temperature !!");
  }else if(isnan(currentPHoRP)){
    Serial.println("Error reading current output value !!");
  }else if ( isnan(warningAlarm) ){
    Serial.println("Error reading warning !!");
  }else{
    
    // Print all values to serial console
    sprintf(Str_Measures,"Measures: pH=%.2f ; Temp=%.2f ; CurrOut=%.2f ; Warning=%u",phMeasure,tempMeasure,currentPHoRP,warningAlarm);
    Serial.println(Str_Measures);
    
  }
    Serial.println();


  delay(1000);
 
}
