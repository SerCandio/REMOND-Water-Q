#include <ModbusMaster.h>
#include "EC_REMOND.h"

#define MAX485_DE          5
#define MAX485_RE_NEG      18


// Constantes Globales
const uint8_t ECslaveID=0x01;    //Direccion default EC

// Variables globales
uint8_t sensorAddr;   //Variable de retorno de sensor ADDR
char Str_Measures[100]; // String para trama de medidas


// Objeto Modbus para el EC
ModbusMaster nodeEC;

void setup() {
  //port 0 debug
  Serial.begin (115200);

  // Setear los pines del MAX485 , 1 solo bus para los sensores
  REMONDWater_begin(MAX485_RE_NEG, MAX485_DE);

  //Communicate with Modbus Slave ID over Serial 2 
  //..............................(TX2, RX2) ESP32  
  Serial2.begin(9600);

  //Modbus node begin "slaveID"
  nodeEC.begin(ECslaveID,Serial2);      // Begin Node with ADDR..

  nodeEC.preTransmission(preTransmission);
  nodeEC.postTransmission(postTransmission);

}

void loop() {
  float conductivity=REMOND_EC_Get_Conductivity(nodeEC);
  float resistivity=REMOND_EC_Get_Resistivity(nodeEC);
  float temperature=REMOND_EC_Get_Temperature(nodeEC);
  float tds=REMOND_EC_Get_Tds(nodeEC);
  float salinity=REMOND_EC_Get_Salinity(nodeEC);

    // Check if valid daata
  if (isnan(conductivity)){
    Serial.println("Error reading conductivity!!");
  }else if (isnan(resistivity)){
    Serial.println("Error reading resistivity !!");
  }else if(isnan(temperature)){
    Serial.println("Error reading temperature !!");
  }else if ( isnan(tds) ){
    Serial.println("Error reading tds !!");
  }else if(isnan(salinity)){
    Serial.println("Error reading salinity");
  }else{
    
    // Print all values to serial console
    sprintf(Str_Measures,"Measures: Cond=%.2f ; Res=%.2f ; Temp=%.2f ; tds=%.2f; salinity=%.2f",conductivity,resistivity,temperature,tds, salinity);
    Serial.println(Str_Measures);
    
  }
    Serial.println();


  delay(1000);

}
