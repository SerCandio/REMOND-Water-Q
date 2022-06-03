#include "Arduino.h"
#include "EC_REMOND.h"
#include <ModbusMaster.h>

#define UPDATE_TIME_EC        250

#define REG_EC_REMOND_DEVICE_ADDRESS  0x0014


float REMOND_EC_Get_Conductivity(ModbusMaster& node){
  if( !ECupdateValues(node) )  //Actualizar si es necesario
    return NAN;         //En caso de error, retorna Nan
    
  return _eCcurrentValues.conductivity;  
}

float REMOND_EC_Get_Resistivity(ModbusMaster& node){
 if( !ECupdateValues(node) )  //Actualizar si es necesario
    return NAN;         //En caso de error, retorna Nan
    
  return _eCcurrentValues.resistivity;  
}

float REMOND_EC_Get_Temperature(ModbusMaster& node){
  if( !ECupdateValues(node) )  //Actualizar si es necesario
    return NAN;         //En caso de error, retorna Nan
    
  return _eCcurrentValues.temperature; 
}

float REMOND_EC_Get_Tds(ModbusMaster& node){
 if( !ECupdateValues(node) )  //Actualizar si es necesario
    return NAN;         //En caso de error, retorna Nan
    
  return _eCcurrentValues.tds;   
}

float REMOND_EC_Get_Salinity(ModbusMaster& node){
 if( !ECupdateValues(node) )  //Actualizar si es necesario
    return NAN;         //En caso de error, retorna Nan
    
  return _eCcurrentValues.salinity;   
}

static bool ECupdateValues(ModbusMaster& node){
  static const uint8_t numReg=10;
  static uint16_t startAddress=0x0000;
  static uint16_t response[numReg];
  static uint8_t j,result;
  static uint32_t registerData=0x00000000;   // Data output pointer

  //Si se lee antes del tiempo limite, no actualizar
  if( (unsigned long)(millis() - ( _lastRead2) )  >  UPDATE_TIME_EC){
        // Graba el tiempo actual
        _lastRead2 = millis();
  } else {
        return true;
  }

  //Leer numReg=(2) registros de 16 bit a partir de la direccion startAddress
  result=node.readHoldingRegisters(startAddress,(uint16_t) numReg);

  if(result == node.ku8MBSuccess)
  {
     for (j = 0; j < numReg; j++)  
     {
      response[j] = node.getResponseBuffer(j);  // Lecturas de medidas son las 2 primeras :  0x0001  -->> C D   ; 0x0002 -->> A B......... por probar
     }

    // Medicion de pH.......
    
                    //AB                                   CD
     registerData=( ( response[1] ) << 16 ) & 0xFFFF0000 | response[0] ;
     _eCcurrentValues.conductivity=*((float*) &registerData );    // Cast HEX float to float value
     
                    //AB                                   CD
     registerData=( ( response[3] ) << 16 ) & 0xFFFF0000 | response[2] ;
     _eCcurrentValues.resistivity=*((float*) &registerData );    // Cast HEX float to float value

                   //AB                                   CD
     registerData=( ( response[5] ) << 16 ) & 0xFFFF0000 | response[4] ;
     _eCcurrentValues.temperature=*((float*) &registerData );    // Cast HEX float to float value

                   //AB                                   CD
     registerData=( ( response[7] ) << 16 ) & 0xFFFF0000 | response[6] ;
     _eCcurrentValues.tds=*((float*) &registerData );    // Cast HEX float to float value

                    //AB                                   CD
     registerData=( ( response[9] ) << 16 ) & 0xFFFF0000 | response[8] ;
     _eCcurrentValues.salinity=*((float*) &registerData );    // Cast HEX float to float value
    

     return true;                //Si hubo lectura exitosa, retorna True
      
  }else{
      return false;               // Si no hubo lectura exitosa, retorna error
  }
}
