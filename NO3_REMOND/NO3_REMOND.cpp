#include "Arduino.h"
#include "NO3_REMOND.h"
#include <ModbusMaster.h>


#define UPDATE_TIME_NO3       250



/*
 * Funcion que retorna el valor de la medida de NO3 ppm
 * 
 *    Parametros : 
 *        node        :  Objeto Modbus asigando al sensor  (&node)
 */
float REMOND_NO3_Get_Measurement(ModbusMaster& node){
  if(!NO3UpdateValues(node))
    return NAN;
  return _NO3currentValues.measurements;
}

/*
 * Funcion que retorna el valor de la temperatura 
 * 
 *    Parametros : 
 *        node        :  Objeto Modbus asigando al sensor  (&node)
 */
float REMOND_NO3_Get_Temperature(ModbusMaster& node){
  if(!NO3UpdateValues(node))
    return NAN;
  return _NO3currentValues.temperature;
}

/*
 * Funcion que retorna el valor de la corriente de salida basado en las mediciones de ION  o MV
 * 
 *    Parametros : 
 *        node        :  Objeto Modbus asigando al sensor  (&node)
 */
float REMOND_NO3_Get_Current_Output(ModbusMaster& node){
  if(!NO3UpdateValues(node))
    return NAN;
  return _NO3currentValues.currentOut; 
}

/*
 * Funcion que retorna el valor de la alarma ( 00, 01, 02 , 03, 04)
 * 
 *    Parametros : 
 *        node        :  Objeto Modbus asigando al sensor  (&node)
 */
uint8_t REMOND_NO3_Get_Warning(ModbusMaster& node){
  if(!NO3UpdateValues(node))
    return NAN;
  return _NO3currentValues.warnings; 
}


/*
 * Funcion que actualiza los valores de medidas del instrumento NO3 ppm
 * Nota : De uso privado solamente
 */
static bool NO3UpdateValues(ModbusMaster& node){
  static const uint8_t numReg=7;
  static uint16_t startAddress=0x0001;
  static uint16_t response[numReg];
  static uint8_t j,result;
  static uint32_t registerData=0x00000000;   // Data output pointer

  //Si se lee antes del tiempo limite, no actualizar
  if( (unsigned long)(millis() - ( _lastRead5) )  >  UPDATE_TIME_NO3){
        // Graba el tiempo actual
        _lastRead5 = millis();
  } else {
        return true;
  }

  //Leer numReg registros de 16 bit a partir de la direccion startAddress=0x001
  result=node.readHoldingRegisters(startAddress,(uint16_t) numReg);

  if(result == node.ku8MBSuccess)
  {
     for (j = 0; j < numReg; j++)  
     {
      response[j] = node.getResponseBuffer(j);  // Lecturas de medidas son las 2 primeras :  0x0001  -->> C D   ; 0x0002 -->> A B...
     }

    // Captura de las medidas ....
    
                    //AB                                   CD
     registerData=( ( response[1] ) << 16 ) & 0xFFFF0000 | response[0] ;
     _NO3currentValues.measurements=*((float*) &registerData );    // Cast HEX float to float value
     
                    //AB                                   CD
     registerData=( ( response[3] ) << 16 ) & 0xFFFF0000 | response[2] ;
     _NO3currentValues.temperature=*((float*) &registerData );    // Cast HEX float to float value

                   //AB                                   CD
     registerData=( ( response[5] ) << 16 ) & 0xFFFF0000 | response[4] ;
     _NO3currentValues.currentOut=*((float*) &registerData );    // Cast HEX float to float value

     _NO3currentValues.warnings= (uint8_t) ( response[6]) & 0x00FF ; // As warnings go from 00 to 04 only !!
     
     return true;                //Si hubo lectura exitosa, retorna True
      
  }else{
      return false;               // Si no hubo lectura exitosa, retorna error
  }

}
