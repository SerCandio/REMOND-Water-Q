#include "Arduino.h"
#include "phREMOND.h"
#include <ModbusMaster.h>


#define UPDATE_TIME_PH        250

#define REG_PH_ZERO_POINT_CALIB_SOLUTION    0x0036
#define REG_PH_SLOPE_POINT_CALIB_SOLUTION   0x0038
#define REG_PH_ZERO_CONFIRMATION            0x003E
#define REG_PH_SLOPE_CONFIRMATION           0x003F
#define REG_PH_MEASURED_ADC                 0x0066




float REMOND_pH_Get_Measurement(ModbusMaster& node){
  if( !pHUpdateValues(node) )  //Actualizar si es necesario
    return NAN;         //En caso de error, retorna Nan
    
  return _pHcurrentValues.measurements;  
}

float REMOND_pH_Get_Temperature(ModbusMaster& node){
  if( !pHUpdateValues(node) )  //Actualizar si es necesario
    return NAN;         //En caso de error, retorna Nan
    
  return _pHcurrentValues.temperature;    
}

float REMOND_pH_Get_Current_Output(ModbusMaster& node){
  if( !pHUpdateValues(node) )  //Actualizar si es necesario
    return NAN;         //En caso de error, retorna Nan
    
  return _pHcurrentValues.currentOut;     
}

uint8_t REMOND_pH_Get_Warning(ModbusMaster& node){
   if( !pHUpdateValues(node) )  //Actualizar si es necesario
    return NAN;         //En caso de error, retorna Nan
    
  return _pHcurrentValues.warnings;   
}

/*
 * Funcion de calibracion interactiva para el sensor de ph REMOND 
 */
void REMOND_pH_Calibration(ModbusMaster& node){
   Serial.println("Before calibration......Choose zero point : ph6.86 (press 6) or ph7.00 (press 7) ???: ");
    
    while(1){
      if ( ((char) Serial.read()) =='6'){
        
        if ( REMOND_ph_Set_Zero_calib(node, zeropH_6_86_Solution) ){
          Serial.println("OK SET ZERO CALIB ph6.86");
          break;
        }
        else{
          Serial.println("SORRY, NO ZERO POINT CALIB, RESET ESP32 !!!"); 
          while (1); 
        }
      }else if ( ((char) Serial.read()) =='7'){

         if ( REMOND_ph_Set_Zero_calib(node, zeropH_7_Solution) ){
          Serial.println("OK SET ZERO CALIB ph7.00");
          break;
         }
        else{
          Serial.println("SORRY, NO ZERO POINT CALIB"); 
           while (1); 
        }
      }
      
      delay(50);
    }
    
    Serial.println("Before calibration......Choose slope point:  ph1.68 (press 1), ph4.01 (press 4), ph9.18 (press 9), ph10.1 (press 3) or  ph12.45 (press 0) ???: ");

    while (1) {
      if(  ((char) Serial.read()) =='1' ){
        if ( REMOND_ph_Set_Slope_calib(node, slopepH_1_68_Solution) ){
          Serial.println("OK SET SLOPE CALIB ph1.68");
          break;
        }
        else{
          Serial.println("SORRY, NO SLOPE POINT CALIB, RESET ESP32 !!!"); 
          while (1); 
        }
      }else if ( ((char) Serial.read()) =='4' ){
         if ( REMOND_ph_Set_Slope_calib(node, slopepH_4_01_Solution) ){
          Serial.println("OK SET SLOPE CALIB ph4.01");
          break;
         }
        else{
          Serial.println("SORRY, NO SLOPE POINT CALIB, RESET ESP32 !!!"); 
          while (1); 
        }
      }else if ( ((char) Serial.read()) =='9' ){
        if ( REMOND_ph_Set_Slope_calib(node, slopepH_9_18_Solution) ){
          Serial.println("OK SET SLOPE CALIB ph9.18");
          break;
        }
        else{
          Serial.println("SORRY, NO SLOPE POINT CALIB, RESET ESP32 !!!"); 
          while (1);
        }
      }else if ( ((char) Serial.read()) =='3' ){
         if ( REMOND_ph_Set_Slope_calib(node, slopepH_10_1_Solution) ){
          Serial.println("OK SET SLOPE CALIB ph10.1");
          break;
         }
        else{
          Serial.println("SORRY, NO SLOPE POINT CALIB, RESET ESP32 !!!"); 
          while (1);
        }
      }else if ( ((char) Serial.read()) =='0' ){
         if ( REMOND_ph_Set_Slope_calib(node, slopepH_12_45_Solution) ){
          Serial.println("OK SET SLOPE CALIB ph12.45");
          break;
         }
        else{
          Serial.println("SORRY, NO SLOPE POINT CALIB, RESET ESP32 !!!"); 
          while (1);
        }
      }

      delay(50);
    }

    Serial.println("Calibration ...First step : Put the sensor onto 6.86 or 7.00 zero point calibration....(press 's' to confirm ADC (stable) Value): ");
    
    while ( ( (char) Serial.read() )!='s'){
      Serial.print("ADC Value : ");
      Serial.println(REMOND_ph_Get_ADC_Value(node),DEC);
      delay(500);
    }

    if ( REMOND_ph_Set_Zero_confirmation(node) ){
      Serial.println("OK ZERO CONFIRMATED !");
    }else{
      Serial.println("COULD NOT ESTABLISH ZERO.....RESET ESP32 !! !");
      while(1);
    }

   Serial.println("Calibration ...Second step : Put the sensor onto 4.01 slope point calibration....(press 'S' to confirm ADC (stable) Value ): ");

   while ( ( (char) Serial.read() )!='S') {
      Serial.print("ADC Value : ");
      Serial.println(REMOND_ph_Get_ADC_Value(node),DEC);
      delay(500);
    }

    if ( REMOND_ph_Set_Slope_confirmation(node) ){
      Serial.println("OK SLOPE CONFIRMATED !");
    }else{
      Serial.println("COULD NOT ESTABLISH SLOPE.....RESET ESP32 !! !");
      while(1);
    }

   Serial.println("Calibration of PH Sucess !! Press 'r' key to continue....");
   while ( ((char) Serial.read())!='r' );
   delay(1000);
}

/*
 * Funcion que pre-setea el punto zero de calibracion del pH REMOND
 * Acceso Privado
 * 
 *         Parametros:
 *             node                :  Objeto Modbus asigando al sensor  (&node)
 *             zeroCalibSolution   :  Valor de la solucion estandard de calibracion de punto zero (6.86 o 7.00)  (ver phREMOND . h)
 *             
 *          Retorna True si Sucess (se pudo pre-setear el zero) o False si failure (fallo la comunicacion)
 */
static bool REMOND_ph_Set_Zero_calib(ModbusMaster& node, uint16_t zeroCalibSolution){
   if ( node.writeSingleRegister(REG_PH_ZERO_POINT_CALIB_SOLUTION, zeroCalibSolution ) == node.ku8MBSuccess  ){
    return true;
  }else
    return false;           //Si ocurre error, retorna falso.......(no se pudo calibrar)   
}

/*
 * Funcion que pre-setea el punto slope de calibracion del pH REMOND
 * Acceso privado
 * 
 *         Parametros:
 *             node                :  Objeto Modbus asigando al sensor  (&node)
 *             slopeCalibSolution  :  Valor de la solucion estandard de calibracion de punto slope (1.68 , 4.01, 9.18,...etc)  (ver phREMOND . h)
 *             
 *         Retorna True si Sucess (se pudo pre-setear el slope) o False si failure (fallo la comunicacion)
 */
static bool REMOND_ph_Set_Slope_calib(ModbusMaster& node,uint16_t slopeCalibSolution){
  if ( node.writeSingleRegister(REG_PH_SLOPE_POINT_CALIB_SOLUTION, slopeCalibSolution ) == node.ku8MBSuccess  ){
    return true;
  }else
    return false;          //Si ocurre error, retorna falso.......(no se pudo calibrar)
}

/*
 * Funcion que confirma el punto zero de calibracion del pH REMOND
 * Acceso privado
 * 
 *         Parametros:
 *             node                :  Objeto Modbus asigando al sensor  (&node)
 *          
 *          Retorna True si Sucess (se pudo confirmar el cero) o False si failure (fallo la comunicacion)
 */
static bool REMOND_ph_Set_Zero_confirmation(ModbusMaster& node){
  if (  node.writeSingleRegister(REG_PH_ZERO_CONFIRMATION, 0x00ff) == node.ku8MBSuccess ) {
    return true;
  }else
    return false;
}

/*
 * Funcion que confirma el punto slope de calibracion del pH REMOND
 * Acceso privado
 * 
 *         Parametros:
 *             node                :  Objeto Modbus asigando al sensor  (&node)
 *          
 *          Retorna True si Sucess (se pudo confirmar el slope) o False si failure (fallo la comunicacion)
 */
static bool REMOND_ph_Set_Slope_confirmation(ModbusMaster& node){
  if (  node.writeSingleRegister(REG_PH_SLOPE_CONFIRMATION, 0x00ff) == node.ku8MBSuccess ) {
    return true;
  }else
    return false;  
}

/*
 * Funcion que retorna el valor del ADC como WORD de 16 bits
 * Acceso privado
 * 
 *        Parametros:
 *            node        :  Objeto Modbus asigando al sensor  (&node)
 *        
 *        Retorna el valor de medicion del ADC o NaN si fallo la comunicacion
 */
static uint16_t REMOND_ph_Get_ADC_Value(ModbusMaster& node){
  if ( !pHcalibrationUpdate(node) )//Actualizar calibracion si es necesario
    return NAN ;            //En caso de error, retorna Nan

  return _pHcalibrationValue.measuredADC;  
}

/*
 * Funcion que actualiza el valor de calibracion del ADC (struct _pHcalibrationValue )
 * Nota : De uso privado solamente
 */
static bool pHcalibrationUpdate(ModbusMaster& node){
  static const uint8_t numReg=1 ;
  static uint16_t response[numReg];
  static uint8_t j;
  
  if (  node.readHoldingRegisters(REG_PH_MEASURED_ADC,(uint16_t) numReg ) == node.ku8MBSuccess){
    for (j=0 ; j<numReg;j++){
      response[j] = node.getResponseBuffer(j); // Lee el valor de lectura del ADC en el registro 0x0066......... solo para calibrar
    }

    _pHcalibrationValue.measuredADC=response[0] ;
    
    return true;
    
  }else
    return false;         // No se pudo leer el registro de ADC... 
}

/*
 * Funcion que actualiza los valores de medidas del instrumentos pH imetro
 * Nota : De uso privado solamente
 */
static bool pHUpdateValues(ModbusMaster& node){
  static const uint8_t numReg=7;
  static uint16_t startAddress=0x0001;
  static uint16_t response[numReg];
  static uint8_t j,result;
  static uint32_t registerData=0x00000000;   // Data output pointer

  //Si se lee antes del tiempo limite, no actualizar
  if( (unsigned long)(millis() - ( _lastRead3) )  >  UPDATE_TIME_PH){
        // Graba el tiempo actual
        _lastRead3 = millis();
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
     _pHcurrentValues.measurements=*((float*) &registerData );    // Cast HEX float to float value
     
                    //AB                                   CD
     registerData=( ( response[3] ) << 16 ) & 0xFFFF0000 | response[2] ;
     _pHcurrentValues.temperature=*((float*) &registerData );    // Cast HEX float to float value

                   //AB                                   CD
     registerData=( ( response[5] ) << 16 ) & 0xFFFF0000 | response[4] ;
     _pHcurrentValues.currentOut=*((float*) &registerData );    // Cast HEX float to float value

     _pHcurrentValues.warnings= (uint8_t) ( response[6]) & 0x00FF ; // As warnings go from 00 to 04 only !!
     
     return true;                //Si hubo lectura exitosa, retorna True
      
  }else{
      return false;               // Si no hubo lectura exitosa, retorna error
  }

}
