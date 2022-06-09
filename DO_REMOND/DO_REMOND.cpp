#include "DO_REMOND.h"


#define UPDATE_TIME_DO                     250

#define REG_DO_ZERO_CALIBRATION            0x003E
#define REG_DO_SLOPE_CALIBRATION           0x003F
#define REG_DO_MEASURED_ADC                0x0066

#define REG_MEASURE_MODE            	   0x0008
#define REG_RESTORE_FACTORY         	   0x001B

/*
 * Funcion que retorna el valor de la medida de DO mg/L
 * 
 *    Parametros : 
 *        node        :  Objeto Modbus asigando al sensor  (&node)
 */
float REMOND_DO_Get_Measurement(ModbusMaster& node){
  if(!DOUpdateValues(node))
    return NAN;
  return _DOcurrentValues.measurements;
}

/*
 * Funcion que retorna el valor de la temperatura 
 * 
 *    Parametros : 
 *        node        :  Objeto Modbus asigando al sensor  (&node)
 */
float REMOND_DO_Get_Temperature(ModbusMaster& node){
  if(!DOUpdateValues(node))
    return NAN;
  return _DOcurrentValues.temperature; 
}

/*
 * Funcion que retorna el valor de la corriente de salida basado en las mediciones de DO
 * 
 *    Parametros : 
 *        node        :  Objeto Modbus asigando al sensor  (&node)
 */
float REMOND_DO_Get_Current_Output(ModbusMaster& node){
  if(!DOUpdateValues(node))
    return NAN;
  return _DOcurrentValues.currentOut;   
}

/*
 * Funcion que retorna el valor de la alarma ( 00, 01, 02 , 03, 04)
 * 
 *    Parametros : 
 *        node        :  Objeto Modbus asigando al sensor  (&node)
 */
uint8_t REMOND_DO_Get_Warning(ModbusMaster& node){
  if(!DOUpdateValues(node))
    return NAN;
  return _DOcurrentValues.warnings;    
}

/*
* Funcion que setea el modo de medicion : %  , mg/L 
*/
bool REMOND_Set_Measure_Mode(ModbusMaster& node, uint16_t measureMode){
  if ( node.writeSingleRegister(REG_MEASURE_MODE, measureMode ) ==node.ku8MBSuccess){
    delay (250) ;                                       // Estabilizar comunicacion
    return true;
  }else
    return false;                                     // No se pudo establecer el modo de medicion

}


/*
 * Funcion de calibracion interactiva para el sensor de DO REMOND 
 * Acceso Publico
 */
void REMOND_DO_Calibration(ModbusMaster& node, bool twoPoints){
  Serial.println("The First Point: zero calibration : Put electrode in a 0 mg/L solution (ie. anhydrous sulfite solution) ....(press 's' to confirm ADC (stable) Value):");

  while ( ( (char) Serial.read() )!='s'){
      Serial.print("ADC Value : ");
      Serial.println(REMOND_DO_Get_ADC_Value(node),DEC);
      delay(500);
  }

  if ( REMOND_DO_Set_Zero_calibration(node)){
    Serial.println("OK ZERO CONFIRMATED !");
  }else{
    Serial.println("COULD NOT ESTABLISH ZERO.....RESET ESP32 !! !");
    while(1);
  }

  
 if (twoPoints==true){             // Es posible hacer la calibracion solo con 1 punto. ...

  Serial.println("The Second Point: slope calibration : Put sensor in saturate air (ie. 100%)....(press 'S' to confirm ADC (stable) Value):");

  while ( ( (char) Serial.read() )!='S'){
    Serial.print("ADC Value : ");
    Serial.println(REMOND_DO_Get_ADC_Value(node),DEC);
    delay(500);
  }

  if (REMOND_DO_Set_Slope_calibration(node)){
    Serial.println("OK SLOPE CONFIRMATED !");
  }else{
    Serial.println("COULD NOT ESTABLISH SLOPE.....RESET ESP32 !! !");
    while(1);
  }
}

 Serial.println("Calibration of DO sensor Sucess !! Press 'r' key to continue....");
 while ( ((char) Serial.read())!='r' );
 delay(1000);
}


/*
 * Funcion que confirma el punto zero de calibracion del DO REMOND
 * Acceso privado
 * 
 *         Parametros:
 *             node                :  Objeto Modbus asigando al sensor  (&node)
 *          
 *          Retorna True si Sucess (se pudo confirmar el cero) o False si failure (fallo la comunicacion)
 */
static bool REMOND_DO_Set_Zero_calibration(ModbusMaster& node){
  if (  node.writeSingleRegister(REG_DO_ZERO_CALIBRATION, 0x00ff) == node.ku8MBSuccess ) {
    return true;
  }else
    return false; 
}

/*
 * Funcion que confirma el punto slope de calibracion del DO REMOND
 * Acceso privado
 * 
 *         Parametros:
 *             node                :  Objeto Modbus asigando al sensor  (&node)
 *          
 *          Retorna True si Sucess (se pudo confirmar el slope) o False si failure (fallo la comunicacion)
 */
static bool REMOND_DO_Set_Slope_calibration(ModbusMaster& node){
  if (  node.writeSingleRegister(REG_DO_SLOPE_CALIBRATION, 0x00ff) == node.ku8MBSuccess ) {
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
static uint16_t REMOND_DO_Get_ADC_Value(ModbusMaster& node){
  if ( !DOcalibrationUpdate(node) )//Actualizar calibracion si es necesario
    return NAN ;            //En caso de error, retorna Nan

  return _DOcalibrationValue.measuredADC;  
}

/*
 * Funcion que actualiza el valor de calibracion del ADC (struct _DOcalibrationValue )
 * Nota : De uso privado solamente
 */
static bool DOcalibrationUpdate(ModbusMaster& node){
  static const uint8_t numReg=1 ;
  static uint16_t response[numReg];
  static uint8_t j;
  
  if (  node.readHoldingRegisters(REG_DO_MEASURED_ADC,(uint16_t) numReg ) == node.ku8MBSuccess){
    for (j=0 ; j<numReg;j++){
      response[j] = node.getResponseBuffer(j); // Lee el valor de lectura del ADC en el registro 0x0066......... solo para calibrar
    }

    _DOcalibrationValue.measuredADC=response[0] ;
    
    return true;
    
  }else
    return false;         // No se pudo leer el registro de ADC... 
}

/*
 * Funcion que actualiza los valores de medicion _DOcurrentValues
 * Nota : De uso privado solamente
 */
static bool DOUpdateValues(ModbusMaster& node){
  static const uint8_t numReg=7;
  static uint16_t startAddress=0x0001;
  static uint16_t response[numReg];
  static uint8_t j,result;
  static uint32_t registerData=0x00000000;   // Data output pointer

  //Si se lee antes del tiempo limite, no actualizar
  if( (unsigned long)(millis() - ( _lastRead4) )  >  UPDATE_TIME_DO){
        // Graba el tiempo actual
        _lastRead4 = millis();
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
     _DOcurrentValues.measurements=*((float*) &registerData );    // Cast HEX float to float value
     
                    //AB                                   CD
     registerData=( ( response[3] ) << 16 ) & 0xFFFF0000 | response[2] ;
     _DOcurrentValues.temperature=*((float*) &registerData );    // Cast HEX float to float value

                   //AB                                   CD
     registerData=( ( response[5] ) << 16 ) & 0xFFFF0000 | response[4] ;
     _DOcurrentValues.currentOut=*((float*) &registerData );    // Cast HEX float to float value

     _DOcurrentValues.warnings= (uint8_t) ( response[6]) & 0x00FF ; // As warnings go from 00 to 04 only !!
     
     return true;                //Si hubo lectura exitosa, retorna True
      
  }else{
      return false;               // Si no hubo lectura exitosa, retorna error
  }

}
