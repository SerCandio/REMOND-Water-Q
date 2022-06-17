#include "Arduino.h"
#include <ModbusMaster.h>
#include "TurbidityCALIBRATION.h"

#define UPDATE_TIME_TURBIDITY    250

#define REG_FIRST_CALIB_VALUE    0x0020
#define REG_SECOND_CALIB_VALUE   0x0024
#define REG_THIRD_CALIB_VALUE    0x0028
#define REG_FOURTH_CALIB_VALUE   0x002C

#define REG_FIRST_ADC_VALUE      0x0022
#define REG_SECOND_ADC_VALUE     0x0026
#define REG_THIRD_ADC_VALUE      0x002A
#define REG_FOURTH_ADC_VALUE     0x0022

#define REG_TURBIDITY_ADC_VALUE  0x0066

#define REG_RESTORE_DEFAULT      0x001B


/*
 * Funcion que retorna el valor de la medida en NTU
 * Puede llamar esta funcion desde el main
 * 
 *    Parametros : 
 *        node        :  Objeto Modbus asigando al sensor  (&node)
 */
float REMOND_Turbidity_Get_Measurement(ModbusMaster& node){
  if(!turbiditYUpdateValues(node))
    return NAN;
    
  return _turbiditYcurrentValues.measurements;
}

/*
 * Funcion principal de calibracion
 * Puede llamar esta funcion desde el main
 * 
 *    Parametros : 
 *        node        :  Objeto Modbus asigando al sensor  (&node)
 */
void sensor_turbidity_Calibration(ModbusMaster& node){
  uint16_t valorADC;
  
  sprintf(Str_Turbidity_frame,"%.3f, %.3f, %.3f, %.3f [NTU]",firstPointCalibSolution,secondPointCalibSolution, thirdPointCalibSolution, fourthPointCalibSolution);
  
  Serial.print("Antes de la calibracion......confirme los valores de los 4 puntos NTU   (presione 'N' para confirmar): ");  
  Serial.println(Str_Turbidity_frame);
  Serial.println();

  while (1){
    if (((char) Serial.read()) =='N' ){
        if ( REMOND_Turbidity_Set_FirstPoint_calib(node,firstPointCalibSolution) ){
          Serial.print("OK PUNTO DE : ");
          Serial.print(firstPointCalibSolution, 3);
          Serial.print(" NTU ");
          Serial.println(" PRE-SETEADO");
          Serial.println();
          break;
        }else{
          Serial.println("SORRY, NO SE PUDO ESTABLECER EL PUNTO, RESET ESP32 !!!");
        }

        delay(50);
  
       if ( REMOND_Turbidity_Set_SecondPoint_calib(node,secondPointCalibSolution) ){
          Serial.print("OK PUNTO DE : ");
          Serial.print(secondPointCalibSolution, 3);
          Serial.print(" NTU ");
          Serial.println(" PRE-SETEADO");
          Serial.println();
          break;
        }else{
          Serial.println("SORRY, NO SE PUDO ESTABLECER EL PUNTO, RESET ESP32 !!!");
        }

        delay(50);

       if ( REMOND_Turbidity_Set_ThirdPoint_calib(node,thirdPointCalibSolution) ){
          Serial.print("OK PUNTO DE : ");
          Serial.print(thirdPointCalibSolution, 3);
          Serial.print(" NTU ");
          Serial.println(" PRE-SETEADO");
          Serial.println();
          break;
        }else{
          Serial.println("SORRY, NO SE PUDO ESTABLECER EL PUNTO, RESET ESP32 !!!");
        }

        delay(50);

        if ( REMOND_Turbidity_Set_FourthPoint_calib(node, fourthPointCalibSolution) ){
          Serial.print("OK PUNTO DE : ");
          Serial.print(fourthPointCalibSolution, 3);
          Serial.print(" NTU ");
          Serial.println(" PRE-SETEADO");
          Serial.println();
          break;
        }else{
          Serial.println("SORRY, NO SE PUDO ESTABLECER EL PUNTO, RESET ESP32 !!!");
        }
        
        delay(50);   
    }
    
  }

//////////////////////////////////////////////////////////////////////////////////////////////////////////

  sprintf(Str_Turbidity_frame,"%.3f [NTU]\n\r",firstPointCalibSolution);
  Serial.print("Calibracion... Primer paso: Colocar el sensor en una solucion de : ");
  Serial.println(Str_Turbidity_frame);
  Serial.println("(Presione '1' para confirmar el valor estable del ADC):\n\r");

  while ( ( (char) Serial.read() )!='1'){
      valorADC=REMOND_Turbidity_Get_ADC_Value(node);
      Serial.print("ADC Valor : ");
      Serial.println(valorADC, DEC);
      delay(500);
  }

  if ( REMOND_Turbidity_Set_FirstADC_value(node, float (valorADC) )  ){
      Serial.println("OK PRIMER PUNTO CALIB CONFIRMADO !");
  }else{
      Serial.println("NO SE PUDO ESTABLECER PUNTO, RESET ESP32 !!!");
  }

//////////////////////////////////////////////////////////////////////////////////////////////////////////

  sprintf(Str_Turbidity_frame,"%.3f [NTU]\n\r",secondPointCalibSolution);
  Serial.print("Calibracion... Segundo paso: Colocar el sensor en una solucion de : ");
  Serial.println(Str_Turbidity_frame);
  Serial.println("(Presione '2' para confirmar el valor estable del ADC):\n\r");

  while ( ( (char) Serial.read() )!='2'){
      valorADC=REMOND_Turbidity_Get_ADC_Value(node);
      Serial.print("ADC Valor : ");
      Serial.println(valorADC, DEC);
      delay(500);
  }

  if ( REMOND_Turbidity_Set_SecondADC_value(node, float (valorADC) )  ){
      Serial.println("OK SEGUNDO PUNTO CALIB CONFIRMADO !");
  }else{
      Serial.println("NO SE PUDO ESTABLECER PUNTO, RESET ESP32 !!!");
  }

//////////////////////////////////////////////////////////////////////////////////////////////////////////

  sprintf(Str_Turbidity_frame,"%.3f [NTU]\n\r",thirdPointCalibSolution);
  Serial.print("Calibracion... Tercer paso: Colocar el sensor en una solucion de : ");
  Serial.println(Str_Turbidity_frame);
  Serial.println("(Presione '3' para confirmar el valor estable del ADC):\n\r");

  while ( ( (char) Serial.read() )!='2'){
      valorADC=REMOND_Turbidity_Get_ADC_Value(node);
      Serial.print("ADC Valor : ");
      Serial.println(valorADC, DEC);
      delay(500);
  }

  if ( REMOND_Turbidity_Set_ThirdADC_value(node, float (valorADC) )  ){
      Serial.println("OK TERCER PUNTO CALIB CONFIRMADO !");
  }else{
      Serial.println("NO SE PUDO ESTABLECER PUNTO, RESET ESP32 !!!");
  }

//////////////////////////////////////////////////////////////////////////////////////////////////////////

  sprintf(Str_Turbidity_frame,"%.3f [NTU]\n\r", fourthPointCalibSolution);
  Serial.print("Calibracion... Cuarto paso: Colocar el sensor en una solucion de : ");
  Serial.println(Str_Turbidity_frame);
  Serial.println("(Presione '4' para confirmar el valor estable del ADC):\n\r");

  while ( ( (char) Serial.read() )!='2'){
      valorADC=REMOND_Turbidity_Get_ADC_Value(node);
      Serial.print("ADC Valor : ");
      Serial.println(valorADC, DEC);
      delay(500);
  }

  if ( REMOND_Turbidity_Set_FourthADC_value(node, float (valorADC) )  ){
      Serial.println("OK SEGUNDO PUNTO CALIB CONFIRMADO !");
  }else{
      Serial.println("NO SE PUDO ESTABLECER PUNTO, RESET ESP32 !!!");
  }
  
 /////////////////////////////////////////////////////////////////////////////////////////////////////////
   Serial.println("Calibracion exitosa del Turbidimetro !! Presione 'r' para continuar....");
   while ( ((char) Serial.read())!='r' );
   delay(1000);
}

/*
 * Funcion que pre-establece el primer punto de calibracion , registro " First calibration value" en direccion 0x0020
 * 
 *    Parametros : 
 *        node                       :  Objeto Modbus asigando al sensor  (&node)
 *        firstPointCalibSolution    :  Punto de calibracion de referencia  ( Ejm : 1.000 NTU)
 */
static bool REMOND_Turbidity_Set_FirstPoint_calib(ModbusMaster& node, float firstPointCalibSolution){
  static uint32_t registerData;
  static uint16_t registerDataH,registerDataL; 

  registerData= *((uint32_t*) & firstPointCalibSolution ); // Convertir de float a HEX Float  de 4 palabras

  registerDataH=(uint16_t) ( (registerData >> 16 ) & 0x0000ffff ) ;   // Obtener el High Word MSB   (31...16)
  registerDataL=(uint16_t) (  registerData & 0x0000ffff         ) ;   // Obtener el Low Word LSB    (15 ...0)

  node.setTransmitBuffer(0, registerDataL); // Escribir el Buffer de TX (word 0) del Master al word menos significativo (LSB) de 'registerData'
  node.setTransmitBuffer(1, registerDataH); // Escribir el Buffer de TX (word 1) del Master al word mas significativo (MSB) de 'registerData'

  if (node.writeMultipleRegisters(REG_FIRST_CALIB_VALUE, 2) ==node.ku8MBSuccess ){  // Esclavo : Transferir los (2) TX Buffer del maestro a partir registro 0x00.... del esclavo
    return true;
  }else{
    return false;                       // No se pudo establecer el registro(s)
  }
  
}

/*
 * Funcion que pre-establece el segundo punto de calibracion , registro " Second calibration value" en direccion 0x0024
 * 
 *    Parametros : 
 *        node                       :  Objeto Modbus asigando al sensor  (&node)
 *        secondPointCalibSolution   :  Punto de calibracion de referencia  ( Ejm : 10.000 NTU)
 */
static bool REMOND_Turbidity_Set_SecondPoint_calib(ModbusMaster& node, float secondPointCalibSolution){
  static uint32_t registerData;
  static uint16_t registerDataH,registerDataL; 

  registerData= *((uint32_t*) & secondPointCalibSolution ); // Convertir de float a HEX Float  de 4 palabras

  registerDataH=(uint16_t) ( (registerData >> 16 ) & 0x0000ffff ) ;   // Obtener el High Word MSB   (31...16)
  registerDataL=(uint16_t) (  registerData & 0x0000ffff         ) ;   // Obtener el Low Word LSB    (15 ...0)

  node.setTransmitBuffer(0, registerDataL); // Escribir el Buffer de TX (word 0) del Master al word menos significativo (LSB) de 'registerData'
  node.setTransmitBuffer(1, registerDataH); // Escribir el Buffer de TX (word 1) del Master al word mas significativo (MSB) de 'registerData'

  if (node.writeMultipleRegisters(REG_SECOND_CALIB_VALUE, 2) ==node.ku8MBSuccess ){  // Esclavo : Transferir los (2) TX Buffer del maestro a partir registro 0x00.... del esclavo
    return true;
  }else{
    return false;                       // No se pudo establecer el registro(s)
  }

}

/*
 * Funcion que pre-establece el segundo punto de calibracion , registro " Third calibration value" en direccion 0x0028
 * 
 *    Parametros : 
 *        node                       :  Objeto Modbus asigando al sensor  (&node)
 *        thirdPointCalibSolution    :  Punto de calibracion de referencia  ( Ejm : 20.000 NTU)
 */
static bool REMOND_Turbidity_Set_ThirdPoint_calib(ModbusMaster& node, float thirdPointCalibSolution){
  static uint32_t registerData;
  static uint16_t registerDataH,registerDataL; 

  registerData= *((uint32_t*) & thirdPointCalibSolution ); // Convertir de float a HEX Float  de 4 palabras

  registerDataH=(uint16_t) ( (registerData >> 16 ) & 0x0000ffff ) ;   // Obtener el High Word MSB   (31...16)
  registerDataL=(uint16_t) (  registerData & 0x0000ffff         ) ;   // Obtener el Low Word LSB    (15 ...0)

  node.setTransmitBuffer(0, registerDataL); // Escribir el Buffer de TX (word 0) del Master al word menos significativo (LSB) de 'registerData'
  node.setTransmitBuffer(1, registerDataH); // Escribir el Buffer de TX (word 1) del Master al word mas significativo (MSB) de 'registerData'

  if (node.writeMultipleRegisters(REG_THIRD_CALIB_VALUE, 2) ==node.ku8MBSuccess ){  // Esclavo : Transferir los (2) TX Buffer del maestro a partir registro 0x00.... del esclavo
    return true;
  }else{
    return false;                       // No se pudo establecer el registro(s)
  }

}

/*
 * Funcion que pre-establece el segundo punto de calibracion , registro " Fourth calibration value" en direccion 0x002C
 * 
 *    Parametros : 
 *        node                       :  Objeto Modbus asigando al sensor  (&node)
 *        fourthPointCalibSolution    :  Punto de calibracion de referencia  ( Ejm : 30.000 NTU)
 */
static bool REMOND_Turbidity_Set_FourthPoint_calib(ModbusMaster& node, float fourthPointCalibSolution){
  static uint32_t registerData;
  static uint16_t registerDataH,registerDataL; 

  registerData= *((uint32_t*) & fourthPointCalibSolution ); // Convertir de float a HEX Float  de 4 palabras

  registerDataH=(uint16_t) ( (registerData >> 16 ) & 0x0000ffff ) ;   // Obtener el High Word MSB   (31...16)
  registerDataL=(uint16_t) (  registerData & 0x0000ffff         ) ;   // Obtener el Low Word LSB    (15 ...0)

  node.setTransmitBuffer(0, registerDataL); // Escribir el Buffer de TX (word 0) del Master al word menos significativo (LSB) de 'registerData'
  node.setTransmitBuffer(1, registerDataH); // Escribir el Buffer de TX (word 1) del Master al word mas significativo (MSB) de 'registerData'

  if (node.writeMultipleRegisters(REG_FOURTH_CALIB_VALUE, 2) ==node.ku8MBSuccess ){  // Esclavo : Transferir los (2) TX Buffer del maestro a partir registro 0x00.... del esclavo
    return true;
  }else{
    return false;                       // No se pudo establecer el registro(s)
  }
  
}

/*
 * Funcion que establece el primer valor de ADC , registro "First ADC Value" en el registro 0x0022
 * 
 *    Parametros : 
 *        node        :  Objeto Modbus asigando al sensor  (&node)
 *        adcValue    :  Punto de calibracion de referencia  ( Ejm : 100.0)
 */
static bool REMOND_Turbidity_Set_FirstADC_value(ModbusMaster& node, float adcValue){
  static uint32_t registerData;
  static uint16_t registerDataH,registerDataL; 

  registerData= *((uint32_t*) & adcValue ); // Convertir de float a HEX Float  de 4 palabras, Ejm si es valor ADC =100 , su valor en u32 seria : 0x42C80000

  registerDataH=(uint16_t) ( (registerData >> 16 ) & 0x0000ffff ) ;   // Obtener el High Word MSB   (31...16)
  registerDataL=(uint16_t) (  registerData & 0x0000ffff         ) ;   // Obtener el Low Word LSB    (15 ...0)

  node.setTransmitBuffer(0, registerDataL); // Escribir el Buffer de TX (word 0) del Master al word menos significativo (LSB) de 'registerData'
  node.setTransmitBuffer(1, registerDataH); // Escribir el Buffer de TX (word 1) del Master al word mas significativo (MSB) de 'registerData'

  if (node.writeMultipleRegisters(REG_FIRST_ADC_VALUE, 2) ==node.ku8MBSuccess){ // Esclavo : Transferir los (2) TX Buffer del maestro a partir registro 0x00.... del esclavo
    return true;
  }else{
    return false;                     // No se pudo establecer el registro(s)
  }

}

/*
 * Funcion que establece el segundo valor de ADC , registro "Second ADC Value" en el registro 0x0026
 * 
 *    Parametros : 
 *        node        :  Objeto Modbus asigando al sensor  (&node)
 *        adcValue    :  Punto de calibracion de referencia  ( Ejm : 1000.0)
 */
static bool REMOND_Turbidity_Set_SecondADC_value(ModbusMaster& node, float adcValue){
  static uint32_t registerData;
  static uint16_t registerDataH,registerDataL; 

  registerData= *((uint32_t*) & adcValue ); // Convertir de float a HEX Float  de 4 palabras, Ejm si es valor ADC =100 , su valor en u32 seria : 0x42C80000

  registerDataH=(uint16_t) ( (registerData >> 16 ) & 0x0000ffff ) ;   // Obtener el High Word MSB   (31...16)
  registerDataL=(uint16_t) (  registerData & 0x0000ffff         ) ;   // Obtener el Low Word LSB    (15 ...0)

  node.setTransmitBuffer(0, registerDataL); // Escribir el Buffer de TX (word 0) del Master al word menos significativo (LSB) de 'registerData'
  node.setTransmitBuffer(1, registerDataH); // Escribir el Buffer de TX (word 1) del Master al word mas significativo (MSB) de 'registerData'

  if (node.writeMultipleRegisters(REG_SECOND_ADC_VALUE, 2) ==node.ku8MBSuccess){ // Esclavo : Transferir los (2) TX Buffer del maestro a partir registro 0x00.... del esclavo
    return true;
  }else{
    return false;                     // No se pudo establecer el registro(s)
  }
  
}

/*
 * Funcion que establece el tercer valor de ADC , registro "Third ADC Value" en el registro 0x002A
 * 
 *    Parametros : 
 *        node        :  Objeto Modbus asigando al sensor  (&node)
 *        adcValue    :  Punto de calibracion de referencia  ( Ejm : 2000.0)
 */
static bool REMOND_Turbidity_Set_ThirdADC_value(ModbusMaster& node, float adcValue){
  static uint32_t registerData;
  static uint16_t registerDataH,registerDataL; 

  registerData= *((uint32_t*) & adcValue ); // Convertir de float a HEX Float  de 4 palabras, Ejm si es valor ADC =100 , su valor en u32 seria : 0x42C80000

  registerDataH=(uint16_t) ( (registerData >> 16 ) & 0x0000ffff ) ;   // Obtener el High Word MSB   (31...16)
  registerDataL=(uint16_t) (  registerData & 0x0000ffff         ) ;   // Obtener el Low Word LSB    (15 ...0)

  node.setTransmitBuffer(0, registerDataL); // Escribir el Buffer de TX (word 0) del Master al word menos significativo (LSB) de 'registerData'
  node.setTransmitBuffer(1, registerDataH); // Escribir el Buffer de TX (word 1) del Master al word mas significativo (MSB) de 'registerData'

  if (node.writeMultipleRegisters(REG_THIRD_ADC_VALUE, 2) ==node.ku8MBSuccess){ // Esclavo : Transferir los (2) TX Buffer del maestro a partir registro 0x00.... del esclavo
    return true;
  }else{
    return false;                     // No se pudo establecer el registro(s)
  }
  
}

/*
 * Funcion que establece el cuarto valor de ADC , registro "Fourth ADC Value" en el registro 0x002E
 * 
 *    Parametros : 
 *        node        :  Objeto Modbus asigando al sensor  (&node)
 *        adcValue    :  Punto de calibracion de referencia  ( Ejm : 3000.0)
 */
static bool REMOND_Turbidity_Set_FourthADC_value(ModbusMaster& node, float adcValue){
  static uint32_t registerData;
  static uint16_t registerDataH,registerDataL; 

  registerData= *((uint32_t*) & adcValue ); // Convertir de float a HEX Float  de 4 palabras, Ejm si es valor ADC =100 , su valor en u32 seria : 0x42C80000

  registerDataH=(uint16_t) ( (registerData >> 16 ) & 0x0000ffff ) ;   // Obtener el High Word MSB   (31...16)
  registerDataL=(uint16_t) (  registerData & 0x0000ffff         ) ;   // Obtener el Low Word LSB    (15 ...0)

  node.setTransmitBuffer(0, registerDataL); // Escribir el Buffer de TX (word 0) del Master al word menos significativo (LSB) de 'registerData'
  node.setTransmitBuffer(1, registerDataH); // Escribir el Buffer de TX (word 1) del Master al word mas significativo (MSB) de 'registerData'

  if (node.writeMultipleRegisters(REG_FOURTH_ADC_VALUE, 2) ==node.ku8MBSuccess){ // Esclavo : Transferir los (2) TX Buffer del maestro a partir registro 0x00.... del esclavo
    return true;
  }else{
    return false;                     // No se pudo establecer el registro(s)
  }

}

/*
 * Funcion que retorna el valor actulizado del ADC
 * 
 *    Parametros : 
 *        node        :  Objeto Modbus asigando al sensor  (&node)
 */
static uint16_t REMOND_Turbidity_Get_ADC_Value(ModbusMaster& node){
  if ( !REMOND_Turbidity_Calibration_Update(node) )//Actualizar calibracion si es necesario
   return NAN ;            //En caso de error, retorna Nan

  return _turbidityCalibrationValue.measuredADC;
  
}

/*
 * Funcion que lee el valor de medicion de NTU en el registro "Measurements" en la direccion 0x0001 y lo devuelve en la variable struct "_turbiditYcurrentValues.measurements"
 * 
 *    Parametros : 
 *        node        :  Objeto Modbus asigando al sensor  (&node)
 */
static bool turbiditYUpdateValues(ModbusMaster& node){
  static const uint8_t numReg=2;                      // Leer solo el registro de medidas
  static uint16_t startAddress=0x0001;
  static uint16_t response[numReg];
  static uint8_t j,result;
  static uint32_t registerData=0x00000000;   // Data output pointer

  //Si se lee antes del tiempo limite, no actualizar
  if( (unsigned long)(millis() - ( _lastReadTurb) )  >  UPDATE_TIME_TURBIDITY){
        // Graba el tiempo actual
        _lastReadTurb = millis();
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
     _turbiditYcurrentValues.measurements=*((float*) &registerData );    // Cast HEX float to float value
        
     return true;                //Si hubo lectura exitosa, retorna True
      
  }else{
      return false;               // Si no hubo lectura exitosa, retorna error
  }

}

/*
 * Funcion que lee el valor de calibracion del ADC , y lo devuelve en la variable struct "_turbidityCalibrationValue.measuredADC"
 * 
 *    Parametros : 
 *        node        :  Objeto Modbus asigando al sensor  (&node)
 */
static bool REMOND_Turbidity_Calibration_Update(ModbusMaster& node){
  static const uint16_t numReg=1 ;  // Cantidad de registros por leer
  static uint16_t response[numReg]; // Array de response
  static uint8_t j;

  if ( node.readHoldingRegisters(REG_TURBIDITY_ADC_VALUE, numReg ) == node.ku8MBSuccess ){  // Lee el valor de lectura del ADC en el registro 0x0066 
    for (j=0 ; j<numReg; j++){
      response[j] = node.getResponseBuffer(j); // Almacena el valor del registro 0x66 en el array response[]
    }

    _turbidityCalibrationValue.measuredADC=response[0]; // ....Hacia el struct global de calibracion...

    return true;                  //Lectura de calibracion exitosa
    
  }else{
    return false ;                // No se pudo leer el registro de calibracion
  }
  
}
