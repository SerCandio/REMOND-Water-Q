#include "Arduino.h"
#include "REMONDWater.h"
#include <ModbusMaster.h>
#include "TURBIDITY_REMOND.h"

#define UPDATE_TIME        250

#define REG_FIRST_CALIBRATION_VALUE   0x0020
#define REG_SECOND_CALIBRATION_VALUE  0x0024
#define REG_THIRD_CALIBRATION_VALUE   0x0028
#define REG_FOURTH_CALIBRATION_VALUE  0x002C

#define REG_FIRST_ADC_VALUE           0x0022
#define REG_SECOND_ADC_VALUE          0x0026
#define REG_THIRD_ADC_VALUE           0x002A
#define REG_FOURTH_ADC_VALUE          0x002E

#define REG_ADC_VALUE                 0x0066

#define REG_1_2_SLOPE                 0x0030
#define REG_1_3_SLOPE                 0x0032
#define REG_1_4_SLOPE                 0x0034

#define REG_UPPER_TEMPERATURE_LIMIT 0x000E
#define REG_LOWER_TEMPERATURE_LIMIT 0x0010

#define REG_BAUD_RATE               0x001A
#define REG_DEVICE_ADDRESS          0x0019

#define REG_RESTORE_FACTORY         0x001B


/*
 * Asignar pines al MAX 485
 */
void REMONDWater_begin(uint8_t  max485ReNegPin,uint8_t max485DePin){
  // Iniciar los pines de MAX485 como salidas ambos
  pinMode(max485ReNegPin, OUTPUT);
  pinMode(max485DePin, OUTPUT);
  
  _max485ReNegPin=max485ReNegPin;
  _max485DePin=max485DePin;
}

/*
 * Rutinas de enable TX/ RX para seteo de modo de solo TX o solo RX (Half Duplex)
 ... https://programarfacil.com/wp-content/uploads/2020/09/MAX485-interno.jpg
 */
void preTransmission()
{
  digitalWrite(_max485ReNegPin, HIGH);  //Deshabilita recepcion
  digitalWrite(_max485DePin, HIGH);    //........................Habilita Transmision
}

void postTransmission()
{
  digitalWrite(_max485ReNegPin, LOW); //........................Habilita recepcion
  digitalWrite(_max485DePin, LOW);  // Deshabilita transmision
}


float REMOND_Get_Measurement(ModbusMaster& node){
  if( !updateValues(node) )  //Actualizar si es necesario
    return NAN;         //En caso de error, retorna Nan
    
  return _currentValues.measurements;  
}

float REMOND_Get_Temperature(ModbusMaster& node){
   if( !updateValues(node) )  //Actualizar si es necesario
    return NAN;            //En caso de error, retorna Nan
    
  return _currentValues.temperature;
}

float REMOND_Get_Output_Current(ModbusMaster& node){
   if( !updateValues(node) )  //Actualizar si es necesario
    return NAN;            //En caso de error, retorna Nan
    
  return _currentValues.currentOut;  
}

uint8_t REMOND_Get_Warning(ModbusMaster& node){
  if( !updateValues(node) )  //Actualizar si es necesario
    return NAN;            //En caso de error, retorna Nan

   return _currentValues.warnings;  
}

uint32_t REMOND_Get_ADC_Value(ModbusMaster& node){                        // Retorna el valor HEX Float del valor del ADC a partir del registro 0x66
  if ( !calibrationUpdate(node) )//Actualizar calibracion si es necesario
    return NAN ;            //En caso de error, retorna Nan

  return (  ( (_calibrationValue.measuredADC_AB << 16 ) & 0xffff0000 ) |  ( _calibrationValue.measuredADC_CD  )  );  
}

bool REMOND_Set_First_ADC_Value(ModbusMaster& node, uint32_t Data){
  static uint16_t DataH,DataL;
  
  if (isnan(Data ) ){
    return false ;         // No se pudo leer el valor del ADC en el registro 0x0066 !
  }else{
    DataH=(uint16_t) (Data >> 16) & 0x0000ffff;
    DataL=(uint16_t) Data & 0x0000ffff;

    // Set TX Buffer to LSB and MSB word of Data  (Data L, Data H)
    node.setTransmitBuffer(0, DataL);
    node.setTransmitBuffer(1, DataH);

    // Transferir los word del buffer de TX a los (2) registros de 16 bit del esclavo a partir de la direccion REG_FIRST_ADC_VALUE
    if (node.writeMultipleRegisters(REG_FIRST_ADC_VALUE, 2) ==node.ku8MBSuccess){
      return true;                                                                // Escritura del primero valor del ADC exitosa
    }else{
      return false;                                                           // No se pudo escribir el primer valor del ADC
    }  
  }
  
}

bool REMOND_Set_Second_ADC_Value(ModbusMaster& node, uint32_t Data){
 static uint16_t DataH,DataL;
  
  if (isnan(Data ) ){
    return false ;         // No se pudo leer el valor del ADC en el registro 0x0066 !
  }else{
    DataH=(uint16_t) (Data >> 16) & 0x0000ffff;
    DataL=(uint16_t) Data & 0x0000ffff;

    // Set TX Buffer to LSB and MSB word of Data  (Data L, Data H)
    node.setTransmitBuffer(0, DataL);
    node.setTransmitBuffer(1, DataH);

    // Transferir los word del buffer de TX a los (2) registros de 16 bit del esclavo a partir de la direccion REG_FIRST_ADC_VALUE
    if (node.writeMultipleRegisters(REG_SECOND_ADC_VALUE, 2) ==node.ku8MBSuccess){
      return true;                                                                // Escritura del primero valor del ADC exitosa
    }else{
      return false;                                                           // No se pudo escribir el primer valor del ADC
    }  
  }
   
}

bool REMOND_Set_Third_ADC_Value(ModbusMaster& node, uint32_t Data){
 static uint16_t DataH,DataL;
  
  if (isnan(Data ) ){
    return false ;         // No se pudo leer el valor del ADC en el registro 0x0066 !
  }else{
    DataH=(uint16_t) (Data >> 16) & 0x0000ffff;
    DataL=(uint16_t) Data & 0x0000ffff;

    // Set TX Buffer to LSB and MSB word of Data  (Data L, Data H)
    node.setTransmitBuffer(0, DataL);
    node.setTransmitBuffer(1, DataH);

    // Transferir los word del buffer de TX a los (2) registros de 16 bit del esclavo a partir de la direccion REG_FIRST_ADC_VALUE
    if (node.writeMultipleRegisters(REG_THIRD_ADC_VALUE, 2) ==node.ku8MBSuccess){
      return true;                                                                // Escritura del primero valor del ADC exitosa
    }else{
      return false;                                                           // No se pudo escribir el primer valor del ADC
    }  
  }
  
}

bool REMOND_Set_Fourth_ADC_Value(ModbusMaster& node, uint32_t Data){
 static uint16_t DataH,DataL;
  
  if (isnan(Data ) ){
    return false ;         // No se pudo leer el valor del ADC en el registro 0x0066 !
  }else{
    DataH=(uint16_t) (Data >> 16) & 0x0000ffff;
    DataL=(uint16_t) Data & 0x0000ffff;

    // Set TX Buffer to LSB and MSB word of Data  (Data L, Data H)
    node.setTransmitBuffer(0, DataL);
    node.setTransmitBuffer(1, DataH);

    // Transferir los word del buffer de TX a los (2) registros de 16 bit del esclavo a partir de la direccion REG_FIRST_ADC_VALUE
    if (node.writeMultipleRegisters(REG_FOURTH_ADC_VALUE , 2) ==node.ku8MBSuccess){
      return true;                                                                // Escritura del primero valor del ADC exitosa
    }else{
      return false;                                                           // No se pudo escribir el primer valor del ADC
    }  
  }
  
}


bool REMOND_TURBIDITY_Set_float_Calibration_Points(ModbusMaster& node, float firstPointCalib, float secondPointCalib, float thirdPointCalib, float fourthPointCalib){
  static uint32_t registerData;
  static uint16_t registerDataH,registerDataL;

  registerData=*((uint32_t*) & firstPointCalib ); // Convertir de float a HEX Float  de 4 palabras
  
  registerDataH=(uint16_t) ( (registerData >> 16 ) & 0x0000ffff ) ; // Descomponer los registros en MSB (bits 31..16) y LSB (bits 15.. 0)
  registerDataL=(uint16_t) (  registerData & 0x0000ffff         ) ; //LSB

  // set word 0 del  TX Buffer al LSB de la variable de  registerData
  node.setTransmitBuffer(0, registerDataL);
  
   // set word 1 del  TX Buffer al MSB de la variable de  registerData
  node.setTransmitBuffer(1, registerDataH);

  // 
  if ( node.writeMultipleRegisters(REG_FIRST_CALIBRATION_VALUE, 2) ==node.ku8MBSuccess  ){  // Enviar la data del TX Buffer al slave Modbus a partir de la direccion 0x20
   delay(250);                                                        // Estabilizar comunicacion
   registerData=*((uint32_t*) & secondPointCalib );
   registerDataH=(uint16_t) ( (registerData >> 16 ) & 0x0000ffff ) ;registerDataL=(uint16_t) (  registerData & 0x0000ffff         ) ;
   node.setTransmitBuffer(0, registerDataL);                        node.setTransmitBuffer(1, registerDataH);

   if ( node.writeMultipleRegisters(REG_SECOND_CALIBRATION_VALUE, 2) ==node.ku8MBSuccess  ){
     delay(250);                                                        // Estabilizar comunicacion
     registerData=*((uint32_t*) & thirdPointCalib );
     registerDataH=(uint16_t) ( (registerData >> 16 ) & 0x0000ffff ) ;registerDataL=(uint16_t) (  registerData & 0x0000ffff         ) ;
     node.setTransmitBuffer(0, registerDataL);                        node.setTransmitBuffer(1, registerDataH);

      if (node.writeMultipleRegisters (REG_THIRD_CALIBRATION_VALUE, 2) ==node.ku8MBSuccess){
        delay(250);
        registerData=*((uint32_t*) & fourthPointCalib );
        registerDataH=(uint16_t) ( (registerData >> 16 ) & 0x0000ffff ) ;registerDataL=(uint16_t) (  registerData & 0x0000ffff         ) ;
        node.setTransmitBuffer(0, registerDataL);                        node.setTransmitBuffer(1, registerDataH);
          if (node.writeMultipleRegisters(REG_FOURTH_CALIBRATION_VALUE, 2) ==node.ku8MBSuccess){
            return true;                                                                              // Escritura de puntos de calibracion exitosa !!!
          }else{
            return false; // No se pudo escribir la data del TX BUFFER
          }
            
      }else{
       return false;//No se pudo escribir la data del TX BUFFER
      }
 
    }else{
      return false; //No se pudo escribir la data del TX BUFFER
    }
   
  }else{
    return false;   // No se pudo escribir la data del TX BUFFER
  }

}

bool REMOND_TURBIDITY_Set_DWord_Calibration_Points(ModbusMaster& node, uint32_t firstPointCalib, uint32_t secondPointCalib, uint32_t thirdPointCalib, uint32_t fourthPointCalib){
  static uint16_t registerDataH,registerDataL;
                                                                   /*
                                                                    * Los puntos de calibracion son DWORD de 32 bit
                                                                    */
  registerDataH=(uint16_t) ( (firstPointCalib >> 16 ) & 0x0000ffff ) ; // Descomponer los registros en MSB (bits 31..16) y LSB (bits 15.. 0)
  registerDataL=(uint16_t) (  firstPointCalib & 0x0000ffff         ) ; //LSB

  node.setTransmitBuffer(0,registerDataL);
  node.setTransmitBuffer(1,registerDataH);

  if ( node.writeMultipleRegisters(REG_FIRST_CALIBRATION_VALUE, 2) ==node.ku8MBSuccess){
    delay(250);
    registerDataH=(uint16_t) ( (secondPointCalib >> 16 ) & 0x0000ffff ) ;registerDataL=(uint16_t) (  secondPointCalib & 0x0000ffff         ) ;
    node.setTransmitBuffer(0, registerDataL);                        node.setTransmitBuffer(1, registerDataH);

    if( node.writeMultipleRegisters(REG_SECOND_CALIBRATION_VALUE, 2) ==node.ku8MBSuccess ){
      delay(250);
      registerDataH=(uint16_t) ( (thirdPointCalib >> 16 ) & 0x0000ffff ) ;registerDataL=(uint16_t) (  thirdPointCalib & 0x0000ffff         ) ;
      node.setTransmitBuffer(0, registerDataL);                        node.setTransmitBuffer(1, registerDataH);

      if ( node.writeMultipleRegisters (REG_THIRD_CALIBRATION_VALUE, 2) ==node.ku8MBSuccess  ){
        delay(250);
        registerDataH=(uint16_t) ( (fourthPointCalib >> 16 ) & 0x0000ffff ) ;registerDataL=(uint16_t) (  fourthPointCalib & 0x0000ffff         ) ;
        node.setTransmitBuffer(0, registerDataL);                        node.setTransmitBuffer(1, registerDataH);

        if ( node.writeMultipleRegisters(REG_FOURTH_CALIBRATION_VALUE, 2) ==node.ku8MBSuccess  ){
          return true;                                                                              // Escritura de puntos DWORD de calibracion existosa
        }else{
          return false; // No se pudo escribir la data del TX BUFFER
        }
        
      }else{
        return false; // No se pudo escribir la data del TX BUFFER
      }
    }else{
      return false; // No se pudo escribir la data del TX BUFFER
    }
  }else{
    return false; // No se pudo escribir la data del TX BUFFER
  }
  
}

bool REMOND_Set_Upper_Temperature_Limit(ModbusMaster& node, float upperTemperatureLimit){
  static uint32_t registerData;
  static uint16_t registerDataH,registerDataL; 
  
  registerData= *((uint32_t*) & upperTemperatureLimit ); // Convertir de float a HEX Float  de 4 palabras

  registerDataH=(uint16_t) ( (registerData >> 16 ) & 0x0000ffff ) ;
  registerDataL=(uint16_t) (  registerData & 0x0000ffff         ) ; 

  if (  node.writeSingleRegister( REG_UPPER_TEMPERATURE_LIMIT , registerDataL ) ==node.ku8MBSuccess   ){
    delay(250);
    if ( node.writeSingleRegister( REG_UPPER_TEMPERATURE_LIMIT + 0x0001, registerDataH) ==node.ku8MBSuccess ) {
      delay(250);                             //Estabilizar comunicacion
      return true;                                       //  Se pudo escribir el Upper Limit de temperatura
    }else{
      return false;
    }
  }else
    return false;                                       // No se pudo escribir el Upper Limit de temperatura                             
}

bool REMOND_Set_Lower_TemperatureLimit(ModbusMaster& node, float lowerTemperatureLimit){
  static uint32_t registerData;
  static uint16_t registerDataH,registerDataL; 
  
  registerData= *((uint32_t*) & lowerTemperatureLimit ); // Convertir de float a HEX Float  de 4 palabras

  registerDataH=(uint16_t) ( (registerData >> 16 ) & 0x0000ffff ) ;
  registerDataL=(uint16_t) (  registerData & 0x0000ffff         ) ; 

  if ( node.writeSingleRegister(REG_LOWER_TEMPERATURE_LIMIT , registerDataL ) ==node.ku8MBSuccess ) {
    delay(250);
    if ( node.writeSingleRegister(REG_LOWER_TEMPERATURE_LIMIT + 0x0001, registerDataH ) ==node.ku8MBSuccess  ){
      delay(250);                             // Estabilizar comunicacion
      return true;                                       //  Se pudo escribir el Lower Limit de temperatura
    }else{
      return false;
    }
     
  }else
    return false;                                     // No se pudo escribir el Lower Limit de temperatura                 

}

bool REMOND_Set_Baudrate (ModbusMaster& node, uint16_t baudrate) {
  if ( node.writeSingleRegister(REG_BAUD_RATE, baudrate ) ==node.ku8MBSuccess    ){
    delay(250);                               // Estabilizar comunicacion
    return true;
  }else
    return false;                                   //No se pudo setear el baudrate
}

bool REMOND_Set_Device_Address (ModbusMaster& node, uint8_t slaveAddressID){
  if ( node.writeSingleRegister(REG_DEVICE_ADDRESS, (uint16_t) slaveAddressID ) ==node.ku8MBSuccess    ){
    delay(250);                     // Estabilizar comunicacion
    return true;
  }else
    return false;                                 // No se pudo setear la direccion
}

uint8_t REMOND_Read_Device_Address (ModbusMaster& node, bool updateAddr, uint8_t &customID){
   static const uint16_t numReg=1 ;
   static uint16_t response[numReg];
   static uint8_t j;
   
  if ( node.readHoldingRegisters( REG_DEVICE_ADDRESS, numReg  ) ==node.ku8MBSuccess  ){
    delay(250);                             // Estabiliza comunicacion
    for (j=0 ; j<numReg;j++){
      response[j] = node.getResponseBuffer(j); // Leer el valor del Address ID
    }

    if (updateAddr){
      customID=response[0];
    }

    return (uint8_t) response[0];
    
  }else
    return 0;   // No se pudo leer la direccion
}



static bool updateValues(ModbusMaster& node){
  static const uint8_t numReg=7;
  static uint16_t startAddress=0x0001;
  static uint16_t response[numReg];
  static uint8_t j,result;
  static uint32_t registerData=0x00000000;   // Data output pointer

  //Si se lee antes del tiempo limite, no actualizar
  if( (unsigned long)(millis() - ( _lastRead) )  >  UPDATE_TIME){
        // Graba el tiempo actual
        _lastRead = millis();
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

    // Medicion de pH.......
    
                    //AB                                   CD
     registerData=( ( response[1] ) << 16 ) & 0xFFFF0000 | response[0] ;
     _currentValues.measurements=*((float*) &registerData );    // Cast HEX float to float value
     
                    //AB                                   CD
     registerData=( ( response[3] ) << 16 ) & 0xFFFF0000 | response[2] ;
     _currentValues.temperature=*((float*) &registerData );    // Cast HEX float to float value

                   //AB                                   CD
     registerData=( ( response[5] ) << 16 ) & 0xFFFF0000 | response[4] ;
     _currentValues.currentOut=*((float*) &registerData );    // Cast HEX float to float value

     _currentValues.warnings= (uint8_t) ( response[6]) & 0x00FF ; // As warnings go from 00 to 04 only !!
     
     return true;                //Si hubo lectura exitosa, retorna True
      
  }else{
      return false;               // Si no hubo lectura exitosa, retorna error
  }
  
}

static bool calibrationUpdate(ModbusMaster& node){
  static const uint8_t numReg=2 ;
  static uint16_t response[numReg];
  static uint8_t j;
  
  if (  node.readHoldingRegisters(REG_ADC_VALUE,(uint16_t) numReg ) == node.ku8MBSuccess){
    for (j=0 ; j<numReg;j++){
      response[j] = node.getResponseBuffer(j); // Lee el valor de lectura del ADC en el registro 0x0066......... solo para calibrar
    }

    _calibrationValue.measuredADC_CD=response[0] ;  // CD
    _calibrationValue.measuredADC_AB=response[1]; // AB
    
    return true;
    
  }else
    return false;         // No se pudo leer el registro de ADC... 
    
}

void sensor_TURBIDITY_Calibration(ModbusMaster& node){
  static char Message[100];
  static uint32_t adcValue;
  
  Serial.println("Before calibration.....Wish to perform float [F] or DOWRD calibration [D]? [F / D]: ");

  while(1){
    if ( ((char) Serial.read()) =='F' ){
      if ( REMOND_TURBIDITY_Set_float_Calibration_Points(node, turbiditYfirstPointCalib, turbiditYSecondPointCalib, turbiditYThirdPointCalib, turbiditYFourthPointCalib)){
        sprintf(Message, "OK SET FLOAT CALIB POINTS TO :: 1.-%.3f 2.-%.3f  3.-%.3f  4.-%.3f", turbiditYSecondPointCalib, turbiditYThirdPointCalib, turbiditYFourthPointCalib);
        Serial.println(Message);
        break;
      }else{
        Serial.println("SORRY, NO SET CALIB POINTS, RESET ESP32 !!!");
        while(1);
      }
      
    }else if (((char) Serial.read()) =='D'){
      if ( REMOND_TURBIDITY_Set_DWord_Calibration_Points(node, turbiditYfirstDWordPointCalib, turbiditYSecondDWordPointCalib, turbiditYThirdDWordPointCalib, turbiditYFourthDWordPointCalib)){
        Serial.println("OK SET DWORD CALIB POINTS");
        break;
      }else{
        Serial.println("SORRY, NO SET CALIB POINTS, RESET ESP32 !!!");
        while(1);
      }      
    }
    delay(50);
  }

  Serial.println("Start Calibration.... First step: Put the sensor on first point of calibration solution....(press '1' to confirm ADC (stable) Value ):");

   while ( ( (char) Serial.read() )!='1'){
    Serial.print("ADC Value : ");
    adcValue=REMOND_Get_ADC_Value(node); 
    Serial.println(adcValue,DEC);
    delay(500);
   }

   if (REMOND_Set_First_ADC_Value(node, adcValue)){
    Serial.println("OK FIRST ADC VALUE CONFIRMED !");
   }else{
    Serial.println("COULD NOT ESTABLISH VALUE.....RESET ESP32 !! !");
    while(1);
   }

   Serial.println("Start Calibration.... Second step: Put the sensor on second point of calibration solution....(press '2' to confirm ADC (stable) Value ):");

   while ( ( (char) Serial.read() )!='2'){
    Serial.print("ADC Value : ");
    adcValue=REMOND_Get_ADC_Value(node); 
    Serial.println(adcValue,DEC);
    delay(500);
   }

   if (REMOND_Set_Second_ADC_Value(node, adcValue)){
    Serial.println("OK SECOND ADC VALUE CONFIRMED !");
   }else{
    Serial.println("COULD NOT ESTABLISH VALUE.....RESET ESP32 !! !");
    while(1);
   }

  Serial.println("Start Calibration.... Third step: Put the sensor on third point of calibration solution....(press '3' to confirm ADC (stable) Value ):");

   while ( ( (char) Serial.read() )!='3'){
    Serial.print("ADC Value : ");
    adcValue=REMOND_Get_ADC_Value(node); 
    Serial.println(adcValue,DEC);
    delay(500);
   }

   if (REMOND_Set_Third_ADC_Value(node, adcValue)){
    Serial.println("OK THIRD ADC VALUE CONFIRMED !");
   }else{
    Serial.println("COULD NOT ESTABLISH VALUE.....RESET ESP32 !! !");
    while(1);
   }


  Serial.println("Start Calibration.... Fourth step: Put the sensor on fourth point of calibration solution....(press '4' to confirm ADC (stable) Value ):");

   while ( ( (char) Serial.read() )!='4'){
    Serial.print("ADC Value : ");
    adcValue=REMOND_Get_ADC_Value(node); 
    Serial.println(adcValue,DEC);
    delay(500);
   }

   if (REMOND_Set_Fourth_ADC_Value(node, adcValue)){
    Serial.println("OK THIRD ADC VALUE CONFIRMED !");
   }else{
    Serial.println("COULD NOT ESTABLISH VALUE.....RESET ESP32 !! !");
    while(1);
   }

  Serial.println("Calibration of TURBIDITY sensor Sucess !! Press 'r' key to continue....");
  while ( ((char) Serial.read())!='r' );
  delay(1000);

 
}
