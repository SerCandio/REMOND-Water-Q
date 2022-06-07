#include "Arduino.h"
#include "REMONDWater.h"
#include <ModbusMaster.h>
#include "phREMOND.h"

#define UPDATE_TIME        250

#define REG_ZERO_POINT_CALIB_SOL  0x0036
#define REG_SLOPE_CALIB_SOL       0x0038
#define REG_ZERO_CONFIRMATION     0x003E
#define REG_SLOPE_CONFIRMATION    0x003F
#define REG_MEASURED_ADC          0x0066

#define REG_MEASURE_MODE            0x0008
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


float REMOND_Get_pH(ModbusMaster& node){
  if( !updateValues(node) )  //Actualizar si es necesario
    return NAN;         //En caso de error, retorna Nan
    
  return _currentValues.measurements;  
}

float REMOND_Get_Temperature(ModbusMaster& node){
   if( !updateValues(node) )  //Actualizar si es necesario
    return NAN;            //En caso de error, retorna Nan
    
  return _currentValues.temperature;
}

float REMOND_Get_Current_PH_ORP(ModbusMaster& node){
   if( !updateValues(node) )  //Actualizar si es necesario
    return NAN;            //En caso de error, retorna Nan
    
  return _currentValues.currentOut;  
}

uint8_t REMOND_Get_Warning(ModbusMaster& node){
  if( !updateValues(node) )  //Actualizar si es necesario
    return NAN;            //En caso de error, retorna Nan

   return _currentValues.warnings;  
}

uint16_t REMOND_Get_ADC_Value(ModbusMaster& node){
  if ( !calibrationUpdate(node) )//Actualizar calibracion si es necesario
    return NAN ;            //En caso de error, retorna Nan

  return _calibrationValue.measuredADC;  
}

bool REMOND_Set_Zero_calib(ModbusMaster& node, uint16_t zeroCalibSolution){
  if ( node.writeSingleRegister(REG_ZERO_POINT_CALIB_SOL ,zeroCalibSolution ) == node.ku8MBSuccess  ){
    return true;
  }else
    return false;           //Si ocurre error, retorna falso.......(no se pudo calibrar)  
}

bool REMOND_Set_Slope_calib(ModbusMaster& node,uint16_t slopeCalibSolution){
  if ( node.writeSingleRegister(REG_SLOPE_CALIB_SOL, slopeCalibSolution ) == node.ku8MBSuccess  ){
    return true;
  }else
    return false;          //Si ocurre error, retorna falso.......(no se pudo calibrar)
  
}

bool REMOND_Set_Zero_confirmation(ModbusMaster& node){
  if (  node.writeSingleRegister(REG_ZERO_CONFIRMATION, 0x00ff) == node.ku8MBSuccess ) {
    return true;
  }else
    return false;
}

bool REMOND_Set_Slope_confirmation(ModbusMaster& node){
 if (  node.writeSingleRegister(REG_SLOPE_CONFIRMATION, 0x00ff) == node.ku8MBSuccess ) {
    return true;
  }else
    return false;  
}

bool REMOND_Set_Measure_Mode(ModbusMaster& node, uint16_t measureMode){
  if ( node.writeSingleRegister(REG_ZERO_POINT_CALIB_SOL, measureMode ) ==node.ku8MBSuccess    ){
    delay (250) ;                                       // Estabilizar comunicacion
    return true;
  }else
    return false;                                     // No se pudo establecer el modo de medicion
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

  //Leer numReg=(2) registros de 16 bit a partir de la direccion startAddress=0x001
  result=node.readHoldingRegisters(startAddress,(uint16_t) numReg);

  if(result == node.ku8MBSuccess)
  {
     for (j = 0; j < numReg; j++)  
     {
      response[j] = node.getResponseBuffer(j);  // Lecturas de PH son las 2 primeras :  0x0001  -->> C D   ; 0x0002 -->> A B......... por probar
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
  static const uint8_t numReg=1 ;
  static uint16_t response[numReg];
  static uint8_t j;
  
  if (  node.readHoldingRegisters(REG_MEASURED_ADC,(uint16_t) numReg ) == node.ku8MBSuccess){
    for (j=0 ; j<numReg;j++){
      response[j] = node.getResponseBuffer(j); // Lee el valor de lectura del ADC en el registro 0x0066......... solo para calibrar
    }

    _calibrationValue.measuredADC=response[0] ;
    
    return true;
    
  }else
    return false;         // No se pudo leer el registro de ADC... 
    
}

void sensor_pH_Calibration(ModbusMaster& node){
     Serial.println("Before calibration......Choose zero point : ph6.86 (press 6) or ph7.00 (press 7) ???: ");
    
    while(1){
      if ( ((char) Serial.read()) =='6'){
        
        if ( REMOND_Set_Zero_calib(node, zeropH_6_86_Solution) ){
          Serial.println("OK SET ZERO CALIB ph6.86");
          break;
        }
        else{
          Serial.println("SORRY, NO ZERO POINT CALIB, RESET ESP32 !!!"); 
          while (1); 
        }
      }else if ( ((char) Serial.read()) =='7'){

         if ( REMOND_Set_Zero_calib(node, zeropH_7_Solution) ){
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
        if ( REMOND_Set_Slope_calib(node, slopepH_1_68_Solution) ){
          Serial.println("OK SET SLOPE CALIB ph1.68");
          break;
        }
        else{
          Serial.println("SORRY, NO SLOPE POINT CALIB, RESET ESP32 !!!"); 
          while (1); 
        }
      }else if ( ((char) Serial.read()) =='4' ){
         if ( REMOND_Set_Slope_calib(node, slopepH_4_01_Solution) ){
          Serial.println("OK SET SLOPE CALIB ph4.01");
          break;
         }
        else{
          Serial.println("SORRY, NO SLOPE POINT CALIB, RESET ESP32 !!!"); 
          while (1); 
        }
      }else if ( ((char) Serial.read()) =='9' ){
        if ( REMOND_Set_Slope_calib(node, slopepH_9_18_Solution) ){
          Serial.println("OK SET SLOPE CALIB ph9.18");
          break;
        }
        else{
          Serial.println("SORRY, NO SLOPE POINT CALIB, RESET ESP32 !!!"); 
          while (1);
        }
      }else if ( ((char) Serial.read()) =='3' ){
         if ( REMOND_Set_Slope_calib(node, slopepH_10_1_Solution) ){
          Serial.println("OK SET SLOPE CALIB ph10.1");
          break;
         }
        else{
          Serial.println("SORRY, NO SLOPE POINT CALIB, RESET ESP32 !!!"); 
          while (1);
        }
      }else if ( ((char) Serial.read()) =='0' ){
         if ( REMOND_Set_Slope_calib(node, slopepH_12_45_Solution) ){
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
      Serial.println(REMOND_Get_ADC_Value(node),DEC);
      delay(500);
    }

    if ( REMOND_Set_Zero_confirmation(node) ){
      Serial.println("OK ZERO CONFIRMATED !");
    }else{
      Serial.println("COULD NOT ESTABLISH ZERO.....RESET ESP32 !! !");
      while(1);
    }

   Serial.println("Calibration ...Second step : Put the sensor onto 4.01 slope point calibration....(press 'S' to confirm ADC (stable) Value ): ");

   while ( ( (char) Serial.read() )!='S') {
      Serial.print("ADC Value : ");
      Serial.println(REMOND_Get_ADC_Value(node),DEC);
      delay(500);
    }

    if ( REMOND_Set_Slope_confirmation(node) ){
      Serial.println("OK SLOPE CONFIRMATED !");
    }else{
      Serial.println("COULD NOT ESTABLISH SLOPE.....RESET ESP32 !! !");
      while(1);
    }

   Serial.println("Calibration of PH Sucess !! Press 'r' key to continue....");
   while ( ((char) Serial.read())!='r' );
   delay(1000);
}
