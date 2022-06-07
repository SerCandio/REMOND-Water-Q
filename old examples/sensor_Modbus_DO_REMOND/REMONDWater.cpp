#include "Arduino.h"
#include "REMONDWater.h"
#include <ModbusMaster.h>

#define UPDATE_TIME        250

#define REG_ZERO_CALIBRATION     0x003E
#define REG_SLOPE_CALIBRATION    0x003F

#define REG_MEASURED_ADC         0x0066

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

uint16_t REMOND_Get_ADC_Value(ModbusMaster& node){
  if ( !calibrationUpdate(node) )//Actualizar calibracion si es necesario
    return NAN ;            //En caso de error, retorna Nan

  return _calibrationValue.measuredADC;  
}

bool REMOND_Set_Zero_calibration(ModbusMaster& node){
  if (  node.writeSingleRegister(REG_ZERO_CALIBRATION, 0x00ff) == node.ku8MBSuccess ) {
    return true;
  }else
    return false;
}

bool REMOND_Set_Slope_calibration(ModbusMaster& node){
 if (  node.writeSingleRegister(REG_SLOPE_CALIBRATION, 0x00ff) == node.ku8MBSuccess ) {
    return true;
  }else
    return false;  
}

bool REMOND_Set_Measure_Mode(ModbusMaster& node, uint16_t measureMode){
  if ( node.writeSingleRegister(REG_MEASURE_MODE, measureMode ) ==node.ku8MBSuccess    ){
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

void sensor_DO_Calibration(ModbusMaster& node, bool twoPoints){
  Serial.println("The First Point: zero calibration : Put electrode in a 0 mg/L solution (ie. anhydrous sulfite solution) ....(press 's' to confirm ADC (stable) Value):");

  while ( ( (char) Serial.read() )!='s'){
      Serial.print("ADC Value : ");
      Serial.println(REMOND_Get_ADC_Value(node),DEC);
      delay(500);
  }

  if ( REMOND_Set_Zero_calibration(node)){
    Serial.println("OK ZERO CONFIRMATED !");
  }else{
    Serial.println("COULD NOT ESTABLISH ZERO.....RESET ESP32 !! !");
    while(1);
  }

  
if (twoPoints==true){             // Es posible hacer la calibracion solo con 1 punto. ...

  Serial.println("The Second Point: slope calibration : Put sensor in saturate air (ie. 100%)....(press 'S' to confirm ADC (stable) Value):");

  while ( ( (char) Serial.read() )!='S'){
    Serial.print("ADC Value : ");
    Serial.println(REMOND_Get_ADC_Value(node),DEC);
    delay(500);
  }

  if (REMOND_Set_Slope_calibration(node)){
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
