#include <ModbusMaster.h>
#include <phREMOND.h>
#include <EC_REMOND.h>
#include <DO_REMOND.h>
#include <NO3_REMOND.h>
#include <TURBIDITY_REMOND.h>

//Actualizar estos pines !!!!!!!!!!!
#define MAX485_DE          12
#define MAX485_RE_NEG      12

#define MOSFET_ENABLE   5 

//#define PH_CALIBRATION_MODE    // Descomentar si se va a calibar el phimetro
//#define DO_CALIBRATION_MODE    //Descomentar si se va a calibar el sensor DO

#define DO_MEASURE_MODE_MGL   // SETEO DE MODO DEL DO

#define MODBUS_SERIAL   Serial2
#define MODBUS_BAUDRATE 9600

#define CONSOLE_SERIAL  Serial
#define CONSOLE_BAUDRATE  115200

/*
 * Pines a asiganr al Modbus Serial (pines remapeables ESP32, cuidado con los INPUT ONLY ! )
 */
#if !defined(MODBUS_RX_PIN) && !defined(MODBUS_TX_PIN)
#define MODBUS_RX_PIN   23                              // Acorde diagrama OK !
#define MODBUS_TX_PIN   19
#endif


// Constantes Globales
const uint8_t ECslaveID=0x01;    //Direccion default EC
const uint8_t pHslaveID=0x02;    //Direccion seteada para el phimetro
const uint8_t NO3slaveID=0x03;  // Direccion seteada para sensor de nitrato NO3
const uint8_t DOslaveID=0x05;  // Direccion seteada para el sensor de DO  (oxigeno disuelto)
const uint8_t TURBslaveID=0x04;  // Direccion seteada para el sensor TURBIDIMETRO

// Variables globales
char Str_Measures[200]; // String para trama de medidas

static struct {         // Struct para guarda de valores de medida
  float ph;
  float ppmNO3;
  float dOmgL;
  float ntu;
  float turbidityWarning;
  float conductivity;
  float resistivity;
  float temperature;
  float tds;
  float salinity;
}_allMeasurements;

// Objeto Modbus para phimetro
ModbusMaster nodePH;

//Objeto Modbus para el NO3 ppm
ModbusMaster nodeNO3;

//Objeto Modbus para el DO  mg/L
ModbusMaster nodeDO;

//Objeto Modbus para el turbidimetro NTU
ModbusMaster nodeTURB;

// Objeto Modbus para el EC
ModbusMaster nodeEC;

void setup() {

  pinMode(MOSFET_ENABLE,OUTPUT);
  digitalWrite(MOSFET_ENABLE,LOW);
  
   //port 0 debug
  CONSOLE_SERIAL.begin(CONSOLE_BAUDRATE);

  //Iniciar pines MAX485
  MAX485_begin();

   //Communicate with Modbus Slave ID over Serial 2 
  //..............................(RX, TX pin) ESP32  
  MODBUS_SERIAL.begin(MODBUS_BAUDRATE,SERIAL_8N1,MODBUS_RX_PIN,MODBUS_TX_PIN);

   //Modbus node begin "slaveID"........ Iniciar comunicacion sensores Modbus
  nodePH.begin(pHslaveID,MODBUS_SERIAL);      // Begin Node with ADDR..
  nodeNO3.begin(NO3slaveID,MODBUS_SERIAL);
  nodeDO.begin(DOslaveID,MODBUS_SERIAL);
  
  nodeTURB.begin(TURBslaveID,MODBUS_SERIAL);
  
  nodeEC.begin(ECslaveID,MODBUS_SERIAL);

   // Callbacks de seteo de modo para los sensores
   
  nodePH.preTransmission(preTransmission);
  nodePH.postTransmission(postTransmission);
  nodeNO3.preTransmission(preTransmission);
  nodeNO3.postTransmission(postTransmission);
  nodeDO.preTransmission(preTransmission);
  nodeDO.postTransmission(postTransmission);
  
  
  nodeTURB.preTransmission(preTransmission);
  nodeTURB.postTransmission(postTransmission);
  
  
  nodeEC.preTransmission(preTransmission);
  nodeEC.postTransmission(postTransmission);

  #if defined (PH_CALIBRATION_MODE)
    REMOND_pH_Calibration(nodePH);
  #endif

  #if defined(DO_CALIBRATION_MODE)
    REMOND_DO_Calibration(nodeDO, false);
  #endif

 #if defined(DO_MEASURE_MODE_MGL)
  if (REMOND_Set_Measure_Mode(nodeDO, 1) ){             // 00 : %   , 01:mg/L
    Serial.println("SENSOR DO ESTABLECIDO A mg/L !!");
  }else{
    Serial.println("NO SE PUDO ESTABLECER EL DO a mg/L");
  }
 #endif
}

void loop() {

   _allMeasurements.ph=REMOND_pH_Get_Measurement(nodePH);
  _allMeasurements.ppmNO3=REMOND_NO3_Get_Measurement(nodeNO3);
  _allMeasurements.dOmgL=REMOND_DO_Get_Measurement(nodeDO);
  _allMeasurements.ntu=REMOND_TURB_Get_Measurement(nodeTURB);
  _allMeasurements.turbidityWarning=REMOND_TURB_Get_Warning(nodeTURB);
  
  _allMeasurements.conductivity=REMOND_EC_Get_Conductivity(nodeEC);
  _allMeasurements.resistivity=REMOND_EC_Get_Resistivity(nodeEC);
  _allMeasurements.temperature=REMOND_EC_Get_Temperature(nodeEC);
  _allMeasurements.tds=REMOND_EC_Get_Tds(nodeEC);
  _allMeasurements.salinity=REMOND_EC_Get_Salinity(nodeEC);

  sprintf(Str_Measures,"pH=%.2f\n\rppmNO3=%.2f\n\rdOmgL=%.2f\n\rNTU=%.2f\n\rCond=%.2f\n\rRes=%.2f\n\rTemp=%.2f\n\rtds=%.2f\n\rsalinity=%.2f",_allMeasurements.ph,_allMeasurements.ppmNO3, _allMeasurements.dOmgL, _allMeasurements.ntu,_allMeasurements.conductivity,_allMeasurements.resistivity,_allMeasurements.temperature,_allMeasurements.tds,_allMeasurements.salinity);
  //sprintf(Str_Measures,"NTU=%.2f Warning=0x%x", _allMeasurements.ntu,_allMeasurements.turbidityWarning);

   // Imprimir data al puerto serial
 CONSOLE_SERIAL.println(Str_Measures);
 CONSOLE_SERIAL.println();

 delay(1000);
}


/*
 * Asignar pines al MAX 485
 */
void MAX485_begin(){
 
  // Iniciar los pines de MAX485 como salidas ambos
  pinMode(MAX485_RE_NEG, OUTPUT);
  pinMode(MAX485_DE, OUTPUT);
  
}

/*
 * Rutinas de enable TX/ RX para seteo de modo de solo TX o solo RX (Half Duplex)
 ... https://programarfacil.com/wp-content/uploads/2020/09/MAX485-interno.jpg
 */
void preTransmission()
{
  digitalWrite(MAX485_RE_NEG, HIGH);  //Deshabilita recepcion
  digitalWrite(MAX485_DE, HIGH);    //........................Habilita Transmision
}

void postTransmission()
{
  digitalWrite(MAX485_RE_NEG, LOW); //........................Habilita recepcion
  digitalWrite(MAX485_DE, LOW);  // Deshabilita transmision
}
