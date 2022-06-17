#include <ModbusMaster.h>
#include "TurbidityCALIBRATION.h"

//Actualizar estos pines !!!!!!!!!!!
#define MAX485_DE          12
#define MAX485_RE_NEG      12

#define MOSFET_ENABLE   5

#define TURB_CALIBRATION_MODE         // Descomentar si se va a calibrar

/*
 * Directivas de comunicacion serial (port0) y comunicacion Modbus
 */
#define MODBUS_SERIAL   Serial2
#define MODBUS_BAUDRATE 9600

#define CONSOLE_SERIAL  Serial
#define CONSOLE_BAUDRATE  115200

// Pines a asiganr al Modbus Serial (pines remapeables ESP32, cuidado con los INPUT ONLY ! )
#if !defined(MODBUS_RX_PIN) && !defined(MODBUS_TX_PIN)
#define MODBUS_RX_PIN   23                              //Acorde diagrama OK !
#define MODBUS_TX_PIN   19
#endif


// Constantes Globales
const uint8_t TURBslaveID=0x01;  // Direccion seteada para el sensor TURBIDIMETRO [NTU]

// Variables globales
char Str_Measures[200]; // String para trama de medidas

static struct {         // Struct para guarda de valores de medida
  float ntu;
}_allMeasurements;


//Objeto Modbus para el turbidimetro NTU
ModbusMaster nodeTURB;


void setup() {

  pinMode(MOSFET_ENABLE,OUTPUT);
  digitalWrite(MOSFET_ENABLE,LOW);

  //port 0 debug
  CONSOLE_SERIAL.begin(CONSOLE_BAUDRATE);

  //Iniciar pines MAX485
  MAX485_begin();

  //Communicarse via Modbus slave ID a traves del Serial X 
  //..............................(RX, TX pin) ESP32  
  MODBUS_SERIAL.begin(MODBUS_BAUDRATE,SERIAL_8N1,MODBUS_RX_PIN,MODBUS_TX_PIN);

  //Modbus node begin "slaveID"........ Iniciar comunicacion sensores Modbus
  nodeTURB.begin(TURBslaveID,MODBUS_SERIAL);

  // Callbacks de seteo de modo para los sensor(es)
  nodeTURB.preTransmission(preTransmission);
  nodeTURB.postTransmission(postTransmission);

  #if defined(TURB_CALIBRATION_MODE)
    sensor_turbidity_Calibration(nodeTURB);
  #endif

}

void loop() {
  _allMeasurements.ntu=REMOND_Turbidity_Get_Measurement(nodeTURB);          // Toma de medidas de prueba del sensor de turbidez

  sprintf(Str_Measures,"NTU=%.2f\n\r",_allMeasurements.ntu);

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
