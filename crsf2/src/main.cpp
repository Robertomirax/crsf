/*
ESP32-S3 adafruit_qtpy_esp32s3_n4r2

Lee por rxd2 (serial1) lo que envía el ESC (voltaje, corriente y mAh consumidos) a 115.200 baudios.
Transmite por Serial (USB C) a la pantalla, los datos decodificados recibidos.
Transmite por el pin TX (Serial0) los datos codificados en formato CRSF

*/
#include <Arduino.h>
#define RXD2 2 // 1  //16
#define TXD2 17

#define WDT_TIMEOUT 2 // segundos de espera del Watchdog timer

unsigned char crc8tab[256] = {
    0x00, 0xD5, 0x7F, 0xAA, 0xFE, 0x2B, 0x81, 0x54, 0x29, 0xFC, 0x56, 0x83, 0xD7, 0x02, 0xA8, 0x7D,
    0x52, 0x87, 0x2D, 0xF8, 0xAC, 0x79, 0xD3, 0x06, 0x7B, 0xAE, 0x04, 0xD1, 0x85, 0x50, 0xFA, 0x2F,
    0xA4, 0x71, 0xDB, 0x0E, 0x5A, 0x8F, 0x25, 0xF0, 0x8D, 0x58, 0xF2, 0x27, 0x73, 0xA6, 0x0C, 0xD9,
    0xF6, 0x23, 0x89, 0x5C, 0x08, 0xDD, 0x77, 0xA2, 0xDF, 0x0A, 0xA0, 0x75, 0x21, 0xF4, 0x5E, 0x8B,
    0x9D, 0x48, 0xE2, 0x37, 0x63, 0xB6, 0x1C, 0xC9, 0xB4, 0x61, 0xCB, 0x1E, 0x4A, 0x9F, 0x35, 0xE0,
    0xCF, 0x1A, 0xB0, 0x65, 0x31, 0xE4, 0x4E, 0x9B, 0xE6, 0x33, 0x99, 0x4C, 0x18, 0xCD, 0x67, 0xB2,
    0x39, 0xEC, 0x46, 0x93, 0xC7, 0x12, 0xB8, 0x6D, 0x10, 0xC5, 0x6F, 0xBA, 0xEE, 0x3B, 0x91, 0x44,
    0x6B, 0xBE, 0x14, 0xC1, 0x95, 0x40, 0xEA, 0x3F, 0x42, 0x97, 0x3D, 0xE8, 0xBC, 0x69, 0xC3, 0x16,
    0xEF, 0x3A, 0x90, 0x45, 0x11, 0xC4, 0x6E, 0xBB, 0xC6, 0x13, 0xB9, 0x6C, 0x38, 0xED, 0x47, 0x92,
    0xBD, 0x68, 0xC2, 0x17, 0x43, 0x96, 0x3C, 0xE9, 0x94, 0x41, 0xEB, 0x3E, 0x6A, 0xBF, 0x15, 0xC0,
    0x4B, 0x9E, 0x34, 0xE1, 0xB5, 0x60, 0xCA, 0x1F, 0x62, 0xB7, 0x1D, 0xC8, 0x9C, 0x49, 0xE3, 0x36,
    0x19, 0xCC, 0x66, 0xB3, 0xE7, 0x32, 0x98, 0x4D, 0x30, 0xE5, 0x4F, 0x9A, 0xCE, 0x1B, 0xB1, 0x64,
    0x72, 0xA7, 0x0D, 0xD8, 0x8C, 0x59, 0xF3, 0x26, 0x5B, 0x8E, 0x24, 0xF1, 0xA5, 0x70, 0xDA, 0x0F,
    0x20, 0xF5, 0x5F, 0x8A, 0xDE, 0x0B, 0xA1, 0x74, 0x09, 0xDC, 0x76, 0xA3, 0xF7, 0x22, 0x88, 0x5D,
    0xD6, 0x03, 0xA9, 0x7C, 0x28, 0xFD, 0x57, 0x82, 0xFF, 0x2A, 0x80, 0x55, 0x01, 0xD4, 0x7E, 0xAB,
    0x84, 0x51, 0xFB, 0x2E, 0x7A, 0xAF, 0x05, 0xD0, 0xAD, 0x78, 0xD2, 0x07, 0x53, 0x86, 0x2C, 0xF9};

u_int8_t datain[64] = {};
uint8_t largoin = 0;
u_int8_t dataout[64] = {};
uint8_t largoout = 0;

unsigned long previousMillis = 0;
const long interval = 20; // milisegundos

// declaración de funciones
void analizar1(uint8_t *vector, uint8_t lar);  // analiza la información recibida y arma el vector a transmitir al receptor
void analizar2(uint8_t *vector, uint8_t lar);  // analiza la informaciónde RPM recibidos y arma el vector a transmitir al receptor
uint8_t crc8(const uint8_t *ptr, uint8_t len); // calcula el crc de los datos recibidos por el puerto serie crsf
void analizar3(uint8_t *vector, uint8_t lar);  // analiza la información de Temperatura recibida y arma el vector a transmitir al receptor
void muestraPan(uint8_t *vector, uint8_t lar); // muestra en pantalla los datos recibidos y el cheksum

void setup()
{
  // inicialización del watchdog timer
  // esp_task_wdt_init(WDT_TIMEOUT, true); // habilita el reseteo del chip cuando se activa el wdt
  // esp_task_wdt_add(NULL);

  Serial.begin(115200);                          // comunicación por el conector USB C
  Serial1.begin(115200, SERIAL_8N1, RXD2, TXD2); // inicia comunicación con esc (recibe)
  Serial0.begin(420000, SERIAL_8N1, -1, -1);     // inicia comunicación crsf con el receptor (transmite). El Serial0 tiene conexión fija a los pines TX y RX

  Serial.println();
  Serial.println();
  Serial.println();
  Serial.println();
}

void loop()
{
  // reseteo del wdt
  // esp_task_wdt_reset();

  if (Serial1.available())
  {

    largoin = Serial1.available(); // obtiene la longitud del vector recibido

    Serial1.readBytes(datain, largoin); // lee los datos provenientes del speed control

    analizar1(datain, largoin); // analiza la informaciónde voltaje, corriente y mAh consumidos recibidos y arma el vector a transmitir al receptor
    Serial0.write(dataout, 12); // envía datos por crsf al receptor

    analizar2(datain, largoin); // aanaliza la informaciónde RPM recibidos y arma el vector a transmitir al receptor
    Serial0.write(dataout, 8); // envía datos por crsf al receptor

    analizar3(datain, largoin); // // analiza la información de Temperatura recibida y arma el vector a transmitir al receptor
    Serial0.write(dataout, 8); // envía datos por crsf al receptor

    // muestra en pantalla los datos recibidos y el cheksum
    muestraPan(datain, largoin);
    
  }
}










// funciones------------------------------------------------------------------------------------------------------------

// calcula el crc8 de un vector descartando los dos primeros y el último byte
uint8_t crc8(const uint8_t *ptr, uint8_t len)
{
  *ptr++;
  *ptr++;
  len = len - 3;
  uint8_t crc = 0;
  for (uint8_t i = 0; i < len; i++)
    crc = crc8tab[crc ^ *ptr++];
  return crc;
}

// analiza la informaciónde voltaje, corriente y mAh consumidos recibidos y arma el vector a transmitir al receptor
void analizar1(uint8_t *vector, uint8_t lar)
{

  // armar dataout
  dataout[0] = 0xC8;      // Syn byte
  dataout[1] = 10;        // largo del vector
  dataout[2] = 0x08;      // Type
  dataout[3] = vector[3]; // Voltaje byte alto
  dataout[4] = vector[4]; // Voltaje byte bajo

  dataout[5] = vector[5]; // corriente byte alto
  dataout[6] = vector[6]; // corriente byte bajo

  dataout[7] = 0; // capacidad usada en mAh byte alto
  dataout[8] = vector[15]; // capacidad usada en mAh byte medio
  dataout[9] = vector[16]; // capacidad usada en mAh byte bajo

  dataout[10] = 0; // Battery remaining (percent)

  // cálculo del crc8
  uint8_t crc = crc8(dataout, 12);
  // Serial.print("    crc calculado ->");
  // Serial.print(crc, DEC);
  dataout[11] = crc; // crc calculado
}

// analiza la informaciónde RPM recibidos y arma el vector a transmitir al receptor
void analizar2(uint8_t *vector, uint8_t lar)
{

  // armar dataout
  dataout[0] = 0xC8;      // Sync byte
  dataout[1] = 6;         // largo del vector
  dataout[2] = 0x0C;      // Type
  dataout[3] = 0;         // id del motor
  dataout[4] = 0;         // 0;         // RPM byte alto
  dataout[5] = vector[8]; // RPM byte medio
  dataout[6] = vector[9]; // rpm byte bajo
 
  // cálculo del crc8
  uint8_t crc = crc8(dataout, 8);
  dataout[7] = crc; // crc calculado
}

// analiza la información de Temperatura recibida y arma el vector a transmitir al receptor
void analizar3(uint8_t *vector, uint8_t lar)
{
  // obtener temperatura 
  int16_t temp = vector[10] * 10;
  int8_t temph = (temp >> 8) & 0xFF ;
  int8_t tempL = temp & 0xFF;
  // armar dataout
  dataout[0] = 0xC8;   // Sync byte
  dataout[1] = 5;     // largo del vector
  dataout[2] = 0x0D;  // Type
  dataout[3] = 0;     // origen
  dataout[4] = temph;     // temp byte alto
  dataout[5] = tempL; // MOS temp byte bajo
  dataout[6] = 0;     // espacio para el crc8

  // cálculo del crc8
  uint8_t crc = crc8(dataout, 7);
  dataout[6] = crc; // crc calculado
}


// muestra en pantalla los datos recibidos y el cheksum
void muestraPan(uint8_t *vector, uint8_t lar)
{

  
  int16_t sum = 0;
  for (size_t i = 0; i < largoin; i++)
  {
    Serial.print("-");
    Serial.print(datain[i], HEX);
  }

  for (size_t i = 0; i < 29; i++)
  {
    sum += datain[i];
  }
  Serial.print(" ------------ ");
  Serial.println(sum);

  // muestra Voltaje  10-1V, data range: 0 ? 0x3e8  100V
  int16_t volt = datain[3] * 256 + datain[4];
  Serial.print("Voltaje ");
  Serial.println(volt);

  // muestra Corriente 10-1A, data range: 0 ? 0x1388 500A
  int16_t amp = datain[5] * 256 + datain[6];
  Serial.print("Corriente ");
  Serial.println(amp);

  // muestra Throttle-PCT 0x01-1%, data range: 0 ? 0x64  100% input
  int16_t thr = datain[7];
  Serial.print("Throttle-PCT ");
  Serial.println(thr);

  // muestra RPM  0x01-10RPM, data range: 0 ? 0xffff
  int16_t rpm = datain[8] * 256 + datain[9];
  Serial.print("RPM ");
  Serial.println(rpm);

  // muestra Mos-temp  0x01 ? 1?, data range: 0 ?0x96 150?
  int16_t mos = datain[10];
  Serial.print("Mos-temp ");
  Serial.println(mos);

  // muestra Motor-temp  0x01 ? 1?, data range: 0 ?0x96 150?
  int16_t mot = datain[11];
  Serial.print("Motor-temp ");
  Serial.println(mot);

  // muestra Throttle-PWM  0x01-1% , data range: 0 ? 0x64  100% output
  int16_t tpwm = datain[12];
  Serial.print("Throttle-PWM ");
  Serial.println(tpwm);

  // muestra Estado-H
  int16_t esth = datain[13];
  Serial.print("Estado-H ");
  Serial.print(esth);

  switch (esth)
  {
  case 0x01:
    Serial.println(" Short-circuit protection");
    break;
  case 0x02:
    Serial.println(" motor wire break");
    break;
  case 0x04:
    Serial.println(" PPM TH loss protection");
    break;
  case 0x08:
    Serial.println(" Power on TH is not zero");
    break;
  case 0x10:
    Serial.println(" Low-voltage protection");
    break;
  case 0x20:
    Serial.println(" Temperature protection");
    break;
  case 0x40:
    Serial.println(" Start locked-rotor");
    break;
  case 0x80:
    Serial.println(" Current protection");
    break;

  default:
    Serial.println(" OK");
    break;
  }

  // muestra Estado-L
  int16_t estL = datain[14];
  Serial.print("Estado-L ");
  Serial.print(estL);

  switch (estL)
  {
  case 0x01:
    Serial.println(" PPM throttle is not within the regulated range, the PPM throttle is in an abnormal state, and the throttle is not within 700us~2500us");
    break;
  case 0x02:
    Serial.println(" UART Throttle is not within the regulated range, UART throttle is in an abnormal state, the throttle value exceeds 1000");
    break;
  case 0x04:
    Serial.println(" UART throttle loss, UART TH loss");
    break;
  case 0x08:
    Serial.println(" CAN throttle loss, CAN TH loss");
    break;
  case 0x10:
    Serial.println(" the battery voltage is not within the regulated range");
    break;

  default:
    Serial.println(" OK");
    break;
  }

  // muestra Mah-used value of the used/consumed power
  int16_t mAu = datain[15] * 256 + datain[16];
  Serial.print("Mah-used ");
  Serial.println(mAu);

  // muestra UART-TH  serial throttle input
  int16_t uarth = datain[17];
  Serial.print("UART-TH ");
  Serial.println(uarth);

  // muestra CAN-TH  can throttle
  int16_t canth = datain[18];
  Serial.print("CAN-TH ");
  Serial.println(canth);

  // muestra BEC voltage  (0-25V)
  int16_t bec = datain[19];
  Serial.print("BEC voltage ");
  Serial.println(bec);
}