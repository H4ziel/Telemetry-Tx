/*==================================================================================================================================================================

Telemetry System - v.01 LoraConfig.h

Author: Joao Ricardo Chaves and Marcelo Haziel
Date: 2024, March.
====================================================================================================================================================================*/

#ifndef LoRa_ConfigTx_
#define LoRa_ConfigTx_

//==========================================================================
//---Bibliotecas---
#include <heltec.h>               //version: 1.1.5
#include <BMP180I2C.h>            //version: 1.0.1
//#include <MPU6050_tockn.h>        //version: 1.5.2
#include <TinyGPSPlus.h>          //version: 0.0.4
#include <SoftwareSerial.h>       

//==========================================================================
//---Variaveis Globais---

//==========================================================================
// Variáveis dos dados do sensor mpu6050
float accxTx, accyTx, acczTx, 
      gyroxTx, gyroyTx, gyrozTx, 
      AngleRoll, AnglePitch, RateRoll,
      RatePitch, RateYaw;

//==========================================================================
// Variáveis dos dados do sensor bmp180
float tempTx, pressure;

//==========================================================================
// Variável que armazena o pacote LoRa
String DadosTx;

//==========================================================================
// Variáveis dos dados do gps NEO 6M
double Lat,Lng,Speed,Alt;
int Sat;

//==========================================================================
//---PIN MCU---

//==========================================================================
//---pinos do display da placa (Comunicação i2c)---
const int DISPLAY_ADDRESS_PIN = 0x3c;
#define DISPLAY_SDA_PIN 4
#define DISPLAY_SCL_PIN 15
#define DISPLAY_RST_PIN 16

//==========================================================================
//---pinos do chip LoRa SX1276 (comunicação spi)---
#define LORA_SCK_PIN 5
#define LORA_MISO_PIN 19
#define LORA_MOSI_PIN 27
#define LORA_SS_PIN 18
#define LORA_RST_PIN 15
#define LORA_DI00_PIN 26

//==========================================================================
//---definição do rádio LoRa---
#define DisplayEstado false //define se o display estará ativo ou não ativo
#define LoRaEstado true //
#define SerialEstado 9600// Baud rate padrão de 1152000
#define AmplificadorDePotencia true //define se o amplificador de potência PABOOST estará ativo ou não 
#define BAND 915E6

//==========================================================================
//---definição pinos SCL e SDA - MODULO GY-87---
#define SDA_PIN 4
#define SCL_PIN 15  

//==========================================================================
//---Acelerômetro e Giroscópio---
//MPU6050 mpu6050(Wire);

//==========================================================================
//---Termômetro e Barômetro---
#define BMP180ADRESS_I2C 0x77
BMP180I2C bmp180(BMP180ADRESS_I2C);

//==========================================================================
//---Gps module definition ---
#define RxPin    23
#define TxPin    17
#define GPSBaud  9600
#define timer    100

TinyGPSPlus gps;

#endif
// ============================================================================
// --- End of Program ---