/*==================================================================================================================================================================

Telemetry System - v.01 --- LoRaTx.ino ---
MODULE: WiFi LoRa 32 (V2)
MCU: ESP32
CHIP: LoRa SX1278
Frequency: 863~928 Hz

Author: Joao Ricardo Chaves and Marcelo Haziel
Date: 2024, March.
====================================================================================================================================================================*/

//=============================================================================================================================
//---Bibliotecas---
#include "LoraConfigTx.h"

//==========================================================================
//--Task Prototipos--
void ReadSensors(void *p);
void SendLoraData(void *p);

//=============================================================================================================================
//---main function---

void setup() 
{
  Serial.begin(9600);                                                                   // Inicialização Serial
  Heltec.begin(DisplayEstado, LoRaEstado, SerialEstado, AmplificadorDePotencia, BAND);    // Inicialização Heltec Lora 
  SetupLoRa(); 
  Wire.begin(SDA_PIN, SCL_PIN);                                                           // Inicialização pin I2C
  Serial2.begin(GPSBaud, SERIAL_8N1,RxPin,TxPin);                                         // Inicialização modulo gps
  //Iniciando comunicação com o mpu6050
  Wire.setClock(400000);
  Wire.begin();
  delay(250);
  Wire.beginTransmission(0x68);
  Wire.write(0x6B);
  Wire.write(0x00);
  Wire.endTransmission();
  //mpu6050.begin();                                                                        // Inicialização Giroscopio e Acelerometro
  //mpu6050.calcGyroOffsets(true);
  if (!bmp180.begin())                                                                    // Inicialização Termometro e barometro
  {
    Serial.println("bmp falhou!");
    while(1);
  }//end if
  bmp180.resetToDefaults();
  bmp180.setSamplingMode(BMP180MI::MODE_UHR);                                             // Setando modo ultra high resolution
  xTaskCreate(ReadSensors,"ReadSensors",7000,NULL,3,NULL);                                // Inicialização ReadSensors, utiliza o nucleo 1 como padrão 
  xTaskCreatePinnedToCore(SendLoraData,"SendLoraData",7000,NULL,2,NULL,0);
  delay(500);
  disableCore0WDT();                                                                      // Desabilita o Watchdog Timer do nucleo 0
  disableCore1WDT();                                                                      // Desabilita o Watchdog Timer do nucleo 1
}//end setup

void loop() 
{
  //==========================
  //---
}//end loop

//==========================================================================
//--Functions/Tasks--

//=============================================================================================================================
//---SetupLoRa---
void SetupLoRa()
{
  SPI.begin(LORA_SCK_PIN, LORA_MISO_PIN, LORA_MOSI_PIN, LORA_SS_PIN);
  LoRa.setPins(LORA_SS_PIN, LORA_RST_PIN, LORA_DI00_PIN);

  if (!LoRa.begin(BAND, AmplificadorDePotencia))
  {
    Serial.println("Possível Erro!!");
  }//end if 

  LoRa.setTxPower(20,RF_PACONFIG_PASELECT_PABOOST);
  LoRa.setSpreadingFactor(10); //define o SF
  LoRa.setSignalBandwidth(250E3); //define a BW
  LoRa.setCodingRate4(5); //define o CR
  LoRa.setPreambleLength(6); // define o comprimento do Preambulo
  LoRa.setSyncWord(0x12); // define a palavra de sincronização 0b00010010
  LoRa.crc(); // ativa a Checagem de Redundância Cilíndrica
}//end SetupLora

//=============================================================================================================================
//---ReadSensors---

void ReadSensors(void *p)
{
  while(1)
  {
    //MPU6050
    Wire.beginTransmission(0x68);
    Wire.write(0x05);
    Wire.endTransmission();
    // Transmissão para configurar a saída do acelerometro
    Wire.beginTransmission(0x68); 
    Wire.write(0x1C);
    Wire.write(0x10);
    Wire.endTransmission();
    // medidas do acelerometro provenientes do sensor
    Wire.beginTransmission(0x68);
    Wire.write(0x3B);
    Wire.endTransmission();
    Wire.requestFrom(0x68,6);
    int16_t AccXLSB = Wire.read() << 8 |
      Wire.read();
    int16_t AccYLSB = Wire.read() << 8 |
      Wire.read();
    int16_t AccZLSB = Wire.read() << 8 |
      Wire.read();

    Wire.beginTransmission(0x68);
    Wire.write(0x1B);
    Wire.write(0x8);
    Wire.endTransmission();
    Wire.beginTransmission(0x68);
    Wire.write(0x43);
    Wire.endTransmission();

    Wire.requestFrom(0x68,6);
    int16_t gyroxTx = Wire.read()<<8 | Wire.read();
    int16_t gyroyTx = Wire.read()<<8 | Wire.read();
    int16_t gyrozTx = Wire.read()<<8 | Wire.read();
    RateRoll = (float)gyroxTx/65.5;
    RatePitch = (float)gyroyTx/65.5;
    RateYaw = (float)gyrozTx/65.5;

    accxTx = (float)AccXLSB/4096;
    accyTx = (float)AccYLSB/4096 + 0.02;  // esses valores de 0.02 e 0.06 são os offsets presentes no eixo y e z
    acczTx = (float)AccZLSB/4096 + 0.06;

    AngleRoll = atan(accyTx/sqrt(accxTx*accxTx+acczTx*acczTx))*1/(3.1415/180) + 2.60;
    AnglePitch = atan(accxTx/sqrt(accyTx*accyTx+acczTx*acczTx))*1/(3.1415/180) - 4;

    //BMP180
    if (!bmp180.measureTemperature())
    {
      Serial.println("Error Temperature!");
    }//end if
    do
    {
      vTaskDelay(100 / portTICK_PERIOD_MS);
    } while (!bmp180.hasValue()); //end do while aninhado
    tempTx = bmp180.getTemperature();
    do
    { 
      vTaskDelay(100 / portTICK_PERIOD_MS);
    } while (!bmp180.hasValue());
    pressure = bmp180.getPressure();

    //GPS
    bool newdata = false;
    for(unsigned long start = millis();millis()-start<1000;)
    {
    while(Serial2.available()>0)
    {
      if(gps.encode(Serial2.read()))
      {
        newdata = true;
        Serial.println("Serial2 avaible.");
      }//end if
   }//end while 
  }//end for

  if(newdata)
  {
    newdata = false;
    Serial.println("Entrou em newdata.");
    if(gps.location.isValid())
    {
      Serial.println("Location valid.");
      Lat   =  gps.location.lat();
      Lng   =  gps.location.lng();
      Speed =  gps.speed.kmph();
      Sat   =  gps.satellites.value();
      Alt   =  gps.altitude.meters();
    }//end if
  }//end if
  else
  {
    Serial.println("No Data.");
  }//end else
  vTaskDelay(100 / portTICK_PERIOD_MS);
  }//end while
}//end ReadSensors

//=============================================================================================================================
//---SendLoraData---
void SendLoraData(void *p)
{
  while(1)
  {
    //envia para serial os dados que serão enviado pelo LoRa
    DadosTx = String(tempTx) + "#" + String(AngleRoll) + "@" + String(AnglePitch) + "%" + String(Alt) + "&" + String(Speed) +
                             + "D" + String(pressure) + "E" + String(Sat) + "A" + String(Lat) + "B" + String(Lng); // o # serve apenas para identificar até onde vai a primeira leitura sensorial.
    //Enviar pacote
    //inicia a montagem do pacote(parâmetro FASE desativa o Header)
    LoRa.beginPacket();

    //enviando os estados ou dados dos sensores de um a um
    LoRa.print(DadosTx);

    LoRa.endPacket(); // finaliza o pacote

    Serial.println("Pacote LoRa enviado.");

    vTaskDelay(1000 / portTICK_PERIOD_MS);
  }//end while
}//end SendLoraData

//=============================================================================================================================
//---End of Program---