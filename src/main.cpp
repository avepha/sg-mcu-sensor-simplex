#define SLAVE_ID 1
#define _TASK_SLEEP_ON_IDLE_RUN
#define _TASK_TIMECRITICAL
#define _TASK_PRIORITY
#define VERSION "1.0.9"

#include <Arduino.h>
#include <Wire.h>
#include <SoftwareSerial.h>
#include <TaskScheduler.h>
#include <FastCRC.h>
#include "util/converter.h"
#include "./config/config.h"
#include "./util/packetUtil.h"

#include "./external_lib/k30/K30_I2C.h"
#include "./external_lib/sht1x/SHT1x.h"
#include "par.h"

#define DIR_485_PIN 8
FastCRC16 crc16;
SoftwareSerial outletPort(SG_STATION_RX, SG_STATION_TX);

Scheduler schCom, schMain;
SHT1x airSensor(AIR_SENSOR_DATA_PIN, AIR_SENSOR_CLK_PIN); // air
SHT1x soilSensor(SOIL_SENSOR_DATA_PIN, SOIL_SENSOR_CLK_PIN); // soil
Par parSensor(PAR_PIN);
K30 k30(0x34);

float temperature = 0;
float humidity = 0;
float soilTemperature = 0;
float vpd = 0;
float soil = 0;
float par = 0;
float parAccumulation = 0;
float co2 = 0;


float getVpd(float _temperature, float _humidity) {
  float spv = 610.7 * pow(10, ((7.5 * _temperature) / (237.3 + _temperature)));
  return (1 - (_humidity / 100)) * spv;
}

void fGetTemperature() {
  temperature = airSensor.readTemperatureC();
  vpd = getVpd(temperature, humidity);
}

void fGetHumidity() {
  humidity = airSensor.readHumidity();
  vpd = getVpd(temperature, humidity);
}

void fGetSoilTemperature() {
  soilTemperature = soilSensor.readTemperatureC();
}

void fGetSoil() {
  soil = soilSensor.readHumidity();
}

void fGetPar() {
  par = parSensor.getPar();
  parAccumulation = (parAccumulation < 10e6) ? parAccumulation + parSensor.getPar() : 0;
}

void fGetCO2() {
  int _co2;
  int rc = k30.readCO2(_co2);
  if (rc == 0) {
    co2 = _co2;
  }
}

void fPrintSensor() {
  float sensors[8];
  sensors[0] = temperature;
  sensors[1] = humidity;
  sensors[2] = vpd;
  sensors[3] = soilTemperature;
  sensors[4] = soil;
  sensors[5] = par;
  sensors[6] = parAccumulation;
  sensors[7] = co2;

  Serial.print("[Info] Read sensor:");
  for (int i = 0; i < sizeof(sensors) / sizeof(sensors[0]); i++) {
    Serial.print(" ");
    Serial.print(String((float) sensors[i]));
  }
  Serial.println();
}

Task tPrintSensor(2000L, TASK_FOREVER, &fPrintSensor, &schMain, true);

Task tGetTemperature(2000L, TASK_FOREVER, &fGetTemperature, &schMain, true);
Task tGetHumidity(2050L, TASK_FOREVER, &fGetHumidity, &schMain, true);
Task tGetSoilTemperature(2200L, TASK_FOREVER, &fGetSoilTemperature, &schMain, true);
Task tGetSoil(2100L, TASK_FOREVER, &fGetSoil, &schMain, true);
Task tGetPar(2150L, TASK_FOREVER, &fGetPar, &schMain, true);
Task tGetCO2(2200L, TASK_FOREVER, &fGetCO2, &schMain, true);

void fDispatchSensor() {
  float sensors[8];
  sensors[0] = temperature;
  sensors[1] = humidity;
  sensors[2] = vpd;
  sensors[3] = soilTemperature;
  sensors[4] = soil;
  sensors[5] = co2;
  sensors[6] = par;
  sensors[7] = parAccumulation;

  // response sensors
  byte packets[100];
  byte data[100];
  uint16_t dataIndex = 0;
  for (uint16_t i = 0; i < sizeof(sensors) / sizeof(sensors[0]); i++) {
    memcpy(&data[dataIndex], &sensors[i], sizeof(sensors[i]));
    dataIndex += 4;
  }

  uint16_t packetSize = generatePacket(packets, SLAVE_ID, 0x04, data, sizeof(sensors));
  digitalWrite(SG_STATION_DIR_PIN, RS485_SEND_MODE);
  outletPort.write(packets, packetSize);
  digitalWrite(SG_STATION_DIR_PIN, RS485_RECV_MODE);

  Serial.print("[Info] write data: ");
  printBytes(packets, packetSize);
}

Task dispatchSensor(3000, TASK_FOREVER, &fDispatchSensor, &schCom, true);

void setup() {
  analogReference(EXTERNAL);

  pinMode(SG_STATION_DIR_PIN, OUTPUT);
  pinMode(SG_STATION_RX, INPUT);
  pinMode(SG_STATION_TX, OUTPUT);

  digitalWrite(SG_STATION_DIR_PIN, RS485_RECV_MODE);
  digitalWrite(SG_STATION_TX, HIGH);

  Wire.begin();
  Serial.begin(9600);
  outletPort.begin(9600);

  schMain.setHighPriorityScheduler(&schCom);

  Serial.println("initializing...");
  Serial.println("SG Version: " + SG_VERSION);
  Serial.println("BUILD VERSION: " + String(VERSION));
#ifdef SG_TEST
  Serial.println("OP MODE: TEST");
#endif
}

void loop() {
  schMain.execute();
}
