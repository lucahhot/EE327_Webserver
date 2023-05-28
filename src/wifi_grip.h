#ifndef WIFI_GRIP_H
#define WIFI_GRIP_H

#include <Arduino.h>

#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <Arduino_JSON.h>
#include <esp_adc_cal.h>
#include "SPIFFS.h"

#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

#include <joint_tracking.h>


extern const char* ssid;
extern const char* password;

extern unsigned long lastTime;  
extern unsigned long lastTimeTemperature;
extern unsigned long lastTimeAcc;
extern unsigned long lastFingerButton;
extern unsigned long fingerDelay;
extern unsigned long gyroDelay;
extern unsigned long temperatureDelay;
extern unsigned long accelerometerDelay;

extern float gyroX, gyroY, gyroZ;
extern float accX, accY, accZ;
extern float temperature;

extern float gyroXerror;
extern float gyroYerror;
extern float gyroZerror;

void initMPU();
void initSPIFFS();
void initWiFi();
void initServer();

String getGyroReadings();
String getAccReadings();
String getTemperature();

void send_mpu_readings();
void send_joint_readings(uint8_t* states);

#endif