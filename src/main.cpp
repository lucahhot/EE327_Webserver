/*********
  Rui Santos
  Complete project details at https://RandomNerdTutorials.com/esp32-mpu-6050-web-server/
  
  Permission is hereby granted, free of charge, to any person obtaining a copy
  of this software and associated documentation files.
  
  The above copyright notice and this permission notice shall be included in all
  copies or substantial portions of the Software.
*********/

#include <Arduino.h>

#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <Arduino_JSON.h>
#include <esp_adc_cal.h>
#include "SPIFFS.h"
#include <wifi_grip.h>

#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <joint_tracking.h>

#include <gesture_recognition.h>



void setup() {
  Serial.begin(115200);

  initMPU();

  initWiFi();
  initSPIFFS();
  initServer();

  init_joints();

  //init_gesture();
}

void loop() {

  send_mpu_readings();

  uint32_t* calculated_data = find_res(Joint_GPIO_list, number_of_joints);
  uint8_t* joint_states = finger_joint_state(calculated_data);

  //print_list<String[number_of_joints]>("Joint names:", Joint_name_list, number_of_joints);
  //print_list<uint8_t[number_of_joints]>("Joint states:", joint_states, number_of_joints);

  send_joint_readings(joint_states);

  delete[] calculated_data;
  delete[] joint_states;

  if (b_calibrate.pressed) {
      Serial.printf("Calibration Initiated\n");
      calibrate_joints();
      b_calibrate.pressed = false;
  }

  //gesture_recognition_loop();

  delay(10);
}