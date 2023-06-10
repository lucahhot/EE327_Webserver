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

// ================================================================
// ===                   GESTURE RECOGNITION                    ===
// ================================================================

// For whatever reason, placing #include "gesture_recognition_v1_inferencing.h" in a file that's not main.cpp causes
// "multiple definition of..." errors, making creating seperate .cpp and .h files for the relevant functions very difficult.

#include "gesture_recognition_v1_inferencing.h"

#define b_gesture_GPIO      17
#define LED_gesture_GPIO    13
#define LED_power_GPIO      35
#define FREQUENCY_HZ        60
#define INTERVAL_MS         (1000 / (FREQUENCY_HZ + 1))

Button b_gesture = {b_gesture_GPIO, false};
float features[EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE];
size_t feature_ix = 0;
unsigned long last_interval_ms = 0;
bool gesture_recognized;

extern Adafruit_MPU6050 mpu;

void ARDUINO_ISR_ATTR isr_gesture()
{
    b_gesture.pressed = true;
}

void init_gesture()
{

  pinMode(LED_gesture_GPIO, OUTPUT);
  pinMode(b_gesture_GPIO, INPUT_PULLUP);
  attachInterrupt(b_gesture_GPIO, isr_gesture, FALLING);

  Serial.print("Features: ");
  Serial.println(EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE);
  Serial.print("Label count: ");
  Serial.println(EI_CLASSIFIER_LABEL_COUNT);

}

void gesture_recognition_loop()
{
    if(b_gesture.pressed){
    digitalWrite(LED_gesture_GPIO, HIGH);
    sensors_event_t a, g, temp;

    if (millis() > last_interval_ms + INTERVAL_MS) {
      last_interval_ms = millis();

      mpu.getEvent(&a, &g, &temp);

      features[feature_ix++] = a.acceleration.x;
      features[feature_ix++] = a.acceleration.y;
      features[feature_ix++] = a.acceleration.z;
      features[feature_ix++] = g.gyro.x;
      features[feature_ix++] = g.gyro.y;
      features[feature_ix++] = g.gyro.z;

      if (feature_ix == EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE) {
        Serial.println("Running the inference...");
        signal_t signal;
        ei_impulse_result_t result;
        int err = numpy::signal_from_buffer(features, EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE, &signal);
        if (err != 0) {
          ei_printf("Failed to create signal from buffer (%d)\n", err);
          return;
        }

        EI_IMPULSE_ERROR res = run_classifier(&signal, &result, true);

        if (res != 0) return;

        ei_printf("Predictions ");
        ei_printf("(DSP: %d ms., Classification: %d ms.)",
                  result.timing.dsp, result.timing.classification);
        ei_printf(": \n");

        gesture_recognized = false;
        for (size_t ix = 0; ix < EI_CLASSIFIER_LABEL_COUNT; ix++) {
          ei_printf("    %s: %.5f\n", result.classification[ix].label, result.classification[ix].value);
          if (result.classification[ix].value > 0.6) {
            gesture_recognized = true;
            if (result.classification[ix].label == "left_to_right")
            {
              ei_printf("Hand moved from left to right.\n");
            } else if (result.classification[ix].label == "right_to_left")
            {
              ei_printf("Hand moved from right to left.\n");
            }
          }
        }

        if(!gesture_recognized){
          ei_printf("Unable to identify gesture.\n");
        }

        b_gesture.pressed = false;
        digitalWrite(LED_gesture_GPIO, LOW);

        feature_ix = 0;
      }

    }
  }
}

void ei_printf(const char *format, ...)
{
  static char print_buf[1024] = { 0 };

  va_list args;
  va_start(args, format);
  int r = vsnprintf(print_buf, sizeof(print_buf), format, args);
  va_end(args);

  if (r > 0) {
    Serial.write(print_buf);
  }
}

// ================================================================
// ===                        MAIN CODE                         ===
// ================================================================

void task_wifi_joints(void * parameters){
  while(1){
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

    vTaskDelay(75/portTICK_PERIOD_MS);
  }
}

void task_gesture_recognition(void * parameters){
  while(1){
    gesture_recognition_loop();
    vTaskDelay(5/portTICK_PERIOD_MS);
  }
}

void setup() {

  Serial.begin(115200);

  pinMode(LED_power_GPIO, OUTPUT);
  digitalWrite(LED_power_GPIO, HIGH);

  initMPU();

  initWiFi();
  initSPIFFS();
  initServer();

  init_joints();

  init_gesture();

  xTaskCreatePinnedToCore(
    task_wifi_joints,
    "task_wifi_joints",
    5000,
    NULL,
    2,
    NULL,
    CONFIG_ARDUINO_RUNNING_CORE
  );

  xTaskCreate(
    task_gesture_recognition,
    "task_gesture_recognition",
    5000,
    NULL,
    1,
    NULL
  );

  vTaskDelete(NULL);
}

void loop() {
}