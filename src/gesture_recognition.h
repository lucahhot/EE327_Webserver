// #ifndef GESTURE_RECOGNITION_H
// #define GESTURE_RECOGNITION_H

// #include <Arduino.h>

// #include <wire.h>
// #include <Adafruit_MPU6050.h>
// #include <Adafruit_Sensor.h>
// #include <joint_tracking.h>
// #include "gesture_recognition_v1_inferencing.h"

// #define b_gesture_GPIO      17
// #define LED_gesture_GPIO    5
// #define FREQUENCY_HZ        60
// #define INTERVAL_MS         (1000 / (FREQUENCY_HZ + 1))

// extern Button b_gesture;
// extern float features[EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE];
// extern size_t feature_ix;
// extern unsigned long last_interval_ms;
// extern bool gesture_recognized;

// void init_gesture();
// void ARDUINO_ISR_ATTR isr_gesture();
// void gesture_recognition_loop();
// void ei_printf(const char *format, ...);

// #endif