#ifndef JOINT_TRACKING_H
#define JOINT_TRACKING_H

#include <arduino.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

#define LED_calibrate_GPIO      19
#define b_calibrate_GPIO        18
#define number_of_joints        2 // number of joints being measured
#define Vin                     3.3
#define known_R                 2200.0

struct Joint {
    String name;
    uint8_t GPIO;
    uint32_t res_opened;
    uint32_t res_closed;
};

struct Button {
    uint8_t GPIO;
    bool pressed;
};

extern String Joint_name_list[number_of_joints]; // variable names of each joint
extern uint8_t Joint_GPIO_list[number_of_joints]; // GPIO number measuring the resistance of each joint
extern Button b_calibrate;

extern Joint Joint_list[number_of_joints];

void init_joints();
void ARDUINO_ISR_ATTR isr_calibrate_joints();
void LED_1flash(uint8_t PIN);

void update_Joint_res_opened(uint32_t* data);
void update_Joint_res_closed(uint32_t* data);
void calibrate_joints();

uint32_t* find_res(uint8_t* pins, uint8_t numPins);
uint8_t* finger_joint_state(uint32_t* data);

template <typename T> 
void print_list(String label, T list, uint8_t numPins)
{
    Serial.println(label);
    for (int i = 0; i < numPins; i++) {
      Serial.print(list[i]);
      if(i != (numPins - 1)){
        Serial.print(", ");
      }
    }
    Serial.println("");
}

#endif