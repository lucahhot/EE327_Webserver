#include <arduino.h>
#include <joint_tracking.h>

String Joint_name_list[number_of_joints] = {"index_finger", "middle_finger"}; // variable names of each joint
uint8_t Joint_GPIO_list[number_of_joints] = {32, 33}; // GPIO number measuring the resistance of each joint
Button b_calibrate = {b_calibrate_GPIO, false};

Joint Joint_list[number_of_joints];

void init_joints()
{
  //set the resolution to 12 bits (0-4096)
  analogReadResolution(12);
  
  // initializing each Joint object
  for (uint8_t i = 0; i < number_of_joints; i++){
      Joint temp_joint = {Joint_name_list[i], Joint_GPIO_list[i], 1000, 0};
      Joint_list[i] = temp_joint;
  }

  // setting the pin modes
  pinMode(LED_calibrate_GPIO, OUTPUT);
  pinMode(b_calibrate_GPIO, INPUT_PULLUP);
  attachInterrupt(b_calibrate_GPIO, isr_calibrate_joints, FALLING);
  for (uint8_t i = 0; i < number_of_joints; i++){
      pinMode(Joint_GPIO_list[i], INPUT);
  }

  // setting the initial states
  digitalWrite(LED_calibrate_GPIO, LOW);
}

void ARDUINO_ISR_ATTR isr_calibrate_joints()
{
    b_calibrate.pressed = true;
}

void LED_1flash(uint8_t PIN)
{
    digitalWrite(PIN, LOW);
    delay(500);
    digitalWrite(PIN, HIGH);
    delay(500);
}

void update_Joint_res_opened(uint32_t* data)
{
    for(uint8_t i = 0; i < number_of_joints; i++) {
        Joint_list[i].res_opened = data[i];
    }
}

void update_Joint_res_closed(uint32_t* data)
{
    for(uint8_t i = 0; i < number_of_joints; i++) {
        Joint_list[i].res_closed = data[i];
    }
}

void calibrate_joints()
{
    digitalWrite(LED_calibrate_GPIO, HIGH);
    Serial.printf("Open hand 1/3\n");
    for (uint8_t i = 0; i < 3; i++){
      LED_1flash(LED_calibrate_GPIO);
    }

    delay(2500); //TAKE THE OPEN HAND READING
    uint32_t* open_data = find_res(Joint_GPIO_list, number_of_joints);
    update_Joint_res_opened(open_data);
    delete[] open_data;

    Serial.printf("Close hand 2/3\n");
    for (uint8_t i = 0; i < 2; i++){
      LED_1flash(LED_calibrate_GPIO);
    }
    
    delay(2500); //TAKE THE CLOSED HAND READING
    uint32_t* closed_data = find_res(Joint_GPIO_list, number_of_joints);
    update_Joint_res_closed(closed_data);
    delete[] closed_data;

    LED_1flash(LED_calibrate_GPIO);
    Serial.printf("Relax hand 3/3\n");
    Serial.printf("Calibration complete\n");
    digitalWrite(LED_calibrate_GPIO, LOW);
}

uint32_t* find_res(uint8_t* pins, uint8_t numPins)
{
    
    uint32_t* GPIOValues = new uint32_t[numPins];
    for (uint8_t i = 0; i < numPins; i++) {
      GPIOValues[i] = analogReadMilliVolts(pins[i]);
    }
    
    uint32_t* res_values = new uint32_t[numPins];
  
    for (uint8_t i = 0; i < numPins; i++) {
      uint32_t val = GPIOValues[i];
      uint32_t res_val = known_R*(Vin/(val/1000.0) - 1.0);
      res_values[i] = res_val;
    }

    delete[] GPIOValues;

    return res_values;
}

uint8_t* finger_joint_state(uint32_t* data)
{
    uint8_t* state_list = new uint8_t[number_of_joints];
    
    for(uint8_t i = 0; i < number_of_joints; i++){
        int32_t open_diff = data[i] - Joint_list[i].res_opened;
        int32_t close_diff = data[i] - Joint_list[i].res_closed;
        if(abs(close_diff) <= abs(open_diff)){
            state_list[i] = 0; // Joint is closed
        }
        else{
            state_list[i] = 1; //Joint is opened
        }
    }

    return state_list;
}