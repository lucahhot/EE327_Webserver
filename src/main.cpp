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
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Arduino_JSON.h>
#include <esp_adc_cal.h>
#include "SPIFFS.h"

// Replace with your network credentials
const char* ssid = "luc";
const char* password = "lucahhot0106";

// Create AsyncWebServer object on port 80
AsyncWebServer server(80);

// Create an Event Source on /events
AsyncEventSource events("/events");

// Json Variable to Hold Sensor Readings
JSONVar readings;

// Timer variables
unsigned long lastTime = 0;  
unsigned long lastTimeTemperature = 0;
unsigned long lastTimeAcc = 0;
unsigned long lastFingerButton = 0;
unsigned long fingerDelay = 10;
unsigned long gyroDelay = 10;
unsigned long temperatureDelay = 1000;
unsigned long accelerometerDelay = 200;

// Create a sensor object
Adafruit_MPU6050 mpu;

sensors_event_t a, g, temp;

float gyroX, gyroY, gyroZ;
float accX, accY, accZ;
float temperature;

//Gyroscope sensor deviation
float gyroXerror = 0.07;
float gyroYerror = 0.03;
float gyroZerror = 0.01;

// Initializing some button stuff to allow for model switching based on button change
const int buttonPin = 36;    // Third pin from the top left
int buttonState;             // the current reading from the input pin

// Init MPU6050
void initMPU(){
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }
  Serial.println("MPU6050 Found!");
}

void initSPIFFS() {
  if (!SPIFFS.begin()) {
    Serial.println("An error has occurred while mounting SPIFFS");
  }
  Serial.println("SPIFFS mounted successfully");
}

// Initialize WiFi
void initWiFi() {
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  Serial.println("");
  Serial.print("Connecting to WiFi...");
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    delay(1000);
  }
  Serial.println("");
  Serial.println(WiFi.localIP());
}

String getGyroReadings(){
  mpu.getEvent(&a, &g, &temp);

  float gyroX_temp = g.gyro.x;
  if(abs(gyroX_temp) > gyroXerror)  {
    gyroX += gyroX_temp/50.00;
  }
  
  float gyroY_temp = g.gyro.y;
  if(abs(gyroY_temp) > gyroYerror) {
    gyroY += gyroY_temp/70.00;
  }

  float gyroZ_temp = g.gyro.z;
  if(abs(gyroZ_temp) > gyroZerror) {
    gyroZ += gyroZ_temp/90.00;
  }

  readings["gyroX"] = String(gyroX);
  readings["gyroY"] = String(gyroY);
  readings["gyroZ"] = String(gyroZ);

  String jsonString = JSON.stringify(readings);
  return jsonString;
}

String getAccReadings() {
  mpu.getEvent(&a, &g, &temp);
  // Get current acceleration values
  accX = a.acceleration.x;
  accY = a.acceleration.y;
  accZ = a.acceleration.z;
  readings["accX"] = String(accX);
  readings["accY"] = String(accY);
  readings["accZ"] = String(accZ);
  String accString = JSON.stringify (readings);
  return accString;
}

String getTemperature(){
  mpu.getEvent(&a, &g, &temp);
  temperature = temp.temperature;
  return String(temperature);
}

//
// OPEN TO CHANGE FOR USER
//

#define number_of_joints      2 // number of joints being measured
String Joint_name_list[number_of_joints] = {"index_finger", "middle_finger"}; // variable names of each joint
uint8_t Joint_GPIO_list[number_of_joints] = {32, 33}; // GPIO number measuring the resistance of each joint

//
// STRUCTS
//

struct Joint {
    String name;
    uint8_t GPIO;
    uint32_t res_opened;
    uint32_t res_closed;
};

struct Button {
    const uint8_t GPIO;
    bool pressed;
};

//
// INITIALIZATION
//

Joint Joint_list[number_of_joints];

//
// PIN NUMBER DECLARATION
//

#define LED_calibrate_GPIO     19
#define b_calibrate_GPIO       18

//
// RAW DATA / adcPIN READING
//

/*
uint8_t* readGPIOs(uint8_t* pins, uint8_t numPins) {
  uint8_t* GPIOValues = new uint8_t[numPins];
  for (int i = 0; i < numPins; i++) {
    GPIOValues[i] = analogReadMilliVolts(pins[i]);
  }
  return GPIOValues;
}
*/

//
// CALIBRATE SETUP
//

Button b_calibrate = {b_calibrate_GPIO, false};

void ARDUINO_ISR_ATTR isr() {
    b_calibrate.pressed = true;
}

void LED_1flash(uint8_t PIN){
    digitalWrite(PIN, LOW);
    delay(500);
    digitalWrite(PIN, HIGH);
    delay(500);
}

void update_Joint_res_opened(uint32_t* data) {
    for(uint8_t i = 0; i < number_of_joints; i++) {
        Joint_list[i].res_opened = data[i];
    }
}

void update_Joint_res_closed(uint32_t* data) {
    for(uint8_t i = 0; i < number_of_joints; i++) {
        Joint_list[i].res_closed = data[i];
    }
}

//
// RESISTANCE READING SETUP
//

float Vin = 3.3;
float known_R = 2200.0;

uint32_t* find_res(uint8_t* pins, uint8_t numPins){
    
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

void calibrate_joints(){
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

//
// HAND_STATE SETUP
//
template <typename T>
void print_list(String label, T list, uint8_t numPins) {
    Serial.println(label);
    for (int i = 0; i < numPins; i++) {
      Serial.print(list[i]);
      if(i != (numPins - 1)){
        Serial.print(", ");
      }
    }
    Serial.println("");
}

uint8_t* finger_joint_state(uint32_t* data){
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

void init_joints(){
  // initializing each Joint object
  for (uint8_t i = 0; i < number_of_joints; i++){
      Joint temp_joint = {Joint_name_list[i], Joint_GPIO_list[i], 1000, 0};
      Joint_list[i] = temp_joint;
  }

  // setting the pin modes
  pinMode(LED_calibrate_GPIO, OUTPUT);
  pinMode(b_calibrate_GPIO, INPUT_PULLUP);
  attachInterrupt(b_calibrate_GPIO, isr, FALLING);
  for (uint8_t i = 0; i < number_of_joints; i++){
      pinMode(Joint_GPIO_list[i], INPUT);
  }

  // setting the initial states
  digitalWrite(LED_calibrate_GPIO, LOW);
}

void setup() {
  Serial.begin(115200);
  initWiFi();
  initSPIFFS();
  initMPU();

  //set the resolution to 12 bits (0-4096)
  analogReadResolution(12);
  init_joints();

  // Handle Web Server
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send(SPIFFS, "/index.html", "text/html");
  });

  server.serveStatic("/", SPIFFS, "/");

  server.on("/reset", HTTP_GET, [](AsyncWebServerRequest *request){
    gyroX=0;
    gyroY=0;
    gyroZ=0;
    request->send(200, "text/plain", "OK");
  });

  server.on("/resetX", HTTP_GET, [](AsyncWebServerRequest *request){
    gyroX=0;
    request->send(200, "text/plain", "OK");
  });

  server.on("/resetY", HTTP_GET, [](AsyncWebServerRequest *request){
    gyroY=0;
    request->send(200, "text/plain", "OK");
  });

  server.on("/resetZ", HTTP_GET, [](AsyncWebServerRequest *request){
    gyroZ=0;
    request->send(200, "text/plain", "OK");
  });

  // Handle Web Server Events
  events.onConnect([](AsyncEventSourceClient *client){
    if(client->lastId()){
      Serial.printf("Client reconnected! Last message ID that it got is: %u\n", client->lastId());
    }
    // send event with message "hello!", id current millis
    // and set reconnect delay to 1 second
    client->send("hello!", NULL, millis(), 10000);
  });
  server.addHandler(&events);

  server.begin();
}

void loop() {
  if ((millis() - lastTime) > gyroDelay) {
    // Send Events to the Web Server with the Sensor Readings
    events.send(getGyroReadings().c_str(),"gyro_readings",millis());
    lastTime = millis();
  }
  if ((millis() - lastTimeAcc) > accelerometerDelay) {
    // Send Events to the Web Server with the Sensor Readings
    events.send(getAccReadings().c_str(),"accelerometer_readings",millis());
    lastTimeAcc = millis();
  }
  if ((millis() - lastTimeTemperature) > temperatureDelay) {
    // Send Events to the Web Server with the Sensor Readings
    events.send(getTemperature().c_str(),"temperature_reading",millis());
    lastTimeTemperature = millis();
  }

  uint32_t* calculated_data = find_res(Joint_GPIO_list, number_of_joints);
  uint8_t* joint_states = finger_joint_state(calculated_data);
  delay(50);

  //print_list<String[number_of_joints]>("Joint names:", Joint_name_list, number_of_joints);
  // print_list<uint32_t[number_of_joints]>("Resistance values:", calculated_data, number_of_joints);
  // print_list<uint8_t[number_of_joints]>("Joint states:", joint_states, number_of_joints);

  if ((millis() - lastFingerButton) > fingerDelay) {
  // Send Events to the Web Server with the Button Reading
  events.send(String(joint_states[0]).c_str(),"index_reading",millis());
  events.send(String(joint_states[1]).c_str(),"middle_reading",millis());
  lastFingerButton = millis();
  }

  delete[] calculated_data;
  delete[] joint_states;

  if (b_calibrate.pressed) {
      Serial.printf("Calibration Initiated\n");
      calibrate_joints();
      b_calibrate.pressed = false;
  }
}

// /*********
//   Rui Santos
//   Complete project details at https://randomnerdtutorials.com  
// *********/

// #include <Wire.h>
// #include <SPI.h>
// #include <Arduino.h>
 
// void setup() {
//   Wire.begin();
//   Serial.begin(115200);
//   Serial.println("\nI2C Scanner");
// }
 
// void loop() {
//   byte error, address;
//   int nDevices;
//   Serial.println("Scanning...");
//   nDevices = 0;
//   for(address = 1; address < 127; address++ ) {
//     Wire.beginTransmission(address);
//     error = Wire.endTransmission();
//     if (error == 0) {
//       Serial.print("I2C device found at address 0x");
//       if (address<16) {
//         Serial.print("0");
//       }
//       Serial.println(address,HEX);
//       nDevices++;
//     }
//     else if (error==4) {
//       Serial.print("Unknow error at address 0x");
//       if (address<16) {
//         Serial.print("0");
//       }
//       Serial.println(address,HEX);
//     }    
//   }
//   if (nDevices == 0) {
//     Serial.println("No I2C devices found\n");
//   }
//   else {
//     Serial.println("done\n");
//   }
//   delay(5000);          
// }
