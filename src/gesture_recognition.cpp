// #include <Arduino.h>
// #include <gesture_recognition.h>

// Button b_gesture = {b_gesture_GPIO, false};
// float features[EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE];
// size_t feature_ix = 0;
// unsigned long last_interval_ms = 0;
// bool gesture_recognized;

// extern Adafruit_MPU6050 mpu;

// void init_gesture()
// {
//   pinMode(LED_gesture_GPIO, OUTPUT);
//   pinMode(b_gesture_GPIO, INPUT_PULLUP);
//   attachInterrupt(b_gesture_GPIO, isr_gesture, FALLING);

//   Serial.print("Features: ");
//   Serial.println(EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE);
//   Serial.print("Label count: ");
//   Serial.println(EI_CLASSIFIER_LABEL_COUNT);

// }

// void ARDUINO_ISR_ATTR isr_gesture()
// {
//     b_gesture.pressed = true;
// }

// void gesture_recognition_loop()
// {
//     if(b_gesture.pressed){
//     digitalWrite(LED_gesture_GPIO, HIGH);
//     sensors_event_t a, g, temp;

//     if (millis() > last_interval_ms + INTERVAL_MS) {
//       last_interval_ms = millis();

//       mpu.getEvent(&a, &g, &temp);

//       features[feature_ix++] = a.acceleration.x;
//       features[feature_ix++] = a.acceleration.y;
//       features[feature_ix++] = a.acceleration.z;
//       features[feature_ix++] = g.gyro.x;
//       features[feature_ix++] = g.gyro.y;
//       features[feature_ix++] = g.gyro.z;

//       if (feature_ix == EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE) {
//         Serial.println("Running the inference...");
//         signal_t signal;
//         ei_impulse_result_t result;
//         int err = numpy::signal_from_buffer(features, EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE, &signal);
//         if (err != 0) {
//           ei_printf("Failed to create signal from buffer (%d)\n", err);
//           return;
//         }

//         EI_IMPULSE_ERROR res = run_classifier(&signal, &result, true);

//         if (res != 0) return;

//         ei_printf("Predictions ");
//         ei_printf("(DSP: %d ms., Classification: %d ms.)",
//                   result.timing.dsp, result.timing.classification);
//         ei_printf(": \n");

//         gesture_recognized = false;
//         for (size_t ix = 0; ix < EI_CLASSIFIER_LABEL_COUNT; ix++) {
//           ei_printf("    %s: %.5f\n", result.classification[ix].label, result.classification[ix].value);
//           if (result.classification[ix].value > 0.6) {
//             gesture_recognized = true;
//             if (result.classification[ix].label == "left_to_right")
//             {
//               ei_printf("Hand moved from left to right.\n");
//             } else if (result.classification[ix].label == "right_to_left")
//             {
//               ei_printf("Hand moved from right to left.\n");
//             }
//           }
//         }

//         if(!gesture_recognized){
//           ei_printf("Unable to identify gesture.\n");
//         }

//         b_gesture.pressed = false;
//         digitalWrite(LED_gesture_GPIO, LOW);

//         feature_ix = 0;
//       }

//     }
//   }
// }

// void ei_printf(const char *format, ...)
// {
//   static char print_buf[1024] = { 0 };

//   va_list args;
//   va_start(args, format);
//   int r = vsnprintf(print_buf, sizeof(print_buf), format, args);
//   va_end(args);

//   if (r > 0) {
//     Serial.write(print_buf);
//   }
// }