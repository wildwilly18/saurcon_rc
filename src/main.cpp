//---
#include <Arduino.h>
#include <U8g2lib.h>

//Include FreeRTOS
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

// Pin Definitions for ESP32
#include "pins.h"

//App Interfaces
#include "ros_interface.h"
#include "display.h"
#include "motor_control.h"
#include "encoder.h"
#include "state_machine.h"


#ifdef LED_BUILTIN
#define LED_PIN LED_BUILTIN
#else
#define LED_PIN 13
#endif


void setup() {
  pinMode(LED_RED, OUTPUT);
  digitalWrite(LED_RED, HIGH);

  pinMode(LED_GRN, OUTPUT);
  digitalWrite(LED_GRN, HIGH);

  // Begin Init Functions these to move into state machine
  init_display();
  xTaskCreate(display_update_task, "display_update_task", 4096, NULL, 1, &display_update_task_handle);

  init_ROS();
  init_pwm();
  //init_servo();
  
  //init_encoder_mutex();
  //init_encoder_isr();

  // create tasks
  //xTaskCreate(state_machine_task, "state_machine_task", 2048, NULL, 1, NULL);
  
  xTaskCreate(ros_executor_task,   "ros_executor_task",   4096, NULL, 1, &ros_executor_task_handle);
  xTaskCreate(task_motion_control, "task_motion_control", 2048, NULL, 2, NULL);
  xTaskCreatePinnedToCore(rpm_filter_task, "rpm_filter_task", 4096, NULL, 1, NULL, 1);
}

void loop() {
  // Empty loop, tasks handle logic
}