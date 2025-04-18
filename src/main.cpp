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


#ifdef LED_BUILTIN
#define LED_PIN LED_BUILTIN
#else
#define LED_PIN 13
#endif


void setup() {


  initROS();
  initDisplay();


  // create tasks
  xTaskCreate(ros_executor_task, "ros_executor_task", 4096, NULL, 1, &ros_executor_task_handle);
  xTaskCreate(display_update_task, "display_update_task", 4096, NULL, 1, &display_update_task_handle);
}

void loop() {
  // Empty loop, tasks handle logic
}