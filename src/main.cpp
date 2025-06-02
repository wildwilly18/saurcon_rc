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
#include "shared_resources.h"

void state_machine_task(void *param){
  stateMachine = new StateMachine(); 
  while (true){stateMachine -> run();}
}

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);

  xTaskCreatePinnedToCore(state_machine_task, "state_machine_task", 8192, NULL, 2, NULL, 1);
}

void loop() {
  // Empty loop, tasks handle logic
}