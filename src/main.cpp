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


void state_machine_task(void *param){
  stateMachine = new StateMachine(); 
  while (true){stateMachine -> run();}
}

void setup() {
  xTaskCreate(state_machine_task, "state_machine_task", 8192, NULL, 1, NULL);
}

void loop() {
  // Empty loop, tasks handle logic
}