#pragma once

//Include FreeRTOS
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

// Arduino and Display Library Dependencies
#include <Arduino.h>
#include <U8g2lib.h>
#include "pins.h"
#include "ros_interface.h"

extern TaskHandle_t display_update_task_handle;

void init_display();
void display_update_task(void *pvParameters);