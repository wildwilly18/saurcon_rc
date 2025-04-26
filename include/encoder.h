#pragma once

//Include FreeRTOS
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

// Arduino and Display Library Dependencies
#include <Arduino.h>
#include <U8g2lib.h>
#include "pins.h"

// Include parameters
#include "types/saurcon_parameters.h"

extern const int8_t enc_transition_table[8][8];

void IRAM_ATTR enc_isr();

void enc_monitor_task();
void init_encoder_isr();
void rpm_filter_task(void *pvParameters);
