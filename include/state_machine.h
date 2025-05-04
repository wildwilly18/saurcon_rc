#pragma once

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

#include "types/saurcon_faults.h"
#include "types/saurcon_states.h"

#include "display.h"
#include "motor_control.h"
#include "ros_interface.h"
#include "encoder.h"

extern SaurconState saurcon_state;

extern SemaphoreHandle_t stateMutex;

// Public API
void state_machine_task(void *pvParameters);

void StateMachine_SetState(SaurconState nextState);
void StateMachine_SetFault(SaurconFaults fault);