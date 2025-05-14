#pragma once

#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>

class StateMachine;

extern StateMachine* stateMachine;

// Shared control variables (protected by controlDataMutex)
extern float velCommand;
extern float steerCommand;

// Shared encoder data (protected by encoderDataMutex)
extern float filteredRPM;

extern SemaphoreHandle_t encoderDataMutex;
extern SemaphoreHandle_t controlDataMutex;
extern TaskHandle_t ros_executor_task_handle;


