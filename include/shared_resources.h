#pragma once

#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>

#include "imu.h"

class StateMachine;

extern StateMachine* stateMachine;
extern IMU* imu;

// Shared control variables (protected by controlDataMutex)
extern float velCommand;
extern float steerCommand;

// Shared encoder data (protected by encoderDataMutex)
extern float filteredRPM;

extern SemaphoreHandle_t encoderDataMutex;
extern SemaphoreHandle_t controlDataMutex;
extern SemaphoreHandle_t i2cMutex;
extern TaskHandle_t ros_subscriber_task_handle;
extern TaskHandle_t ros_sensor_publisher_task_handle;
extern TaskHandle_t ros_state_publisher_task_handle;


