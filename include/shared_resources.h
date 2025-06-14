#pragma once

#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>

#include "imu.h"
#include "encoder.h"

class StateMachine;

extern StateMachine* stateMachine;
extern IMU* imu;
extern Encoder* encoder;

// Shared control variables (protected by controlDataMutex)
extern float velCommand;
extern float steerCommand;

extern SemaphoreHandle_t controlDataMutex;
extern SemaphoreHandle_t i2cMutex;
extern TaskHandle_t ros_control_subscriber_task_handle;
extern TaskHandle_t ros_state_subscriber_task_handle;
extern TaskHandle_t ros_sensor_publisher_task_handle;
extern TaskHandle_t ros_state_publisher_task_handle;


