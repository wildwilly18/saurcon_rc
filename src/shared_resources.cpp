#include "shared_resources.h"

StateMachine* stateMachine = nullptr;
IMU* imu = nullptr;

//Control variables
float velCommand = 0.0f;
float steerCommand = 0.0f;
float filteredRPM = 0.0f;

// global semaphore definition
SemaphoreHandle_t encoderDataMutex = nullptr;
SemaphoreHandle_t controlDataMutex = nullptr;
SemaphoreHandle_t i2cMutex = nullptr;
TaskHandle_t ros_executor_task_handle = nullptr;