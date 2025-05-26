#include "shared_resources.h"

StateMachine* stateMachine = nullptr;
IMU* imu = nullptr;
Encoder* encoder = nullptr;

//Control variables
float velCommand = 0.0f;
float steerCommand = 0.0f;

// global semaphore definition
SemaphoreHandle_t controlDataMutex = nullptr;
SemaphoreHandle_t i2cMutex = nullptr;
TaskHandle_t ros_subscriber_task_handle = nullptr;
TaskHandle_t ros_sensor_publisher_task_handle = nullptr;
TaskHandle_t ros_state_publisher_task_handle = nullptr;