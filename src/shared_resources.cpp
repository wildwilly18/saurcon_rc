#include "shared_resources.h"

StateMachine* stateMachine = nullptr;
IMU* imu = nullptr;
Encoder* encoder = nullptr;

//Control variables
float velCommand = 0.0f;
float steerCommand = 0.0f;


HardwareSerial SerialDBug(2);  // Use UART2

// global semaphore definition
SemaphoreHandle_t controlDataMutex = nullptr;
SemaphoreHandle_t i2cMutex = nullptr;
TaskHandle_t ros_ctrl_sub_task_handle = nullptr;
TaskHandle_t ros_state_sub_task_handle = nullptr;
TaskHandle_t ros_sensor_publisher_task_handle = nullptr;
TaskHandle_t ros_state_publisher_task_handle = nullptr;