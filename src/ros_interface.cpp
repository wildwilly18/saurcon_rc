#include "ros_interface.h"

SemaphoreHandle_t controlDataMutex;

// Define the task handle
TaskHandle_t ros_executor_task_handle = NULL;

//Setup a time handle
TimerHandle_t watchdog_ros_timer;

//Micro-Ros objects
rcl_subscription_t subscriber;
geometry_msgs__msg__Twist msg;
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t ros_allocator;
rcl_node_t node;
rcl_timer_t timer;

ControlCommand cmd;

static volatile uint32_t last_msg_tick = 0;
#define TIMEOUT_ROS_MS 120

float velCommand = 0.0;
float steerCommand = 0.0;

void init_ROS(){

    //Setup control data mutex
    controlDataMutex = xSemaphoreCreateMutex();
    if(controlDataMutex == NULL){
        digitalWrite(LED_PIN, LOW);
        error_loop();
    }

    Serial.begin(921600);
    Serial.setRxBufferSize(1024);

    set_microros_serial_transports(Serial);
    
    delay(2000);

    ros_allocator = rcl_get_default_allocator();

    //create init_options
    RCCHECK(rclc_support_init(&support, 0, NULL, &ros_allocator));

    // create node
    RCCHECK(rclc_node_init_default(&node, "saurcon_rc", "", &support));

    // create subscriber with best-effort Qos
    RCCHECK(rclc_subscription_init_best_effort(
        &subscriber,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
        "ctrl_output"));

    // create executor
    RCCHECK(rclc_executor_init(&executor, &support.context, 1, &ros_allocator));
    RCCHECK(rclc_executor_add_subscription(&executor, &subscriber, &msg, &subscription_callback, ON_NEW_DATA));

    //setup_watchdog_ros_timer();
}

void setup_watchdog_ros_timer(){
  watchdog_ros_timer = xTimerCreate(
    "msg_watchdog",
    pdMS_TO_TICKS(10), //check every 10ms
    pdTRUE,
    NULL,
    watchdog_ros_callback
  );
  xTimerStart(watchdog_ros_timer, 0);
}

//Setup watchdog_ros timer callback
void watchdog_ros_callback(TimerHandle_t xTimer){
  uint32_t now = xTaskGetTickCount();
  if((now-last_msg_tick) > pdMS_TO_TICKS(TIMEOUT_ROS_MS)) {
    //StateMachine_SetFault(ROS_CONNECTION_LOSS);
  }
}

// Define the Micro-ROS objects declared in the header file
void error_loop(){
  while(1){
    StateMachine_SetFault(ROS_CONNECTION_LOSS);
    vTaskDelay(pdMS_TO_TICKS(1000));
    ESP.restart();
    
  }
}

void subscription_callback(const void * msgin)
{  
  const geometry_msgs__msg__Twist * msg = (const geometry_msgs__msg__Twist *)msgin;
  last_msg_tick = xTaskGetTickCount();

  // Lock the mutex before updating shared variable
  if(xSemaphoreTake(controlDataMutex, portMAX_DELAY) == pdTRUE)
  {
    // Extract the velocity and steering commands from the message
    velCommand = msg->linear.x;
    steerCommand = msg->angular.z;

    cmd.steer    = steerCommand;
    cmd.throttle = velCommand;

    xSemaphoreGive(controlDataMutex);
  };

  xQueueSend(controlQueue, &cmd, pdMS_TO_TICKS(5));  // Send to queue without waiting  
}

//Ros2 Executor Task
void ros_executor_task(void *pvParameters) 
{
  while(1) {
    // Spin the executor to process incoming messages with a timeout
    rcl_ret_t ret = rclc_executor_spin_some(&executor, RCL_MS_TO_NS(50));

    if (ret != RCL_RET_OK) {StateMachine_SetFault(ROS_CONNECTION_LOSS);}

    vTaskDelay(10 / portTICK_PERIOD_MS);
  }

}