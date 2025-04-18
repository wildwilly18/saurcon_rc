#include "ros_interface.h"


SemaphoreHandle_t controlDataMutex;

// Define the task handle
TaskHandle_t ros_executor_task_handle = NULL;

//Micro-Ros objects
rcl_subscription_t subscriber;
geometry_msgs__msg__Twist msg;
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t timer;


float velCommand = 0.0;
float steerCommand = 0.0;

void initROS(){

    //Setup control data mutex
    controlDataMutex = xSemaphoreCreateMutex();
    if(controlDataMutex == NULL){
        //error_loop();
    }

    Serial.begin(921600);
    Serial.setRxBufferSize(1024);

    set_microros_serial_transports(Serial);
    
    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, HIGH);  
    
    delay(2000);

    allocator = rcl_get_default_allocator();

    //create init_options
    RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

    // create node
    RCCHECK(rclc_node_init_default(&node, "saurcon_rc", "", &support));

    // create subscriber with best-effort Qos
    RCCHECK(rclc_subscription_init_best_effort(
        &subscriber,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
        "ctrl_output"));

    // create executor
    RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
    RCCHECK(rclc_executor_add_subscription(&executor, &subscriber, &msg, &subscription_callback, ON_NEW_DATA));

}

// Define the Micro-ROS objects declared in the header file
void error_loop(){
    while(1){
      digitalWrite(LED_PIN, !digitalRead(LED_PIN));
      delay(100);
    }
  }
  
  void subscription_callback(const void * msgin)
  {  
    const geometry_msgs__msg__Twist * msg = (const geometry_msgs__msg__Twist *)msgin;
    
    // Lock the mutex before updating shared variable
    if(xSemaphoreTake(controlDataMutex, portMAX_DELAY) == pdTRUE)
    {
      // Extract the velocity and steering commands from the message
      velCommand = msg->linear.x;
      steerCommand = msg->angular.z;
      xSemaphoreGive(controlDataMutex);
    };
  
    digitalWrite(LED_PIN, (msg->angular.z == 0) ? LOW : HIGH);  
  }
  
  //Ros2 Executor Task
  void ros_executor_task(void *pvParameters) 
  {
    while(1) {
      // Spin the executor to process incoming messages
      RCCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10)));
      vTaskDelay(10 / portTICK_PERIOD_MS);
    }
  
  }