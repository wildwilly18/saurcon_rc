//---
#include <Arduino.h>
#include <U8g2lib.h>

//Include FreeRTOS
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

// Pin Definitions for ESP32
#include "pins.h"

// Micro-Ros includes
#include <micro_ros_platformio.h>
#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <geometry_msgs/msg/twist.h>

//Micro-Ros objects
rcl_subscription_t subscriber;
geometry_msgs__msg__Twist msg;
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t timer;

#ifdef LED_BUILTIN
#define LED_PIN LED_BUILTIN
#else
#define LED_PIN 13
#endif

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}


// Velocity variable to hold throttle command
static float velCommand = 0.0;
static float steerCommand = 0.0;

// UI display buffers
static char formattedVEL[10];
static char formattedSetVEL[10];
static char formattedSetSTEER[10];

// FreeRTOS task handles
TaskHandle_t ros_executor_task_handle = NULL;
TaskHandle_t display_update_task_handle = NULL;

SemaphoreHandle_t controlDataMutex;

// OLED display object
U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, SCL_PIN, SDA_PIN, U8X8_PIN_NONE);


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

//Display update task
void display_update_task(void *pvParameters)
{
  while(true) {
    if(xSemaphoreTake(controlDataMutex, portMAX_DELAY) == pdTRUE){
      // Format velocityCommand into display string
      sprintf(formattedSetVEL, "%03.4f", velCommand);
      sprintf(formattedSetSTEER, "%03.4f", steerCommand);
      xSemaphoreGive(controlDataMutex); //Release Mutex
    };

    // Update display
    u8g2.clearBuffer();
    u8g2.setFont(u8g2_font_t0_12_tf);
    u8g2.drawStr(0, 10, "______SauRCon_______");
    u8g2.drawStr(0, 25, "SET STR: ");
    u8g2.drawStr(0, 40, "SET VEL: ");
    u8g2.drawStr(85, 25, formattedSetSTEER);      // Your velocity reading (maybe add later)
    u8g2.drawStr(85, 40, formattedSetVEL);   // Incoming throttle value
    u8g2.drawStr(25, 55, "STATE: RUNNING");

    u8g2.sendBuffer();
    
    vTaskDelay(33 / portTICK_PERIOD_MS);
  }
}

void setup() {
  //Setup control data mutex
  controlDataMutex = xSemaphoreCreateMutex();
  if(controlDataMutex == NULL){
    error_loop();
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

  // OLED setup
    // OLED setup
    u8g2.begin();
    u8g2.clearBuffer();
    u8g2.setFont(u8g2_font_t0_12_tf);
    u8g2.drawStr(0, 10, "______SauRCon_______");
    u8g2.drawStr(0, 25, "SET STR: ");
    u8g2.drawStr(0, 40, "SET VEL: ");
    u8g2.drawStr(25, 55, "STATE: START");
    u8g2.sendBuffer();

  // create tasks
  xTaskCreate(
    ros_executor_task,
    "ros_executor_task",
    4096,
    NULL,
    1,
    &ros_executor_task_handle);

  xTaskCreate(
    display_update_task,
    "display_update_task",
    4096,
    NULL,
    1,
    &display_update_task_handle
  );
}

void loop() {
  // Empty loop, tasks handle logic
}