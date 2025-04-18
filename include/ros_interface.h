#pragma once

//Include FreeRTOS
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

// Micro-Ros includes
#include <micro_ros_platformio.h>
#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <geometry_msgs/msg/twist.h>

// Arduino and Pins
#define LED_PIN 13
#include "pins.h"
#include <Arduino.h>

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

//Shared Variables Set
extern float velCommand;
extern float steerCommand;

extern TaskHandle_t ros_executor_task_handle;

extern SemaphoreHandle_t controlDataMutex;

//Micro-Ros objects
extern rcl_subscription_t subscriber;
extern geometry_msgs__msg__Twist msg;
extern rclc_executor_t executor;
extern rclc_support_t support;
extern rcl_allocator_t allocator;
extern rcl_node_t node;
extern rcl_timer_t timer;

//Function
void initROS();
void error_loop();
void subscription_callback(const void * msgin);
void ros_executor_task(void *pvParameters);