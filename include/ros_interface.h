#pragma once

//Include FreeRTOS
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/timers.h>

// Micro-Ros includes
#include <micro_ros_platformio.h>
#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

// Types
#include <std_msgs/msg/u_int8.h>
#include <geometry_msgs/msg/twist.h>
#include <sensor_msgs/msg/imu.h>
#include <sensor_msgs/msg/magnetic_field.h>

#include "types/control_cmd_type.h"

// Arduino and Pins
#define LED_PIN 13
#include "pins.h"
#include <Arduino.h>

// Include needed to pipe the RTOS QUEUE of the CMDs
#include "motor_control.h"
#include "state_machine.h"

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

//Micro-Ros objects
extern rcl_subscription_t control_subscriber;
extern rcl_subscription_t state_cmd_subscriber;

extern rcl_publisher_t imu_pub;
extern rcl_publisher_t mag_pub;
extern rcl_publisher_t state_pub;

extern std_msgs__msg__UInt8 state_msg;
extern geometry_msgs__msg__Twist ctrl_msg;
extern sensor_msgs__msg__Imu msg_imu;
extern sensor_msgs__msg__MagneticField msg_mag;

extern rclc_executor_t executor;
extern rclc_support_t support;
extern rcl_allocator_t allocator;
extern rcl_node_t node;
extern rcl_timer_t timer;

//Function
void init_ROS();
void setup_watchdog_ros_timer();
void watchdog_ros_callback(TimerHandle_t xTimer);
void error_loop();
void ctrl_sub_callback(const void * msgin);
void state_cmd_sub_callback(const void * msgin);

// Task Functions
void ros_ctrl_sub_task(void *pvParameters);
void ros_state_sub_task(void *pvParameters);
void ros_sensor_publisher_task(void *pvParameters);
void ros_state_publisher_task(void *pvParameters);
void fill_msg_header(std_msgs__msg__Header &header, const char *frame_id_str);
builtin_interfaces__msg__Time get_time();