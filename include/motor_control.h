#pragma

#include <Arduino.h>
#include <ESP32Servo.h>
#include <algorithm>

//Include FreeRTOS
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include "pins.h"
#include "control_cmd_type.h"



using namespace std;

// Declare some values for controlling the Steer Servo.
#define SERVO_MIN (uint32_t)0
#define SERVO_MAX (uint32_t)180

#define STEER_MIN (uint32_t)45
#define STEER_MAX (uint32_t)135

extern QueueHandle_t controlQueue;

extern Servo SteerServo;

void task_motion_control(void *pv);
void init_pwm();
void init_servo();
void set_servo(uint32_t angle);
void init_motor();

uint32_t map_steering(float steerValue);