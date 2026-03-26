#pragma

#include <Arduino.h>
#include <ESP32Servo.h>
#include <algorithm>

//Include FreeRTOS
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include "pins.h"
#include "types/control_cmd_type.h"

#include "state_machine.h"
#include "encoder.h"


using namespace std;

// Declare some values for controlling the Steer Servo.
#define SERVO_MIN (uint32_t)0
#define SERVO_MAX (uint32_t)180

#define STEER_MIN (uint32_t)45
#define STEER_MAX (uint32_t)135

#define THROTTLE_MAX (int32_t)255
#define THROTTLE_MIN (int32_t)0

extern QueueHandle_t controlQueue;

extern Servo SteerServo;

void task_motion_control(void *pv);
void init_pwm();
void init_servo();
void set_servo(uint32_t angle);

void init_throttle();
void set_throttle(int32_t throttle);

int32_t map_throttle(float speed);
uint32_t map_steering(float steerValue);