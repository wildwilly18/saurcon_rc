#pragma once

// Encoder Parameters
#define RPM_FILTER_SIZE 10

// Control Parameters
#define MIN_TURN_ANGLE 45
#define MAX_TURN_ANGLE 135

#define MIN_ESC_PWM (uint32_t)1000
#define MAX_ESC_PWM (uint32_t)2000

#define MIN_RUN_ESC_PWM (uint32_t)1400
#define MAX_RUN_ESC_PWM (uint32_t)1600

// Physical Parameters
#define WHEEL_RADIUS 0.033f
#define ENCODER_TICKS_PER_REV 6.0f
#define GEAR_RATIO 3.14f
#define TICKS_PER_WHEEL_REV (ENCODER_TICKS_PER_REV * GEAR_RATIO)

//In this instance our car frame is North West Up
#define IMU_ROTATION  90
#define MAG_ROTATION 180

//Addresses
#define MPU_9250_ADDRESS 0x68