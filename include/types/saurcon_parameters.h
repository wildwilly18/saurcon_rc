#pragma once

// Encoder Parameters
#define RPM_FILTER_SIZE 10

// Control Parameters
#define MIN_TURN_ANGLE 45
#define MAX_TURN_ANGLE 135

#define MIN_MOT_PWM (uint8_t)255
#define MAX_MOT_PWM (uint8_t)0

// Physical Parameters
#define WHEEL_RADIUS 0.033f
#define ENCODER_TICKS_PER_REV 192 //(11 PPR * 4 Quadrature * 4.4 Gearbox)
#define GEAR_RATIO 3.14f
#define TICKS_PER_WHEEL_REV (ENCODER_TICKS_PER_REV * GEAR_RATIO)

//In this instance our car frame is North West Up
#define IMU_ROTATION  90

//Addresses
#define MPU_6050_ADDRESS 0x68

// Use Display
#define DISPLAY_ENABLE 0