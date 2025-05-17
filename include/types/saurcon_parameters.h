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
#define WHEEL_RADIUS 33