#pragma once

#ifdef LED_BUILTIN
#define LED_PIN LED_BUILTIN
#else
#define LED_PIN 13
#endif

// Servo Out Pin
#define ESC_PIN    19
#define SERVO_PIN  18

//Pins for LED
#define LED_RED 25
#define LED_GRN 33
#define LED_YEL 32

//Pins for hall sensor
#define A_PIN 13
#define B_PIN 12
#define C_PIN 14

//Pins to monitor current
#define I_in_1 A9
#define I_in_2 A8

#define SCL_PIN 22
#define SDA_PIN 21