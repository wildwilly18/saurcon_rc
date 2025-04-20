#include "motor_control.h"

void init_pwm(){
    ESP32PWM::allocateTimer(0);
	ESP32PWM::allocateTimer(1);
	ESP32PWM::allocateTimer(2);
	ESP32PWM::allocateTimer(3);
}

void init_servo()
{
    Servo SteerServo;
    SteerServo.setPeriodHertz(50);
    SteerServo.attach(SERVO_PIN);
}

void set_servo(uint32_t angle)
{
    uint32_t steer_angle = max(STEER_MIN, angle);
    SteerServo.write(angle);
}