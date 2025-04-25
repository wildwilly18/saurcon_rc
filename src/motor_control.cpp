#include "motor_control.h"

Servo SteerServo;

//Initialize control queue to handle to queue commands. 
QueueHandle_t controlQueue = xQueueCreate(10, sizeof(ControlCommand));

void task_motion_control(void *pv){
    ControlCommand receivedCommand;

    while(true){
        if(xQueueReceive(controlQueue, &receivedCommand, portMAX_DELAY)){
            uint32_t servo_angle = map_steering(receivedCommand.steer);

            set_servo(servo_angle);
        }
        
        vTaskDelay(1);
    }
}

void init_pwm(){
    ESP32PWM::allocateTimer(0);
	ESP32PWM::allocateTimer(1);
	ESP32PWM::allocateTimer(2);
	ESP32PWM::allocateTimer(3);
}

void init_servo()
{
    SteerServo.setPeriodHertz(50);
    SteerServo.attach(SERVO_PIN);

    set_servo(90);
}

void set_servo(uint32_t angle)
{
    uint32_t steer_angle = max(STEER_MIN, angle);
    SteerServo.write(angle);
}

uint32_t map_steering(float steerValue)
{
    // Clamp input to [-1.0, 1.0]
    steerValue = constrain(steerValue, -1.0f, 1.0f);

    // Map to [0, 180]
    return (uint32_t)((steerValue + 1.0f) * 90.0f);  // (-1 maps to 0, 0 to 90, 1 to 180)
}