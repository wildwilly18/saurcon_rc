#include "motor_control.h"

Servo SteerServo;

//Initialize control queue to handle to queue commands. 
QueueHandle_t controlQueue = xQueueCreate(10, sizeof(ControlCommand));

void task_motion_control(void *pv){
    ControlCommand receivedCommand;
    uint32_t servo_angle = 90;
    uint32_t servo_angle_mapped = 90;

    while(true){
        if(xQueueReceive(controlQueue, &receivedCommand, pdMS_TO_TICKS(200))){
            servo_angle_mapped = map_steering(receivedCommand.steer);
            set_servo(servo_angle_mapped);
            /**
            switch (saurcon_state)
            {
            case RUN_SCON:
                servo_angle_mapped = map_steering(receivedCommand.steer);

                set_servo(servo_angle_mapped);
                break;

            default:
                set_servo(servo_angle);
                break;
            }
            //digitalWrite(LED_RED, LOW);
            **/

            //set_servo(servo_angle);
        } else {
            //StateMachine_SetFault(ROS_CONNECTION_LOSS);
            set_servo(servo_angle);
            digitalWrite(LED_RED, HIGH);
            // Handle timeout case here, e.g., set servo to a default position
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
    vTaskDelay(pdMS_TO_TICKS(1000));
    set_servo(180);
    vTaskDelay(pdMS_TO_TICKS(1000));
    set_servo(0);
    vTaskDelay(pdMS_TO_TICKS(1000));
    set_servo(90);
}

void set_servo(uint32_t angle)
{
    uint32_t steer_angle = max(STEER_MIN, angle);
    steer_angle = min(STEER_MAX, steer_angle);
    SteerServo.write(steer_angle);
}

uint32_t map_steering(float steerValue)
{
    // Clamp input to [-1.0, 1.0]
    steerValue = constrain(steerValue, -1.0f, 1.0f);

    // Map to [0, 180]
    return (uint32_t)((steerValue + 1.0f) * 90.0f);  // (-1 maps to 0, 0 to 90, 1 to 180)
}