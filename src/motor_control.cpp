#include "motor_control.h"
#include "shared_resources.h"

Servo SteerServo;

//Throttle Setup
#define ESC_CHANNEL   4
#define ESC_FREQ     50
#define ESC_RES_BITS 16

SaurconState localState;


//Initialize control queue to handle to queue commands. 
QueueHandle_t controlQueue = xQueueCreate(10, sizeof(ControlCommand));

void task_motion_control(void *pv){
    ControlCommand receivedCommand;
    uint32_t servo_angle        = 90;
    uint32_t servo_angle_mapped = 90;

    int32_t throttle        = 0;
    int32_t throttle_mapped = 0;

    while(true){
        // after done, transition to SETUP
        if (stateMachine) {
            localState = stateMachine->getState();
        }
        
        if(xQueueReceive(controlQueue, &receivedCommand, pdMS_TO_TICKS(120))){
            servo_angle_mapped = map_steering(receivedCommand.steer);
            throttle_mapped    = map_throttle(receivedCommand.throttle);

            switch (localState)
            {
            case SaurconState::RUN_AUTONOMOUS:
            case SaurconState::RUN_CONTROL:
                servo_angle_mapped = map_steering(receivedCommand.steer);

                set_servo(servo_angle_mapped);
                set_throttle(throttle_mapped);
                break;

            default:
                set_servo(servo_angle);
                set_throttle(throttle);
                break;
            }

        } else {
            if(stateMachine)
            {
                //If fail Stop Motors
                stateMachine->setFault(ROS_CONNECTION_LOSS);
                set_servo(servo_angle);
                set_throttle(throttle);
            }
        }
        
        vTaskDelay(5);
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
    vTaskDelay(pdMS_TO_TICKS(500));
    set_servo(180);
    vTaskDelay(pdMS_TO_TICKS(500));
    set_servo(0);
    vTaskDelay(pdMS_TO_TICKS(500));
    set_servo(90);
}

void init_throttle()
{
    //Setup here for the Dual H Bridge Motor Control
    pinMode(MOT_ENABLE, OUTPUT);
    pinMode(MOT_1, OUTPUT);
    pinMode(MOT_2, OUTPUT);
    
    digitalWrite(MOT_1, LOW);
    digitalWrite(MOT_2, LOW);
    analogWrite(MOT_ENABLE, 0);

    digitalWrite(LED_RED, LOW);
    digitalWrite(LED_GRN, HIGH);
}

void set_servo(uint32_t angle)
{
    uint32_t steer_angle = max(STEER_MIN, angle);
    steer_angle = min(STEER_MAX, steer_angle);
    SteerServo.write(steer_angle);
}

void set_throttle(int32_t throttle)
{
    // Use abs() for PWM magnitude, preserve sign for direction
    uint8_t throttle_value = (uint8_t)min((int32_t)THROTTLE_MAX, abs(throttle));

    if(throttle >= 0){
        digitalWrite(MOT_1, LOW);
        digitalWrite(MOT_2, HIGH);
        analogWrite(MOT_ENABLE, throttle_value);
    } else {
        digitalWrite(MOT_1, HIGH);
        digitalWrite(MOT_2, LOW);
        analogWrite(MOT_ENABLE, throttle_value);
    }
}

int32_t map_throttle(float speed)
{
    // Clamp input to [-1.0, 1.0]
    speed = constrain(speed, -1.0f, 1.0f);

    // Preserve sign for direction control in set_throttle()
    return (int32_t)(speed * THROTTLE_MAX);
}

uint32_t map_steering(float steerValue)
{
    // Clamp input to [-1.0, 1.0]
    steerValue = constrain(steerValue, -1.0f, 1.0f);

    // Map to [180, 0]
    return (uint32_t)((1.0f - steerValue) * 90.0f);  // (-1 maps to 180, 0 to 90, 1 to 0)
}