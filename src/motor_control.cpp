#include "motor_control.h"

Servo SteerServo;
Servo Throttle;

SaurconState localState;

//Initialize control queue to handle to queue commands. 
QueueHandle_t controlQueue = xQueueCreate(10, sizeof(ControlCommand));

void task_motion_control(void *pv){
    ControlCommand receivedCommand;
    uint32_t servo_angle        = 90;
    uint32_t servo_angle_mapped = 90;

    uint32_t throttle        = 1500;
    uint32_t throttle_mapped = 1500;

    while(true){
        // after done, transition to SETUP
        if (stateMutex && xSemaphoreTake(stateMutex, (TickType_t)10) == pdTRUE) {
            localState = saurcon_state;
    
            xSemaphoreGive(stateMutex);
        }
        
        if(xQueueReceive(controlQueue, &receivedCommand, pdMS_TO_TICKS(200))){
            servo_angle_mapped = map_steering(receivedCommand.steer);
            throttle_mapped    = map_throttle(receivedCommand.throttle);

            switch (localState)
            {
            case RUN_SCON:
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
            StateMachine_SetFault(ROS_CONNECTION_LOSS);
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
    vTaskDelay(pdMS_TO_TICKS(400));
    set_servo(180);
    vTaskDelay(pdMS_TO_TICKS(400));
    set_servo(0);
    vTaskDelay(pdMS_TO_TICKS(400));
    set_servo(90);
}

void init_throttle()
{
    Throttle.setPeriodHertz(200);
    Throttle.attach(ESC_PIN);

    //set_throttle(1500);
    //vTaskDelay(pdMS_TO_TICKS(1000));
    //set_throttle(2000);
    //vTaskDelay(pdMS_TO_TICKS(1500));
    //set_throttle(1000);
    //vTaskDelay(pdMS_TO_TICKS(1500));
    //set_throttle(1500);
}

void set_servo(uint32_t angle)
{
    uint32_t steer_angle = max(STEER_MIN, angle);
    steer_angle = min(STEER_MAX, steer_angle);
    SteerServo.write(steer_angle);
}

void set_throttle(uint32_t throttle)
{
    uint32_t throttle_set = max(MIN_ESC_PWM, throttle);
    throttle_set = min(MAX_ESC_PWM, throttle_set);
    throttle_set = min(MAX_ESC_PWM, throttle_set);
    Throttle.write(throttle_set);
}

uint32_t map_throttle(float speed)
{
    // Clamp input to [-1.0, 1.0]
    speed = constrain(speed, -1.0f, 1.0f);
    uint32_t mapped_throttle;

    if (speed > 0.0f) {
        mapped_throttle = (uint32_t)(1500 + speed * (MAX_RUN_ESC_PWM - 1500));
    } else {
        mapped_throttle = (uint32_t)(1500 + speed * (1500 - MIN_RUN_ESC_PWM));
    }

    return mapped_throttle;
}

uint32_t map_steering(float steerValue)
{
    // Clamp input to [-1.0, 1.0]
    steerValue = constrain(steerValue, -1.0f, 1.0f);

    // Map to [0, 180]
    return (uint32_t)((steerValue + 1.0f) * 90.0f);  // (-1 maps to 0, 0 to 90, 1 to 180)
}