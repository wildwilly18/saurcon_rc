#include "motor_control.h"

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
    ledcSetup(ESC_CHANNEL, ESC_FREQ, ESC_RES_BITS);
    ledcAttachPin(ESC_PIN, ESC_CHANNEL);

    digitalWrite(LED_RED,   HIGH);
    digitalWrite(LED_GRN, HIGH);

    //Need to watch for the rpm and if > some small number skip the throttle setup. 

    vTaskDelay(pdMS_TO_TICKS(4000));
    set_throttle(2000);
    digitalWrite(LED_RED,   LOW);
    vTaskDelay(pdMS_TO_TICKS(5000));

    set_throttle(1000);
    digitalWrite(LED_GRN,   LOW);
    vTaskDelay(pdMS_TO_TICKS(5000));

    set_throttle(1500);
    digitalWrite(LED_RED,   HIGH);
    digitalWrite(LED_GRN, HIGH);
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

    //write us
    write_esc_us(throttle_set);
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

void write_esc_us(uint32_t microseconds){
    // Convert microseconds to 16-bit duty (50Hz = 20,000 us period)
    uint32_t duty = microseconds * 65535 / 20000;
    ledcWrite(ESC_CHANNEL, duty);
}