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

    uint32_t throttle        = 1500;
    uint32_t throttle_mapped = 1500;

    while(true){
        // after done, transition to SETUP
        if (stateMachine) {
            localState = stateMachine->getState();
        }
        
        if(xQueueReceive(controlQueue, &receivedCommand, pdMS_TO_TICKS(5))){
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

        }
        /* else {
            if(stateMachine && (localState == SaurconState::RUN_CONTROL || localState == SaurconState::RUN_AUTONOMOUS))
            {
                stateMachine->setFault(ROS_COM_LOSS);
                set_servo(servo_angle);
            }
        }
            */
        
        vTaskDelay(pdMS_TO_TICKS(2));
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
    ledcSetup(ESC_CHANNEL, ESC_FREQ, ESC_RES_BITS);
    ledcAttachPin(ESC_PIN, ESC_CHANNEL);

    digitalWrite(LED_RED, HIGH);
    digitalWrite(LED_GRN, HIGH);

    float motor_rpm = 0;

    // Check if already spinning before attempting calibration
    motor_rpm = encoder->getFilteredRPM();

    if (fabs(motor_rpm) > 10.0) {
        // ESC already armed or spinning — skip calibration
        return;
    }

    // STEP 1A: Send some throttle and monitor RPM
    set_throttle(1600);

    const int check_duration_ms = 1000;
    const int sample_interval_ms = 10;
    const int max_checks = check_duration_ms / sample_interval_ms;

    for (int i = 0; i < max_checks; ++i) {
        motor_rpm = encoder->getFilteredRPM();

        if (fabs(motor_rpm) > 10.0) {
            // Motor responded — assume ESC already calibrated
            set_throttle(1500);
            return;
        }

        vTaskDelay(pdMS_TO_TICKS(sample_interval_ms));
    }

    // STEP 1B: Send FULL throttle
    set_throttle(2000);
    vTaskDelay(pdMS_TO_TICKS(4000));

    // STEP 2: Send MIN throttle
    set_throttle(1000);
    vTaskDelay(pdMS_TO_TICKS(4000));

    // STEP 3: Neutral throttle to finish
    set_throttle(1500);
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

    // Map to [180, 0]
    return (uint32_t)((1.0f - steerValue) * 90.0f);  // (-1 maps to 180, 0 to 90, 1 to 0)
}

void write_esc_us(uint32_t microseconds){
    // Convert microseconds to 16-bit duty (50Hz = 20,000 us period)
    uint32_t duty = microseconds * 65535 / 20000;
    ledcWrite(ESC_CHANNEL, duty);
}