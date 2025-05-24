#include "state_machine.h"
#include "shared_resources.h"
#include "display.h"

// Initialize the State Machine
StateMachine::StateMachine() {
    stateMutex = xSemaphoreCreateMutex();
    currentState = SaurconState::STARTUP;
    previousState = SaurconState::NO_STATE;

    Wire.begin(SDA_PIN, SCL_PIN);

    vTaskDelay(pdMS_TO_TICKS(2000));

    i2cMutex = xSemaphoreCreateMutex();

    display.init();
    display.startTask();

    imu = new IMU(0x68);
    imu->begin();

    led.init();
}

void StateMachine::setState(SaurconState nextState) {
    if (xSemaphoreTake(stateMutex, 10) == pdTRUE) {
        if (nextState != currentState) {
            currentState = nextState;
        }
        xSemaphoreGive(stateMutex);
    }
}

void StateMachine::setFault(SaurconFaults fault) {
    if (xSemaphoreTake(stateMutex, 10) == pdTRUE) {
        currentState = (fault == ROS_CONNECTION_LOSS) ? SaurconState::FAULT_ROS : SaurconState::FAULT;
        xSemaphoreGive(stateMutex);
    }
}

SaurconState StateMachine::getState() {
    SaurconState state;
    if (xSemaphoreTake(stateMutex, 10) == pdTRUE) {
        state = currentState;
        xSemaphoreGive(stateMutex);
    }
    return state;
}

void StateMachine::run() {
    SaurconState localState = getState();

    if (localState != previousState) {
        onExit(previousState);
        onEnter(localState);
        previousState = localState;
    }

    handle(localState);
    vTaskDelay(pdMS_TO_TICKS(100));
}

// --- State Routing ---

void StateMachine::onEnter(SaurconState state) {
    switch (state) {
        case SaurconState::STARTUP: onEnter_STARTUP(); break;
        case SaurconState::STARTUP_ROS: onEnter_STARTUP_ROS(); break;
        case SaurconState::SETUP: onEnter_SETUP(); break;
        case SaurconState::STANDBY: onEnter_STANDBY(); break;
        case SaurconState::RUN_CONTROL: onEnter_RUN_CONTROL(); break;
        case SaurconState::RUN_AUTONOMOUS: onEnter_RUN_AUTONOMOUS(); break;
        case SaurconState::FAULT: onEnter_FAULT(); break;
        case SaurconState::FAULT_ROS: onEnter_FAULT_ROS(); break;
        default: break;
    }
}

void StateMachine::handle(SaurconState state) {
    switch (state) {
        case SaurconState::STARTUP: handle_STARTUP(); break;
        case SaurconState::STARTUP_ROS: handle_STARTUP_ROS(); break;
        case SaurconState::SETUP: handle_SETUP(); break;
        case SaurconState::STANDBY: handle_STANDBY(); break;
        case SaurconState::RUN_CONTROL: handle_RUN_CONTROL(); break;
        case SaurconState::RUN_AUTONOMOUS: handle_RUN_AUTONOMOUS(); break;
        case SaurconState::FAULT: handle_FAULT(); break;
        case SaurconState::FAULT_ROS: handle_FAULT_ROS(); break;
        default: break;
    }
}

void StateMachine::onExit(SaurconState state) {
    switch (state) {
        case SaurconState::RUN_CONTROL: onExit_RUN_CONTROL(); break;
        default: break; // No exit behavior for other states yet
    }
}

// --- Individual State Methods ---

void StateMachine::onEnter_STARTUP() {
    led.setLEDState(LEDState::OFF, LEDState::OFF, LEDState::OFF);
    vTaskDelay(pdMS_TO_TICKS(50));
}

void StateMachine::handle_STARTUP() {
    vTaskDelay(pdMS_TO_TICKS(50));
    setState(SaurconState::STARTUP_ROS);
}

void StateMachine::onEnter_STARTUP_ROS() {
    led.setLEDState(LEDState::ON, LEDState::OFF, LEDState::BLINK_FAST);
    init_ROS();
    if (!ros_subscriber_task_handle) {
        xTaskCreate(ros_subscriber_task, "ros_subscriber_task", 4096, NULL, 1, &ros_subscriber_task_handle);
    }

    if(!ros_sensor_publisher_task_handle){
        xTaskCreate(ros_sensor_publisher_task, "ros_sensor_publisher_task", 4096, NULL, 1, &ros_sensor_publisher_task_handle);
    }

    if(!ros_state_publisher_task_handle){
        xTaskCreate(ros_state_publisher_task, "ros_state_publisher_task", 2048, NULL, 1, &ros_state_publisher_task_handle);
    }
}

void StateMachine::handle_STARTUP_ROS() {
    setState(SaurconState::SETUP);
}

void StateMachine::onEnter_SETUP() {
    //display.setState(STARTUP_DISPLAY);
}

void StateMachine::handle_SETUP() {
    init_encoder_mutex();
    init_encoder_isr();
    xTaskCreatePinnedToCore(rpm_filter_task, "rpm_filter_task", 4096, NULL, 1, NULL, 1);
    init_pwm();
    init_servo();
    init_throttle();
    xTaskCreate(task_motion_control, "task_motion_control", 2048, NULL, 1, NULL);
    setState(SaurconState::RUN_CONTROL);
}

void StateMachine::onEnter_STANDBY(){
    //Placeholder
}

void StateMachine::handle_STANDBY(){
    //Placeholder
}

void StateMachine::onEnter_RUN_CONTROL() {
    led.setLEDState(LEDState::OFF, LEDState::ON, LEDState::BLINK_SLOW);
    display.setState(IMU_DISPLAY);
}

void StateMachine::handle_RUN_CONTROL() {
    // Placeholder for main loop operations
}

void StateMachine::onExit_RUN_CONTROL() {
    // Placeholder for cleanup logic
}

void StateMachine::onEnter_RUN_AUTONOMOUS() {
    led.setLEDState(LEDState::OFF, LEDState::ON, LEDState::BLINK_SLOW);
    display.setState(IMU_DISPLAY);
}

void StateMachine::handle_RUN_AUTONOMOUS() {
    // Placeholder for main loop operations
}

void StateMachine::onExit_RUN_AUTONOMOUS() {
    // Placeholder for cleanup logic
}

void StateMachine::onEnter_FAULT() {
    led.setLEDState(LEDState::ON, LEDState::BLINK_FAST, LEDState::OFF);
    display.setState(FAULT_DISPLAY);
}

void StateMachine::handle_FAULT() {
    // Placeholder for fault handling logic
}

void StateMachine::onEnter_FAULT_ROS() {
    led.setLEDState(LEDState::ON, LEDState::OFF, LEDState::BLINK_FAST);
    display.setState(FAULT_DISPLAY);
    vTaskDelay(pdMS_TO_TICKS(3000));
}

void StateMachine::handle_FAULT_ROS() {
    vTaskDelay(pdMS_TO_TICKS(1000));
    led.setLEDState(LEDState::ON, LEDState::OFF, LEDState::OFF);
    vTaskDelay(pdMS_TO_TICKS(2000));
    ESP.restart();
}
