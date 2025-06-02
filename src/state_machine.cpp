#include "state_machine.h"
#include "shared_resources.h"
#include "display.h"

// Initialize the State Machine
StateMachine::StateMachine() {
    stateMutex = xSemaphoreCreateMutex();
    currentState = SaurconState::STARTUP;
    previousState = SaurconState::NO_STATE;
    commandedState = SaurconState::STARTUP;
    currentFault = SaurconFaults::NONE;

    Wire.begin(SDA_PIN, SCL_PIN);

    vTaskDelay(pdMS_TO_TICKS(2000));

    i2cMutex = xSemaphoreCreateMutex();

    display.init();
    display.startTask();

    //imu = new IMU(0x68);
    //imu->begin();

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
        currentFault = fault;
        currentState = SaurconState::FAULT;
        xSemaphoreGive(stateMutex);
    }
}

void StateMachine::setCommandState(SaurconState cmd) {
    if (xSemaphoreTake(stateMutex, 10) == pdTRUE) {
        commandedState = cmd;
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

SaurconFaults StateMachine::getFault() {
    SaurconFaults fault;
    if (xSemaphoreTake(stateMutex, 10) == pdTRUE) {
        fault = currentFault;
        xSemaphoreGive(stateMutex);
    }
    return fault;
}

SaurconState StateMachine::getCommandedState() {
    SaurconState state;
    if (xSemaphoreTake(stateMutex, 10) == pdTRUE) {
        state = commandedState;
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

void StateMachine::onExit(SaurconState state) {
    switch (state) {
        case SaurconState::RUN_CONTROL: onExit_RUN_CONTROL(); break;
        default: break; // No exit behavior for other states yet
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

    if(checkStateCommandChange() && isExternalCommandAllowed(StateMachine::getState(), StateMachine::getCommandedState()))
    { StateMachine::setState(StateMachine::getCommandedState());}
}

bool StateMachine::checkStateCommandChange(){
    SaurconState current_state = StateMachine::getState();
    SaurconState commanded_state = StateMachine::getCommandedState();

    return (current_state != commanded_state) ? true : false;
}

bool StateMachine::isExternalCommandAllowed(SaurconState current, SaurconState cmd) {
    switch(current) {
        case SaurconState::STANDBY:        return true;
        case SaurconState::RUN_AUTONOMOUS: return true;
        case SaurconState::RUN_CONTROL:    return true;
        default: return false;
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
    //led.setLEDState(LEDState::ON, LEDState::OFF, LEDState::BLINK_FAST);
    display.setState(ROS_STARTUP_DISPLAY);

    init_ROS();

    if (!ros_ctrl_sub_task_handle) {
        xTaskCreatePinnedToCore(ros_ctrl_sub_task, "ros_ctrl_sub_task", 4096, NULL, 3, &ros_ctrl_sub_task_handle, 0);
    }

    if (!ros_state_sub_task_handle) {
        xTaskCreatePinnedToCore(ros_state_sub_task, "ros_state_sub_task", 2048, NULL, 2, &ros_state_sub_task_handle, 0);
    }

    //if(!ros_sensor_publisher_task_handle){
    //    xTaskCreate(ros_sensor_publisher_task, "ros_sensor_publisher_task", 8192, NULL, 1, &ros_sensor_publisher_task_handle);
    //}

    if(!ros_state_publisher_task_handle){
        xTaskCreate(ros_state_publisher_task, "ros_state_publisher_task", 4096, NULL, 2, &ros_state_publisher_task_handle);
    }
    vTaskDelay(pdMS_TO_TICKS(100));

    setup_watchdog_ros_timer();
}

void StateMachine::handle_STARTUP_ROS() {
    
    setState(SaurconState::SETUP);
}

void StateMachine::onEnter_SETUP() {
    display.setState(STARTUP_DISPLAY);
}

void StateMachine::handle_SETUP() {
    encoder = new Encoder(A_PIN, B_PIN, C_PIN);
    encoder->begin();

    init_pwm();
    init_servo();
    init_throttle();
    xTaskCreatePinnedToCore(task_motion_control, "task_motion_control", 4096, NULL, 3, NULL,1);
    setState(SaurconState::STANDBY);
}

void StateMachine::onEnter_STANDBY(){
    led.setLEDState(LEDState::OFF, LEDState::BLINK_SLOW, LEDState::BLINK_SLOW);
    display.setState(STANDBY_DISPLAY);
}

void StateMachine::handle_STANDBY(){
    //Placeholder, wait to be sent elsewhere.
    vTaskDelay(pdMS_TO_TICKS(100));
}

void StateMachine::onEnter_RUN_CONTROL() {
    led.setLEDState(LEDState::OFF, LEDState::ON, LEDState::BLINK_SLOW);
    display.setState(RUN_DISPLAY);
}

void StateMachine::handle_RUN_CONTROL() {
    // Placeholder for main loop operations
    vTaskDelay(pdMS_TO_TICKS(100));
}

void StateMachine::onExit_RUN_CONTROL() {
    // Placeholder for cleanup logic
}

void StateMachine::onEnter_RUN_AUTONOMOUS() {
    led.setLEDState(LEDState::OFF, LEDState::ON, LEDState::BLINK_SLOW);
    display.setState(ENCODER_DISPLAY);
}

void StateMachine::handle_RUN_AUTONOMOUS() {
    // Placeholder for main loop operations
}

void StateMachine::onExit_RUN_AUTONOMOUS() {
    // Placeholder for cleanup logic
}

void StateMachine::onEnter_FAULT() {
    led.setLEDState(LEDState::ON, LEDState::OFF, LEDState::OFF);
    display.setFault(currentFault);
}

void StateMachine::handle_FAULT() {
    // Placeholder for fault handling logic
    vTaskDelay(pdMS_TO_TICKS(2000));
    ESP.restart();
}


//Following to be deprecated handle all in fault.
void StateMachine::onEnter_FAULT_ROS() {
    led.setLEDState(LEDState::ON, LEDState::OFF, LEDState::BLINK_FAST);
    display.setFault(currentFault);
    vTaskDelay(pdMS_TO_TICKS(3000));
}

void StateMachine::handle_FAULT_ROS() {
    vTaskDelay(pdMS_TO_TICKS(1000));
    led.setLEDState(LEDState::ON, LEDState::OFF, LEDState::OFF);
    vTaskDelay(pdMS_TO_TICKS(2000));
    ESP.restart();
}
