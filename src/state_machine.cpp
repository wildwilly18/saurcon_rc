#include "state_machine.h"
#include "display.h"

// Initialize the State Machine
StateMachine::StateMachine() {
    stateMutex = xSemaphoreCreateMutex();
    currentState = STARTUP_SCON;
    previousState = NO_STATE_SCON;

    imu.begin();

    display.init();
    display.startTask();

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
        currentState = (fault == ROS_CONNECTION_LOSS) ? FAULT_ROS_SCON : FAULT_SCON;
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
        case STARTUP_SCON: onEnter_STARTUP_SCON(); break;
        case STARTUP_ROS_SCON: onEnter_STARTUP_ROS_SCON(); break;
        case SETUP_SCON: onEnter_SETUP_SCON(); break;
        case RUN_SCON: onEnter_RUN_SCON(); break;
        case FAULT_SCON: onEnter_FAULT_SCON(); break;
        case FAULT_ROS_SCON: onEnter_FAULT_ROS_SCON(); break;
        default: break;
    }
}

void StateMachine::handle(SaurconState state) {
    switch (state) {
        case STARTUP_SCON: handle_STARTUP_SCON(); break;
        case STARTUP_ROS_SCON: handle_STARTUP_ROS_SCON(); break;
        case SETUP_SCON: handle_SETUP_SCON(); break;
        case RUN_SCON: handle_RUN_SCON(); break;
        case FAULT_SCON: handle_FAULT_SCON(); break;
        case FAULT_ROS_SCON: handle_FAULT_ROS_SCON(); break;
        default: break;
    }
}

void StateMachine::onExit(SaurconState state) {
    switch (state) {
        case RUN_SCON: onExit_RUN_SCON(); break;
        default: break; // No exit behavior for other states yet
    }
}

// --- Individual State Methods ---

void StateMachine::onEnter_STARTUP_SCON() {
    led.setLEDState(LEDState::OFF, LEDState::OFF, LEDState::OFF);
    vTaskDelay(pdMS_TO_TICKS(500));
}

void StateMachine::handle_STARTUP_SCON() {
    vTaskDelay(pdMS_TO_TICKS(500));
    setState(STARTUP_ROS_SCON);
}

void StateMachine::onEnter_STARTUP_ROS_SCON() {
    led.setLEDState(LEDState::ON, LEDState::OFF, LEDState::BLINK_FAST);
    init_ROS();
    if (!ros_executor_task_handle) {
        xTaskCreate(ros_executor_task, "ros_executor_task", 4096, NULL, 1, &ros_executor_task_handle);
    }
}

void StateMachine::handle_STARTUP_ROS_SCON() {
    setState(SETUP_SCON);
}

void StateMachine::onEnter_SETUP_SCON() {
    //display.setState(STARTUP_DISPLAY);
}

void StateMachine::handle_SETUP_SCON() {
    init_encoder_mutex();
    init_encoder_isr();
    xTaskCreatePinnedToCore(rpm_filter_task, "rpm_filter_task", 4096, NULL, 1, NULL, 1);
    init_pwm();
    init_servo();
    init_throttle();
    xTaskCreate(task_motion_control, "task_motion_control", 2048, NULL, 2, NULL);
    setState(RUN_SCON);
}

void StateMachine::onEnter_RUN_SCON() {
    led.setLEDState(LEDState::OFF, LEDState::ON, LEDState::BLINK_SLOW);
    display.setState(IMU_DISPLAY);
}

void StateMachine::handle_RUN_SCON() {
    // Placeholder for main loop operations
}

void StateMachine::onExit_RUN_SCON() {
    // Placeholder for cleanup logic
}

void StateMachine::onEnter_FAULT_SCON() {
    led.setLEDState(LEDState::ON, LEDState::BLINK_FAST, LEDState::OFF);
    display.setState(FAULT_DISPLAY);
}

void StateMachine::handle_FAULT_SCON() {
    // Placeholder for fault handling logic
}

void StateMachine::onEnter_FAULT_ROS_SCON() {
    led.setLEDState(LEDState::ON, LEDState::OFF, LEDState::BLINK_FAST);
    display.setState(FAULT_DISPLAY);
    vTaskDelay(pdMS_TO_TICKS(3000));
}

void StateMachine::handle_FAULT_ROS_SCON() {
    vTaskDelay(pdMS_TO_TICKS(1000));
    led.setLEDState(LEDState::ON, LEDState::OFF, LEDState::OFF);
    vTaskDelay(pdMS_TO_TICKS(2000));
    ESP.restart();
}
