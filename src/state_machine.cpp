#include "state_machine.h"
#include "shared_resources.h"
#include "display.h"
#include <map>
#include <set>

// Initialize the State Machine
StateMachine::StateMachine() {
    stateMutex = xSemaphoreCreateMutex();

    currentState = SaurconState::STARTUP;
    previousState = SaurconState::NO_STATE;
    requestedState = SaurconState::NO_STATE;

    Wire.begin(SDA_PIN, SCL_PIN);

    vTaskDelay(pdMS_TO_TICKS(2000));

    i2cMutex = xSemaphoreCreateMutex();

    display.init();
    display.startTask();

    imu = new IMU(0x68);
    imu->begin();

    led.init();
}

void StateMachine::setRequestedState(SaurconState reqState) {
    if (xSemaphoreTake(stateMutex, portMAX_DELAY)) {
        requestedState = reqState;
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
    SaurconState localRequestedState = SaurconState::NO_STATE;

    bool gotRequested = false;

    //Grab our safe copy of requested state
    if(xSemaphoreTake(stateMutex, pdMS_TO_TICKS(10))) {
        localRequestedState = requestedState;
        xSemaphoreGive(stateMutex);
        gotRequested = true;
    }

    // Handle state transition if need be
    if(gotRequested &&
    localRequestedState != SaurconState::NO_STATE &&
    localRequestedState != localState)
    {
        if(StateMachine::isValidTransition(localState, localRequestedState)) {

            onExit(localState);
            
            if (xSemaphoreTake(stateMutex, pdMS_TO_TICKS(10))) {
                currentState = localRequestedState;
                xSemaphoreGive(stateMutex);
            }

            previousState = localState;
            localState = localRequestedState;
            onEnter(localState);
        }
    }

    handle(localState);

    vTaskDelay(pdMS_TO_TICKS(50));
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

bool StateMachine::isValidTransition(SaurconState from, SaurconState to) {
    static const std::map<SaurconState, std::set<SaurconState>> validTransitions = {
        { SaurconState::STARTUP, { SaurconState::STARTUP_ROS } },
        { SaurconState::STARTUP_ROS, { SaurconState::SETUP } },
        { SaurconState::SETUP, { SaurconState::STANDBY, SaurconState::FAULT, SaurconState::FAULT_ROS} },
        { SaurconState::RUN_CONTROL, { SaurconState::RUN_AUTONOMOUS, SaurconState::FAULT, SaurconState::FAULT_ROS, SaurconState::STANDBY } },
        { SaurconState::RUN_AUTONOMOUS, { SaurconState::RUN_CONTROL, SaurconState::FAULT, SaurconState::FAULT_ROS, SaurconState::STANDBY } },
        { SaurconState::STANDBY, { SaurconState::RUN_CONTROL, SaurconState::RUN_AUTONOMOUS, SaurconState::FAULT, SaurconState::FAULT_ROS } },
        { SaurconState::FAULT, { SaurconState::STARTUP } },
        { SaurconState::FAULT_ROS, { SaurconState::STARTUP } }
    };

    auto it = validTransitions.find(from);
    if (it != validTransitions.end()) {
        return it->second.count(to) > 0;
    }
    return false;
}

// --- Individual State Methods ---

void StateMachine::onEnter_STARTUP() {
    led.setLEDState(LEDState::BLINK_FAST, LEDState::OFF, LEDState::OFF);
    vTaskDelay(pdMS_TO_TICKS(50));
}

void StateMachine::handle_STARTUP() {
    vTaskDelay(pdMS_TO_TICKS(50));
    setRequestedState(SaurconState::STARTUP_ROS);
}

void StateMachine::onEnter_STARTUP_ROS() {
    led.setLEDState(LEDState::ON, LEDState::OFF, LEDState::BLINK_FAST);
    display.setState(ROS_STARTUP_DISPLAY);
    init_ROS();
    if (!ros_control_subscriber_task_handle) {
        xTaskCreatePinnedToCore(
            ros_control_subscriber_task, 
            "ros_control_subscriber_task", 
            4096, 
            NULL, 
            3, 
            &ros_control_subscriber_task_handle,
            1);
    }

    if (!ros_state_subscriber_task_handle) {
        xTaskCreatePinnedToCore(
            ros_state_subscriber_task, 
            "ros_state_subscriber_task", 
            4096, 
            NULL, 
            2, 
            &ros_state_subscriber_task_handle,
            0);
    }

    if(!ros_sensor_publisher_task_handle){
        xTaskCreate(
            ros_sensor_publisher_task, 
            "ros_sensor_publisher_task", 
            4096, 
            NULL, 
            2, 
            &ros_sensor_publisher_task_handle);
    }

    if(!ros_state_publisher_task_handle){
        xTaskCreatePinnedToCore(
            ros_state_publisher_task, 
            "ros_state_publisher_task", 
            2048, 
            NULL, 
            2, 
            &ros_state_publisher_task_handle,
            0);
    }

    setup_watchdog_ros_timer();
}

void StateMachine::handle_STARTUP_ROS() {
    setRequestedState(SaurconState::SETUP);
}

void StateMachine::onEnter_SETUP() {
    led.setLEDState(LEDState::ON, LEDState::BLINK_FAST, LEDState::ON);
    display.setState(SETUP_DISPLAY);
    setRequestedState(SaurconState::STANDBY);

    encoder = new Encoder(A_PIN, B_PIN, C_PIN);
    encoder->begin();

    init_pwm();
    init_servo();
    //init_throttle();
    xTaskCreate(
        task_motion_control, 
        "task_motion_control", 
        2048, 
        NULL, 
        1, 
        NULL);

}

void StateMachine::handle_SETUP() {
    setRequestedState(SaurconState::STANDBY);
}

void StateMachine::onEnter_STANDBY(){
    led.setLEDState(LEDState::OFF, LEDState::OFF, LEDState::ON);
    display.setState(STANDBY_DISPLAY);
    //Placeholder
}

void StateMachine::handle_STANDBY(){
    //Placeholder
}

void StateMachine::onEnter_RUN_CONTROL() {
    led.setLEDState(LEDState::OFF, LEDState::ON, LEDState::OFF);
    display.setState(RUN_DISPLAY);
}

void StateMachine::handle_RUN_CONTROL() {
    // Placeholder for main loop operations
}

void StateMachine::onExit_RUN_CONTROL() {
    // Placeholder for cleanup logic
}

void StateMachine::onEnter_RUN_AUTONOMOUS() {
    led.setLEDState(LEDState::OFF, LEDState::ON, LEDState::ON);
    display.setState(ENCODER_DISPLAY);
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
