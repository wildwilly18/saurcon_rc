#pragma once

#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>

#include "display.h"
#include "ros_interface.h"
#include "motor_control.h"
#include "encoder.h"
#include "saurcon_LED.h"
#include "imu.h"
#include "encoder.h"

#include "types/saurcon_enums.h"

extern TaskHandle_t ros_executor_task_handle;

class StateMachine {
public:
    StateMachine();
    void run();
    void setRequestedState(SaurconState reqState);
    void setFault(SaurconFaults fault);
    SaurconState getState();

private:
    SaurconState currentState;
    SaurconState previousState;
    SaurconState requestedState;

    SemaphoreHandle_t stateMutex;

    DisplayManager display;
    LEDManager led;

    void onEnter(SaurconState state);
    void onExit(SaurconState state);
    void handle(SaurconState state);

    bool isValidTransition(SaurconState from, SaurconState to);

    void onEnter_STARTUP();
    void handle_STARTUP();

    void onEnter_STARTUP_ROS();
    void handle_STARTUP_ROS();

    void onEnter_SETUP();
    void handle_SETUP();

    void onEnter_STANDBY();
    void handle_STANDBY();

    void onEnter_RUN_CONTROL();
    void handle_RUN_CONTROL();
    void onExit_RUN_CONTROL();

    void onEnter_RUN_AUTONOMOUS();
    void handle_RUN_AUTONOMOUS();
    void onExit_RUN_AUTONOMOUS();

    void onEnter_FAULT();
    void handle_FAULT();
    
    void onEnter_FAULT_ROS();
    void handle_FAULT_ROS();

};
