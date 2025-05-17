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

#include "types/saurcon_enums.h"

extern TaskHandle_t ros_executor_task_handle;

class StateMachine {
public:
    StateMachine();
    void run();
    void setState(SaurconState nextState);
    void setFault(SaurconFaults fault);
    SaurconState getState();

private:
    SaurconState currentState;
    SaurconState previousState;
    SemaphoreHandle_t stateMutex;

    DisplayManager display;
    LEDManager led;
    IMU imu;

    void onEnter(SaurconState state);
    void onExit(SaurconState state);
    void handle(SaurconState state);

    void onEnter_STARTUP_SCON();
    void handle_STARTUP_SCON();
    void onEnter_STARTUP_ROS_SCON();
    void handle_STARTUP_ROS_SCON();
    void onEnter_SETUP_SCON();
    void handle_SETUP_SCON();
    void onEnter_RUN_SCON();
    void handle_RUN_SCON();
    void onExit_RUN_SCON();
    void onEnter_FAULT_SCON();
    void handle_FAULT_SCON();
    void onEnter_FAULT_ROS_SCON();
    void handle_FAULT_ROS_SCON();
};
