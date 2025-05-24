#pragma once

// Robot operating state machine states
enum class SaurconState : uint8_t {
    NO_STATE = 0,     // Initial or invalid state
    STARTUP,          // Power-on init
    STARTUP_ROS,          // ROS2 initialization
    SETUP,            // Motor and encoder setup
    STANDBY,          // Startup complete wait for action
    RUN_CONTROL,      // Operation via controller
    RUN_AUTONOMOUS,   // Operation via raspi
    FAULT,            // General fault state
    FAULT_ROS         // ROS-specific fault
};

// Display rendering states
enum DisplayState {
    OFF_DISPLAY,
    STARTUP_DISPLAY,
    ROS_STARTUP_DISPLAY,
    RUN_DISPLAY,
    ENCODER_DISPLAY,
    IMU_DISPLAY,
    DEBUG_DISPLAY,
    INIT_DISPLAY,
    FAULT_DISPLAY
};

// Display LED states
enum class LEDState {
    OFF = 0,
    ON,
    BLINK_FAST,
    BLINK_SLOW
};

// Fault types
enum SaurconFaults {
    NONE = 0,                  // No fault
    GENERIC_FAULT,            // General error
    IMU_START_FAULT,          // IMU Failed Setup
    ROS_CONNECTION_LOSS       // Lost comms with ROS
};