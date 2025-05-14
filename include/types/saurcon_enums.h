#pragma once

// Robot operating state machine states
enum SaurconState {
    NO_STATE_SCON = 0,     // Initial or invalid state
    STARTUP_SCON,          // Power-on init
    STARTUP_ROS_SCON,      // ROS2 initialization
    SETUP_SCON,            // Motor and encoder setup
    RUN_SCON,              // Normal operation
    FAULT_SCON,            // General fault state
    FAULT_ROS_SCON         // ROS-specific fault
};

// Display rendering states
enum DisplayState {
    OFF_DISPLAY,
    STARTUP_DISPLAY,
    ROS_STARTUP_DISPLAY,
    RUN_DISPLAY,
    ENCODER_DISPLAY,
    DEBUG_DISPLAY,
    INIT_DISPLAY,
    FAULT_DISPLAY
};

// Fault types
enum SaurconFaults {
    NONE = 0,                  // No fault
    GENERIC_FAULT,            // General error
    ROS_CONNECTION_LOSS       // Lost comms with ROS
};