#pragma once

enum SaurconState {
    STARTUP_SCON = 0,
    STARTUP_ROS_SCON,
    SETUP_SCON,
    RUN_SCON,
    DEBUG_SCON,
    SAFE_SCON,
    FAULT_SCON,
    FAULT_ROS_SCON,
    NO_STATE_SCON
};