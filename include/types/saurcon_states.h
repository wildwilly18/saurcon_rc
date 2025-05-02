#pragma once

enum SaurconState {
    STARTUP_SCON = 0,
    SETUP_SCON,
    RUN_SCON,
    DEBUG_SCON,
    SAFE_SCON,
    FAULT_SCON,
    FAULT_ROS_SCON
};