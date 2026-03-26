// encoder.h
#pragma once

#include <Arduino.h>
#include "driver/pcnt.h"
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/semphr.h>

#define RPM_FILTER_SIZE 10

class Encoder {
public:
    Encoder();
    void begin();
    float getAngularWheelSpeed();
    float getLinearSpeed();
    bool isStopped(){return stopped;};
    void update();
    static void enc_update_task(void *param);
    void resetDistanceTraveled(){wheel_distance_traveled = 0.0;};

private:
    static Encoder* instance;
    SemaphoreHandle_t enc_data_mutex;

    TaskHandle_t encTaskHandle = nullptr;

    int16_t enc_count;
    int16_t enc_count_prev;

    uint32_t last_time_ms_{0};

    bool stopped{true};

    float wheel_rpm{0.0};
    float wheel_angular_speed{0.0};     // rad/s
    float wheel_linear_speed{0.0};      // m/s
    float wheel_distance_traveled{0.0}; // m
};