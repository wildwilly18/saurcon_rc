// encoder.h
#pragma once

#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/semphr.h>

#define RPM_FILTER_SIZE 10

class Encoder {
public:
    Encoder(uint8_t pinA, uint8_t pinB, uint8_t pinC);
    void begin();
    float getFilteredRPM();
    void getWheelInfo(float &ang_speed, float &lin_speed, float &dist_traveled);
    int getPosition();
    void resetDistanceTraveled();

private:
    static void isrWrapperA();
    static void isrWrapperB();
    static void isrWrapperC();
    void handleISR();

    static Encoder* instance;

    uint8_t pinA, pinB, pinC;
    volatile uint8_t last_hall_state;
    volatile int motor_position;
    volatile int motor_direction;
    volatile unsigned long last_transition_time;
    volatile float raw_motor_rpm;

    float rpm_buffer[RPM_FILTER_SIZE];
    uint8_t rpm_buffer_index;

    QueueHandle_t dt_queue;
    SemaphoreHandle_t data_mutex;

    static void rpmFilterTask(void* pvParams);

    float filtered_motor_rpm;
    float filtered_motor_rad_sec;

    float wheel_rpm;
    float wheel_angular_speed;     // rad/s
    float wheel_linear_speed;      // m/s
    float wheel_distance_traveled; // m

    const int8_t transition_table[8][8] = {
        { 0, 1, -1, 0, -1, 0, 0, 0 },
        { -1, 0, 0, 1, 0, 0, 0, -1 },
        { 1, 0, 0, -1, 0, 0, 1, 0 },
        { 0, -1, 1, 0, 0, 1, 0, 0 },
        { 1, 0, 0, 0, 0, -1, 0, 1 },
        { 0, 0, 0, -1, 1, 0, -1, 0 },
        { 0, 0, -1, 0, 0, 1, 0, 1 },
        { 0, 1, 0, 0, -1, 0, -1, 0 },
    };
};