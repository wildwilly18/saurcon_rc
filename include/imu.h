#pragma once

#include <Arduino.h>
#include <Wire.h>

#include <MPU6050_light.h>
#include <QMC5883LCompass.h>

#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>

#include "types/saurcon_parameters.h"
#include "pins.h"

class IMU {
public:
    IMU();

    bool begin();
    void update();
    static void imu_update_task(void *param);

    void getAccel(float& ax, float& ay, float& az);
    void getGyro(float& gx, float& gy, float& gz);

private:
    MPU6050 mpu;
    SemaphoreHandle_t imuDataMutex;

    struct {
        float ax, ay, az;
        float gx, gy, gz;
    } imuData;

    TaskHandle_t imuTaskHandle = nullptr;

};