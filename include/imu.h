#pragma once

#include <Arduino.h>
#include <Wire.h>
#include <MPU9250.h>

#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>

#include "types/saurcon_parameters.h"
#include "pins.h"

class IMU {
public:
    IMU(uint8_t i2c_addr = 0x68);

    bool begin();
    void update();

    static void imu_update_task(void *param);

    void getAccel(float& ax, float& ay, float& az);
    void getGyro(float& gx, float& gy, float& gz);
    void getMag(float& mx, float& my, float& mz);

private:
    MPU9250 mpu;
    SemaphoreHandle_t imuDataMutex;

    struct {
        float ax, ay, az;
        float gx, gy, gz;
        float mx, my, mz;
    } imuData;

    uint8_t address;

    TaskHandle_t imuTaskHandle = nullptr;

};