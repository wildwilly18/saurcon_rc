#include "imu.h"
#include "shared_resources.h"

IMU::IMU() : mpu(Wire) {
    imuDataMutex = xSemaphoreCreateMutex();
}

// Initializes the IMU. Returns true if initialization is successful and the IMU is ready to use, 
// otherwise returns false if initialization fails.
bool IMU::begin() {
    mpu = MPU6050(Wire);
    bool imuOk;
    byte status = mpu.begin();

    if(status) {
        imuOk = true;

        //Initialize MPU 6050
        mpu.calcGyroOffsets();
        vTaskDelay(pdMS_TO_TICKS(1000));
        
        //Initialize Mag
        mag.init();

        xTaskCreate(imu_update_task, "imu_update_task", 2048, this, 1, &imuTaskHandle);
    } else {
        imuOk = false;
    }

    return imuOk;
}

void IMU::update() {
    mpu.update();

    if(xSemaphoreTake(imuDataMutex, pdMS_TO_TICKS(5))== pdTRUE) {
        imuData.ax = mpu.getAccX();
        imuData.ay = mpu.getAccY();
        imuData.az = mpu.getAccZ();

        imuData.gx = mpu.getGyroX();
        imuData.gy = mpu.getGyroY();
        imuData.gz = mpu.getGyroZ();
        
        mag.read();
        imuData.mx = mag.getX();
        imuData.my = mag.getY();
        imuData.mz = mag.getZ();

        xSemaphoreGive(imuDataMutex);
    }
}   

void IMU::imu_update_task(void *param) {
    //task to update the imu values
    IMU* self = static_cast<IMU*>(param);

    while (true) {
        if (xSemaphoreTake(i2cMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
            self->update();
            xSemaphoreGive(i2cMutex);
        }
        vTaskDelay(pdMS_TO_TICKS(50)); // Add a delay to avoid busy looping
    }    
}

void IMU::getAccel(float& ax, float& ay, float& az) {
    if (xSemaphoreTake(imuDataMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
        ax =  imuData.ay;
        ay = -imuData.ax;
        az =  imuData.az;
        xSemaphoreGive(imuDataMutex);
    }
}

void IMU::getGyro(float& gx, float& gy, float& gz) {
    if (xSemaphoreTake(imuDataMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
        gx =  imuData.gy;
        gy = -imuData.gx;
        gz =  imuData.gz;
        xSemaphoreGive(imuDataMutex);
    }
}

void IMU::getMag(float& mx, float& my, float& mz) {
    if (xSemaphoreTake(imuDataMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
        mx = -imuData.mx;
        my = -imuData.my;
        mz =  imuData.mz;
        xSemaphoreGive(imuDataMutex);
    }
}