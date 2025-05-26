#include "imu.h"
#include "shared_resources.h"

IMU::IMU(uint8_t i2c_addr) : address(i2c_addr) {
    imuDataMutex = xSemaphoreCreateMutex();
}

// Initializes the IMU. Returns true if initialization is successful and the IMU is ready to use, 
// otherwise returns false if initialization fails.
bool IMU::begin() {
    bool status = mpu.setup(address);

    if(status){
        xTaskCreate(imu_update_task, "imu_update_task", 2048, this, 1, &imuTaskHandle);
    }

    return status;
}

void IMU::update() {
    //if(!mpu.update()) return;
    bool ok = mpu.update();

    if(ok){
        if(xSemaphoreTake(imuDataMutex, pdMS_TO_TICKS(5))== pdTRUE) {
            imuData.ax = mpu.getAccX();
            imuData.ay = mpu.getAccY();
            imuData.az = mpu.getAccZ();

            imuData.gx = mpu.getGyroX();
            imuData.gy = mpu.getGyroY();
            imuData.gz = mpu.getGyroZ();

            imuData.mx = mpu.getMagX();
            imuData.my = mpu.getMagY();
            imuData.mz = mpu.getMagZ();

            xSemaphoreGive(imuDataMutex);
        }
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
        ax = imuData.ax;
        ay = imuData.ay;
        az = imuData.az;
        xSemaphoreGive(imuDataMutex);
    }
}

void IMU::getGyro(float& gx, float& gy, float& gz) {
    if (xSemaphoreTake(imuDataMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
        gx = imuData.gx;
        gy = imuData.gy;
        gz = imuData.gz;
        xSemaphoreGive(imuDataMutex);
    }
}

void IMU::getMag(float& mx, float& my, float& mz) {
    if (xSemaphoreTake(imuDataMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
        mx = imuData.mx;
        my = imuData.my;
        mz = imuData.mz;
        xSemaphoreGive(imuDataMutex);
    }
}

void IMU::getQuaternion(float& qx, float& qy, float& qz, float& qw) {
    if (xSemaphoreTake(imuDataMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
        qx = mpu.getQuaternionX();
        qy = mpu.getQuaternionY();
        qz = mpu.getQuaternionZ();
        qw = mpu.getQuaternionW();
        xSemaphoreGive(imuDataMutex);
    }
}