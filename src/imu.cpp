#include "imu.h"

IMU::IMU(uint8_t i2c_addr) : address(i2c_addr) {
    imuDataMutex = xSemaphoreCreateMutex();
}

bool IMU::begin() {
    Wire.begin(); // Adjust SDA/SCL pins as needed
    vTaskDelay(pdMS_TO_TICKS(2000));
    
    bool status = mpu.setup(address);

    if(status){
        xTaskCreate(imu_update_task, "imu_update_task", 2048, this, 1, &imuTaskHandle);
    }
    
    return status;
}

void IMU::update() {
    if(!mpu.update()) return;

    if(xSemaphoreTake(imuDataMutex, (TickType_t)10) == pdTRUE) {
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

void IMU::imu_update_task(void *param) {
    //task to update the imu values
    IMU* self = static_cast<IMU*>(param);

    while(true){
        self->update();

        vTaskDelay(pdMS_TO_TICKS(10)); //100 hz 
    }
    
}

void IMU::getAccel(float& ax, float& ay, float& az) {
    if (xSemaphoreTake(imuDataMutex, (TickType_t)10) == pdTRUE) {
        ax = imuData.ax;
        ay = imuData.ay;
        az = imuData.az;
        xSemaphoreGive(imuDataMutex);
    }
}

void IMU::getGyro(float& gx, float& gy, float& gz) {
    if (xSemaphoreTake(imuDataMutex, (TickType_t)10) == pdTRUE) {
        gx = imuData.gx;
        gy = imuData.gy;
        gz = imuData.gz;
        xSemaphoreGive(imuDataMutex);
    }
}

void IMU::getMag(float& mx, float& my, float& mz) {
    if (xSemaphoreTake(imuDataMutex, (TickType_t)10) == pdTRUE) {
        mx = imuData.mx;
        my = imuData.my;
        mz = imuData.mz;
        xSemaphoreGive(imuDataMutex);
    }
}