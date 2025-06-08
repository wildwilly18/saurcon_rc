#include "display.h"
#include "shared_resources.h"

DisplayManager::DisplayManager() : currentDisplayState(STARTUP_DISPLAY) {
    sprintf(formattedSetVEL, "0.0000");
    sprintf(formattedSetSTEER, "0.0000");
    sprintf(formattedRPM, "0.0000");
    sprintf(formattedVEL, "0.0000");
    sprintf(faultSTR, "0");
    display_update_task_handle = NULL;
}

void DisplayManager::init() {
    u8g2.begin();
    u8g2.clearBuffer();
    u8g2.setFont(u8g2_font_t0_12_tf);
    u8g2.drawStr(0, 10, "______SauRCon_______");
    u8g2.drawStr(0, 25, "SET STR: ");
    u8g2.drawStr(0, 40, "SET VEL: ");
    u8g2.drawStr(25, 55, "STATE: INIT");
    u8g2.sendBuffer();
    setState(STARTUP_DISPLAY);
}

void DisplayManager::setState(DisplayState state) {
    currentDisplayState = state;
}

void DisplayManager::startTask() {
    xTaskCreate(DisplayManager::display_update_task, "display_update_task", 4096, this, 1, &display_update_task_handle);
}

void DisplayManager::display_update_task(void *pvParameters) {
    DisplayManager* display = static_cast<DisplayManager*>(pvParameters);
    while (true) {
        if(xSemaphoreTake(i2cMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
            display->u8g2.clearBuffer();
            display->u8g2.setFont(u8g2_font_t0_12_tf);

            switch (display->currentDisplayState) {
                case RUN_DISPLAY:
                    if (xSemaphoreTake(controlDataMutex, portMAX_DELAY) == pdTRUE) {
                        sprintf(display->formattedSetVEL, "%03.4f", velCommand);
                        sprintf(display->formattedSetSTEER, "%03.4f", steerCommand);
                        xSemaphoreGive(controlDataMutex);
                    }
                    display->u8g2.drawStr(0, 10, "______SauRCon_______");
                    display->u8g2.drawStr(0, 25, "SET STR: ");
                    display->u8g2.drawStr(0, 40, "SET VEL: ");
                    display->u8g2.drawStr(85, 25, display->formattedSetSTEER);
                    display->u8g2.drawStr(85, 40, display->formattedSetVEL);
                    display->u8g2.drawStr(25, 55, "STATE: RUN");
                    break;

                case IMU_DISPLAY:
                    float ax, ay, az;
                    float gx, gy, gz;

                    imu->getAccel(ax, ay, az);
                    imu->getGyro(gx, gy, gz);
                
                    display->u8g2.drawStr(0, 10, "______SauRCon_______");
                    display->u8g2.drawStr(0, 25, "AZ:");
                    display->u8g2.drawStr(0, 40, "GZ:");
                    // Format values if needed
                    char azStr[12], gzStr[12];
                    dtostrf(az, 6, 4, azStr);
                    dtostrf(gz, 6, 4, gzStr);
                    display->u8g2.drawStr(35, 25, azStr);
                    display->u8g2.drawStr(35, 40, gzStr);
                    display->u8g2.drawStr(25, 55, "STATE: IMU");
                    break;
                    

                case ENCODER_DISPLAY:
                    float ang_spd, lin_spd, dist;
                    encoder->getWheelInfo(ang_spd, lin_spd, dist);

                    sprintf(display->formattedRPM, "%03.4f", ang_spd);
                    sprintf(display->formattedVEL, "%03.4f", lin_spd);

                    display->u8g2.drawStr(0, 10, "______SauRCon_______");
                    display->u8g2.drawStr(0, 25, "ENC RPM: ");
                    display->u8g2.drawStr(0, 40, "ENC VEL: ");
                    display->u8g2.drawStr(85, 25, display->formattedRPM);
                    display->u8g2.drawStr(85, 40, display->formattedVEL);
                    display->u8g2.drawStr(25, 55, "STATE: ENCODER");
                    break;

                case FAULT_DISPLAY:
                    sprintf(display->faultSTR, "%d", ROS_CONNECTION_LOSS);
                    display->u8g2.drawStr(0, 10, "______SauRCon_______");
                    display->u8g2.drawStr(0, 25, "FAULT: ");
                    display->u8g2.drawStr(0, 40, "RESET: ");
                    display->u8g2.drawStr(85, 25, display->faultSTR);
                    display->u8g2.drawStr(85, 40, display->faultSTR);
                    display->u8g2.drawStr(25, 55, "STATE: FAULTED");
                    break;

                case STARTUP_DISPLAY:
                    display->u8g2.drawStr(0, 10, "______SauRCon_______");
                    display->u8g2.drawStr(0, 25, "STARTING UP SAURCON");
                    display->u8g2.drawStr(0, 40, "PLEASE BE PATIENT");
                    display->u8g2.drawStr(20, 55, "STATE: STARTUP");
                    break;

                case SETUP_DISPLAY:
                    display->u8g2.drawStr(0, 10, "______SauRCon_______");
                    display->u8g2.drawStr(0, 25, "SETTING UP SAURCON");
                    display->u8g2.drawStr(0, 40, "PLEASE BE PATIENT");
                    display->u8g2.drawStr(20, 55, "STATE: SETUP");
                    break;

                case STANDBY_DISPLAY:
                    display->u8g2.drawStr(0, 10, "______SauRCon_______");
                    display->u8g2.drawStr(0, 25, "IN STANDBY");
                    display->u8g2.drawStr(0, 40, "WAITING PATIENTLY");
                    display->u8g2.drawStr(20, 55, "STATE: STANDBY");
                    break;

                case ROS_STARTUP_DISPLAY:
                    display->u8g2.drawStr(0, 10, "______SauRCon_______");
                    display->u8g2.drawStr(0, 25, "SETTING UP ROS");
                    display->u8g2.drawStr(0, 40, "BE PATIENT WIZARD");
                    display->u8g2.drawStr(0, 55, "STATE: ROS START");
                    break;

                default:
                    display->u8g2.drawStr(0, 10, "______SauRCon_______");
                    display->u8g2.drawStr(0, 25, "WHERE AM I");
                    display->u8g2.drawStr(0, 40, "PLEASE HELP ME");
                    display->u8g2.drawStr(25, 55, "STATE: UNKNOWN");
                    break;
            }


                display->u8g2.sendBuffer();

            xSemaphoreGive(i2cMutex);
        };
        vTaskDelay(pdMS_TO_TICKS(200));
    }
}
