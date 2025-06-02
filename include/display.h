#pragma once

//Include FreeRTOS
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

// Arduino and Display Library Dependencies
#include <Arduino.h>
#include <U8g2lib.h>
#include "pins.h"

#include "types/saurcon_enums.h"
#include "shared_resources.h"

class DisplayManager {
    public:
        DisplayManager();
    
        void init();
        void setState(DisplayState state);
        void setFault(SaurconFaults fault);
        void startTask();

        static void display_update_task(void *param);
    
    private:
        DisplayState currentDisplayState;
        TaskHandle_t display_update_task_handle;
        SaurconFaults currentFault = SaurconFaults::NONE;
    
        char formattedSetVEL[10];
        char formattedSetSTEER[10];
        char formattedRPM[10];
        char formattedVEL[10];
        char faultSTR[10];
    
        U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2 =
            U8G2_SSD1306_128X64_NONAME_F_HW_I2C(U8G2_R0, SCL_PIN, SDA_PIN, U8X8_PIN_NONE);
    };