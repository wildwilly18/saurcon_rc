#pragma once

#include <Arduino.h>
#include <freertos/FreeRTOS.h>

#include "types/saurcon_enums.h"
#include "pins.h"

class LEDManager {
    public:
        LEDManager();

        void init();
        void setLEDState(LEDState r, LEDState g, LEDState y);

        static void led_update_task(void *param);

    private:
        struct Led {
            uint8_t pin;
            LEDState state;
            bool currentLevel;
            TickType_t lastToggle;
            TickType_t blinkInterval;
        };

        Led red;
        Led green;
        Led yellow;

        TaskHandle_t ledTaskHandle;
};