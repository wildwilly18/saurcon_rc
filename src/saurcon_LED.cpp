#include "saurcon_LED.h"

#define BLINK_SLOW_MS 1000
#define BLINK_FAST_MS  333

LEDManager::LEDManager()
    : red{LED_RED, LEDState::OFF, LOW, 0, 0},
      green{LED_GRN, LEDState::OFF, LOW, 0, 0},
      yellow{LED_YEL, LEDState::OFF, LOW, 0, 0},
      ledTaskHandle(nullptr)
{}

void LEDManager::init() {
    pinMode(red.pin, OUTPUT);
    pinMode(green.pin, OUTPUT);
    pinMode(yellow.pin, OUTPUT);

    xTaskCreate(led_update_task, "led_update_task", 2048, this, 1, &ledTaskHandle);
}

void LEDManager::setLEDState(LEDState r, LEDState g, LEDState y) {
    red.state = r;
    green.state = g;
    yellow.state = y;

    red.blinkInterval = (r == LEDState::BLINK_FAST) ? pdMS_TO_TICKS(BLINK_FAST_MS) :
                        (r == LEDState::BLINK_SLOW) ? pdMS_TO_TICKS(BLINK_SLOW_MS) : 0;

    green.blinkInterval = (g == LEDState::BLINK_FAST) ? pdMS_TO_TICKS(BLINK_FAST_MS) :
                          (g == LEDState::BLINK_SLOW) ? pdMS_TO_TICKS(BLINK_SLOW_MS) : 0;

    yellow.blinkInterval = (y == LEDState::BLINK_FAST) ? pdMS_TO_TICKS(BLINK_FAST_MS) :
                           (y == LEDState::BLINK_SLOW) ? pdMS_TO_TICKS(BLINK_SLOW_MS) : 0;
}

void LEDManager::led_update_task(void *param) {
    LEDManager* self = static_cast<LEDManager*>(param);
    TickType_t lastWake = xTaskGetTickCount();
    const TickType_t loopDelay = pdMS_TO_TICKS(50);

    while(true) {
        auto update = [](Led &led) {
            TickType_t now = xTaskGetTickCount();
            switch(led.state) {
                case LEDState::ON:
                    digitalWrite(led.pin, HIGH);
                    led.currentLevel = HIGH;
                    break;

                case LEDState::OFF:
                    digitalWrite(led.pin, LOW);
                    led.currentLevel = LOW;
                    break;
                case LEDState::BLINK_SLOW:
                case LEDState::BLINK_FAST:
                    if(now - led.lastToggle >= led.blinkInterval) {
                        led.currentLevel = !led.currentLevel;
                        digitalWrite(led.pin, led.currentLevel);
                        led.lastToggle = now;
                    }
                    break;
            }
        };

        update(self->red);
        update(self->green);
        update(self->yellow);

        vTaskDelayUntil(&lastWake, loopDelay);
    }
}