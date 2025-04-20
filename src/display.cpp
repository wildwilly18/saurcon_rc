#include "display.h"

// UI display buffers
static char formattedSetVEL[10];
static char formattedSetSTEER[10];

// FreeRTOS task handles
TaskHandle_t display_update_task_handle = NULL;

// OLED display object
U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, SCL_PIN, SDA_PIN, U8X8_PIN_NONE);

void init_display(){
    // OLED setup
    u8g2.begin();
    u8g2.clearBuffer();
    u8g2.setFont(u8g2_font_t0_12_tf);
    u8g2.drawStr(0, 10, "______SauRCon_______");
    u8g2.drawStr(0, 25, "SET STR: ");
    u8g2.drawStr(0, 40, "SET VEL: ");
    u8g2.drawStr(25, 55, "STATE: START");
    u8g2.sendBuffer();
}

//Display update task
void display_update_task(void *pvParameters)
{
  while(true) {
    if(xSemaphoreTake(controlDataMutex, portMAX_DELAY) == pdTRUE){
      // Format velocityCommand into display string
      sprintf(formattedSetVEL, "%03.4f", velCommand);
      sprintf(formattedSetSTEER, "%03.4f", steerCommand);
      xSemaphoreGive(controlDataMutex); //Release Mutex
    };

    // Update display
    u8g2.clearBuffer();
    u8g2.setFont(u8g2_font_t0_12_tf);
    u8g2.drawStr(0, 10, "______SauRCon_______");
    u8g2.drawStr(0, 25, "SET STR: ");
    u8g2.drawStr(0, 40, "SET VEL: ");
    u8g2.drawStr(85, 25, formattedSetSTEER);      // Your velocity reading (maybe add later)
    u8g2.drawStr(85, 40, formattedSetVEL);   // Incoming throttle value
    u8g2.drawStr(25, 55, "STATE: RUNNING");

    u8g2.sendBuffer();
    
    vTaskDelay(33 / portTICK_PERIOD_MS);
  }
}