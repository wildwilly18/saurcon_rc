#include "display.h"

// UI display buffers
static char formattedSetVEL[10];
static char formattedSetSTEER[10];
static char formattedRPM[10];
static char formattedVEL[10];
static char faultSTR[10];

// Display State
DisplayState currentDisplayState = STARTUP_DISPLAY;

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
    u8g2.drawStr(25, 55, "STATE: INIT");
    u8g2.sendBuffer();
}

void set_display_state(DisplayState state){
  //Probably safe like this, no need for a semaphore.
  currentDisplayState = state;
}

//Display update task
void display_update_task(void *pvParameters)
{
  while(true) {

    switch(currentDisplayState) {
      case RUN_DISPLAY:
        // In run state look at control data mutex for the vel and steer data. Info here may change.
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
        u8g2.drawStr(25, 55, "STATE: RUN");
        
        break;

      case ENCODER_DISPLAY:
        //In Encoder State look at encoder data mutex for the rpm and velocity
        if(xSemaphoreTake(encoderDataMutex, portMAX_DELAY) == pdTRUE){
          // Format velocityCommand into display string
          sprintf(formattedRPM, "%03.4f", filteredRPM);
          sprintf(formattedVEL, "%03.4f", filteredRPM);
          xSemaphoreGive(encoderDataMutex); //Release Mutex
        }

        // Update display
        u8g2.clearBuffer();
        u8g2.setFont(u8g2_font_t0_12_tf);
        u8g2.drawStr(0, 10, "______SauRCon_______");
        u8g2.drawStr(0, 25, "ENC RPM: ");
        u8g2.drawStr(0, 40, "ENC VEL: ");
        u8g2.drawStr(85, 25, formattedRPM);      // Your velocity reading (maybe add later)
        u8g2.drawStr(85, 40, formattedVEL);   // Incoming throttle value
        u8g2.drawStr(25, 55, "STATE: ENCODER");
        
        break;

      case FAULT_DISPLAY:
        SaurconFaults fault = ROS_CONNECTION_LOSS;

        sprintf(faultSTR, "%d", fault);
        // Update display
        u8g2.clearBuffer();
        u8g2.setFont(u8g2_font_t0_12_tf);
        u8g2.drawStr(0, 10, "______SauRCon_______");
        u8g2.drawStr(0, 25, "FAULT: ");
        u8g2.drawStr(0, 40, "RESET: ");
        u8g2.drawStr(85, 25, faultSTR);      // Your velocity reading (maybe add later)
        u8g2.drawStr(85, 40, faultSTR);   // Incoming throttle value
        u8g2.drawStr(25, 55, "STATE: FAULTED");

        break;
    }

    u8g2.sendBuffer();
    
    vTaskDelay(100 / portTICK_PERIOD_MS);
  }
}