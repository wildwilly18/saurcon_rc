#include "state_machine.h"

// Current state
SaurconState saurcon_state;
static SaurconFaults current_fault = NONE;

// Mutex to protect state transitions if needed (optional but good practice)
static SemaphoreHandle_t stateMutex = NULL;

void StateMachine_SetFault(SaurconFaults fault) {
    //Receive fault, set the current state based on fault.
    if (stateMutex && xSemaphoreTake(stateMutex, (TickType_t)10) == pdTRUE) {
        if(fault == ROS_CONNECTION_LOSS){
            saurcon_state = FAULT_ROS_SCON;
        }   else {
            saurcon_state = FAULT_SCON;
        }
        xSemaphoreGive(stateMutex);
    }
}

void handle_startup() {
    // do startup stuff
    set_display_state(STARTUP_DISPLAY);

    // after done, transition to SETUP
    if (stateMutex && xSemaphoreTake(stateMutex, (TickType_t)10) == pdTRUE) {
        saurcon_state = RUN_SCON;
        xSemaphoreGive(stateMutex);
    }

    //init_pwm();
    //init_servo();

    //vTaskDelay(pdMS_TO_TICKS(5000));

    //saurcon_state = RUN_SCON;
}

void handle_setup() {
    set_display_state(STARTUP_DISPLAY);
    // initialize sensors, hardware, etc
    bool setup_success = true; // pretend we checked things

    if (stateMutex && xSemaphoreTake(stateMutex, (TickType_t)10) == pdTRUE) {
        saurcon_state = setup_success ? RUN_SCON : FAULT_SCON;
        xSemaphoreGive(stateMutex);
    }
}

void handle_run() {
    // normal running mode
    set_display_state(RUN_DISPLAY);
}

void handle_fault() {
    // stop motors, blink LEDs, whatever is needed
    set_display_state(FAULT_DISPLAY);
    
}

void handle_ros_fault() {
    set_display_state(FAULT_DISPLAY);
    // Upon startup if ros unable to establish, handle fault here
    vTaskDelay(pdMS_TO_TICKS(10000));
    ESP.restart();
}

void state_machine_task(void *pvParameters) {
    // Create the mutex
    stateMutex = xSemaphoreCreateMutex();

    if (stateMutex == NULL) {
        // Could not create mutex, stay here or log error
        while (1) { vTaskDelay(pdMS_TO_TICKS(1000)); }
    }

    while (1) {
        SaurconState local_state;

        // Safely read the current state
        if (xSemaphoreTake(stateMutex, (TickType_t)10) == pdTRUE) {
            local_state = saurcon_state;
            xSemaphoreGive(stateMutex);
        }

        // State handling
        switch (local_state) {
            case STARTUP_SCON:
                handle_startup();
                break;
            case SETUP_SCON:
                handle_setup();
                break;
            case RUN_SCON:
                digitalWrite(LED_RED, LOW);
                digitalWrite(LED_GRN, HIGH);
                handle_run();
                break;
            case FAULT_SCON:
                handle_fault();
                break;
            case FAULT_ROS_SCON:
                digitalWrite(LED_RED, HIGH);
                digitalWrite(LED_GRN, LOW);
                handle_ros_fault();;
                break;
            default:
                // shouldn't happen
                break;
        }

        // sleep for a little bit to avoid hogging CPU
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}