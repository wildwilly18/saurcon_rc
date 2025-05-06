#include "state_machine.h"

// Current state
SaurconState saurcon_state;
static SaurconFaults current_fault = NONE;

// Mutex to protect state transitions if needed (optional but good practice)
SemaphoreHandle_t stateMutex = NULL;

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

void StateMachine_SetState(SaurconState nextState) {
    SaurconState currentState;
    // after done, transition to SETUP
    if (stateMutex && xSemaphoreTake(stateMutex, (TickType_t)10) == pdTRUE) {
        currentState = saurcon_state;

        if(nextState != currentState){
            saurcon_state = nextState;
        }

        xSemaphoreGive(stateMutex);
    }
}

void handle_startup() {
    // do startup stuff
    init_display();
    
    if(display_update_task_handle == NULL) {
        xTaskCreate(display_update_task, "display_update_task", 4096, NULL, 1, &display_update_task_handle);
    }

    vTaskDelay(pdMS_TO_TICKS(2000));

    StateMachine_SetState(STARTUP_ROS_SCON);
}

void handle_ros_startup(){
    set_display_state(ROS_STARTUP_DISPLAY);
    digitalWrite(LED_RED, HIGH);
    digitalWrite(LED_GRN, LOW);

    // startup ros
    init_ROS();

    // Create the ROS executor task only after ROS is initialized
    if (ros_executor_task_handle == NULL) {
        xTaskCreate(ros_executor_task, "ros_executor_task", 4096, NULL, 1, &ros_executor_task_handle);
    }
   
    StateMachine_SetState(SETUP_SCON);
}

void handle_setup() {
    set_display_state(STARTUP_DISPLAY);

    init_encoder_mutex();
    init_encoder_isr();

    xTaskCreatePinnedToCore(rpm_filter_task, "rpm_filter_task", 4096, NULL, 1, NULL, 1);

    init_pwm();
    init_servo();
    init_throttle();

    xTaskCreate(task_motion_control, "task_motion_control", 2048, NULL, 2, NULL);

    StateMachine_SetState(RUN_SCON);
}

void handle_run() {
    // normal running mode
    set_display_state(ENCODER_DISPLAY);
}

void handle_fault() {
    // stop motors, blink LEDs, whatever is needed
    set_display_state(FAULT_DISPLAY);
    
}

void handle_ros_fault() {
    set_display_state(FAULT_DISPLAY);
    // Upon startup if ros unable to establish, handle fault here
    vTaskDelay(pdMS_TO_TICKS(3000));
    ESP.restart();
}

void state_machine_task(void *pvParameters) {
    // Create the mutex
    stateMutex = xSemaphoreCreateMutex();

    if (stateMutex == NULL) {
        // Could not create mutex, stay here or log error
        while (1) { vTaskDelay(pdMS_TO_TICKS(1000)); }
    }

    SaurconState previous_state = NO_STATE_SCON;

    while (1) {
        SaurconState local_state;

        // Safely read the current state
        if (xSemaphoreTake(stateMutex, (TickType_t)10) == pdTRUE) {
            local_state = saurcon_state;
            xSemaphoreGive(stateMutex);
        }

        if(previous_state != local_state){
            // State handling
            switch (local_state) {
                case STARTUP_SCON:
                    digitalWrite(LED_RED, LOW);
                    digitalWrite(LED_GRN, LOW);
                    handle_startup();
                    break;
                case STARTUP_ROS_SCON:
                    digitalWrite(LED_GRN, LOW);
                    handle_ros_startup();
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
        }

        // sleep for a little bit to avoid hogging CPU
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}