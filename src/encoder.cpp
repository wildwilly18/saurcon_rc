#include "encoder.h"

uint8_t last_hall_state = 0;
int motor_position = 0;
unsigned long last_transition_time;
float raw_motor_rpm;


// Queues
QueueHandle_t dt_queue; // Queue to hold dt measurements

// Filtered RPM (Global so it can be accessed elsewhere)
float filtered_motor_rpm;

//Moving Average Filter Buffer
float rpm_buffer[RPM_FILTER_SIZE] = {0};
uint8_t rpm_buffer_index;

// Lookup table for transition -> direction
// Initialized with placeholder values; update as needed.
const int8_t enc_transition_table[8][8] = {
    //  0  1  2  3  4  5  6  7 (new state)
    {  0, 1,-1, 0,-1, 0, 0, 0 }, // old 0
    { -1, 0, 0, 1, 0, 0, 0,-1 }, // old 1
    {  1, 0, 0,-1, 0, 0, 1, 0 }, // old 2
    {  0,-1, 1, 0, 0, 1, 0, 0 }, // old 3
    {  1, 0, 0, 0, 0,-1, 0, 1 }, // old 4
    {  0, 0, 0,-1, 1, 0,-1, 0 }, // old 5
    {  0, 0,-1, 0, 0, 1, 0, 1 }, // old 6
    {  0, 1, 0, 0,-1, 0,-1, 0 }, // old 7
};

void init_encoder_isr(){
    pinMode(A_PIN, INPUT_PULLUP);
    pinMode(B_PIN, INPUT_PULLUP);
    pinMode(C_PIN, INPUT_PULLUP);

    // Create dt queur (holds 20 entries, each uint32_t size)
    dt_queue = xQueueCreate(20, sizeof(uint32_t));
    if (dt_queue == NULL) {
        while(1); // Stop, need debugging of sorts here
    }

    //Attatch Interrupts for the pins
    attachInterrupt(digitalPinToInterrupt(A_PIN), enc_isr, CHANGE);
    attachInterrupt(digitalPinToInterrupt(B_PIN), enc_isr, CHANGE);
    attachInterrupt(digitalPinToInterrupt(C_PIN), enc_isr, CHANGE);
}

void IRAM_ATTR enc_isr(){
    unsigned long now = micros();

    uint8_t a = digitalRead(A_PIN);
    uint8_t b = digitalRead(B_PIN);
    uint8_t c = digitalRead(C_PIN);

    uint8_t hall_state = (a << 2) | (b << 1) | (c << 0);

    if (hall_state != last_hall_state) {
        int8_t direction = enc_transition_table[last_hall_state][hall_state];

        if(direction == 1){
            motor_position++;
        } else if(direction == -1) {
            motor_position--;
        }

        //Calculate time difference
        uint32_t dt = now - last_transition_time; // u_seconds

        //Push dt into the queue
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        xQueueSendFromISR(dt_queue, &dt, &xHigherPriorityTaskWoken);

        last_transition_time = now;
        last_hall_state = hall_state;

        if (xHigherPriorityTaskWoken) {
            portYIELD_FROM_ISR();
        }
    }
}

void rpm_filter_task(void *pvParameters){
    uint32_t dt;
    float raw_rpm;

    for(;;){
        if(xQueueReceive(dt_queue, &dt, portMAX_DELAY) == pdTRUE){
            if (dt > 0) {
                raw_rpm = 10000000.0f / (float)dt; // 10,000,000 / Δt

                // Update moving average buffer
                rpm_buffer[rpm_buffer_index] = raw_rpm;
                rpm_buffer_index = (rpm_buffer_index + 1) % RPM_FILTER_SIZE;

                //Calculate the filtered RPM
                float sum = 0.0f;
                for(int i = 0; i < RPM_FILTER_SIZE; i++) {
                    sum += rpm_buffer[i];
                }
                filtered_motor_rpm = sum / RPM_FILTER_SIZE;

                //PRINT OUTPUT, NO SERIAL RIGHT NOW.
            }
        }
    }
}