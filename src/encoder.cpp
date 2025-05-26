#include "encoder.h"
#include "shared_resources.h"

Encoder* Encoder::instance = nullptr;

Encoder::Encoder(uint8_t a, uint8_t b, uint8_t c)
    : pinA(a), pinB(b), pinC(c), last_hall_state(0),
      motor_position(0), rpm_buffer_index(0), filtered_motor_rpm(0.0f)
{
    instance = this;  // Singleton-style for ISR access
    for (int i = 0; i < RPM_FILTER_SIZE; ++i) rpm_buffer[i] = 0.0f;
}

void Encoder::begin() {
    pinMode(pinA, INPUT_PULLUP);
    pinMode(pinB, INPUT_PULLUP);
    pinMode(pinC, INPUT_PULLUP);

    dt_queue = xQueueCreate(10, sizeof(uint32_t));
    data_mutex = xSemaphoreCreateMutex();

    attachInterrupt(digitalPinToInterrupt(pinA), isrWrapperA, CHANGE);
    attachInterrupt(digitalPinToInterrupt(pinB), isrWrapperB, CHANGE);
    attachInterrupt(digitalPinToInterrupt(pinC), isrWrapperC, CHANGE);

    xTaskCreatePinnedToCore(rpmFilterTask, "encoder_rpm_filter", 4096, this, 1, NULL, 1);
}

float Encoder::getFilteredRPM() {
    if (xSemaphoreTake(data_mutex, pdMS_TO_TICKS(10))) {
        float val = filtered_motor_rpm;
        xSemaphoreGive(data_mutex);
        return val;
    }
    return 0.0f;
}

int Encoder::getPosition() {
    return motor_position;
}

void Encoder::getWheelInfo(float &ang_speed, float &lin_speed, float &dist_traveled) {
    if (xSemaphoreTake(data_mutex, pdMS_TO_TICKS(10))) {
        ang_speed = wheel_angular_speed;
        lin_speed = wheel_linear_speed;
        dist_traveled = wheel_distance_traveled;
    }
    return;
}

void Encoder::resetDistanceTraveled() {
    if (xSemaphoreTake(data_mutex, pdMS_TO_TICKS(10))) {
        wheel_distance_traveled = 0.0f;
    }
}

void Encoder::isrWrapperA() { if (instance) instance->handleISR(); }
void Encoder::isrWrapperB() { if (instance) instance->handleISR(); }
void Encoder::isrWrapperC() { if (instance) instance->handleISR(); }

void Encoder::handleISR() {
    unsigned long now = micros();
    uint8_t a = digitalRead(pinA);
    uint8_t b = digitalRead(pinB);
    uint8_t c = digitalRead(pinC);
    uint8_t hall_state = (a << 2) | (b << 1) | c;

    if (hall_state != last_hall_state) {
        motor_direction = transition_table[last_hall_state][hall_state];

        uint32_t dt = now - last_transition_time;
        last_transition_time = now;
        last_hall_state = hall_state;

        BaseType_t hp_task_woken = pdFALSE;
        xQueueSendFromISR(dt_queue, &dt, &hp_task_woken);
        if (hp_task_woken) portYIELD_FROM_ISR();
    }
}

void Encoder::rpmFilterTask(void* pvParams) {
    Encoder* self = static_cast<Encoder*>(pvParams);
    uint32_t dt;

    while (true) {
        if (xQueueReceive(self->dt_queue, &dt, pdMS_TO_TICKS(100))) {
            float raw_rpm = (dt > 0) ? (60.0f * 1000000.0f) / (dt * ENCODER_TICKS_PER_REV) : 0.0f;
            float dt_sec = dt/1000000.0f;
            self->rpm_buffer[self->rpm_buffer_index++] = raw_rpm * static_cast<float>(self->motor_direction);
            self->rpm_buffer_index %= RPM_FILTER_SIZE;

            float sum = 0.0f;
            for (int i = 0; i < RPM_FILTER_SIZE; ++i) sum += self->rpm_buffer[i];
            float avg = sum / RPM_FILTER_SIZE;

            if (xSemaphoreTake(self->data_mutex, pdMS_TO_TICKS(10))) {
                self->filtered_motor_rpm = avg;
                self->wheel_rpm = avg / GEAR_RATIO;
                self->wheel_angular_speed = self->wheel_rpm * 2.0f * M_PI / 60.0f;
                self->wheel_linear_speed  = self->wheel_angular_speed * WHEEL_RADIUS;
                self->wheel_distance_traveled += (self->wheel_linear_speed * dt_sec);
                xSemaphoreGive(self->data_mutex);
            }
        } else {
            // This acts to clear rpm filter for 0 speed
            for (int i = 0; i < RPM_FILTER_SIZE; ++i) self->rpm_buffer[i] = 0.0f;
            if (xSemaphoreTake(self->data_mutex, pdMS_TO_TICKS(10))) {
                self->filtered_motor_rpm  = 0.0f;
                self->wheel_rpm = 0.0f;
                self->wheel_angular_speed = 0.0f;
                self->wheel_linear_speed = 0.0f;
                self->wheel_distance_traveled = self->wheel_distance_traveled;
                xSemaphoreGive(self->data_mutex);
            }
        }
    }
}