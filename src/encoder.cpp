#include "encoder.h"
#include "shared_resources.h"

Encoder* Encoder::instance = nullptr;

Encoder::Encoder()
{
    instance = this;

    // Set up the pulse counter
    pcnt_config_t pcnt_config = {
        .pulse_gpio_num = MOT_ENC_1,         // Channel A
        .ctrl_gpio_num  = MOT_ENC_2,         // Channel B
        .lctrl_mode     = PCNT_MODE_REVERSE, // Reverse Count direction when B is low
        .hctrl_mode     = PCNT_MODE_KEEP,    // Keep count directio when B is high
        .pos_mode       = PCNT_COUNT_DEC,    // Count down on rising edge of A
        .neg_mode       = PCNT_COUNT_INC,    // Count up on falling edge of A
        .counter_h_lim  = 32767,
        .counter_l_lim  = -32768,
        .unit           = PCNT_UNIT_0,
        .channel        = PCNT_CHANNEL_0,
    };
    pcnt_unit_config(&pcnt_config);

    // Configure channel 1 for full quadrature resoltuion (4x counting)
    pcnt_config.pulse_gpio_num = MOT_ENC_2;
    pcnt_config.ctrl_gpio_num  = MOT_ENC_1;
    pcnt_config.channel        = PCNT_CHANNEL_1;
    pcnt_config.pos_mode       = PCNT_COUNT_INC; // Swapped
    pcnt_config.neg_mode       = PCNT_COUNT_DEC; // Swapped
    pcnt_unit_config(&pcnt_config);

    // Set filter to ignore pulses shorter than 100 APB clock cycles
    pcnt_set_filter_value(PCNT_UNIT_0, 100);
    pcnt_filter_enable(PCNT_UNIT_0);

    // Initialize and start counter
    pcnt_counter_pause(PCNT_UNIT_0);
    pcnt_counter_clear(PCNT_UNIT_0);
    pcnt_counter_resume(PCNT_UNIT_0);
}

void Encoder::begin() {
    enc_data_mutex = xSemaphoreCreateMutex();

    // Initialize Encode task
    xTaskCreatePinnedToCore(
        enc_update_task, 
        "enc_update_task", 
        2048, 
        this, 
        1, 
        &encTaskHandle,
        1);
}

void Encoder::enc_update_task(void *param){
    Encoder* self = static_cast<Encoder*>(param);

    while(true){
        if(xSemaphoreTake(self->enc_data_mutex, pdMS_TO_TICKS(1)) == pdTRUE){
            self->update();
            xSemaphoreGive(self->enc_data_mutex);
        }
        vTaskDelay(pdMS_TO_TICKS(10)); // Add a delay to avoid busy loops
    }
}

void Encoder::update(){
    // Update Loop. Mutex protected
    int16_t current_enc_cnt{0};
    pcnt_get_counter_value(PCNT_UNIT_0, &current_enc_cnt);

    uint32_t current_time_ms = pdTICKS_TO_MS(xTaskGetTickCount());

    float hall_rpm{0};
    float linear_speed{0.0};

    if(last_time_ms_ > 0){
        // Calculate the time difference
        uint32_t dt = current_time_ms - last_time_ms_;
        if(dt == 0) return;

        int32_t delta_counts = (int32_t)current_enc_cnt - (int32_t)enc_count_prev;

        //Handle Integer wrap
        if(enc_count_prev > 16000 && delta_counts < -16000){
            delta_counts = (32767 - (int32_t)enc_count_prev) + 1 + (int32_t)current_enc_cnt;
        }

        else if(enc_count_prev < -16000 && delta_counts > 16000){
            delta_counts = (-32768 - (int32_t)enc_count_prev) - 1 + (int32_t)current_enc_cnt;
        }
        
        hall_rpm = (delta_counts * 60000.0f) / (ENCODER_TICKS_PER_REV * (float)dt);
    }

    // Linear Speed
    linear_speed = (hall_rpm * 3.14159265358979323846f * WHEEL_RADIUS) / 60.0f;

    wheel_rpm = hall_rpm;
    wheel_linear_speed = linear_speed;
    
    enc_count_prev = current_enc_cnt;
    last_time_ms_ = current_time_ms;
}