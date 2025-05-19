#pragma once

#include "driver/ledc.h"

#define MAX_PWM_CHANNELS 6
#define MAX_LEDCTIMERS   4

typedef struct {
    int freq;              // frequency in Hz
    ledc_timer_t timer;    // timer ID (0–3)
    bool used;             //
} LedcTimerInfo;

static LedcTimerInfo ledc_timers[MAX_LEDCTIMERS] = { 0 };
static ledc_timer_t pwm_timer_map[MAX_PWM_CHANNELS]; // CHn -> TIMERn

ledc_timer_t get_or_allocate_ledc_timer(int freq) {

    for (int i = 0; i < MAX_LEDCTIMERS; ++i) {
        if (ledc_timers[i].used && ledc_timers[i].freq == freq)
            return ledc_timers[i].timer;
    }

    for (int i = 0; i < MAX_LEDCTIMERS; ++i) {
        if (!ledc_timers[i].used) {
            ledc_timers[i].used = true;
            ledc_timers[i].freq = freq;
            ledc_timers[i].timer = (ledc_timer_t)i;

            ledc_timer_config_t config = {
                .speed_mode = LEDC_HIGH_SPEED_MODE,
                .duty_resolution = LEDC_TIMER_10_BIT,
                .timer_num = (ledc_timer_t)i,
                .freq_hz = freq,
                .clk_cfg = LEDC_AUTO_CLK,
                .deconfigure = false
            };
            ledc_timer_config(&config);

            return (ledc_timer_t)i;
        }
    }

    return (ledc_timer_t)-1;
}

bool configure_pwm_channel(int ch, int gpio, int freq) {
    ledc_timer_t timer = get_or_allocate_ledc_timer(freq);
    if (timer == (ledc_timer_t)-1)
        return false; // hiba: nincs szabad timer

    ledc_channel_config_t chan_cfg = {
        .gpio_num = gpio,
        .speed_mode = LEDC_HIGH_SPEED_MODE,
        .channel = (ledc_channel_t)ch,
        .intr_type = LEDC_INTR_DISABLE,
        .timer_sel = timer,
        .duty = 0,
        .hpoint = 0,
        .sleep_mode = LEDC_SLEEP_MODE_NO_ALIVE_NO_PD,
        .flags = {
            .output_invert = 0
        }
    };
    if (ledc_channel_config(&chan_cfg) != ESP_OK)
        return false;

    pwm_timer_map[ch] = timer;
    return true;
}

void set_pwm_duty(int ch, uint16_t duty) {
    if (ch < 0 || ch >= MAX_PWM_CHANNELS)
        return;

    duty = (duty > 1023) ? 1023 : duty;

    ledc_set_duty(LEDC_HIGH_SPEED_MODE, (ledc_channel_t)ch, duty);
    ledc_update_duty(LEDC_HIGH_SPEED_MODE, (ledc_channel_t)ch);
}
