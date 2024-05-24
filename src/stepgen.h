#pragma once

#include "globals.h"
#include "hardware.h"

#include <stdint.h>

#include "driver/gpio.h"
#include "driver/timer.h"

#include "soc/timer_group_struct.h"

// #define timer_0_set_alarm_value(alarm_val) timer_group_set_alarm_value_in_isr(TIMER_GROUP_0, TIMER_0, alarm_val)
// #define timer_0_set_alarm_value(alarm_val) TIMERG0.hw_timer[0].alarmlo.tx_alarm_lo = alarm_val
#define timer_0_set_alarm_value(alarm_val) TIMERG0.hw_timer[0].alarm_low = alarm_val
// #define timer_0_clear_interrupt timer_group_clr_intr_status_in_isr(TIMER_GROUP_0, TIMER_0)
// #define timer_0_clear_interrupt TIMERG0.int_clr_timers.t0_int_clr = 1
#define timer_0_clear_interrupt TIMERG0.int_clr_timers.t0 = 1
// #define timer_0_enable_alarm timer_group_enable_alarm_in_isr(TIMER_GROUP_0, TIMER_0)
// #define timer_0_enable_alarm TIMERG0.hw_timer[0].config.tx_alarm_en = 1
#define timer_0_enable_alarm TIMERG0.hw_timer[0].config.alarm_en = 1

// #define timer_1_set_alarm_value(alarm_val) timer_group_set_alarm_value_in_isr(TIMER_GROUP_0, TIMER_1, alarm_val)
// #define timer_1_set_alarm_value(alarm_val) TIMERG0.hw_timer[1].alarmlo.tx_alarm_lo = alarm_val
#define timer_1_set_alarm_value(alarm_val) TIMERG0.hw_timer[1].alarm_low = alarm_val
// #define timer_1_clear_interrupt timer_group_clr_intr_status_in_isr(TIMER_GROUP_0, TIMER_1)
//  #define timer_1_clear_interrupt TIMERG0.int_clr_timers.t1_int_clr = 1
#define timer_1_clear_interrupt TIMERG0.int_clr_timers.t1 = 1
// #define timer_1_enable_alarm timer_group_enable_alarm_in_isr(TIMER_GROUP_0, TIMER_1)
// #define timer_1_enable_alarm TIMERG0.hw_timer[1].config.tx_alarm_en = 1
#define timer_1_enable_alarm TIMERG0.hw_timer[1].config.alarm_en = 1

// #define timer_2_set_alarm_value(alarm_val) timer_group_set_alarm_value_in_isr(TIMER_GROUP_1, TIMER_0, alarm_val)
// #define timer_2_set_alarm_value(alarm_val) TIMERG1.hw_timer[0].alarmlo.tx_alarm_lo = alarm_val
#define timer_2_set_alarm_value(alarm_val) TIMERG1.hw_timer[0].alarm_low = alarm_val
// #define timer_2_clear_interrupt timer_group_clr_intr_status_in_isr(TIMER_GROUP_1, TIMER_0)
// #define timer_2_clear_interrupt TIMERG1.int_clr_timers.t0_int_clr = 1
#define timer_2_clear_interrupt TIMERG1.int_clr_timers.t0 = 1
// #define timer_2_enable_alarm timer_group_enable_alarm_in_isr(TIMER_GROUP_1, TIMER_0)
// #define timer_2_enable_alarm TIMERG1.hw_timer[0].config.tx_alarm_en = 1
#define timer_2_enable_alarm TIMERG1.hw_timer[0].config.alarm_en = 1

void IRAM_ATTR timer_0_isr(void* arg)
{
    static int _step = 0;
    static int _dir = 0;
    static uint32_t _t = 0;

    if (_step == 0) { // end of period
        _t = T_half[0];
        if (_t) { // there is a period time so a pulse must be started
            STEP_0_H;
            timer_0_set_alarm_value(_t);
            (_dir == 0) ? --fb.pos[0] : ++fb.pos[0];
            _step = 1;
            math[0] = 1;
        } else { // no need to step pulse
            if (dirChange[0]) { // change dir
                (_dir == 0) ? DIR_0_H : DIR_0_L;
                timer_0_set_alarm_value(dirSetup[0]);
                dir[0] = _dir ^= 1;
                dirChange[0] = 0;
                math[0] = 1;
            } else { // 4 kHz listen
                timer_0_set_alarm_value(10000UL);
                math[0] = 1;
            }
        }
    } else { // the middle of the period time (step H-L transition)
        STEP_0_L;
        timer_0_set_alarm_value(_t);
        _step = 0;
    }

    timer_0_clear_interrupt;
    timer_0_enable_alarm;
}

void IRAM_ATTR timer_1_isr(void* arg)
{
    static int _step = 0;
    static int _dir = 0;
    static uint32_t _t = 0;

    if (_step == 0) {
        _t = T_half[1];
        if (_t) {
            STEP_1_H;
            timer_1_set_alarm_value(_t);
            (_dir == 0) ? --fb.pos[1] : ++fb.pos[1];
            _step = 1;
            math[1] = 1;
        } else {
            if (dirChange[1]) {
                (_dir == 0) ? DIR_1_H : DIR_1_L;
                timer_1_set_alarm_value(dirSetup[1]);
                dir[1] = _dir ^= 1;
                dirChange[1] = 0;
                math[1] = 1;
            } else {
                timer_1_set_alarm_value(10000UL);
                math[1] = 1;
            }
        }
    } else {
        STEP_1_L;
        timer_1_set_alarm_value(_t);
        _step = 0;
    }

    timer_1_clear_interrupt;
    timer_1_enable_alarm;
}

void IRAM_ATTR timer_2_isr(void* arg)
{
    static int _step = 0;
    static int _dir = 0;
    static uint32_t _t = 0;

    if (_step == 0) {
        _t = T_half[2];
        if (_t) {
            STEP_2_H;
            timer_2_set_alarm_value(_t);
            (_dir == 0) ? --fb.pos[2] : ++fb.pos[2];
            _step = 1;
            math[2] = 1;
        } else {
            if (dirChange[2]) {
                (_dir == 0) ? DIR_2_H : DIR_2_L;
                timer_2_set_alarm_value(dirSetup[2]);
                dir[2] = _dir ^= 1;
                dirChange[2] = 0;
                math[2] = 1;
            } else {
                timer_2_set_alarm_value(10000UL);
                math[2] = 1;
            }
        }
    } else {
        STEP_2_L;
        timer_2_set_alarm_value(_t);
        _step = 0;
    }

    timer_2_clear_interrupt;
    timer_2_enable_alarm;
}

static void timer__init(timer_group_t group, timer_idx_t idx)
{
    /* Select and initialize basic parameters of the timer */
    /*
    timer_config_t timer_config = {
        .divider = 2UL,
        .counter_dir = TIMER_COUNT_UP,
        .counter_en = TIMER_PAUSE,
        .alarm_en = TIMER_ALARM_EN,
        .auto_reload = TIMER_AUTORELOAD_EN,
    }; // default clock source is APB
    */

    timer_config_t timer_config = {
        .alarm_en = TIMER_ALARM_EN,
        .counter_en = TIMER_PAUSE,
        .intr_type = TIMER_INTR_LEVEL,
        .counter_dir = TIMER_COUNT_UP,
        .auto_reload = TIMER_AUTORELOAD_EN,
        .divider = 2UL,
    };

    timer_init(group, idx, &timer_config);

    /* Timer's counter will initially start from value below.
       Also, if auto_reload is set, this value will be automatically reload on alarm */
    timer_set_counter_value(group, idx, 0ULL);

    /* Configure the alarm value and the interrupt on alarm. */
    timer_set_alarm_value(group, idx, 40000000ULL);
    timer_enable_intr(group, idx);
    if (group == TIMER_GROUP_0) {
        if (idx == TIMER_0)
            timer_isr_register(group, idx, timer_0_isr, NULL, ESP_INTR_FLAG_IRAM, NULL);
        else if (idx == TIMER_1)
            timer_isr_register(group, idx, timer_1_isr, NULL, ESP_INTR_FLAG_IRAM, NULL);
    } else if (group == TIMER_GROUP_1) {
        if (idx == TIMER_0)
            timer_isr_register(group, idx, timer_2_isr, NULL, ESP_INTR_FLAG_IRAM, NULL);
        // else if (idx == TIMER_1)
        //  timer_isr_register(group, idx, timer_3_isr, (void*)((idx << 1) | group), ESP_INTR_FLAG_IRAM, NULL);
    }

    timer_start(group, idx);
}

static float IRAM_ATTR fastInvSqrt(const float x)
{
    const float xhalf = x * 0.5f;
    union {
        float x;
        uint32_t i;
    } u = { .x = x };
    u.i = 0x5f3759df - (u.i >> 1);
    return u.x * (1.5f - xhalf * u.x * u.x);
}

static void IRAM_ATTR deceleration(const int i)
{
    if (accelStep[i] != 0) {
        if (--accelStep[i] != 0)
            T_half[i] = fastInvSqrt(accel_x2[i] * (float)accelStep[i]) * 20000000.0f;
        else
            T_half[i] = 0;
    }
}

static void IRAM_ATTR acceleration(const int i)
{
    if (cmd.control & CTRL_ENABLE) {
        ++accelStep[i];
        T_half[i] = fastInvSqrt(accel_x2[i] * (float)accelStep[i]) * 20000000.0f;
    } else
        deceleration(i);
}

static void IRAM_ATTR stepgen_task(void* arg)
{
    gpio_reset_pin((gpio_num_t)STEP_0_PIN);
    gpio_set_direction((gpio_num_t)STEP_0_PIN, GPIO_MODE_OUTPUT);

    gpio_reset_pin((gpio_num_t)DIR_0_PIN);
    gpio_set_direction((gpio_num_t)DIR_0_PIN, GPIO_MODE_OUTPUT);

    gpio_reset_pin((gpio_num_t)STEP_1_PIN);
    gpio_set_direction((gpio_num_t)STEP_1_PIN, GPIO_MODE_OUTPUT);

    gpio_reset_pin((gpio_num_t)DIR_1_PIN);
    gpio_set_direction((gpio_num_t)DIR_1_PIN, GPIO_MODE_OUTPUT);

    gpio_reset_pin((gpio_num_t)STEP_2_PIN);
    gpio_set_direction((gpio_num_t)STEP_2_PIN, GPIO_MODE_OUTPUT);

    gpio_reset_pin((gpio_num_t)DIR_2_PIN);
    gpio_set_direction((gpio_num_t)DIR_2_PIN, GPIO_MODE_OUTPUT);

    timer__init(TIMER_GROUP_0, TIMER_0);
    timer__init(TIMER_GROUP_0, TIMER_1);
    timer__init(TIMER_GROUP_1, TIMER_0);

    for (;;) {
        for (int i = 0; i < 3; ++i) {
            if (math[i]) {
                if (accelStep[i]) {
                    if (dir[i] == cmd_dir[i]) {
                        if (T_half[i] > cmd_T_half[i])
                            acceleration(i);
                        else if (T_half[i] < cmd_T_half[i])
                            deceleration(i);
                    } else
                        deceleration(i);
                } else {
                    int pos_error = cmd.pos[i] - fb.pos[i];
                    if (pos_error < 0) {
                        if (dir[i] == 0)
                            acceleration(i);
                        else
                            dirChange[i] = 1;
                    } else if (pos_error > 0) {
                        if (dir[i] == 0)
                            dirChange[i] = 1;
                        else
                            acceleration(i);
                    }
                }
                math[i] = 0;
            }
        }
    }
}
