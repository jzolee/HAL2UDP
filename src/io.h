#pragma once

#include "driver/gpio.h"
#include "driver/ledc.h"
#include "esp_attr.h"

#include "globals.h"

void IRAM_ATTR outputHandler(void)
{
    static uint16_t last_pwm[6] = { 0 };
    uint8_t out = cmd.io;
    bool enable = cmd.control & CTRL_ENABLE;

    for (int i = 0; i < 6; ++i) {
        if (pwm_enable[i] != 0) {
            if (enable) {
                if (last_pwm[i] != cmd.pwm[i]) {
                    last_pwm[i] = cmd.pwm[i];
                    ledc_set_duty(LEDC_HIGH_SPEED_MODE, i, last_pwm[i]); // Set duty to
                    ledc_update_duty(LEDC_HIGH_SPEED_MODE, i); // Update duty to apply the new value
                }
            } else {
                ledc_set_duty(LEDC_HIGH_SPEED_MODE, i, 0); // Set duty to 0
                ledc_update_duty(LEDC_HIGH_SPEED_MODE, i); // Update duty to apply the new value
                last_pwm[i] = 0;
            }
        } else {
            switch (i) {
            case 0:
                enable ? ((out & IO_00) ? OUT_00_H : OUT_00_L) : OUT_00_L;
                break;
            case 1:
                enable ? ((out & IO_01) ? OUT_01_H : OUT_01_L) : OUT_01_L;
                break;
            case 2:
                enable ? ((out & IO_02) ? OUT_02_H : OUT_02_L) : OUT_02_L;
                break;
            case 3:
                enable ? ((out & IO_03) ? OUT_03_H : OUT_03_L) : OUT_03_L;
                break;
            case 4:
                enable ? ((out & IO_04) ? OUT_04_H : OUT_04_L) : OUT_04_L;
                break;
            case 5:
                enable ? ((out & IO_05) ? OUT_05_H : OUT_05_L) : OUT_05_L;
            }
        }
    }
}

void IRAM_ATTR inputHandler(void)
{
    uint8_t in = 0;
    if (IN_00) in = IO_00;
    if (IN_01) in |= IO_01;
    if (IN_02) in |= IO_02;
    if (IN_03) in |= IO_03;
    if (IN_04) in |= IO_04;
    if (IN_05) in |= IO_05;
    if (IN_06) in |= IO_06;
    fb.io = in;
}

void io_init(void)
{
    // GPIO initialization

    gpio_set_direction((gpio_num_t)IN_00_PIN, GPIO_MODE_INPUT);
    gpio_set_pull_mode((gpio_num_t)IN_00_PIN, GPIO_PULLUP_ONLY);

    gpio_set_direction((gpio_num_t)IN_01_PIN, GPIO_MODE_INPUT);
    gpio_set_pull_mode((gpio_num_t)IN_01_PIN, GPIO_PULLUP_ONLY);

    gpio_set_direction((gpio_num_t)IN_02_PIN, GPIO_MODE_INPUT);
    gpio_set_pull_mode((gpio_num_t)IN_02_PIN, GPIO_PULLUP_ONLY);

    gpio_set_direction((gpio_num_t)IN_03_PIN, GPIO_MODE_INPUT);
    gpio_set_pull_mode((gpio_num_t)IN_03_PIN, GPIO_PULLUP_ONLY);

    gpio_set_direction((gpio_num_t)IN_04_PIN, GPIO_MODE_INPUT);
    //gpio_set_pull_mode((gpio_num_t)IN_04_PIN, GPIO_FLOATING);

    gpio_set_direction((gpio_num_t)IN_05_PIN, GPIO_MODE_INPUT);
    //gpio_set_pull_mode((gpio_num_t)IN_05_PIN,GPIO_FLOATING);

    gpio_set_direction((gpio_num_t)IN_06_PIN, GPIO_MODE_INPUT);
    //gpio_set_pull_mode((gpio_num_t)IN_06_PIN, GPIO_FLOATING);
}