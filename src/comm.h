#pragma once

#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_attr.h"
#include "driver/ledc.h"

#include "globals.h"
#include "hardware.h"
#include "w5500.h"
#include "io.h"
#include "pwm.h"

void IRAM_ATTR watchdog_task(void* arg)
{
    TickType_t xLastWakeTime = xTaskGetTickCount();

    for (;;) {
        xTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(100)); // 0.1 sec
        if (watchdog) {
            watchdog = 0;
        } else {
            fb.control = 0;
            cmd.control = 0;
            outputHandler();
        }
    }
}

void IRAM_ATTR  w5500_socket_isr(void* arg)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    vTaskNotifyGiveFromISR(comm_task_handle, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

void IRAM_ATTR commandHandler()
{
    if (cmd.control & CTRL_READY) {

        for (int i = 0; i < 3; ++i) {
            if (cmd.vel[i] > 0.0f) {
                cmd_dir[i] = 1;
                cmd_T_half[i] = (uint32_t)(20000000.0f / cmd.vel[i]);
            } else if (cmd.vel[i] < 0.0f) {
                cmd_dir[i] = 0;
                cmd_T_half[i] = (uint32_t)(20000000.0f / -cmd.vel[i]);
            } else {
                cmd_dir[i] = -1;
                cmd_T_half[i] = UINT32_MAX;
            }
        }
    }

    if (!(fb.control & CTRL_READY)) {

        if ((fb.control & CTRL_DIRSETUP) &&
            (fb.control & CTRL_ACCEL) &&
            (fb.control & CTRL_PWMFREQ)) {

            fb.control |= CTRL_READY;

        } else if (cmd.control & CTRL_DIRSETUP) {
            fb.control |= CTRL_DIRSETUP;
            for (int i = 0; i < 3; ++i)
                dirSetup[i] = cmd.dirSetup[i] / 25; //   25ns / timer tic

        } else if (cmd.control & CTRL_ACCEL) {
            fb.control |= CTRL_ACCEL;
            for (int i = 0; i < 3; ++i)
                accel_x2[i] = (float)cmd.accel[i] * 2.0f;

        } else if (cmd.control & CTRL_PWMFREQ) {
            fb.control |= CTRL_PWMFREQ;
            for (int i = 0; i < 6; ++i) {
                if (cmd.pwm[i] != 0) {
                    configure_pwm_channel(i, out_pins[i], cmd.pwm[i]);
                    pwm_enable[i] = 1;
                } else {
                    gpio_set_direction((gpio_num_t)out_pins[i], GPIO_MODE_OUTPUT);
                    gpio_set_level((gpio_num_t)out_pins[i], 0);
                }
            }
        }
    }
}

void IRAM_ATTR comm_task(void* arg)
{
    xSemaphoreTake(startMutex, portMAX_DELAY);

    // W5500 config
    w5500_config_t  cfg = {
        .spi_host = VSPI_HOST,
        .spi_clock_hz = SPI_CLOCK_MHZ * 1000 * 1000,
        .pin_cs = SPI_CS_PIN,
        .pin_miso = SPI_MISO_PIN,
        .pin_mosi = SPI_MOSI_PIN,
        .pin_sclk = SPI_SCLK_PIN,
        .gateway = GATEWAY,
        .subnet = SUBNET,
        .mac = { 0x6A, 0x7A, 0x6F, 0x6C, 0x65, 0x65 },
        .ip = LOCAL_IP,
        .port = 58427,
        .remotePort = 58428
    };

    // Initialize W5500
    w5500_init(&cfg);

    // interrupt
    gpio_set_direction(W5500_INT_PIN, GPIO_MODE_INPUT);
    gpio_set_intr_type(W5500_INT_PIN, GPIO_INTR_NEGEDGE);
    gpio_install_isr_service(0);
    gpio_isr_handler_add(W5500_INT_PIN, w5500_socket_isr, NULL);

    xSemaphoreGive(startMutex);

    for (;;) {
        
        inputHandler();
        for (int i = 0; i < 3; ++i) {
            uint32_t t = T_half[i];
            if (t) {
                if (dir[i])
                    fb.vel[i] = 20000000.0f / (float)t;
                else
                    fb.vel[i] = -20000000.0f / (float)t;
            } else
                fb.vel[i] = 0.0f;
        }
        uint8_t fb_buf[sizeof(fb)];
        memcpy(&fb_buf, &fb, sizeof(fb));

        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        w5500_recv_t* received = w5500_data_exchange(sizeof(cmd) + 1, fb_buf, sizeof(fb));
        if (received->dataLength) {
            uint8_t chk = 71;
            for (int i = 0; i < sizeof(cmd); ++i)
                chk ^= received->data[i];
            if (received->data[sizeof(cmd)] == chk) {
                memcpy(&cmd, &received->data, sizeof(cmd));
                commandHandler();
                outputHandler();
                watchdog = watchdog + 1;
            }
        }
    }
}
