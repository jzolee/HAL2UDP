// file: comm.c
#include "comm.h"

#include <stdint.h>
#include <stddef.h>
#include <math.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

#include "esp_attr.h"
#include "driver/gpio.h"
#include "esp_timer.h"

#include "hardware.h"
#include "w5500.h"
#include "stepgen.h"
#include "io.h"

#ifndef STATIC_INLINE
#define STATIC_INLINE __attribute__((always_inline)) static inline
#endif

/* =========================================
*  Control bit definitions (host ↔ firmware)
*  ========================================= */
#define CTRL_DIRSETUP 0b00000001  // Direction setup time configured
#define CTRL_ACCEL    0b00000010  // Acceleration configured
#define CTRL_PWMFREQ  0b00000100  // PWM frequencies configured
#define CTRL_READY    0b01000000  // All mandatory configuration received
#define CTRL_ENABLE   0b10000000  // Motion/output enable from host

/* =======================================================
*  Command packet (host → device)
*  Packed to guarantee binary compatibility over Ethernet.
*  ======================================================= */
#pragma pack(push, 1)
typedef struct {  // max 55 byte !!! (64 byte SPI data register size - 8 byte w5500 header - 1 byte crc)
    union {
        int32_t pos[3];
        uint32_t dirSetup[3];
        uint32_t accel[3];
    };
    int32_t  T[3];
    uint8_t  control;
    uint8_t  io;
    uint16_t pwm[6];
} comm_cmd_t;
#pragma pack(pop)

/* ===============================
*  Feedback packet (device → host)
*  =============================== */
#pragma pack(push, 1)
typedef struct {  // max 55 byte !!! (64 byte SPI data register size - 8 byte w5500 header - 1 byte crc)
    int32_t pos[3];
    int32_t T[3];
    uint8_t control;
    uint8_t io;
} comm_fb_t;
#pragma pack(pop)

static comm_fb_t fb = { 0 };
static volatile uint32_t timeout = 1;
static volatile uint32_t w5500_irq_flag = 0;

STATIC_INLINE void update_stepgen(const int ch, const sg_cmd_t* restrict new_cmd) {
    static uint32_t idx[3] = { 1, 1, 1 };
    sg_ctx_t* const act = &sg_ctx[ch];
    volatile sg_cmd_t* const cmd = &act->cmd[idx[ch]];
    *cmd = *new_cmd;
    asm volatile ("memw");
    act->active_cmd = cmd;
    idx[ch] ^= 1;
}

STATIC_INLINE void feedback_handler(void) {

    input_handler(&fb.io);

    for (int i = 0; i < 3; ++i) {
        const sg_ctx_t* const act = &sg_ctx[i];
        fb.pos[i] = act->pos;
        const uint32_t T = act->T;
        const int32_t dir = act->dir;
        fb.T[i] = T ? dir * T : 0;
    }
}

STATIC_INLINE void command_handler(const comm_cmd_t* restrict cmd) {

    static uint32_t cntrl_en = 0;

    /* -------- Enable / Disable -------- */
    if (__builtin_expect(cmd->control & CTRL_ENABLE, 1)) {
        if (cntrl_en == 0) {
            cntrl_en = 1;
            stepGen_enable = 1;
        }
    } else if (cntrl_en != 0) {
        cntrl_en = 0;
        stepGen_enable = 0;
        fb.control = 0;
    }

    /* -------- Runtime motion -------- */
    if (__builtin_expect(cmd->control & CTRL_READY, 1)) {
        for (int i = 0; i < 3; ++i) {

            sg_cmd_t new_cmd;
            const int32_t T = cmd->T[i];

            if (T > 0) {
                new_cmd.dir = 1;
                new_cmd.T = T;
            } else if (T < 0) {
                new_cmd.dir = -1;
                new_cmd.T = -T;
            } else {
                new_cmd.dir = 0;
                new_cmd.T = UINT32_MAX;
            }

            new_cmd.pos = cmd->pos[i];
            update_stepgen(i, &new_cmd);
        }
    }

    /* -------- Configuration -------- */
    if (__builtin_expect(!(fb.control & CTRL_READY), 0)) {

        if ((fb.control & CTRL_DIRSETUP) &&
            (fb.control & CTRL_ACCEL) &&
            (fb.control & CTRL_PWMFREQ)) {

            fb.control |= CTRL_READY;
            startup_sequence = STEPGEN_INIT2;

        } else if (cmd->control & CTRL_DIRSETUP) {
            fb.control |= CTRL_DIRSETUP;
            for (int i = 0; i < 3; ++i)
                sg_cfg[i].dirSetup = (uint32_t)cmd->dirSetup[i] / 25;

        } else if (cmd->control & CTRL_ACCEL) {
            fb.control |= CTRL_ACCEL;
            for (int i = 0; i < 3; ++i) {
                const float T_scale = 40000000.0f / sqrtf((float)(cmd->accel[i] * 2)); // (timer tic = 25ns)
                sg_cfg[i].T1 = (uint32_t)(T_scale + 0.5f);
#if STEPGEN_USE_LUT
                sg_cfg[i].accel = cmd->accel[i];
#else
                sg_cfg[i].T_scale = T_scale;
#endif
            }

        } else if (cmd->control & CTRL_PWMFREQ) {
            fb.control |= CTRL_PWMFREQ;
            init_outputs(cmd->pwm);
        }
    }

    /* -------- Output update -------- */
    output_handler(cmd->io, cmd->pwm, cntrl_en);
}

static void IRAM_ATTR watchdog_timer_cb(void* arg) {
    if (__builtin_expect(timeout, 0)) {
        if (fb.control) {
            fb.control = 0;
            const comm_cmd_t cmd = { 0 };
            command_handler(&cmd);
        }
    }
    timeout = 1;
}

STATIC_INLINE void init_watchdog(void) {
    esp_timer_handle_t watchdog_timer_handle;
    const esp_timer_create_args_t args = {
        .arg = NULL,
        .callback = &watchdog_timer_cb,
        .name = "wd_periodic",
        .dispatch_method = ESP_TIMER_ISR
    };
    esp_timer_create(&args, &watchdog_timer_handle);
    esp_timer_start_periodic(watchdog_timer_handle, 100000ULL); // 100ms
}

static void IRAM_ATTR w5500_socket_isr(void* arg) { w5500_irq_flag = 1; }

void IRAM_ATTR comm_loop(void) {

    init_inputs();

    init_watchdog();

    while (startup_sequence != COMM_INIT);

    gpio_set_direction(W5500_INT_PIN, GPIO_MODE_INPUT);
    gpio_set_intr_type(W5500_INT_PIN, GPIO_INTR_NEGEDGE);
    gpio_install_isr_service(ESP_INTR_FLAG_LEVEL3 | ESP_INTR_FLAG_IRAM);
    gpio_isr_handler_add(W5500_INT_PIN, w5500_socket_isr, NULL);

    const w5500_cfg_t  w5500_cfg = {
        .gateway = GATEWAY,
        .subnet = SUBNET,
        .mac = { 0x6A, 0x7A, 0x6F, 0x6C, 0x65, 0x65 },
        .ip = LOCAL_IP,
        .port = 58427,
        .remotePort = 58428
    };

    w5500_recv_t* recv = w5500_init(&w5500_cfg);

    for (;;) {
        //asm volatile ("waiti 0");
        if (w5500_irq_flag) {
            w5500_irq_flag = 0;
            feedback_handler();
            w5500_send((const uint8_t*)&fb, sizeof(comm_fb_t));
            w5500_recv(sizeof(comm_cmd_t) + 1);
            uint8_t chk = 71;
            for (int i = 0; i < sizeof(comm_cmd_t); ++i) chk ^= recv->data[i];
            if (__builtin_expect(recv->data[sizeof(comm_cmd_t)] == chk, 1)) {
                command_handler((const comm_cmd_t*)recv->data);
                timeout = 0;
            }
        }
    }
}
