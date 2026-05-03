// file: stepgen.c

#include <stdint.h>
#include <stdlib.h>   // malloc / free

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

#include "esp_attr.h"            // IRAM_ATTR / DRAM_ATTR
#include "esp_intr_alloc.h"      // ISR registration flags

#include "driver/timer.h"        // Legacy timer driver (direct register access)
#include "driver/gpio.h"

#include "stepgen.h"
#include "stepgen.defs.h"
#include "hardware.h"        // Pin definitions and board-specific configuration

#if STEPGEN_USE_LUT
#include <math.h>
#endif

#if STEPGEN_USE_RMT
#include "soc/soc.h"
#include "soc/rmt_reg.h"
#include "soc/periph_defs.h"
#include "soc/gpio_sig_map.h"
#include "rom/gpio.h"
#include "driver/periph_ctrl.h"
#endif

/* ============================================================================
 * Global instances
 * ============================================================================ */

volatile start_seq_t startup_sequence = STEPGEN_INIT1;
volatile uint32_t stepGen_enable = 0;

sg_cfg_t sg_cfg[3] = { 0 };

sg_ctx_t sg_ctx[3] = {
    {.dir = -1, .dirSetup = 40000000},
    {.dir = -1, .dirSetup = 40000000},
    {.dir = -1, .dirSetup = 40000000}
};

/* ============================================================================
 * Static instances
 * ============================================================================ */

enum Action : uint32_t {
    ACTION_NULL,
    ACTION_DIR_C,
    ACTION_START,
    ACTION_ACCEL,
    ACTION_DECEL,
    ACTION_BRAKE,
} sg_action_t;

/* ============================================================================
 * Math helpers (non-LUT mode)
 * ============================================================================ */

#if !STEPGEN_USE_LUT

 /**
  * @brief Fast inverse square root approximation.
  *
  * Used to compute acceleration ramp without LUT.
  */
STATIC_INLINE float fastInvSqrt(const float x) {
    const float xhalf = 0.5f * x;
    union { float f; uint32_t i; } u = { .f = x };
    u.i = 0x5F3759DF - (u.i >> 1); // Magic constant
    u.f = u.f * (1.5f - xhalf * u.f * u.f); // 1st Newton-Raphson iteration
    //u.f = u.f * (1.5f - xhalf * u.f * u.f); // 2nd Newton-Raphson iteration
    return u.f;
}

#endif /* !STEPGEN_USE_LUT */

/* ============================================================================
 * LUT helpers (STEPGEN_USE_LUT)
 * ============================================================================ */

#if STEPGEN_USE_LUT

STATIC_INLINE void free_lut(sg_ctx_t* restrict sg) {
    if (sg->lut_T) {
        free(sg->lut_T);
        sg->lut_T = NULL;
    }
    if (sg->lut_dn) {
        free(sg->lut_dn);
        sg->lut_dn = NULL;
    }
    sg->T_size = 0;
    sg->idx_max = 0;
    sg->dn_counter = 0;
}

STATIC_INLINE uint32_t calc_T(const float K, const uint32_t n) {
    if (n == 0) return UINT32_MAX;
    const float T = K / sqrtf((float)n);
    if (T < 1.0f) return 1;
    return (uint32_t)(T + 0.5f);
}

static void IRAM_ATTR build_lut(sg_ctx_t* restrict sg, const float freq_max) {

    free_lut(sg);

    const uint32_t T_min = (uint32_t)(40000000.0f / freq_max + 0.5f);
    const float K = 40000000.0f / sqrtf((float)(sg->accel * 2));

    /* --- 1. Direct T lookup table --- */
    uint32_t idx = 1;
    uint32_t T_last = calc_T(K, idx);

    while (1) {
        const uint32_t T = calc_T(K, ++idx);
        if (T <= T_min) break;
        if (idx == 65535) break;
        if (T_last == T) break;
        T_last = T;
    }

    sg->lut_T = (uint32_t*)malloc(sizeof(uint32_t) * idx);
    if (!sg->lut_T) return;

    sg->T_size = idx;

    sg->lut_T[0] = UINT32_MAX;
    for (uint32_t i = 1; i < idx; ++i)
        sg->lut_T[i] = calc_T(K, i);

    /* --- 2. Delta-n table (dn) --- */
    T_last = sg->lut_T[idx - 1];
    uint32_t dn_size = 0;

    if (T_last > T_min) {
        dn_size = T_last - T_min;
        sg->lut_dn = (uint32_t*)malloc(sizeof(uint32_t) * dn_size);
        if (sg->lut_dn) {
            uint16_t dn_idx = 0;
            while (T_last >= T_min && dn_idx < dn_size) {
                uint32_t steps_count = 0;
                while (calc_T(K, idx++) == T_last)
                    if (++steps_count == 10000) break; // safety limit
                sg->lut_dn[dn_idx++] = steps_count;
                T_last--;
            }
        } else dn_size = 0;
    }

    /* --- 3. Reset runtime state --- */
    sg->idx_max = sg->T_size + dn_size - 1;
    sg->dn_counter = 0;
}

#endif /* STEPGEN_USE_LUT */

/* ============================================================================
 * Timer ISRs
 * ============================================================================ */

#if STEPGEN_USE_RMT // ISR with RMT

#define GENERATE_TIMER_ISR_IMPL(CH_NUM)                                 \
static void IRAM_ATTR timer_##CH_NUM##_isr(void* arg) {                 \
                                                                        \
    sg_ctx_t* const c = &sg_ctx[CH_NUM];                                \
    const uint32_t T = c->T;                                            \
                                                                        \
    if (__builtin_expect(T > 4, 1)) {                                   \
        /* RMT pulse: T/2 High (max 32767 RMT tick) */                  \
        /* (Hardware limit: 15 bit) */                                  \
        const uint32_t Thalf = (T >> 1) & 0x7FFFUL;                     \
        /* START STEP PULSE */                                          \
        /* The low 16-bit entry is sent first */                        \
        /* Entry: [Level: 1 bit][Duration: 15 bit] */                   \
        REGISTER_WRITE(RMT_CHANNEL_MEM(CH_NUM), Thalf | (1UL << 15));   \
        /* RESET MEM POINTER & START */                                 \
        REGISTER_WRITE(RMT_CH_CONF1_REG(CH_NUM), RMT_CONF1_RESETnSTART);\
        /* POSITION UPDATE */                                           \
        c->pos += c->dir;                                               \
        /* LOAD PERIOD TO TIMER */                                      \
        TMR_##CH_NUM##_ALM = T;                                         \
    } else {                                                            \
        /*  DIR CHANGE / IDLE  */                                       \
        if (c->dir_change) {                                            \
            const int32_t dir = -c->dir;                                \
            if (dir < 0) DIR_##CH_NUM##_LOW;                            \
            else DIR_##CH_NUM##_HIGH;                                   \
            c->dir = dir;                                               \
            c->dir_change = 0;                                          \
            TMR_##CH_NUM##_ALM = c->dirSetup;                           \
        } else {                                                        \
            /* Idle polling 4kHz */                                     \
            TMR_##CH_NUM##_ALM = 10000UL;                               \
        }                                                               \
    }                                                                   \
    c->math = 1;                                                        \
    TMR_##CH_NUM##_CLR;                                                 \
    TMR_##CH_NUM##_EN;                                                  \
}

#else /* non-RMT */

#define GENERATE_TIMER_ISR_IMPL(CH_NUM)                     \
static void IRAM_ATTR timer_##CH_NUM##_isr(void* arg) {     \
                                                            \
    sg_ctx_t* const c = &sg_ctx[CH_NUM];                    \
                                                            \
    if (c->step_state) {                                    \
        /* STEP H->L TRANSITION */                          \
        STEP_##CH_NUM##_LOW;                                \
        c->step_state = 0;                                  \
        TMR_##CH_NUM##_ALM = c->T_low;                      \
    } else {                                                \
        /* STEP L->H TRANSITION OR DIR CHANGE OR IDLE */    \
        const uint32_t T = c->T;                            \
        if (__builtin_expect(T > 0, 1)) {                   \
            /* This branching is the most common */         \
            STEP_##CH_NUM##_HIGH;                           \
            c->pos += c->dir;                               \
            const uint32_t T_high = T >> 1;                 \
            c->T_low = T - T_high;                          \
            c->step_state = 1;                              \
            TMR_##CH_NUM##_ALM = T_high;                    \
        } else {                                            \
            /* DIR CHANGE OR IDLE (RARE OCCASIONS) */       \
            if (c->dir_change) {                            \
                const int32_t dir = -c->dir;                \
                if (dir < 0) DIR_##CH_NUM##_LOW;            \
                else DIR_##CH_NUM##_HIGH;                   \
                c->dir = dir;                               \
                c->dir_change = 0;                          \
                TMR_##CH_NUM##_ALM = c->dirSetup;           \
            } else {                                        \
                /* Idle polling 4kHz */                     \
                TMR_##CH_NUM##_ALM = 10000UL;               \
            }                                               \
        }                                                   \
        c->math = 1;                                        \
    }                                                       \
    TMR_##CH_NUM##_CLR; /* timer interrupt clear */         \
    TMR_##CH_NUM##_EN;  /* timer re-enable alarm */         \
}
#endif /* STEPGEN_USE_RMT */

GENERATE_TIMER_ISR_IMPL(0)
GENERATE_TIMER_ISR_IMPL(1)
GENERATE_TIMER_ISR_IMPL(2)

/* ==============================
*  Step generator core processing
*  ============================== */

STATIC_INLINE void stepgen_process(sg_ctx_t* const sg, const uint32_t sg_enable) {

    volatile const sg_cmd_t* restrict const cmd = sg->active_cmd;
    const uint32_t cmd_T = cmd->T;
    uint32_t T = sg->T;
    uint32_t idx = sg->idx;
    uint32_t action = ACTION_NULL;

    if (__builtin_expect(idx, 1)) {
        if (__builtin_expect(sg_enable, 1)) {
            if (__builtin_expect(sg->dir == cmd->dir, 1)) {
                if (T > cmd_T) action = ACTION_ACCEL;
                else if (T < cmd_T) action = ACTION_DECEL;
            } else action = ACTION_BRAKE;
        } else action = ACTION_BRAKE;
    } else {
        const int32_t pos_error = cmd->pos - sg->pos;
        if (pos_error && __builtin_expect(sg_enable, 1)) {
            const int32_t target_dir = (pos_error > 0) ? 1 : -1;
            action = (sg->dir == target_dir) ? ACTION_START : ACTION_DIR_C;
        }
    }

#if STEPGEN_USE_LUT
    const uint32_t T_size = sg->T_size;
    uint32_t dn_counter = sg->dn_counter;
#endif

    switch (action) {
    case ACTION_DIR_C:
        sg->dir_change = 1;
        break;
    case ACTION_START:
        sg->idx = 1;
        sg->T = sg->T1;
        break;
    case ACTION_ACCEL:
#if STEPGEN_USE_LUT
        if (__builtin_expect(++idx >= T_size, 1)) {
            if (__builtin_expect(sg->lut_dn[idx - T_size] > dn_counter, 1)) {
                sg->dn_counter = ++dn_counter;
                break;
            }
            sg->T = --T;
            sg->idx = idx;
            sg->dn_counter = 0;
            break;
        }
        T = sg->lut_T[idx];
#else
        T = (uint32_t)(sg->T_scale * fastInvSqrt((float)++idx) + 0.5f);
#endif
        if (__builtin_expect(T < cmd_T, 0)) {
            sg->T = cmd_T;
            break;
        }
        sg->T = T;
        sg->idx = idx;
        break;
    case ACTION_DECEL:
    case ACTION_BRAKE:
#if STEPGEN_USE_LUT
        if (__builtin_expect(dn_counter > 0, 1)) {
            sg->dn_counter = --dn_counter;
            break;
        }
#endif
        if (__builtin_expect(--idx, 1)) {
#if STEPGEN_USE_LUT
            if (__builtin_expect(idx >= T_size, 1)) {
                sg->T = ++T;
                sg->idx = idx;
                sg->dn_counter = sg->lut_dn[idx - T_size];
                break;
            }
            T = sg->lut_T[idx];
#else
            T = (uint32_t)(sg->T_scale * fastInvSqrt((float)idx) + 0.5f);
#endif
            if (__builtin_expect(action == ACTION_DECEL, 1) && (T > cmd_T)) {
                sg->T = cmd_T;
                break;
            }
            sg->T = T;
            sg->idx = idx;
            break;
        }
        sg->T = 0;
        sg->idx = 0;
        break;
    default:
        break;
    }
    sg->math = 0;
}

static void IRAM_ATTR stepgen_loop(void) {

    sg_ctx_t* const sg0 = &sg_ctx[0];
    sg_ctx_t* const sg1 = &sg_ctx[1];
    sg_ctx_t* const sg2 = &sg_ctx[2];

    for (;;) {
        //asm volatile ("waiti 0");
        const uint32_t sg_en = stepGen_enable;
        if (__builtin_expect(sg0->math, 1)) stepgen_process(sg0, sg_en);
        if (__builtin_expect(sg1->math, 1)) stepgen_process(sg1, sg_en);
        if (__builtin_expect(sg2->math, 1)) stepgen_process(sg2, sg_en);
    }
}

/* ============================================================================
 * Timer / RMT initialization
 * ============================================================================ */

#if STEPGEN_USE_RMT // RMT init

static void IRAM_ATTR rmt_init(const int ch, const int gpio) {

    periph_module_enable(PERIPH_RMT_MODULE);

    gpio_pad_select_gpio(gpio);
    gpio_set_direction(gpio, GPIO_MODE_OUTPUT);

    gpio_matrix_out(gpio, RMT_SIG_OUT0_IDX + ch, 0, 0);
    // RMT setup
    REGISTER_WRITE(RMT_CH_CONF0_REG(ch), RMT_CONF0_BASE_VAL);//
    REGISTER_WRITE(RMT_CH_CONF1_REG(ch), RMT_CONF1_BASE_VAL);//
    REGISTER_SET_BIT(RMT_APB_CONF_REG, RMT_APB_FIFO_MASK);//
    // Transmit end marker
    REGISTER_WRITE(RMT_CHANNEL_MEM(ch) + 4, 0);
}
#endif //USE_RMT

static void IRAM_ATTR timer__init(timer_group_t group, timer_idx_t idx) {

    // Select and initialize basic parameters of the timer
    timer_config_t cfg = {
        .alarm_en = TIMER_ALARM_EN,
        .counter_en = TIMER_PAUSE,
        .intr_type = TIMER_INTR_LEVEL,
        .counter_dir = TIMER_COUNT_UP,
        .auto_reload = TIMER_AUTORELOAD_EN,
        .clk_src = TIMER_SRC_CLK_APB,
        .divider = 2UL,
    };

    timer_init(group, idx, &cfg);

    // Timer's counter will initially start from value below.
    // Also, if auto_reload is set, this value will be automatically reload on alarm
    timer_set_counter_value(group, idx, 0ULL);

    // Configure the alarm value and the interrupt on alarm.
    timer_set_alarm_value(group, idx, 40000000ULL);
    timer_enable_intr(group, idx);

    if (group == TIMER_GROUP_0 && idx == TIMER_0) timer_isr_register(group, idx, timer_0_isr, NULL, ESP_INTR_FLAG_IRAM, NULL);
    if (group == TIMER_GROUP_0 && idx == TIMER_1) timer_isr_register(group, idx, timer_1_isr, NULL, ESP_INTR_FLAG_IRAM, NULL);
    if (group == TIMER_GROUP_1 && idx == TIMER_0) timer_isr_register(group, idx, timer_2_isr, NULL, ESP_INTR_FLAG_IRAM, NULL);

    timer_start(group, idx);
}

/* ============================================================================
 * Step generator task
 * ============================================================================ */
static void IRAM_ATTR stepgen_task(void* arg) {

    //Configure STEP/DIR GPIOs
#if STEPGEN_USE_RMT
    rmt_init(0, STEP_0_PIN);
    rmt_init(1, STEP_1_PIN);
    rmt_init(2, STEP_2_PIN);
#else // Direct GPIO manipulation
    gpio_reset_pin((gpio_num_t)STEP_0_PIN);
    gpio_set_direction((gpio_num_t)STEP_0_PIN, GPIO_MODE_OUTPUT);
    gpio_reset_pin((gpio_num_t)STEP_1_PIN);
    gpio_set_direction((gpio_num_t)STEP_1_PIN, GPIO_MODE_OUTPUT);
    gpio_reset_pin((gpio_num_t)STEP_2_PIN);
    gpio_set_direction((gpio_num_t)STEP_2_PIN, GPIO_MODE_OUTPUT);
#endif
    gpio_reset_pin((gpio_num_t)DIR_0_PIN);
    gpio_set_direction((gpio_num_t)DIR_0_PIN, GPIO_MODE_OUTPUT);
    gpio_reset_pin((gpio_num_t)DIR_1_PIN);
    gpio_set_direction((gpio_num_t)DIR_1_PIN, GPIO_MODE_OUTPUT);
    gpio_reset_pin((gpio_num_t)DIR_2_PIN);
    gpio_set_direction((gpio_num_t)DIR_2_PIN, GPIO_MODE_OUTPUT);

    for (size_t i = 0; i < 3; ++i)
        sg_ctx[i].active_cmd = &sg_ctx[i].cmd[0];

    startup_sequence = COMM_INIT;

    // Waiting for config data from comm task
    while (startup_sequence != STEPGEN_INIT2);

    // Processing of config data
    for (size_t i = 0; i < 3; ++i) {
        sg_ctx_t* const act = &sg_ctx[i];
        act->dirSetup = sg_cfg[i].dirSetup;
        act->T1 = sg_cfg[i].T1;
#if STEPGEN_USE_LUT
        act->accel = sg_cfg[i].accel;
        build_lut(act, 300000.0f);
#else
        act->T_scale = sg_cfg[i].T_scale;
#endif
    }

    timer__init(TIMER_GROUP_0, TIMER_0);
    timer__init(TIMER_GROUP_0, TIMER_1);
    timer__init(TIMER_GROUP_1, TIMER_0);

    // Suspend scheduler
    vTaskSuspendAll();

    stepgen_loop();
}

void IRAM_ATTR start_stepgen_task(void) {
    xTaskCreatePinnedToCore(stepgen_task, "stepgen_task", (2 * 1024), NULL, 1, NULL, 1);
}