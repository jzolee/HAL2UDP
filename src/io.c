// file: io.c

#include "driver/gpio.h"
#include "driver/ledc.h"
#include "soc/gpio_reg.h"
#include "esp_attr.h"

#include "io.h"
#include "hardware.h"

/* ============================================================
*  Digital I/O and PWM handling
*
*  - Fast digital input sampling via direct GPIO register reads
*  - Combined digital output and PWM output handling
*  - Runtime allocation and sharing of LEDC timers by frequency
*  ============================================================ */

#define OUTPUTS_NUM (sizeof(out_pins) / sizeof(out_pins[0]))
#define INPUTS_NUM (sizeof(in_pins) / sizeof(in_pins[0]))

typedef struct {
    uint32_t mask0; // GPIO 0-31
    uint32_t mask1; // GPIO 32-39
} gpio_mask_t;

typedef struct {
    gpio_num_t   gpio;     // GPIO number
    uint32_t     duty;     // PWM duty (10 bit)
    gpio_mask_t  mask;     // Precomputed GPIO register masks for ultra-fast handling.
    bool         is_pwm;   // the pin is pwm output
} out_info_t;

typedef struct {
    uint32_t     freq;    // PWM frequency in Hz
    ledc_timer_t timer;   // LEDC timer index
    bool         used;    // allocation flag
} timer_info_t;

static out_info_t out[OUTPUTS_NUM];
static timer_info_t timers[LEDC_TIMER_MAX] = { 0 };
static gpio_mask_t in_mask[INPUTS_NUM]; // Precomputed GPIO register masks

static uint32_t io_set_masks0[256];
static uint32_t io_clr_masks0[256];
static uint32_t io_set_masks1[256];
static uint32_t io_clr_masks1[256];

static ledc_timer_t IRAM_ATTR get_or_allocate_ledc_timer(const uint32_t freq) {

    // Try to reuse an existing timer with the same frequency
    for (int i = 0; i < LEDC_TIMER_MAX; i++) {
        if (timers[i].used && timers[i].freq == freq)
            return timers[i].timer;
    }

    // Allocate a new timer if available
    for (int i = 0; i < LEDC_TIMER_MAX; i++) {
        if (!timers[i].used) {
            timers[i].used = true;
            timers[i].freq = freq;
            timers[i].timer = (ledc_timer_t)i;

            ledc_timer_config_t cfg = {
                .speed_mode = LEDC_HIGH_SPEED_MODE,
                .duty_resolution = LEDC_TIMER_10_BIT,
                .timer_num = (ledc_timer_t)i,
                .freq_hz = freq,
                .clk_cfg = LEDC_AUTO_CLK,
                .deconfigure = false,
            };
            ledc_timer_config(&cfg);
            return (ledc_timer_t)i;
        }
    }

    // No free timer available
    return (ledc_timer_t)-1;
}

static bool IRAM_ATTR configure_pwm_channel(const int ch, const uint32_t freq) {
    ledc_timer_t timer = get_or_allocate_ledc_timer(freq);
    if (timer == (ledc_timer_t)-1)
        return false;

    ledc_channel_config_t cfg = {
        .gpio_num = (int)out[ch].gpio,
        .speed_mode = LEDC_HIGH_SPEED_MODE,
        .channel = (ledc_channel_t)ch,
        .intr_type = LEDC_INTR_DISABLE,
        .timer_sel = timer,
        .duty = 0,
        .hpoint = 0,
        .sleep_mode = LEDC_SLEEP_MODE_NO_ALIVE_NO_PD,
        .flags = {.output_invert = 0 }
    };

    if (ledc_channel_config(&cfg) != ESP_OK)
        return false;

    return true;
}

void IRAM_ATTR init_inputs(void) {
    for (int i = 0; i < INPUTS_NUM; i++) {
        gpio_num_t pin = in_pins[i];
        gpio_set_direction(pin, GPIO_MODE_INPUT);
        if (pin < 34)
            gpio_set_pull_mode(pin, GPIO_PULLUP_ONLY); // Internal pull-ups are enabled where supported.
        if (pin < 32) {
            in_mask[i].mask0 = (1ULL << pin);
            in_mask[i].mask1 = 0;
        } else {
            in_mask[i].mask0 = 0;
            in_mask[i].mask1 = (1ULL << (pin - 32));
        }
    }
}

void IRAM_ATTR init_outputs(const uint16_t* restrict pwm_freq) {
    for (int i = 0; i < OUTPUTS_NUM; i++) {
        gpio_num_t pin = out_pins[i];
        out[i].gpio = pin;
        out[i].duty = UINT16_MAX;
        out[i].is_pwm = false;

        if (pin < 32) {
            out[i].mask.mask0 = (1UL << pin);
            out[i].mask.mask1 = 0;
        } else {
            out[i].mask.mask0 = 0;
            out[i].mask.mask1 = (1UL << (pin - 32));
        }

        if (pwm_freq && pwm_freq[i]) {
            if (configure_pwm_channel(i, pwm_freq[i])) {
                out[i].is_pwm = true;
                continue;
            }
        }
        gpio_set_direction(pin, GPIO_MODE_OUTPUT);
        gpio_set_level(pin, 0);
    }

    for (int i = 0; i < 256; i++) {
        io_set_masks0[i] = 0; io_clr_masks0[i] = 0;
        io_set_masks1[i] = 0; io_clr_masks1[i] = 0;
        for (int j = 0; j < OUTPUTS_NUM; j++) {
            if (!out[j].is_pwm) {
                if (i & (1 << j)) {
                    io_set_masks0[i] |= out[j].mask.mask0;
                    io_set_masks1[i] |= out[j].mask.mask1;
                } else {
                    io_clr_masks0[i] |= out[j].mask.mask0;
                    io_clr_masks1[i] |= out[j].mask.mask1;
                }
            }
        }
    }
}

void IRAM_ATTR input_handler(uint8_t* restrict fb_io) {
    const uint32_t in0 = *(volatile uint32_t*)GPIO_IN_REG;
    const uint32_t in1 = *(volatile uint32_t*)GPIO_IN1_REG;

    uint8_t val = 0;
    for (int i = 0; i < INPUTS_NUM; i++) {
        if ((in0 & in_mask[i].mask0) | (in1 & in_mask[i].mask1))
            val |= (1U << i);
    }

    *fb_io = val;
}

void IRAM_ATTR output_handler(const uint8_t outputs, const uint16_t* restrict pwm_freq, const uint8_t enable) {
    for (int i = 0; i < OUTPUTS_NUM; i++) {
        if (out[i].is_pwm) {
            uint32_t duty = enable ? pwm_freq[i] : 0;
            if (duty > 1023)
                duty = 1023;
            if (out[i].duty != duty) {
                out[i].duty = duty;
                ledc_set_duty(LEDC_HIGH_SPEED_MODE, (ledc_channel_t)i, duty);
                ledc_update_duty(LEDC_HIGH_SPEED_MODE, (ledc_channel_t)i);
            }
        }
    }

    uint32_t idx = enable ? outputs : 0;
    *(volatile uint32_t*)GPIO_OUT_W1TC_REG = io_clr_masks0[idx];
    *(volatile uint32_t*)GPIO_OUT_W1TS_REG = io_set_masks0[idx];
    *(volatile uint32_t*)GPIO_OUT1_W1TC_REG = io_clr_masks1[idx];
    *(volatile uint32_t*)GPIO_OUT1_W1TS_REG = io_set_masks1[idx];
}
