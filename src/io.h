// file: io.h
#pragma once

/* ============================
*  Digital I/O and PWM handling
*  ============================ */

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

void init_inputs(void);
void init_outputs(const uint16_t* restrict pwm_freq);

void input_handler(uint8_t* restrict fb_io);
void output_handler(const uint8_t outputs, const uint16_t* restrict pwm_freq, const uint8_t enable);

#ifdef __cplusplus
}
#endif
