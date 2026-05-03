// file: stepgen.defs.h
#pragma once

#include "soc/timer_group_reg.h" // TIMERG register definitions
#include "soc/gpio_reg.h"        // GPIO register definitions

#include "stepgen.h"
#include "hardware.h"            // Board-specific pin definitions

#if STEPGEN_USE_RMT
#include "soc/rmt_reg.h"
#endif

/* Force aggressive inlining for timing-critical helpers. */

#ifndef STATIC_INLINE
#define STATIC_INLINE __attribute__((always_inline)) static inline
#endif

/* Register access helpers */

#ifndef REGISTER_WRITE
#define REGISTER_WRITE(reg, val)   ((*(volatile uint32_t*)(reg)) = (val))
#endif

#ifndef REGISTER_GET_BIT
#define REGISTER_GET_BIT(reg, m)   (*(volatile uint32_t*)(reg) & (m))
#endif

#ifndef REGISTER_SET_BIT
#define REGISTER_SET_BIT(reg, m)   (*(volatile uint32_t*)(reg) |= (m))
#endif

/* GPIO STEP / DIR macros */

#if !STEPGEN_USE_RMT

#define STEP_0_HIGH  REGISTER_WRITE(GPIO_OUT_W1TS_REG, 1UL << STEP_0_PIN)
#define STEP_1_HIGH  REGISTER_WRITE(GPIO_OUT_W1TS_REG, 1UL << STEP_1_PIN)
#define STEP_2_HIGH  REGISTER_WRITE(GPIO_OUT_W1TS_REG, 1UL << STEP_2_PIN)

#define STEP_0_LOW   REGISTER_WRITE(GPIO_OUT_W1TC_REG, 1UL << STEP_0_PIN)
#define STEP_1_LOW   REGISTER_WRITE(GPIO_OUT_W1TC_REG, 1UL << STEP_1_PIN)
#define STEP_2_LOW   REGISTER_WRITE(GPIO_OUT_W1TC_REG, 1UL << STEP_2_PIN)

#endif

#define DIR_0_HIGH   REGISTER_WRITE(GPIO_OUT_W1TS_REG, 1UL << DIR_0_PIN)
#define DIR_1_HIGH   REGISTER_WRITE(GPIO_OUT_W1TS_REG, 1UL << DIR_1_PIN)
#define DIR_2_HIGH   REGISTER_WRITE(GPIO_OUT_W1TS_REG, 1UL << DIR_2_PIN)

#define DIR_0_LOW    REGISTER_WRITE(GPIO_OUT_W1TC_REG, 1UL << DIR_0_PIN)
#define DIR_1_LOW    REGISTER_WRITE(GPIO_OUT_W1TC_REG, 1UL << DIR_1_PIN)
#define DIR_2_LOW    REGISTER_WRITE(GPIO_OUT_W1TC_REG, 1UL << DIR_2_PIN)

/* Timer alarm / interrupt helpers */

#define TMR_0_ALM    (*(volatile uint32_t*)TIMG_T0ALARMLO_REG(0))
#define TMR_1_ALM    (*(volatile uint32_t*)TIMG_T1ALARMLO_REG(0))
#define TMR_2_ALM    (*(volatile uint32_t*)TIMG_T0ALARMLO_REG(1))

#define TMR_0_EN     REGISTER_SET_BIT(TIMG_T0CONFIG_REG(0), TIMG_T0_ALARM_EN)
#define TMR_1_EN     REGISTER_SET_BIT(TIMG_T1CONFIG_REG(0), TIMG_T1_ALARM_EN)
#define TMR_2_EN     REGISTER_SET_BIT(TIMG_T0CONFIG_REG(1), TIMG_T0_ALARM_EN)

#define TMR_0_CLR    REGISTER_WRITE(TIMG_INT_CLR_TIMERS_REG(0), TIMG_T0_INT_CLR)
#define TMR_1_CLR    REGISTER_WRITE(TIMG_INT_CLR_TIMERS_REG(0), TIMG_T1_INT_CLR)
#define TMR_2_CLR    REGISTER_WRITE(TIMG_INT_CLR_TIMERS_REG(1), TIMG_T0_INT_CLR)

/* RMT configuration (STEPGEN_USE_RMT) */

#if STEPGEN_USE_RMT

/*
*  RMT base configuration:
*   - APB clock (80 MHz)
*   - Divider = 2  -> 25 ns tick
*   - One memory block per channel
*/
#define RMT_CONF0_BASE_VAL ((2 << RMT_DIV_CNT_CH0_S) | (1 << RMT_MEM_SIZE_CH0_S))

/*
*  RMT channel CONF1 base:
*   - Idle output enabled
*   - Idle level = LOW
*   - APB clock always on
*/
#define RMT_CONF1_BASE_VAL (RMT_IDLE_OUT_EN_CH0 | RMT_REF_ALWAYS_ON_CH0)

/* Reset memory pointer and start transmission */
#define RMT_CONF1_RESETnSTART (RMT_CONF1_BASE_VAL | RMT_MEM_RD_RST_CH0 | RMT_TX_START_CH0)

/* Channel register helpers */
#define RMT_CH_CONF0_REG(ch) (RMT_CH0CONF0_REG + ((ch) * 8))
#define RMT_CH_CONF1_REG(ch) (RMT_CH0CONF1_REG + ((ch) * 8))

#endif /* STEPGEN_USE_RMT */
