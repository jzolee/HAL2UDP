// file: stepgen.h
#pragma once

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// ============================================================================
// Configuration switches
// ============================================================================

#define STEPGEN_USE_RMT  1
#define STEPGEN_USE_LUT  1

// ============================================================================
// Step generator command state
// ============================================================================

/**
 * @brief Commanded motion state (double-buffered).
 *
 * Written by communication task, read by step generator task.
 */
typedef struct {
    volatile int32_t  pos;   ///< Absolute target position (steps)
    volatile int32_t  dir;   ///< Direction (+1 / -1)
    volatile uint32_t T;     ///< Step period in timer ticks (25 ns)
} sg_cmd_t;

// ============================================================================
// Configuration
// ============================================================================
typedef struct {
    volatile uint32_t dirSetup;   ///< DIR setup time (ticks)
    volatile uint32_t T1;         ///< Initial step period
#if STEPGEN_USE_LUT
    volatile uint32_t accel;      ///< Acceleration (steps/s²)
#else
    volatile float    T_scale;    ///< Runtime acceleration scale
#endif
} sg_cfg_t;

// ============================================================================
// Step generator context
// ============================================================================

typedef struct __attribute__((aligned(32))) {

    // --- ISR DATA ---
    volatile int32_t  pos;
    volatile int32_t  dir;
    volatile uint32_t T;
    volatile uint32_t dir_change;
    volatile uint32_t math;
#if !STEPGEN_USE_RMT
    uint32_t step_state;
    uint32_t T_low;
#endif
    // --- CONFIG DATA ---
    uint32_t dirSetup;
    uint32_t T1;
#if STEPGEN_USE_LUT
    uint32_t accel;
#else
    float    T_scale;
#endif
    // --- TASK DATA  ---
    uint32_t idx;
#if STEPGEN_USE_LUT
    uint32_t dn_counter;
    uint32_t idx_max;
    uint32_t T_size;
    uint32_t* lut_T;
    uint32_t* lut_dn;
#endif
    // --- COMMAND (double buffered) ---
    volatile sg_cmd_t cmd[2];
    volatile sg_cmd_t* active_cmd;
} sg_ctx_t;

// ============================================================================
// Global instances
// ============================================================================

typedef enum {
    STEPGEN_INIT1,
    COMM_INIT,
    STEPGEN_INIT2
} start_seq_t;

extern volatile start_seq_t startup_sequence;
extern volatile uint32_t stepGen_enable;
extern sg_cfg_t sg_cfg[3];
extern sg_ctx_t sg_ctx[3];

// ============================================================================
// Task entry point
// ============================================================================

void start_stepgen_task(void);

#ifdef __cplusplus
}
#endif
