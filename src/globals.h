#pragma once

#include "hardware.h"
#include <stdbool.h>
#include <stdint.h>

#define CTRL_DIRSETUP 0b00000001
#define CTRL_ACCEL 0b00000010
#define CTRL_PWMFREQ 0b00000100
#define CTRL_READY 0b01000000
#define CTRL_ENABLE 0b10000000

#define IO_00 0b00000001
#define IO_01 0b00000010
#define IO_02 0b00000100
#define IO_03 0b00001000
#define IO_04 0b00010000
#define IO_05 0b00100000
#define IO_06 0b01000000
#define IO_07 0b10000000

uint8_t mac[] = { 0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED };
IPAddress ip(192, 168, 96, 54);
uint16_t port = 58427;

const uint8_t out_pins[6] = { OUT_00_PIN, OUT_01_PIN, OUT_02_PIN, OUT_03_PIN, OUT_04_PIN, OUT_05_PIN };

struct cmdPacket {
    uint8_t control;
    uint8_t io;
    uint16_t pwm[6];
    int32_t pos[3];
    float vel[3];
} cmd = { 0 };

struct fbPacket {
    uint8_t control;
    uint8_t io;
    int32_t pos[3];
    float vel[3];
} fb = { 0 };

volatile uint32_t dirSetup[3] = { 1000, 1000, 1000 }; // x 25 nanosec
volatile float accel_x2[3] = { 1000.0f, 1000.0f, 1000.0f }; // acceleration*2 step/sec2

volatile uint32_t cmd_T_half[3] = { 0 };
volatile int cmd_dir[3] = { 0 };

volatile uint32_t T_half[3] = { 0 };
volatile int dir[3] = { 0 };
volatile int dirChange[3] = { 0 };
volatile int math[3] = { 0 };

volatile uint32_t watchdog = 0;

int pwm_enable[6] = { 0 };

uint32_t accelStep[3] = { 0 };

TaskHandle_t comm_task_handle = NULL;
