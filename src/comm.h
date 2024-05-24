#pragma once

#include <Arduino.h>

#include "globals.h"

#include "hardware.h"

// #include <WiFi.h>

#include <Ethernet.h>

#include "w5100_mod.h" // Adapted from Ethernet library, adds access to socket IR registers

#include <EthernetUdp.h>

/*==================================================================*/

void IRAM_ATTR socket_isr()
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    vTaskNotifyGiveFromISR(comm_task_handle, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

inline void enableSIRs() { W5100.writeSIMR(0xFF); } // enable interrupts for all sockets

inline void disableSIRs() { W5100.writeSIMR(0x00); } // disable interrupts for all sockets

inline void clearSIRs() // After a socket IR, SnIR and SIR need to be reset
{
    for (int i = 0; i < 8; ++i)
        W5100.writeSnIR(i, 0xFF); // Clear socket i interrupt
    W5100.writeSIR(0xFF); // Clear SIR
}

void IRAM_ATTR outputHandler()
{
    static uint16_t last_pwm[6] = { 0 };

    for (int i = 0; i < 6; ++i) {

        bool enable = cmd.control & CTRL_ENABLE;

        if (pwm_enable[i] != 0) {
            if (enable) {
                if (last_pwm[i] != cmd.pwm[i]) {
                    last_pwm[i] = cmd.pwm[i];
                    ledcWrite(i * 2, last_pwm[i]);
                }
            } else {
                ledcWrite(i * 2, 0);
                last_pwm[i] = 0;
            }
        } else {
            switch (i) {
            case 0:
                enable ? ((cmd.io & IO_00) ? OUT_00_H : OUT_00_L) : OUT_00_L;
            case 1:
                enable ? ((cmd.io & IO_01) ? OUT_01_H : OUT_01_L) : OUT_01_L;
            case 2:
                enable ? ((cmd.io & IO_02) ? OUT_02_H : OUT_02_L) : OUT_02_L;
            case 3:
                enable ? ((cmd.io & IO_03) ? OUT_03_H : OUT_03_L) : OUT_03_L;
            case 4:
                enable ? ((cmd.io & IO_04) ? OUT_04_H : OUT_04_L) : OUT_04_L;
            case 5:
                enable ? ((cmd.io & IO_05) ? OUT_05_H : OUT_05_L) : OUT_05_L;
            }
        }
    }
}

void IRAM_ATTR inputHandler()
{
    (IN_00) ? fb.io = IO_00 : fb.io = 0;
    if (IN_01)
        fb.io |= IO_01;
    if (IN_02)
        fb.io |= IO_02;
    if (IN_03)
        fb.io |= IO_03;
    if (IN_04)
        fb.io |= IO_04;
    if (IN_05)
        fb.io |= IO_05;
    if (IN_06)
        fb.io |= IO_06;
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

        if ((fb.control & CTRL_DIRSETUP)
            && (fb.control & CTRL_ACCEL)
            && (fb.control & CTRL_PWMFREQ)) {
            fb.control |= CTRL_READY;

        } else if (cmd.control & CTRL_DIRSETUP) {
            fb.control |= CTRL_DIRSETUP;
            for (int i = 0; i < 3; ++i)
                dirSetup[i] = cmd.pos[i] / 25; //   25ns / timer tic

        } else if (cmd.control & CTRL_ACCEL) {
            fb.control |= CTRL_ACCEL;
            for (int i = 0; i < 3; ++i)
                accel_x2[i] = (float)cmd.pos[i] * 2.0f;

        } else if (cmd.control & CTRL_PWMFREQ) {
            fb.control |= CTRL_PWMFREQ;
            for (int i = 0; i < 6; ++i) {
                if (cmd.pwm[i] != 0) {
                    ledcAttachPin(out_pins[i], i * 2);
                    ledcSetup(i * 2, cmd.pwm[i], 10);
                    ledcWrite(i * 2, 0);
                    pwm_enable[i] = 1;
                } else {
                    pinMode(out_pins[i], OUTPUT);
                    digitalWrite(out_pins[i], LOW);
                }
            }
        }
    }
}

static void IRAM_ATTR watchdog_task(void* arg)
{
    TickType_t xLastWakeTime = xTaskGetTickCount();

    for (;;) {
        xTaskDelayUntil(&xLastWakeTime, 10);
        if (watchdog) {
            watchdog = 0;
        } else {
            fb.control = 0;
            cmd.control = 0;
            outputHandler();
        }
    }
}

static void IRAM_ATTR comm_task(void* arg)
{
    pinMode(IN_00_PIN, INPUT_PULLUP);
    pinMode(IN_01_PIN, INPUT_PULLUP);
    pinMode(IN_02_PIN, INPUT_PULLUP);
    pinMode(IN_03_PIN, INPUT_PULLUP);

    pinMode(IN_04_PIN, INPUT);
    pinMode(IN_05_PIN, INPUT);
    pinMode(IN_06_PIN, INPUT);

    pinMode(W5500_INT_GPIO, INPUT);

    EthernetUDP Udp;

    Ethernet.init(SPI_CS_PIN); /* You can use Ethernet.init(pin) to configure the CS pin */
    Ethernet.begin(mac, ip); /* start the Ethernet */

    Udp.begin(port); /* start UDP */

    //  https://forum.arduino.cc/t/getting-interrupts-from-ethernet-shield/673984
    //  Configure Wiznet interrupts:
    //  Need to select specific interrupt types for each socket. By default, all types are enabled.
    //  We will only use the RECV interrupt (bit 2 of SnIMR):
    for (int i = 0; i < 8; ++i)
        W5100.writeSnIMR(i, 0x04); // Socket IR mask: RECV for all sockets

    enableSIRs();

    // Configure socket_isr() as the interrupt service routine upon falling edge on W5500_INT_GPIO pin:
    attachInterrupt(digitalPinToInterrupt(W5500_INT_GPIO), socket_isr, FALLING); // For some reason, socket_isr gets called by this line...

    for (;;) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        if (Udp.parsePacket() == sizeof(cmd) + 1) {

            char buf[60];

            Udp.read(buf, sizeof(cmd) + 1);

            uint8_t chk = 71;
            for (int i = 0; i < sizeof(cmd); ++i)
                chk ^= buf[i];

            if (buf[sizeof(cmd)] == chk) {
                memcpy(&cmd, &buf, sizeof(cmd));
                commandHandler();
                ++watchdog;
            }

            inputHandler();

            for (int i = 0; i < 3; ++i) {
                uint32_t t = T_half[i];
                if (t)
                    dir[i] ? fb.vel[i] = 20000000.0f / (float)t : fb.vel[i] = -20000000.0f / (float)t;
                else
                    fb.vel[i] = 0.0f;
            }

            memcpy(&buf, &fb, sizeof(fb));
            Udp.beginPacket(Udp.remoteIP(), Udp.remotePort());
            Udp.write(buf, sizeof(fb));
            Udp.endPacket();

            outputHandler();
        }
        clearSIRs();
    }
}