/*    External step generator and IO interface for LinuxCNC over Ethernet
 *    with dual-core ESP32 and W5500 modules.
 *
 *    Copyright 2022 Juhász Zoltán <juhasz.zoltan at freemail dot hu>
 * 
 *    version:20220330
 *
 *    This program is free software; you can redistribute it and/or modify
 *    it under the terms of the GNU General Public License as published by
 *    the Free Software Foundation; either version 2 of the License, or
 *    (at your option) any later version.
 *
 *    This program is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *    GNU General Public License for more details.
 *
 *    You should have received a copy of the GNU General Public License
 *    along with this program; if not, write to the Free Software
 *    Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 */

/*    Hardware:
 *
 *           3v3 -- 3.3v W5500
 *           GND -- GND  W5500
 *           GND -- (DB15- 7)
 *           GND -- (DB15- 8)
 * 
 *       GPIO  1 -> OUT-03 or PWM-03
 *       GPIO  2 -> OUT-00 or PWM-00  -[100R]-(DB15 - 9) & onboard blue LED
 *       GPIO  4 -> OUT-01 or PWM-01  -[100R]-(DB15 -10)
 *       GPIO  5 -> W5500 SCS
 *       GPIO 12 -> step-0            -[100R]-(DB15 - 1)
 *       GPIO 13 -> dir-0             -[100R]-(DB15 - 2)
 *       GPIO 14 -> OUT-04 or PWM-04
 *       GPIO 15 -> OUT-05 or PWM-05
 * (RX2) GPIO 16 -> step-1            -[100R]-(DB15 - 3)
 * (TX2) GPIO 17 -> dir-1             -[100R]-(DB15 - 4)
 *       GPIO 18 -> W5500 SCLK
 *       GPIO 19 <- W5500 MISO
 *       GPIO 21 -> step-2            -[100R]-(DB15 - 5)
 *       GPIO 22 -> dir-2             -[100R]-(DB15 - 6)
 *       GPIO 23 -> W5500 MOSI
 *       GPIO 25 -> OUT-02 or PWM-02  -[100R]-(DB15 -11)
 *       GPIO 26 <- IN-00 {pullup}    -[100R]-(DB15 -12)
 *       GPIO 27 <- IN-01 {pullup}    -[100R]-(DB15 -13)
 *       GPIO 32 <- IN-02 {pullup}    -[100R]-(DB15 -14)
 *       GPIO 33 <- IN-03 {pullup}    -[100R]-(DB15 -15)
 *       GPIO 34 <- IN-04 {no pullup!}
 *       GPIO 35 <- IN-05 {no pullup!}
 *  (VP) GPIO 36 <- IN-06 {no pullup!}
 *  (VN) GPIO 39 <- IN-07 {no pullup!}
 */

/*
 *  WROOM 32 GPIO NOTES
 *
 *  https://randomnerdtutorials.com/esp32-pinout-reference-gpios/
 *
 *  GPIO    Input           Output      Notes
 *
 *  0     ? pulled up ?   ? OK ?        outputs PWM signal at boot
 *  1    !! TX pin !!     ? OK ?        debug output at boot
 *  2       OK              OK          connected to on-board LED
 *  3     ? OK ?         !! RX pin !!   HIGH at boot
 *  4       OK              OK
 *  5       OK              OK          outputs PWM signal at boot
 *  6-11    X               X           connected to the integrated SPI flash
 *  12    ? OK ?            OK          boot fail if pulled high
 *  13      OK              OK
 *  14      OK              OK          outputs PWM signal at boot
 *  15      OK              OK          outputs PWM signal at boot
 *  16      OK              OK
 *  17      OK              OK
 *  18      OK              OK
 *  19      OK              OK
 *  21      OK              OK
 *  22      OK              OK
 *  23      OK              OK
 *  25      OK              OK
 *  26      OK              OK
 *  27      OK              OK
 *  32      OK              OK
 *  33      OK              OK
 *  34      OK              --          input only, no pullup
 *  35      OK              --          input only, no pullup
 *  36      OK              --          input only, no pullup
 *  39      OK              --          input only, no pullup
 */

#include <Arduino.h>

#include <Ethernet.h>
#include <EthernetUdp.h>

/*==================================================================*/

byte mac[] = { 0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED };
IPAddress ip(192, 168, 96, 54);
unsigned int port = 58427;

/*==================================================================*/

#define SPI_CS_PIN 5 // hardcoded in Ethernet_mod/src/utility/w5100.h !!!!!!!!!!!!!!

/*==================================================================*/

#define STEP_0_PIN 12
#define STEP_0_H REG_WRITE(GPIO_OUT_W1TS_REG, BIT12)
#define STEP_0_L REG_WRITE(GPIO_OUT_W1TC_REG, BIT12)
#define DIR_0_PIN 13
#define DIR_0_H REG_WRITE(GPIO_OUT_W1TS_REG, BIT13)
#define DIR_0_L REG_WRITE(GPIO_OUT_W1TC_REG, BIT13)

#define STEP_1_PIN 16
#define STEP_1_H REG_WRITE(GPIO_OUT_W1TS_REG, BIT16)
#define STEP_1_L REG_WRITE(GPIO_OUT_W1TC_REG, BIT16)
#define DIR_1_PIN 17
#define DIR_1_H REG_WRITE(GPIO_OUT_W1TS_REG, BIT17)
#define DIR_1_L REG_WRITE(GPIO_OUT_W1TC_REG, BIT17)

#define STEP_2_PIN 21
#define STEP_2_H REG_WRITE(GPIO_OUT_W1TS_REG, BIT21)
#define STEP_2_L REG_WRITE(GPIO_OUT_W1TC_REG, BIT21)
#define DIR_2_PIN 22
#define DIR_2_H REG_WRITE(GPIO_OUT_W1TS_REG, BIT22)
#define DIR_2_L REG_WRITE(GPIO_OUT_W1TC_REG, BIT22)

/*==================================================================*/

#define OUT_00_PIN 2
#define OUT_00_H REG_WRITE(GPIO_OUT_W1TS_REG, BIT2)
#define OUT_00_L REG_WRITE(GPIO_OUT_W1TC_REG, BIT2)
#define OUT_01_PIN 4
#define OUT_01_H REG_WRITE(GPIO_OUT_W1TS_REG, BIT4)
#define OUT_01_L REG_WRITE(GPIO_OUT_W1TC_REG, BIT4)
#define OUT_02_PIN 25
#define OUT_02_H REG_WRITE(GPIO_OUT_W1TS_REG, BIT25)
#define OUT_02_L REG_WRITE(GPIO_OUT_W1TC_REG, BIT25)
#define OUT_03_PIN 1
#define OUT_03_H REG_WRITE(GPIO_OUT_W1TS_REG, BIT1)
#define OUT_03_L REG_WRITE(GPIO_OUT_W1TC_REG, BIT1)
#define OUT_04_PIN 14
#define OUT_04_H REG_WRITE(GPIO_OUT_W1TS_REG, BIT14)
#define OUT_04_L REG_WRITE(GPIO_OUT_W1TC_REG, BIT14)
#define OUT_05_PIN 15
#define OUT_05_H REG_WRITE(GPIO_OUT_W1TS_REG, BIT15)
#define OUT_05_L REG_WRITE(GPIO_OUT_W1TC_REG, BIT15)

/*==================================================================*/

#define IN_00_PIN 26
#define IN_00 REG_READ(GPIO_IN_REG) & BIT26 //26
#define IN_01_PIN 27
#define IN_01 REG_READ(GPIO_IN_REG) & BIT27 //27
#define IN_02_PIN 32
#define IN_02 REG_READ(GPIO_IN1_REG) & BIT0 //32 -32
#define IN_03_PIN 33
#define IN_03 REG_READ(GPIO_IN1_REG) & BIT1 //33 -32
#define IN_04_PIN 34
#define IN_04 REG_READ(GPIO_IN1_REG) & BIT2 //34 -32
#define IN_05_PIN 35
#define IN_05 REG_READ(GPIO_IN1_REG) & BIT3 //35 -32
#define IN_06_PIN 36
#define IN_06 REG_READ(GPIO_IN1_REG) & BIT4 //36 -32
#define IN_07_PIN 39
#define IN_07 REG_READ(GPIO_IN1_REG) & BIT7 //39 -32

/*==================================================================*/

#define CTRL_DIRSETUP 0b00000001
#define CTRL_ACCEL    0b00000010
#define CTRL_PWMFREQ  0b00000100
#define CTRL_READY    0b01000000
#define CTRL_ENABLE   0b10000000

#define IO_00 0b00000001
#define IO_01 0b00000010
#define IO_02 0b00000100
#define IO_03 0b00001000
#define IO_04 0b00010000
#define IO_05 0b00100000
#define IO_06 0b01000000
#define IO_07 0b10000000

/*==================================================================*/

EthernetUDP Udp; // An EthernetUDP instance to let us send and receive packets over UDP

struct cmdPacket {
    uint8_t control;
    uint8_t io;
    uint16_t pwm[6];
    int32_t pos[3];
    float vel[3];
} cmd = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0.0f, 0.0f, 0.0f };

struct fbPacket {
    uint8_t control;
    uint8_t io;
    int32_t pos[3];
    float vel[3];
} fb = { 0, 0, 0, 0, 0, 0.0f, 0.0f, 0.0f };

/*==================================================================*/

volatile unsigned long ul_dirSetup[3] = { 1000, 1000, 1000 }; // x 25 nanosec
volatile float f_accel_x2[3] = { 1000.0, 1000.0, 1000.0 }; // acceleration*2 step/sec2

/*==================================================================*/

volatile unsigned long ul_cmd_T[3];

unsigned long ul_accelStep[3] = { 0, 0, 0 };

volatile unsigned long ul_T[3] = { 0, 0, 0 };
volatile unsigned long ul_TH[3] = { 0, 0, 0 };
volatile unsigned long ul_TL[3] = { 0, 0, 0 };
volatile bool b_math[3] = { false, false, false };
volatile bool b_dirSignal[3] = { LOW, LOW, LOW };
volatile bool b_dirChange[3] = { false, false, false };

const uint8_t pwm_pin[6] = { OUT_00_PIN, OUT_01_PIN, OUT_02_PIN, OUT_03_PIN, OUT_04_PIN, OUT_05_PIN };
bool pwm_enable[6] = { false, false, false, false, false, false };

/*==================================================================*/

hw_timer_t* stepGen_0 = NULL;
hw_timer_t* stepGen_1 = NULL;
hw_timer_t* stepGen_2 = NULL;

/*==================================================================*/
float IRAM_ATTR fastInvSqrt(const float x)
{
    const float xhalf = x * 0.5f;
    union {
        float x;
        uint32_t i;
    } u = { .x = x };
    u.i = 0x5f3759df - (u.i >> 1);
    return u.x * (1.5f - xhalf * u.x * u.x);
}
/*==================================================================*/
/*float IRAM_ATTR Q_InvSqrt(float x)
{
    float xhalf = x * 0.5f;
    int i = *(int*)&x; // evil floating point bit level hacking
    i = 0x5f3759df - (i >> 1); // what the fuck?
    x = *(float*)&i; // convert new bits into float
    x = x * (1.5f - xhalf * x * x); // 1st iteration of Newton's method
    //x = x * (1.5f - xhalf * x * x); // 2nd iteration, this can be removed
    return x;
}*/
/*==================================================================*/
void IRAM_ATTR onTime_0()
{
    static bool b_stepState = LOW;
    static bool b_dirState = LOW;

    if (b_stepState == LOW) { // end of period
        if (ul_T[0]) { // there is a period time so a pulse must be started
            if (!b_dirChange[0]) { //direction is good, start a new pulse (step L-H transition)
                STEP_0_H;
                timerAlarmWrite(stepGen_0, ul_TH[0], true);
                b_stepState = HIGH;
                b_dirState ? fb.pos[0]++ : fb.pos[0]--;
                b_math[0] = true; // calculation of a new period time
            } else { // need to change direction
                if (b_dirSignal[0]) {
                    DIR_0_H;
                    b_dirState = HIGH;
                } else {
                    DIR_0_L;
                    b_dirState = LOW;
                }
                timerAlarmWrite(stepGen_0, ul_dirSetup[0], true);
                b_dirChange[0] = false;
            }
        } else { // no need to step pulse
            timerAlarmWrite(stepGen_0, 4000ul, true); // 0,1 millis scan
            b_math[0] = true; // calculation of a new period time
        }
    } else { // the middle of the period time (step H-L transition)
        STEP_0_L;
        timerAlarmWrite(stepGen_0, ul_TL[0], true);
        b_stepState = LOW;
    }
}
/*==================================================================*/
void IRAM_ATTR onTime_1()
{
    static bool b_stepState = LOW;
    static bool b_dirState = LOW;

    if (b_stepState == LOW) { // end of period
        if (ul_T[1]) { // there is a period time so a pulse must be started
            if (!b_dirChange[1]) { //direction is good, start a new pulse (step L-H transition)
                STEP_1_H;
                timerAlarmWrite(stepGen_1, ul_TH[1], true);
                b_stepState = HIGH;
                b_dirState ? fb.pos[1]++ : fb.pos[1]--;
                b_math[1] = true; // calculation of a new period time
            } else { // need to change direction
                if (b_dirSignal[1]) {
                    DIR_1_H;
                    b_dirState = HIGH;
                } else {
                    DIR_1_L;
                    b_dirState = LOW;
                }
                timerAlarmWrite(stepGen_1, ul_dirSetup[1], true);
                b_dirChange[1] = false;
            }
        } else { // no need to step pulse
            timerAlarmWrite(stepGen_1, 4000ul, true); // 0,1 millis scan
            b_math[1] = true; // calculation of a new period time
        }
    } else { // the middle of the period time (step H-L transition)
        STEP_1_L;
        timerAlarmWrite(stepGen_1, ul_TL[1], true);
        b_stepState = LOW;
    }
}
/*==================================================================*/
void IRAM_ATTR onTime_2()
{
    static bool b_stepState = LOW;
    static bool b_dirState = LOW;

    if (b_stepState == LOW) { // end of period
        if (ul_T[2]) { // there is a period time so a pulse must be started
            if (!b_dirChange[2]) { //direction is good, start a new pulse (step L-H transition)
                STEP_2_H;
                timerAlarmWrite(stepGen_2, ul_TH[2], true);
                b_stepState = HIGH;
                b_dirState ? fb.pos[2]++ : fb.pos[2]--;
                b_math[2] = true; // calculation of a new period time
            } else { // need to change direction
                if (b_dirSignal[2]) {
                    DIR_2_H;
                    b_dirState = HIGH;
                } else {
                    DIR_2_L;
                    b_dirState = LOW;
                }
                timerAlarmWrite(stepGen_2, ul_dirSetup[2], true);
                b_dirChange[2] = false;
            }
        } else { // no need to step pulse
            timerAlarmWrite(stepGen_2, 4000ul, true); // 0,1 millis scan
            b_math[2] = true; // calculation of a new period time
        }
    } else { // the middle of the period time (step H-L transition)
        STEP_2_L;
        timerAlarmWrite(stepGen_2, ul_TL[2], true);
        b_stepState = LOW;
    }
}
/*==================================================================*/
void IRAM_ATTR newT(const int i) // period time calculation
{
    //ul_T[i] = 40000000.0f / sqrtf((float)(ul_accelStep[i] * ul_accel_x2[i]));
    ul_T[i] = 40000000.0f * fastInvSqrt((float)ul_accelStep[i] * f_accel_x2[i]); // fast method (>3x)
    ul_TH[i] = ul_T[i] >> 1;
    ul_TL[i] = ul_T[i] - ul_TH[i];
}
/*==================================================================*/
void IRAM_ATTR deceleration(const int i)
{
    if (ul_accelStep[i]) {
        ul_accelStep[i]--;
        if (ul_accelStep[i])
            newT(i);
        else
            ul_T[i] = 0;
    }
}
/*==================================================================*/
void IRAM_ATTR acceleration(const int i)
{
    if (cmd.control & CTRL_ENABLE) {
        ul_accelStep[i]++;
        newT(i);
    } else
        deceleration(i);
}
/*==================================================================*/
void IRAM_ATTR outputHandler()
{
    static int last_pwm[6] = { 0, 0, 0, 0, 0, 0 };

    bool enable = cmd.control & CTRL_ENABLE;

    if (pwm_enable[0]) {
        if (enable) {
            if (last_pwm[0] != cmd.pwm[0]) {
                last_pwm[0] = cmd.pwm[0];
                ledcWrite(0, last_pwm[0]);
            }
        } else {
            ledcWrite(0, 0);
            last_pwm[0] = 0;
        }
    } else {
        if (enable)
            (cmd.io & IO_00) ? OUT_00_H : OUT_00_L;
        else
            OUT_00_L;
    }

    if (pwm_enable[1]) {
        if (enable) {
            if (last_pwm[1] != cmd.pwm[1]) {
                last_pwm[1] = cmd.pwm[1];
                ledcWrite(2, last_pwm[1]);
            }
        } else {
            ledcWrite(2, 0);
            last_pwm[1] = 0;
        }
    } else {
        if (enable)
            (cmd.io & IO_01) ? OUT_01_H : OUT_01_L;
        else
            OUT_01_L;
    }

    if (pwm_enable[2]) {
        if (enable) {
            if (last_pwm[2] != cmd.pwm[2]) {
                last_pwm[2] = cmd.pwm[2];
                ledcWrite(4, last_pwm[2]);
            }
        } else {
            ledcWrite(4, 0);
            last_pwm[2] = 0;
        }
    } else {
        if (enable)
            (cmd.io & IO_02) ? OUT_02_H : OUT_02_L;
        else
            OUT_02_L;
    }

    if (pwm_enable[3]) {
        if (enable) {
            if (last_pwm[3] != cmd.pwm[3]) {
                last_pwm[3] = cmd.pwm[3];
                ledcWrite(6, last_pwm[3]);
            }
        } else {
            ledcWrite(6, 0);
            last_pwm[3] = 0;
        }
    } else {
        if (enable)
            (cmd.io & IO_03) ? OUT_03_H : OUT_03_L;
        else
            OUT_03_L;
    }

    if (pwm_enable[4]) {
        if (enable) {
            if (last_pwm[4] != cmd.pwm[4]) {
                last_pwm[4] = cmd.pwm[4];
                ledcWrite(8, last_pwm[4]);
            }
        } else {
            ledcWrite(8, 0);
            last_pwm[4] = 0;
        }
    } else {
        if (enable)
            (cmd.io & IO_04) ? OUT_04_H : OUT_04_L;
        else
            OUT_04_L;
    }

    if (pwm_enable[5]) {
        if (enable) {
            if (last_pwm[5] != cmd.pwm[5]) {
                last_pwm[5] = cmd.pwm[5];
                ledcWrite(10, last_pwm[5]);
            }
        } else {
            ledcWrite(10, 0);
            last_pwm[5] = 0;
        }
    } else {
        if (enable)
            (cmd.io & IO_05) ? OUT_05_H : OUT_05_L;
        else
            OUT_05_L;
    }
}
/*==================================================================*/
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
    if (IN_07)
        fb.io |= IO_07;
}
/*==================================================================*/
void IRAM_ATTR commandHandler()
{
    if (cmd.control & CTRL_READY) {

        for (int i = 0; i < 3; i++) {

            if (cmd.vel[i] > 0.0f)
                ul_cmd_T[i] = (unsigned long)(40000000.0f / cmd.vel[i]);
            else if (cmd.vel[i] < 0.0f)
                ul_cmd_T[i] = (unsigned long)(40000000.0f / -cmd.vel[i]);
            else
                ul_cmd_T[i] = 40000000ul;
        }
    }

    if (!(fb.control & CTRL_READY)) {

        if ((fb.control & CTRL_DIRSETUP)
            && (fb.control & CTRL_ACCEL)
            && (fb.control & CTRL_PWMFREQ)) {
            fb.control |= CTRL_READY;
            //log_printf("READY\n");

        } else if (cmd.control & CTRL_DIRSETUP) {
            fb.control |= CTRL_DIRSETUP;
            for (int i = 0; i < 3; i++)
                ul_dirSetup[i] = cmd.pos[i] / 25; //   25ns / timer tic

        } else if (cmd.control & CTRL_ACCEL) {
            fb.control |= CTRL_ACCEL;
            for (int i = 0; i < 3; i++)
                f_accel_x2[i] = (float)cmd.pos[i] * 2.0;

        } else if (cmd.control & CTRL_PWMFREQ) {
            fb.control |= CTRL_PWMFREQ;
            for (int i = 0; i < 6; i++) {
                if (cmd.pwm[i]) {
                    ledcAttachPin(pwm_pin[i], i * 2);
                    ledcSetup(i * 2, cmd.pwm[i], 10);
                    ledcWrite(i * 2, 0);
                    pwm_enable[i] = true;
                } else {
                    if (i == 0) {
                        pinMode(OUT_00_PIN, OUTPUT);
                        digitalWrite(OUT_00_PIN, 0);
                    } else if (i == 1) {
                        pinMode(OUT_01_PIN, OUTPUT);
                        digitalWrite(OUT_01_PIN, 0);
                    } else if (i == 2) {
                        pinMode(OUT_02_PIN, OUTPUT);
                        digitalWrite(OUT_02_PIN, 0);
                    } else if (i == 3) {
                        pinMode(OUT_03_PIN, OUTPUT);
                        digitalWrite(OUT_03_PIN, 0);
                    } else if (i == 4) {
                        pinMode(OUT_04_PIN, OUTPUT);
                        digitalWrite(OUT_04_PIN, 0);
                    } else if (i == 5) {
                        pinMode(OUT_05_PIN, OUTPUT);
                        digitalWrite(OUT_05_PIN, 0);
                    }
                }
            }
        }
    }
}
/*==================================================================*/
void IRAM_ATTR loop_Core0(void* parameter)
{
    delay(500);
    Udp.parsePacket(); /* to empty buffer */

    for (;;) {

        static unsigned long ul_watchdog;

        if (Udp.parsePacket() == sizeof(cmd) + 1) {

            char packetBuffer[60]; // buffer for receiving and sending data

            Udp.read(packetBuffer, sizeof(cmd) + 1);

            uint8_t chk = 71;
            for (int i = 0; i < sizeof(cmd); i++)
                chk ^= packetBuffer[i];

            if (packetBuffer[sizeof(cmd)] == chk) {
                memcpy(&cmd, &packetBuffer, sizeof(cmd));
                commandHandler();
                ul_watchdog = millis();
            }

            inputHandler();

            for (int i = 0; i < 3; i++) {
                unsigned long ul_t = ul_T[i];
                if (ul_t)
                    b_dirSignal[i] ? fb.vel[i] = 40000000.0f / (float)ul_t : fb.vel[i] = -40000000.0f / (float)ul_t;
                else
                    fb.vel[i] = 0.0f;
            }

            memcpy(&packetBuffer, &fb, sizeof(fb));

            Udp.beginPacket(Udp.remoteIP(), Udp.remotePort());
            Udp.write(packetBuffer, sizeof(fb));
            Udp.endPacket();

            outputHandler();
        }

        if (millis() - ul_watchdog > 10ul) {
            fb.control = 0;
            cmd.control = 0;
            outputHandler();
            ul_watchdog = millis();
        }
    }
}
/*==================================================================*/
void setup_Core0(void* parameter)
{
    pinMode(IN_00_PIN, INPUT_PULLUP);
    pinMode(IN_01_PIN, INPUT_PULLUP);
    pinMode(IN_02_PIN, INPUT_PULLUP);
    pinMode(IN_03_PIN, INPUT_PULLUP);

    pinMode(IN_04_PIN, INPUT);
    pinMode(IN_05_PIN, INPUT);
    pinMode(IN_06_PIN, INPUT);
    pinMode(IN_07_PIN, INPUT);

    delay(500);
    Ethernet.init(SPI_CS_PIN); /* You can use Ethernet.init(pin) to configure the CS pin */
    Ethernet.begin(mac, ip); /* start the Ethernet */

    delay(500);
    Udp.begin(port); /* start UDP */

    delay(100);
    xTaskCreatePinnedToCore(
        loop_Core0, // Task function.
        "loopTask_Core0", // name of task.
        4096, // Stack size of task
        NULL, // parameter of the task
        0, // priority of the task
        NULL, // Task handle to keep track of created task
        0); // pin task to core 0

    disableCore0WDT();

    vTaskDelete(NULL);
}
/*==================================================================*/
void setup()
{
    pinMode(STEP_0_PIN, OUTPUT);
    digitalWrite(STEP_0_PIN, 0);
    pinMode(DIR_0_PIN, OUTPUT);
    digitalWrite(DIR_0_PIN, 0);
    pinMode(STEP_1_PIN, OUTPUT);
    digitalWrite(STEP_1_PIN, 0);
    pinMode(DIR_1_PIN, OUTPUT);
    digitalWrite(DIR_1_PIN, 0);
    pinMode(STEP_2_PIN, OUTPUT);
    digitalWrite(STEP_2_PIN, 0);
    pinMode(DIR_2_PIN, OUTPUT);
    digitalWrite(DIR_2_PIN, 0);

    // Configure the Prescaler
    // 80 000 000 / 2 = 40 000 000 tics/second ===>   25 ns/tic
    stepGen_0 = timerBegin(0, 2, true);
    stepGen_1 = timerBegin(1, 2, true);
    stepGen_2 = timerBegin(2, 2, true);
    timerAttachInterrupt(stepGen_0, &onTime_0, true);
    timerAttachInterrupt(stepGen_1, &onTime_1, true);
    timerAttachInterrupt(stepGen_2, &onTime_2, true);
    timerAlarmWrite(stepGen_0, 40000000, true);
    timerAlarmWrite(stepGen_1, 40000000, true);
    timerAlarmWrite(stepGen_2, 40000000, true);
    timerAlarmEnable(stepGen_0);
    timerAlarmEnable(stepGen_1);
    timerAlarmEnable(stepGen_2);

    xTaskCreatePinnedToCore(
        setup_Core0, // Task function.
        "setup_Core0Task", // name of task.
        1024, // Stack size of task
        NULL, // parameter of the task
        0, // priority of the task
        NULL, // Task handle to keep track of created task
        0); // pin task to core 0
}
/*==================================================================*/
void IRAM_ATTR loop()
{
    for (int i = 0; i < 3; i++) {
        if (b_math[i]) {
            b_math[i] = false;
            if (!ul_accelStep[i]) { // the axis is stationary
                long l_pos_error = cmd.pos[i] - fb.pos[i];
                if (l_pos_error) { // if there is a position error
                    if ((l_pos_error > 0 && b_dirSignal[i] == HIGH) || (l_pos_error < 0 && b_dirSignal[i] == LOW)) // the direction is good
                        acceleration(i);
                    else { // need to change direction
                        (l_pos_error > 0) ? b_dirSignal[i] = HIGH : b_dirSignal[i] = LOW;
                        b_dirChange[i] = true;
                    }
                }
            } else { // the axis moves
                if ((cmd.vel[i] > 0.0f && b_dirSignal[i] == HIGH) || (cmd.vel[i] < 0.0f && b_dirSignal[i] == LOW)) { // the direction is good
                    if (ul_T[i] > ul_cmd_T[i]) // the speed is low
                        acceleration(i);
                    else if (ul_T[i] < ul_cmd_T[i]) // the speed is high
                        deceleration(i);
                } else // the direction is wrong or the target speed is zero
                    deceleration(i);
            }
        }
    }
}
