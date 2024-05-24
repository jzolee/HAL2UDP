/*    Hardware:
 *
 *           3v3 -- 3.3v W5500
 *           GND -- GND  W5500
 *           GND -- (DSUB - 7)
 *           GND -- (DSUB - 8)
 *
 *       GPIO  1 -> OUT-03 or PWM-03
 *       GPIO  2 -> OUT-00 or PWM-00  -[100R]-(DSUB - 9) & onboard blue LED
 *       GPIO  4 -> OUT-01 or PWM-01  -[100R]-(DSUB -10)
 *       GPIO  5 -> W5500 SCS
 *       GPIO 12 -> step-0            -[100R]-(DSUB - 1)
 *       GPIO 13 -> dir-0             -[100R]-(DSUB - 2)
 *       GPIO 14 -> OUT-04 or PWM-04
 *       GPIO 15 -> OUT-05 or PWM-05
 * (RX2) GPIO 16 -> step-1            -[100R]-(DSUB - 3)
 * (TX2) GPIO 17 -> dir-1             -[100R]-(DSUB - 4)
 *       GPIO 18 -> W5500 SCLK
 *       GPIO 19 <- W5500 MISO
 *       GPIO 21 -> step-2            -[100R]-(DSUB - 5)
 *       GPIO 22 -> dir-2             -[100R]-(DSUB - 6)
 *       GPIO 23 -> W5500 MOSI
 *       GPIO 25 -> OUT-02 or PWM-02  -[100R]-(DSUB -11)
 *       GPIO 26 <- IN-00 {pullup}    -[100R]-(DSUB -12)
 *       GPIO 27 <- IN-01 {pullup}    -[100R]-(DSUB -13)
 *       GPIO 32 <- IN-02 {pullup}    -[100R]-(DSUB -14)
 *       GPIO 33 <- IN-03 {pullup}    -[100R]-(DSUB -15)
 *       GPIO 34 <- IN-04 {no pullup!}
 *       GPIO 35 <- IN-05 {no pullup!}
 *  (VP) GPIO 36 <- IN-06 {no pullup!}
 *  (VN) GPIO 39 <- W5500 INT
 */

/*  WROOM 32 GPIO NOTES
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

#pragma once

#include "soc/gpio_reg.h"

// write value to register
#ifndef REGISTER_WRITE
#define REGISTER_WRITE(_r, _v) ((*(volatile uint32_t*)(_r)) = (_v))
#endif

//get bit or get bits from register
#ifndef REGISTER_GET_BIT
#define REGISTER_GET_BIT(_r, _b) (*(volatile uint32_t*)(_r) & (_b))
#endif

/*==================================================================*/

#define SPI_CLOCK_MHZ 8
#define SPI_CS_PIN 5
#define W5500_INT_GPIO 39

/*==================================================================*/

#define STEP_0_PIN 12
#define STEP_0_H REGISTER_WRITE(GPIO_OUT_W1TS_REG, BIT12)
#define STEP_0_L REGISTER_WRITE(GPIO_OUT_W1TC_REG, BIT12)
#define DIR_0_PIN 13
#define DIR_0_H REGISTER_WRITE(GPIO_OUT_W1TS_REG, BIT13)
#define DIR_0_L REGISTER_WRITE(GPIO_OUT_W1TC_REG, BIT13)

#define STEP_1_PIN 16
#define STEP_1_H REGISTER_WRITE(GPIO_OUT_W1TS_REG, BIT16)
#define STEP_1_L REGISTER_WRITE(GPIO_OUT_W1TC_REG, BIT16)
#define DIR_1_PIN 17
#define DIR_1_H REGISTER_WRITE(GPIO_OUT_W1TS_REG, BIT17)
#define DIR_1_L REGISTER_WRITE(GPIO_OUT_W1TC_REG, BIT17)

#define STEP_2_PIN 21
#define STEP_2_H REGISTER_WRITE(GPIO_OUT_W1TS_REG, BIT21)
#define STEP_2_L REGISTER_WRITE(GPIO_OUT_W1TC_REG, BIT21)
#define DIR_2_PIN 22
#define DIR_2_H REGISTER_WRITE(GPIO_OUT_W1TS_REG, BIT22)
#define DIR_2_L REGISTER_WRITE(GPIO_OUT_W1TC_REG, BIT22)

/*==================================================================*/

#define OUT_00_PIN 2
#define OUT_00_H REGISTER_WRITE(GPIO_OUT_W1TS_REG, BIT2)
#define OUT_00_L REGISTER_WRITE(GPIO_OUT_W1TC_REG, BIT2)
#define OUT_01_PIN 4
#define OUT_01_H REGISTER_WRITE(GPIO_OUT_W1TS_REG, BIT4)
#define OUT_01_L REGISTER_WRITE(GPIO_OUT_W1TC_REG, BIT4)
#define OUT_02_PIN 25
#define OUT_02_H REGISTER_WRITE(GPIO_OUT_W1TS_REG, BIT25)
#define OUT_02_L REGISTER_WRITE(GPIO_OUT_W1TC_REG, BIT25)
#define OUT_03_PIN 1
#define OUT_03_H REGISTER_WRITE(GPIO_OUT_W1TS_REG, BIT1)
#define OUT_03_L REGISTER_WRITE(GPIO_OUT_W1TC_REG, BIT1)
#define OUT_04_PIN 14
#define OUT_04_H REGISTER_WRITE(GPIO_OUT_W1TS_REG, BIT14)
#define OUT_04_L REGISTER_WRITE(GPIO_OUT_W1TC_REG, BIT14)
#define OUT_05_PIN 15
#define OUT_05_H REGISTER_WRITE(GPIO_OUT_W1TS_REG, BIT15)
#define OUT_05_L REGISTER_WRITE(GPIO_OUT_W1TC_REG, BIT15)

/*==================================================================*/

#define IN_00_PIN 26
#define IN_00 REGISTER_GET_BIT(GPIO_IN_REG, BIT26) // 26
#define IN_01_PIN 27
#define IN_01 REGISTER_GET_BIT(GPIO_IN_REG, BIT27) // 27
#define IN_02_PIN 32
#define IN_02 REGISTER_GET_BIT(GPIO_IN1_REG, BIT0) // 32 -32
#define IN_03_PIN 33
#define IN_03 REGISTER_GET_BIT(GPIO_IN1_REG, BIT1) // 33 -32
#define IN_04_PIN 34
#define IN_04 REGISTER_GET_BIT(GPIO_IN1_REG, BIT2) // 34 -32
#define IN_05_PIN 35
#define IN_05 REGISTER_GET_BIT(GPIO_IN1_REG, BIT3) // 35 -32
#define IN_06_PIN 36
#define IN_06 REGISTER_GET_BIT(GPIO_IN1_REG, BIT4) // 36 -32

/*==================================================================*/