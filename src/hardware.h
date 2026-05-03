// file: hardware.h
#pragma once

/* ===================================================
*  Board-level hardware configuration and pin mapping.
*  ---------------------------------------------------
*  - pin assignments
*  - network configuration
*  - example wiring documentation
*  =================================================== */

#include <stdint.h>
#include "driver/gpio.h"
#include "esp_attr.h"

/* =====================
*  Network configuration
*  ===================== */
#define LOCAL_IP    {192, 168, 96, 54}
#define GATEWAY     {192, 168, 96, 1}
#define SUBNET      {255, 255, 255, 0}

/* =========================
*  W5500 configuration
*  ========================= */
#define W5500_INT_PIN 39

/* ===================
*  Step generator pins
*  =================== */
#define STEP_0_PIN     12
#define DIR_0_PIN      13

#define STEP_1_PIN     16
#define DIR_1_PIN      17

#define STEP_2_PIN     21
#define DIR_2_PIN      22

/* =====================
*  Digital / PWM outputs
*  ===================== */
static const gpio_num_t DRAM_ATTR out_pins[] = { 2, 4, 25, 1, 14, 15 };

/* ==============
*  Digital inputs
*  ============== */
static const gpio_num_t DRAM_ATTR in_pins[] = { 26, 27, 32, 33, 34, 35, 36 };


/* =================================================
*  Hardware reference wiring (example configuration)
*  -------------------------------------------------
*  This section documents a minimal, direct wiring setup used as a
*  reference design. It is NOT a hard requirement, but a working example.
*
*  Legend:
*  DSUB-x   : External connector pin
*  -[100R]- : Series resistor for current limiting / protection
*
*            3v3 -- 3.3v W5500
*            GND -- GND  W5500
*            GND -- (DSUB - 7)
*            GND -- (DSUB - 8)
*
*        GPIO  1 -> OUT-03 or PWM-03
*        GPIO  2 -> OUT-00 or PWM-00  -[100R]-(DSUB - 9) & onboard blue LED
*        GPIO  4 -> OUT-01 or PWM-01  -[100R]-(DSUB -10)
*        GPIO  5 -> W5500 SCS
*        GPIO 12 -> step-0            -[100R]-(DSUB - 1)
*        GPIO 13 -> dir-0             -[100R]-(DSUB - 2)
*        GPIO 14 -> OUT-04 or PWM-04
*        GPIO 15 -> OUT-05 or PWM-05
*  (RX2) GPIO 16 -> step-1            -[100R]-(DSUB - 3)
*  (TX2) GPIO 17 -> dir-1             -[100R]-(DSUB - 4)
*        GPIO 18 -> W5500 SCLK
*        GPIO 19 <- W5500 MISO
*        GPIO 21 -> step-2            -[100R]-(DSUB - 5)
*        GPIO 22 -> dir-2             -[100R]-(DSUB - 6)
*        GPIO 23 -> W5500 MOSI
*        GPIO 25 -> OUT-02 or PWM-02  -[100R]-(DSUB -11)
*        GPIO 26 <- IN-00 {pullup}    -[100R]-(DSUB -12)
*        GPIO 27 <- IN-01 {pullup}    -[100R]-(DSUB -13)
*        GPIO 32 <- IN-02 {pullup}    -[100R]-(DSUB -14)
*        GPIO 33 <- IN-03 {pullup}    -[100R]-(DSUB -15)
*        GPIO 34 <- IN-04 {no pullup!}
*        GPIO 35 <- IN-05 {no pullup!}
*   (VP) GPIO 36 <- IN-06 {no pullup!}
*   (VN) GPIO 39 <- W5500 INT {no pullup!}
*/


/* ==========================
*  ESP32-WROOM-32 GPIO notes
*  --------------------------
*  source: https://randomnerdtutorials.com/esp32-pinout-reference-gpios/
*  ---------------------------------------------------------------------
*
*   GPIO    Input           Output      Notes
*
*   0     ? pulled up ?   ? OK ?        outputs PWM signal at boot
*   1    !! TX pin !!     ? OK ?        debug output at boot
*   2       OK              OK          connected to on-board LED
*   3     ? OK ?         !! RX pin !!   HIGH at boot
*   4       OK              OK
*   5       OK              OK          outputs PWM signal at boot
*   6-11    X               X           connected to the integrated SPI flash
*   12    ? OK ?            OK          boot fail if pulled high
*   13      OK              OK
*   14      OK              OK          outputs PWM signal at boot
*   15      OK              OK          outputs PWM signal at boot
*   16      OK              OK
*   17      OK              OK
*   18      OK              OK
*   19      OK              OK
*   21      OK              OK
*   22      OK              OK
*   23      OK              OK
*   25      OK              OK
*   26      OK              OK
*   27      OK              OK
*   32      OK              OK
*   33      OK              OK
*   34      OK              --          input only, no pullup
*   35      OK              --          input only, no pullup
*   36      OK              --          input only, no pullup
*   39      OK              --          input only, no pullup
*/
