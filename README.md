# HAL2UDP
External step generator and IO interface for LinuxCNC over Ethernet with dual-core ESP32 and W5500 modules.

Settings:
--------
set your eth0 to 192.168.96.55
(the Esp32 with Ethernet will have 192.168.96.54)

LinuxCNC driver:
----------------
install linuxcnc-uspace-dev
install build-essential

halcompile --install udp.comp

Hardware:
---------
ESP32 3v3 -- 3.3v W5500
ESP32 GND -- GND  W5500
 
  GPIO  1 -> OUT-03 or PWM-03
  GPIO  2 -> OUT-00 or PWM-00 & onboard blue LED
  GPIO  4 -> OUT-01 or PWM-01
  GPIO  5 -> W5500 SCS
  GPIO 12 -> step-0
  GPIO 13 -> dir-0
  GPIO 14 -> OUT-04 or PWM-04
  GPIO 15 -> OUT-05 or PWM-05
  GPIO 16 -> step-1
  GPIO 17 -> dir-1
  GPIO 18 -> W5500 SCLK
  GPIO 19 <- W5500 MISO
  GPIO 21 -> step-2
  GPIO 22 -> dir-2
  GPIO 23 -> W5500 MOSI
  GPIO 25 -> OUT-02 or PWM-02
  GPIO 26 <- IN-00 {pullup}
  GPIO 27 <- IN-01 {pullup}
  GPIO 32 <- IN-02 {pullup}
  GPIO 33 <- IN-03 {pullup}
  GPIO 34 <- IN-04 {no pullup!}
  GPIO 35 <- IN-05 {no pullup!}
  GPIO 36 <- IN-06 {no pullup!}
  GPIO 39 <- IN-07 {no pullup!}
