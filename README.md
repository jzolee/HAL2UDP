# HAL2UDP
External step generator and IO interface for LinuxCNC over Ethernet with dual-core ESP32 and W5500 modules.<br>

The hardware is connected to LinuxCNC over Ethernet. The controller operates in position mode at low speed and at higher speeds in velocity mode.

### Features
* **step** and **dir** signals for 3 axes
* 8 input pins
* 6 output pins, any can be pwm signal
* step frequency up to 40 kHz
### Install
Clone this repository and open it with platformIO
### Settings
set your eth0 to 192.168.96.55<br>
(the Esp32 with Ethernet will have 192.168.96.54)
### LinuxCNC driver
```bash
sudo apt-get install linuxcnc-uspace-dev build-essential
```
```bash
sudo halcompile --install udp.comp
```
### Hardware
ESP-WROOM-32 dev board + W5500 ethernet module<br>

`ESP32 3v3 -- 3v3 W5500`<br>
`ESP32 GND -- GND W5500`<br>
 
`GPIO  1 -> OUT-03 or PWM-03`<br>
`GPIO  2 -> OUT-00 or PWM-00 & onboard blue LED`<br>
`GPIO  4 -> OUT-01 or PWM-01`<br>
`GPIO  5 -> W5500 SCS`<br>
`GPIO 12 -> step-0`<br>
`GPIO 13 -> dir-0`<br>
`GPIO 14 -> OUT-04 or PWM-04`<br>
`GPIO 15 -> OUT-05 or PWM-05`<br>
`GPIO 16 -> step-1`<br>
`GPIO 17 -> dir-1`<br>
`GPIO 18 -> W5500 SCLK`<br>
`GPIO 19 <- W5500 MISO`<br>
`GPIO 21 -> step-2`<br>
`GPIO 22 -> dir-2`<br>
`GPIO 23 -> W5500 MOSI`<br>
`GPIO 25 -> OUT-02 or PWM-02`<br>
`GPIO 26 <- IN-00 {pullup}`<br>
`GPIO 27 <- IN-01 {pullup}`<br>
`GPIO 32 <- IN-02 {pullup}`<br>
`GPIO 33 <- IN-03 {pullup}`<br>
`GPIO 34 <- IN-04 {no pullup!}`<br>
`GPIO 35 <- IN-05 {no pullup!}`<br>
`GPIO 36 <- IN-06 {no pullup!}`<br>
`GPIO 39 <- IN-07 {no pullup!}`<br>
### LinuxCNC HAL pins
udp.0.position_cmd (in - float) commanded position in position units<br>
udp.1.position_cmd<br>
udp.2.position_cmd<br>

udp.0.velocity_cmd (in - float) commanded velocity in position units/s<br>
udp.1.velocity_cmd<br>
udp.2.velocity_cmd<br>

udp.0.position_fb (out - float) feedback position in position units<br>
udp.1.position_fb<br>
udp.2.position_fb<br>

udp.0.velocity_fb (out - float) feedback velocity in position units/s<br>
udp.1.velocity_fb<br>
udp.2.velocity_fb<br>

udp.out.00 (in - bit) digital output<br>
udp.out.01<br>
udp.out.02<br>
udp.out.03<br>
udp.out.04<br>
udp.out.05<br>

udp.pwm.00 (in - float) PWM output 0...1<br>
udp.pwm.01<br>
udp.pwm.02<br>
udp.pwm.03<br>
udp.pwm.04<br>
udp.pwm.05<br>

udp.in.00 (out - bit) digital input<br>
udp.in.01<br>
udp.in.02<br>
udp.in.03<br>
udp.in.04<br>
udp.in.05<br>
udp.in.06<br>
udp.in.07<br>

udp.ready (out - bit) module state<br>
udp.enable (in - bit) module enable<br>
udp.packets (out - s32) lost packets<br>
### LinuxCNC HAL parameters
udp.0.dirsetup (rw - u32) minimum time from a direction change to the beginning of the next step pulse in ns<br>
udp.1.dirsetup<br>
udp.2.dirsetup<br>

udp.0.scale (rw - float) steps per position unit<br>
udp.1.scale<br>
udp.2.scale<br>

udp.0.accel (rw - float) acceleration in position units/s<sup>2</sup><br>
udp.1.accel<br>
udp.2.accel<br>

udp.pwm.00.freq (rw - u32) PWM frequency in Hz 0..65000<br>
udp.pwm.01.freq<br>
udp.pwm.02.freq<br>
udp.pwm.03.freq<br>
udp.pwm.04.freq<br>
udp.pwm.05.freq<br>
### PWM usage
If the udp.pwm.xx.freq parameter is set to 0, the udp.out.xx pin works and the udp.pwm.xx pin does not.
If the value of the udp.pwm.xx.freq parameter is not 0, the udp.out.xx pin does not work and the udp.pwm.xx pin does.

