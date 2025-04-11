# HAL2UDP
External step generator and IO interface for LinuxCNC over Ethernet with dual-core ESP32 and W5500 modules.<br>

The hardware is connected to LinuxCNC over Ethernet. The controller operates in velocity mode.<br>

[video on operation](https://youtu.be/UXWcg7PwRJs)

### Changelog
The INT pin of the W5500 module has been connected to the GPIO39 pin of the esp32 module.

Therefore, the number of inputs is reduced to 7.

The code has been optimized to be faster, so a higher step frequency is possible.

ESP-IDF framework, native W5500 driver -> there is no external library.

The LinuxCNC HAL pins' and parameters' names have been changed for a better understanding.

(I couldn't test the pwm function of output pins, I hope they work)

### Features
* **step** and **dir** signals for 3 axes
* 7 input pins
* 6 output pins, any can be pwm signal
* step frequency up to 100 kHz
### Install
Clone/Copy this repository and open it with platformIO
### Settings
set your eth0 to 192.168.96.XX<br>
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
`GPIO 39 <- W5500 INT`<br>
### LinuxCNC HAL pins
udp.stepgen.0.position-cmd (in - float) commanded position in position units<br>
udp.stepgen.1.position-cmd<br>
udp.stepgen.2.position-cmd<br>

udp.stepgen.0.velocity-cmd (in - float) commanded velocity in position units/s<br>
udp.stepgen.1.velocity-cmd<br>
udp.stepgen.2.velocity-cmd<br>

udp.stepgen.0.position-fb (out - float) feedback position in position units<br>
udp.stepgen.1.position-fb<br>
udp.stepgen.2.position-fb<br>

udp.stepgen.0.velocity-fb (out - float) feedback velocity in position units/s<br>
udp.stepgen.1.velocity-fb<br>
udp.stepgen.2.velocity-fb<br>

udp.out.0 (in - bit) digital output<br>
udp.out.1<br>
udp.out.2<br>
udp.out.3<br>
udp.out.4<br>
udp.out.5<br>

udp.pwm.0 (in - float) PWM output 0...1<br>
udp.pwm.1<br>
udp.pwm.2<br>
udp.pwm.3<br>
udp.pwm.4<br>
udp.pwm.5<br>

udp.in.0 (out - bit) digital input<br>
udp.in.1<br>
udp.in.2<br>
udp.in.3<br>
udp.in.4<br>
udp.in.5<br>
udp.in.6<br>

udp.ready (out - bit) module state<br>
udp.enable (in - bit) module enable<br>
udp.lost (out - s32) lost packets<br>
### LinuxCNC HAL parameters
udp.stepgen.0.dirsetup (rw - u32) minimum time between a direction change and the beginning of the next step pulse in ns<br>
udp.stepgen.1.dirsetup<br>
udp.stepgen.2.dirsetup<br>

udp.stepgen.0.position-scale (rw - float) steps per position unit<br>
udp.stepgen.1.position-scale<br>
udp.stepgen.2.position-scale<br>

udp.stepgen.0.maxaccel (rw - float) acceleration in position units/s<sup>2</sup><br>
udp.stepgen.1.maxaccel<br>
udp.stepgen.2.maxaccel<br>

udp.pwm.0.freq (rw - u32) PWM frequency in Hz 0..65000<br>
udp.pwm.1.freq<br>
udp.pwm.2.freq<br>
udp.pwm.3.freq<br>
udp.pwm.4.freq<br>
udp.pwm.5.freq<br>
### PWM usage
If the udp.pwm.#.freq parameter is set to 0 (or is not set), then udp.out.# pin works but the udp.pwm.# pin doesn't.<br>
If the value of the udp.pwm.#.freq parameter is not 0, then udp.out.# pin doesn't work but the udp.pwm.# pin does.<br>

