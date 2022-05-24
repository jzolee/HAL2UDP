# HAL2UDP
External step generator and IO interface for LinuxCNC over Ethernet with dual-core ESP32 and W5500 modules.

set your eth0 to 192.168.96.55
the Esp32 with Ethernet will have 192.168.96.54

LinuxCNC driver:
install linuxcnc-uspace-dev

halcompile --install udp.comp
