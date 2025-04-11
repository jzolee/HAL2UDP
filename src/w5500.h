#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include "w5500.defs.h"

#include "driver/spi_master.h"
#include "driver/gpio.h"

#define W5500_RECV_BUFFER 80

    typedef struct {
        spi_host_device_t spi_host;
        int spi_clock_hz;
        gpio_num_t pin_cs;
        gpio_num_t pin_miso;
        gpio_num_t pin_mosi;
        gpio_num_t pin_sclk;
        uint8_t gateway[4];
        uint8_t subnet[4];
        uint8_t mac[6];
        uint8_t ip[4];
        uint16_t port;
        uint16_t remotePort;
    } w5500_config_t;

#pragma pack(push, 1)
    typedef union {
        uint8_t buffer[W5500_RECV_BUFFER];
        struct {
            uint8_t remoteIP[4];
            uint16_t remotePort;
            uint16_t dataLength;
            uint8_t data[W5500_RECV_BUFFER - 8];
        };
    } w5500_recv_t;
#pragma pack(pop)

    void  w5500_init(const w5500_config_t* config);
    w5500_recv_t* w5500_recv(const uint16_t len);
    void  w5500_send(const uint8_t* buf, const uint16_t len);
    w5500_recv_t* w5500_data_exchange(const uint16_t recv_len, const uint8_t* send_buf, const uint16_t send_len);

#ifdef __cplusplus
}
#endif
