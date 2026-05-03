// file: w5500.h
#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#include "driver/spi_master.h"
#include "driver/gpio.h"

#define W5500_RECV_BUFFER 64

typedef struct {
    uint8_t gateway[4];
    uint8_t subnet[4];
    uint8_t mac[6];
    uint8_t ip[4];
    uint16_t port;
    uint16_t remotePort;
} w5500_cfg_t;

typedef union __attribute__((aligned(4))) {
    uint8_t buff[W5500_RECV_BUFFER];
    struct {
        uint8_t remote_ip[4];
        uint16_t remote_port;
        uint16_t data_length;
        uint8_t data[W5500_RECV_BUFFER - 8];
    };
} w5500_recv_t;

w5500_recv_t* w5500_init(const w5500_cfg_t* cfg);
void w5500_reinit(const w5500_cfg_t* cfg);
w5500_recv_t* w5500_recv(uint16_t len);
void w5500_send(const uint8_t* restrict buf, uint16_t len);
w5500_recv_t* w5500_get_recv_ptr(void);

#ifdef __cplusplus
}
#endif
