// file: w5500.c
#include "w5500.h"
#include "w5500.defs.h"

#include "hardware.h"

#include <string.h>

#include "esp_attr.h"

#include "soc/spi_reg.h"
#include "soc/io_mux_reg.h"
#include "soc/periph_defs.h"
#include "driver/periph_ctrl.h"

#include "esp_timer.h"

/* =====================  HELPERS  ===================== */

#define W5500_SPI_NUM       3          // VSPI

#ifndef STATIC_INLINE
#define STATIC_INLINE __attribute__((always_inline)) static inline
#endif

#ifndef REGISTER_READ
#define REGISTER_READ(addr) (*(volatile uint32_t*)(addr))
#endif

#ifndef REGISTER_WRITE
#define REGISTER_WRITE(reg, val) ((*(volatile uint32_t*)(reg)) = (val))
#endif

#ifndef REGISTER_GET_BIT
#define REGISTER_GET_BIT(reg, mask) (*(volatile uint32_t*)(reg) & (mask))
#endif

#ifndef REGISTER_SET_BIT
#define REGISTER_SET_BIT(reg, mask) (*(volatile uint32_t*)(reg) |= (mask))
#endif

#ifndef REGISTER_CLR_BIT
#define REGISTER_CLR_BIT(reg, mask) (*(volatile uint32_t*)(reg) &= ~(mask))
#endif

#define SPI_U_MOSI (SPI_USR_ADDR | SPI_DOUTDIN | SPI_USR_MOSI)
#define SPI_U_MISO (SPI_USR_ADDR | SPI_DOUTDIN | SPI_USR_MISO)

#define SPI_DATA_BUF (volatile uint32_t*)SPI_W0_REG(W5500_SPI_NUM)

/* ===================== LOCAL STATE ===================== */

static w5500_recv_t recv = { 0 };
static uint16_t tx_wr = 0;
static uint16_t rx_rd = 0;

/* ===================== LOW LEVEL SPI ===================== */

STATIC_INLINE void spi_hw_init(void) {

    periph_module_enable(PERIPH_VSPI_MODULE);

    PIN_FUNC_SELECT(PERIPHS_IO_MUX_GPIO5_U, FUNC_GPIO5_VSPICS0);
    PIN_FUNC_SELECT(PERIPHS_IO_MUX_GPIO18_U, FUNC_GPIO18_VSPICLK);
    PIN_FUNC_SELECT(PERIPHS_IO_MUX_GPIO23_U, FUNC_GPIO23_VSPID);
    PIN_FUNC_SELECT(PERIPHS_IO_MUX_GPIO19_U, FUNC_GPIO19_VSPIQ);

    // 24 bit address (W5500 header)
    REGISTER_WRITE(SPI_USER1_REG(W5500_SPI_NUM), (24 - 1) << SPI_USR_ADDR_BITLEN_S);
    // Clock = 40 MHz
    REGISTER_WRITE(SPI_CLOCK_REG(W5500_SPI_NUM), (1 << SPI_CLKCNT_N_S) | (1 << SPI_CLKCNT_L_S));
}

STATIC_INLINE uint8_t spi_rx_u8(const uint32_t header) {

    while (REGISTER_GET_BIT(SPI_CMD_REG(W5500_SPI_NUM), SPI_USR));

    REGISTER_WRITE(SPI_ADDR_REG(W5500_SPI_NUM), header);
    REGISTER_WRITE(SPI_USER_REG(W5500_SPI_NUM), SPI_U_MISO);
    REGISTER_WRITE(SPI_MISO_DLEN_REG(W5500_SPI_NUM), (1 << 3) - 1);

    REGISTER_SET_BIT(SPI_CMD_REG(W5500_SPI_NUM), SPI_USR);
    while (REGISTER_GET_BIT(SPI_CMD_REG(W5500_SPI_NUM), SPI_USR));

    return (uint8_t)*SPI_DATA_BUF;
}

STATIC_INLINE uint16_t spi_rx_u16(const uint32_t header) {

    while (REGISTER_GET_BIT(SPI_CMD_REG(W5500_SPI_NUM), SPI_USR));

    REGISTER_WRITE(SPI_ADDR_REG(W5500_SPI_NUM), header);
    REGISTER_WRITE(SPI_USER_REG(W5500_SPI_NUM), SPI_U_MISO);
    REGISTER_WRITE(SPI_MISO_DLEN_REG(W5500_SPI_NUM), (2 << 3) - 1);

    REGISTER_SET_BIT(SPI_CMD_REG(W5500_SPI_NUM), SPI_USR);
    while (REGISTER_GET_BIT(SPI_CMD_REG(W5500_SPI_NUM), SPI_USR));

    return (uint16_t)*SPI_DATA_BUF;
}

STATIC_INLINE void spi_rx_buf(const uint32_t header, uint8_t* restrict rx, const uint32_t len) {

    while (REGISTER_GET_BIT(SPI_CMD_REG(W5500_SPI_NUM), SPI_USR));

    REGISTER_WRITE(SPI_ADDR_REG(W5500_SPI_NUM), header);
    REGISTER_WRITE(SPI_USER_REG(W5500_SPI_NUM), SPI_U_MISO);
    REGISTER_WRITE(SPI_MISO_DLEN_REG(W5500_SPI_NUM), (len << 3) - 1);

    REGISTER_SET_BIT(SPI_CMD_REG(W5500_SPI_NUM), SPI_USR);
    while (REGISTER_GET_BIT(SPI_CMD_REG(W5500_SPI_NUM), SPI_USR));

    const uint32_t words = (len + 3) >> 2;
    volatile const uint32_t* restrict src = SPI_DATA_BUF;
    uint32_t* restrict dst = (uint32_t*)rx;
    for (int i = 0; i < words; ++i) dst[i] = src[i];
}

STATIC_INLINE void spi_tx_u8(const uint32_t header, const uint8_t data) {

    while (REGISTER_GET_BIT(SPI_CMD_REG(W5500_SPI_NUM), SPI_USR));

    *SPI_DATA_BUF = (uint32_t)data;
    REGISTER_WRITE(SPI_ADDR_REG(W5500_SPI_NUM), header);
    REGISTER_WRITE(SPI_USER_REG(W5500_SPI_NUM), SPI_U_MOSI);
    REGISTER_WRITE(SPI_MOSI_DLEN_REG(W5500_SPI_NUM), (1 << 3) - 1);

    REGISTER_SET_BIT(SPI_CMD_REG(W5500_SPI_NUM), SPI_USR);
}

STATIC_INLINE void spi_tx_u16(const uint32_t header, const uint16_t data) {

    while (REGISTER_GET_BIT(SPI_CMD_REG(W5500_SPI_NUM), SPI_USR));

    *SPI_DATA_BUF = (uint32_t)data;
    REGISTER_WRITE(SPI_ADDR_REG(W5500_SPI_NUM), header);
    REGISTER_WRITE(SPI_USER_REG(W5500_SPI_NUM), SPI_U_MOSI);
    REGISTER_WRITE(SPI_MOSI_DLEN_REG(W5500_SPI_NUM), (2 << 3) - 1);

    REGISTER_SET_BIT(SPI_CMD_REG(W5500_SPI_NUM), SPI_USR);
}

STATIC_INLINE void spi_tx_buf(const uint32_t header, const uint8_t* restrict tx, const uint32_t len) {

    const uint32_t words = (len + 3) >> 2;
    const uint32_t* restrict src = (const uint32_t*)tx;
    volatile uint32_t* restrict dst = SPI_DATA_BUF;

    while (REGISTER_GET_BIT(SPI_CMD_REG(W5500_SPI_NUM), SPI_USR));

    for (int i = 0; i < words; ++i) dst[i] = src[i];

    REGISTER_WRITE(SPI_ADDR_REG(W5500_SPI_NUM), header);
    REGISTER_WRITE(SPI_USER_REG(W5500_SPI_NUM), SPI_U_MOSI);
    REGISTER_WRITE(SPI_MOSI_DLEN_REG(W5500_SPI_NUM), (len << 3) - 1);

    REGISTER_SET_BIT(SPI_CMD_REG(W5500_SPI_NUM), SPI_USR);
}

/* ===================== W5500 PROTOCOL ===================== */

// Format: [Addr High] [Addr Low] [Control]
// The header must be placed in the upper bytes of the spi address register
STATIC_INLINE uint32_t make_w5500_header(const uint16_t addr, const uint8_t block, const uint8_t rw, const uint8_t mode) {
    return ((((uint32_t)addr << 8) | (block | rw | mode)) << 8);
}

/* ----------------- BASIC ACCESS ----------------- */

STATIC_INLINE uint8_t w5500_read_u8(const uint16_t addr, const uint8_t block) {
    const uint32_t header = make_w5500_header(addr, block, W5500_READ, W5500_MODE_FDM1);
    return spi_rx_u8(header);
}

STATIC_INLINE uint16_t w5500_read_u16(const uint16_t addr, const uint8_t block) {
    const uint32_t header = make_w5500_header(addr, block, W5500_READ, W5500_MODE_FDM2);
    return __builtin_bswap16(spi_rx_u16(header));
}

STATIC_INLINE void w5500_read_buf(const uint16_t addr, const uint8_t block, uint8_t* restrict buf, const uint16_t len) {
    const uint32_t header = make_w5500_header(addr, block, W5500_READ, W5500_MODE_VDM);
    spi_rx_buf(header, buf, len);
}

STATIC_INLINE void w5500_write_u8(const uint16_t addr, const uint8_t block, const uint8_t data) {
    const uint32_t header = make_w5500_header(addr, block, W5500_WRITE, W5500_MODE_FDM1);
    spi_tx_u8(header, data);
}

STATIC_INLINE void w5500_write_u16(const uint16_t addr, const uint8_t block, const uint16_t data) {
    const uint32_t header = make_w5500_header(addr, block, W5500_WRITE, W5500_MODE_FDM2);
    spi_tx_u16(header, __builtin_bswap16(data));
}

STATIC_INLINE void w5500_write_buf(const uint16_t addr, const uint8_t block, const uint8_t* restrict buf, const uint16_t len) {
    const uint32_t header = make_w5500_header(addr, block, W5500_WRITE, W5500_MODE_VDM);
    spi_tx_buf(header, buf, len);
}

/* ----------------- SOCKET COMMAND ----------------- */

STATIC_INLINE void w5500_SnCmd(const uint8_t sock, const uint8_t cmd) {
    w5500_write_u8(SnCR(sock), cmd);
    while (w5500_read_u8(SnCR(sock))) asm volatile("nop; nop;");
}

STATIC_INLINE void w5500_SnCmd_noWait(const uint8_t sock, const uint8_t cmd) {
    w5500_write_u8(SnCR(sock), cmd);
}

/* ===================== HIGH LEVEL ===================== */

static void IRAM_ATTR w5500_reset(void) {
    w5500_write_u8(_MR, MR_RESET);
    while (w5500_read_u8(_MR) & MR_RESET) asm volatile("nop; nop;");
}

static void IRAM_ATTR w5500_config(const w5500_cfg_t* restrict cfg) {
    w5500_write_buf(GAR, cfg->gateway, 4);
    w5500_write_buf(SUBR, cfg->subnet, 4);
    w5500_write_buf(SHAR, cfg->mac, 6);
    w5500_write_buf(SIPR, cfg->ip, 4);
    uint8_t bcast[4];
    for (int i = 0; i < 4; ++i)
        bcast[i] = cfg->ip[i] | ~cfg->subnet[i];
    w5500_write_buf(SnDIPR(0), bcast, 4);
    w5500_write_u16(SnDPORT(0), cfg->remotePort);
    w5500_write_u8(SnIMR(0), SnIMR_RECV);
    w5500_write_u8(SIMR, 0x01);
    w5500_write_u8(SnMR(0), SnMR_UDP);
    w5500_write_u16(SnPORT(0), cfg->port);
    w5500_SnCmd(0, SnCR_OPEN);
}

/* ===================== PUBLIC API ===================== */

w5500_recv_t* IRAM_ATTR w5500_init(const w5500_cfg_t* cfg) {
    spi_hw_init();
    uint64_t last_time = esp_timer_get_time();
    while (esp_timer_get_time() - last_time < 100000) asm volatile("nop; nop;"); // 100ms
    w5500_reset();
    memset(&recv, 0, sizeof(recv));
    tx_wr = 0;
    rx_rd = 0;
    w5500_config(cfg);
    return &recv;
}

void IRAM_ATTR w5500_reinit(const w5500_cfg_t* cfg) {
    w5500_reset();
    memset(&recv, 0, sizeof(recv));
    tx_wr = 0;
    rx_rd = 0;
    w5500_config(cfg);
}

w5500_recv_t* IRAM_ATTR w5500_get_recv_ptr(void) { return &recv; }

w5500_recv_t* IRAM_ATTR w5500_recv(uint16_t len) {
    const uint16_t size = w5500_read_u16(SnRX_RSR(0));
    rx_rd += size;
    w5500_read_buf(rx_rd - len, W5500_RX_BLOCK(0), recv.data, len);
    w5500_write_u16(SnRX_RD(0), rx_rd);
    w5500_SnCmd_noWait(0, SnCR_RECV);
    w5500_write_u8(SnIR(0), 0xFF);
    w5500_write_u8(SIR, 0xFF);
    return &recv;
}

void IRAM_ATTR w5500_send(const uint8_t* restrict buf, uint16_t len) {
    w5500_write_buf(tx_wr, W5500_TX_BLOCK(0), buf, len);
    tx_wr += len;
    w5500_write_u16(SnTX_WR(0), tx_wr);
    w5500_SnCmd_noWait(0, SnCR_SEND);
}
