#include "w5500.h"

#include <stdint.h>
#include <stddef.h>
#include <string.h>

#include "esp_attr.h"
#include "esp_log.h"

static const char* TAG = "W5500";
static spi_device_handle_t spi_handle = NULL;
static w5500_recv_t received = { 0 };

static void IRAM_ATTR swap_u16(uint16_t* data) { *data = (*data >> 8) | (*data << 8); }

// construct 24 bit header: 16bit address + 5bit register block + 1bit read/write + 2bit transmit mode
static inline uint32_t IRAM_ATTR make_header(const uint16_t address, const uint8_t block, const uint8_t rw, const uint8_t mode) {
    return (((uint32_t)address) << 8) | ((uint32_t)(block | rw | mode));
}

static uint8_t IRAM_ATTR w5500_read_u8(const uint16_t addr, const uint8_t block) {
    spi_transaction_t t = {
        .flags = SPI_TRANS_USE_RXDATA,
        .addr = make_header(addr, block, W5500_READ, W5500_MODE_FDM1),
        .length = 8
    };
    spi_device_polling_transmit(spi_handle, &t);
    return t.rx_data[0];
}

static uint16_t IRAM_ATTR w5500_read_u16(const uint16_t addr, const uint8_t block) {
    spi_transaction_t t = {
        .flags = SPI_TRANS_USE_RXDATA,
        .addr = make_header(addr, block, W5500_READ, W5500_MODE_FDM2),
        .length = 16
    };
    spi_device_polling_transmit(spi_handle, &t);
    return ((uint16_t)t.rx_data[0]) << 8 | (uint16_t)t.rx_data[1];
}

static void IRAM_ATTR w5500_read_buf(const uint16_t addr, const uint8_t block, uint8_t* buf, const uint16_t len) {
    spi_transaction_t t = {
        .addr = make_header(addr, block, W5500_READ, W5500_MODE_VDM),
        .length = len * 8,
        .rx_buffer = buf
    };
    spi_device_polling_transmit(spi_handle, &t);
}

static void IRAM_ATTR w5500_write_u8(const uint16_t addr, const uint8_t block, const uint8_t data) {
    spi_transaction_t t = {
        .flags = SPI_TRANS_USE_TXDATA,
        .addr = make_header(addr, block, W5500_WRITE, W5500_MODE_FDM1),
        .length = 8,
        .tx_data[0] = data
    };
    spi_device_polling_transmit(spi_handle, &t);
}

static void IRAM_ATTR w5500_write_u16(const uint16_t addr, const uint8_t block, const uint16_t data) {
    spi_transaction_t t = {
        .flags = SPI_TRANS_USE_TXDATA,
        .addr = make_header(addr, block, W5500_WRITE, W5500_MODE_FDM2),
        .length = 16,
        .tx_data[0] = (uint8_t)(data >> 8),
        .tx_data[1] = (uint8_t)(data & 0xff)
    };
    spi_device_polling_transmit(spi_handle, &t);
}

static void IRAM_ATTR w5500_write_buf(const uint16_t addr, const uint8_t block, const uint8_t* buf, const uint16_t len) {
    spi_transaction_t t = {
        .addr = make_header(addr, block, W5500_WRITE, W5500_MODE_VDM),
        .length = len * 8,
        .tx_buffer = buf
    };
    spi_device_polling_transmit(spi_handle, &t);
}

static void IRAM_ATTR w5500_SnCmd(const uint8_t socket, const uint8_t command) {
    w5500_write_u8(SnCR(socket), command); // Socket command
    while (w5500_read_u8(SnCR(socket))); // Wait for command to complete
}

static void IRAM_ATTR w5500_SnCmd_noWait(const uint8_t socket, const uint8_t command) {
    w5500_write_u8(SnCR(socket), command);
}

static void IRAM_ATTR w5500_reset() {
    w5500_write_u8(_MR, MR_RESET); //soft reset
    while (w5500_read_u8(_MR) & MR_RESET)
        vTaskDelay(pdMS_TO_TICKS(1)); // Wait for reset to complete
}

static void IRAM_ATTR w5500_spi_init(const w5500_config_t* config) {

    spi_bus_config_t buscfg = {
        .mosi_io_num = config->pin_mosi,
        .miso_io_num = config->pin_miso,
        .sclk_io_num = config->pin_sclk,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 4096,
        .flags = SPICOMMON_BUSFLAG_MASTER,
    };

    spi_device_interface_config_t devcfg = {
        .address_bits = 24,
        .mode = 0,  // SPI MODE 0 (CPOL=0, CPHA=0)
        .clock_speed_hz = config->spi_clock_hz,
        .input_delay_ns = 20, // 20 ns delay for MISO data valid
        .spics_io_num = config->pin_cs,
        .queue_size = 1,
    };

    spi_bus_initialize(config->spi_host, &buscfg, SPI_DMA_DISABLED);
    spi_bus_add_device(config->spi_host, &devcfg, &spi_handle);
    spi_device_acquire_bus(spi_handle, portMAX_DELAY);

    ESP_LOGI(TAG, "SPI initialized");
}

void IRAM_ATTR w5500_init(const w5500_config_t* config) {

    w5500_spi_init(config);

    vTaskDelay(pdMS_TO_TICKS(100));

    w5500_reset();

    // set local IP address, subnet mask, gateway, MAC address
    w5500_write_buf(GAR, config->gateway, 4);
    w5500_write_buf(SUBR, config->subnet, 4);
    w5500_write_buf(SHAR, config->mac, 6);
    w5500_write_buf(SIPR, config->ip, 4);

    // set destination IP and port
    uint8_t broadcastIP[4];
    for (int i = 0; i < 4; ++i) broadcastIP[i] = config->ip[i] | ~config->subnet[i];
    w5500_write_buf(SnDIPR(0), broadcastIP, 4); // remote ip
    w5500_write_u16(SnDPORT(0), config->remotePort); // remote port

    // socket receive interrupt
    w5500_write_u8(SnIMR(0), SnIMR_RECV);// Socket n IR mask: RECV
    w5500_write_u8(SIMR, 0x01); // enable interrupts for socket 0

    w5500_write_u8(SnMR(0), SnMR_UDP); // UDP mode
    w5500_write_u16(SnPORT(0), config->port); // PORT number
    w5500_SnCmd(0, SnCR_OPEN); // Open socket

    ESP_LOGI(TAG, "W5500 initialized successfully");
}

w5500_recv_t* IRAM_ATTR w5500_recv(const uint16_t len)
{
    static uint16_t rx_rd = 0; // w5500_read_u16(SnRX_RD(0));

    uint16_t recvSize = w5500_read_u16(SnRX_RSR(0));

    if (recvSize == len + 8) {
        //w5500_read_buf(rx_rd, W5500_RX_BLOCK(0), received.buffer, recvSize);
        //swap_u16((uint16_t*)&received.remotePort);
        //swap_u16((uint16_t*)&received.dataLength);
        w5500_read_buf(rx_rd + 8, W5500_RX_BLOCK(0), received.data, len);
        received.dataLength = len;
    } else {
        received.dataLength = 0;
        ESP_LOGE(TAG, "Packet size error: %d", recvSize);
    }

    rx_rd += recvSize;
    w5500_write_u16(SnRX_RD(0), rx_rd); // Update read pointer
    w5500_SnCmd_noWait(0, SnCR_RECV); // RECEIVE command

    // clear interrupt
    w5500_write_u8(SnIR(0), 0xFF); // Clear socket n interrupts
    w5500_write_u8(SIR, 0xFF); // Clear socket interrupt

    return &received;
}

void IRAM_ATTR w5500_send(const uint8_t* buf, const uint16_t len)
{
    static uint16_t tx_wr = 0; // w5500_read_u16(SnTX_WR(0));

    w5500_write_buf(tx_wr, W5500_TX_BLOCK(0), buf, len);

    tx_wr += len;
    w5500_write_u16(SnTX_WR(0), tx_wr); // Update write pointer
    w5500_SnCmd_noWait(0, SnCR_SEND); // SEND command
}

w5500_recv_t* IRAM_ATTR w5500_data_exchange(const uint16_t recv_len, const uint8_t* send_buf, const uint16_t send_len)
{
    // send

    static uint16_t tx_wr = 0; //w5500_read_u16(SnTX_WR(0));

    w5500_write_buf(tx_wr, W5500_TX_BLOCK(0), send_buf, send_len);

    tx_wr += send_len;
    w5500_write_u16(SnTX_WR(0), tx_wr); // Update write pointer
    w5500_SnCmd_noWait(0, SnCR_SEND); // SEND command

    // receive

    static uint16_t rx_rd = 0; //w5500_read_u16(SnRX_RD(0));

    uint16_t recvSize = w5500_read_u16(SnRX_RSR(0));

    if (recvSize == recv_len + 8) {
        //w5500_read_buf(rx_rd, W5500_RX_BLOCK(0), w5500_received.buffer, recvSize);
        //swap_u16((uint16_t*)&w5500_received.remotePort);
        //swap_u16((uint16_t*)&w5500_received.dataLength);
        w5500_read_buf(rx_rd + 8, W5500_RX_BLOCK(0), received.data, recv_len);
        received.dataLength = 1;
    } else {
        received.dataLength = 0;
        ESP_LOGE(TAG, "Packet size error: %d", recvSize);
    }

    rx_rd += recvSize;
    w5500_write_u16(SnRX_RD(0), rx_rd); // Update read pointer
    w5500_SnCmd_noWait(0, SnCR_RECV); // RECEIVE command

    // clear interrupt

    w5500_write_u8(SnIR(0), 0xFF); // Clear socket n interrupts
    w5500_write_u8(SIR, 0xFF); // Clear socket interrupt

    return &received;
}
