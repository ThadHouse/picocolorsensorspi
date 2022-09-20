#pragma once

#include "hardware/pio.h"
#include "hardware/dma.h"
#include <stdint.h>

typedef void(*pio_spi_transaction_started_func)(void* ctx);
typedef void(*pio_spi_data_request_function)(void* ctx, uint8_t reg);
typedef void(*pio_spi_transaction_ended_func)(void* ctx, uint8_t num_bytes_read, uint8_t num_bytes_written);

typedef struct pio_spi_config_t {
    int pio_idx;
    int cs_pin;
    int sck_pin;
    int copi_pin;
    int cipo_pin;
    int dbg_pin;
    pio_spi_transaction_started_func transaction_started;
    pio_spi_data_request_function data_request;
    pio_spi_transaction_ended_func transaction_ended;
    void* callback_ctx;
} pio_spi_config_t;

typedef struct pio_spi_t {
    bool allocated;
    PIO pio;
    uint sm_write;
    uint offset_write;
    uint sm_cs;
    uint offset_cs;
    uint sm_initial;
    uint offset_initial;
    uint channel_write;
    uint sm_read;
    uint offset_read;
    uint channel_read;
    uint32_t startstop_mask;
    pio_spi_config_t config;
    uint8_t write_buf_len;
    uint8_t read_buf_len;
} pio_spi_t;

#ifdef __cplusplus
extern "C" {
#endif

pio_spi_t* pio_spi_init(const pio_spi_config_t* config);
void pio_spi_free(pio_spi_t* spi);

inline static void pio_spi_provide_read_buffer(pio_spi_t* spi, volatile uint8_t* buf, uint8_t buf_bytes) {
    spi->read_buf_len = buf_bytes;
    dma_channel_transfer_to_buffer_now(spi->channel_read, buf, buf_bytes);
}

inline static void pio_spi_provide_write_buffer(pio_spi_t* spi, volatile uint8_t* buf, uint8_t buf_bytes) {
    spi->write_buf_len = buf_bytes;
    dma_channel_transfer_from_buffer_now(spi->channel_write, buf, buf_bytes);
}

void pio_spi_start(const pio_spi_t* spi);
void pio_spi_stop(pio_spi_t* spi);

#ifdef __cplusplus
}
#endif
