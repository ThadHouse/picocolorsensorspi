#pragma once

#include "hardware/pio.h"
#include "hardware/dma.h"
#include <stdint.h>

// For all callbacks, it is recommended they all be marked __time_critical_func,
// as well as any functions they call. Callbacks are not called if they are null.

// After each transaction, both DMA buffers are cleared. This means for each
// transaction, they must be provided if required. The transaction will work
// if only 1 of the 2 is provided (Technically it will work if none are provided
// but whats the point in that).

// Called when CS goes low
// This function is when it is recommended to provide the read buffer.
// The state machines are already started, so it is recommended to provide
// the read buffer, and then do any further computations necessary (Like
// grab all data to be written after first byte.
// If write data does not depend on data received, write buffer can be provided as well.
typedef void(*pio_spi_transaction_started_func)(void* ctx);

// Called when first byte of data is received, containing value of first byte.
// Recommended place to provide the write buffer if first byte is necessary.
// This register can be received even if no read buffer is provided.
typedef void(*pio_spi_data_request_function)(void* ctx, uint8_t reg);

// Called when CS goes high. Provides the number of bytes read, and the
// number of bytes written.
// If read or write buffers are known, they can be provided here for the next
// transaction. If this is done, for the first transaction to be successful,
// the buffers for the first transaction would need to be provided before
// starting, as this callback will not occur before the first transaction occurs.
typedef void(*pio_spi_transaction_ended_func)(void* ctx, uint8_t num_bytes_read, uint8_t num_bytes_written);

// Configuration options for pio spi
// All items necessary except for callbacks and callback context
typedef struct pio_spi_config_t {
    int pio_idx; // pio index
    int cs_pin; // CS Pin (Can be any pin)
    int sck_pin; // SCK Pin (Must be 1 pin after COPI)
    int copi_pin; // COPI Pin (Can be any pin)
    int cipo_pin; // CIPO Pin (Can be any pin)
    int dbg_pin;    // Debug pin is written high at the beginning of each ISR
                    // and written low at the end. Useful for debugging timing
                    // issues between ISRs and data bytes

    pio_spi_transaction_started_func transaction_started;
    pio_spi_data_request_function data_request;
    pio_spi_transaction_ended_func transaction_ended;
    void* callback_ctx; // Passed to each callback
} pio_spi_config_t;

// Internal representation of PIO SPI
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

// Initialize a PIO SPI object. Asserts if already allocated.
// The engine is stopped upon initialization, and must be started.
pio_spi_t* pio_spi_init(const pio_spi_config_t* config);
// Frees a PIO SPI object, stopping if its already running
void pio_spi_free(pio_spi_t* spi);

// Provide the read buffer for DMA. Must be called per transaction if read data is requested,
// but at max once per transaction.
inline static void pio_spi_provide_read_buffer(pio_spi_t* spi, volatile uint8_t* buf, uint8_t buf_bytes) {
    spi->read_buf_len = buf_bytes;
    dma_channel_transfer_to_buffer_now(spi->channel_read, buf, buf_bytes);
}

// Provide the write buffer for DMA. Must be called per transaction if write data is requested,
// but at max once per transaction.
inline static void pio_spi_provide_write_buffer(pio_spi_t* spi, volatile uint8_t* buf, uint8_t buf_bytes) {
    spi->write_buf_len = buf_bytes;
    dma_channel_transfer_from_buffer_now(spi->channel_write, buf, buf_bytes);
}

// Start the pio spi engine.
void pio_spi_start(const pio_spi_t* spi);
// Stop the pio spi engine.
void pio_spi_stop(pio_spi_t* spi);

#ifdef __cplusplus
}
#endif
