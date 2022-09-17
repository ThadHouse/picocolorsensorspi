#pragma once

#include <stdint.h>

typedef void(*stretched_spi_transaction_started_func)(void* ctx);
typedef const volatile uint8_t*(*stretched_spi_data_request_function)(void* ctx, uint32_t reg, uint32_t* length);
typedef void(*stretched_spi_transaction_ended_func)(void* ctx, const uint8_t* buffer, uint32_t buffer_len);

typedef struct stretched_spi_config_t {
    int pio_idx;
    int cs_pin;
    int sck_pin;
    int copi_pin;
    int cipo_pin;
    int dbg_pin;
    stretched_spi_transaction_started_func transaction_started;
    stretched_spi_data_request_function data_request;
    stretched_spi_transaction_ended_func transaction_ended;
    void* callback_ctx;
} stretched_spi_config_t;

typedef struct stretched_spi_t stretched_spi_t;

#ifdef __cplusplus
extern "C" {
#endif

const stretched_spi_t* stretched_spi_init(const stretched_spi_config_t* config);
void stretched_spi_free(const stretched_spi_t* spi);
//void stretched_spi_start(const stretched_spi_t* spi);
//void stretched_spi_stop(const stretched_spi_t* spi);

#ifdef __cplusplus
}
#endif
