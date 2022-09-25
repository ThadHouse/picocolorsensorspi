#pragma once

#include "hardware/spi.h"
#include <stdint.h>

typedef struct spi_controller_config_t {
    int spi_idx; // spi index
    int cs_pin; // CS Pin
    int sck_pin; // SCK Pin
    int copi_pin; // COPI Pin
    int cipo_pin; // CIPO Pin
    int frequency;

    bool cs_active_high;

    int mode; // SPI Mode
} spi_controller_config_t;

typedef struct spi_controller_t {
    bool allocated;
    spi_inst_t* spi;
    spi_controller_config_t config;
} spi_controller_t ;

#ifdef __cplusplus
extern "C" {
#endif

spi_controller_t* spi_controller_initialize(const spi_controller_config_t* config);

void spi_controller_write(spi_controller_t* spi, uint8_t* buffer, uint32_t length);

int spi_controller_register_transaction(spi_controller_t* spi, uint8_t* buffer, uint32_t length, int num_retries);

#ifdef __cplusplus
}
#endif
