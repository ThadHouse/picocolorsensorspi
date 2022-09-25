#include "spi_controller.h"

#include "hardware/spi.h"
#include "hardware/gpio.h"
#include <string.h>
#include "SEGGER_RTT.h"

spi_controller_t spi_controller[2];

spi_controller_t* spi_controller_initialize(const spi_controller_config_t* config) {
    assert(config);
    assert(config->spi_idx == 0 || config->spi_idx == 1);
    spi_controller_t* spi = &spi_controller[config->spi_idx];
    assert(!spi->allocated);
    memset(spi, 0, sizeof(*spi));
    spi->allocated = true;

    spi->config = *config;
    spi->spi = config->spi_idx == 0 ? spi0 : spi1;

    spi_init(spi->spi, spi->config.frequency);
    gpio_set_function(spi->config.cipo_pin, GPIO_FUNC_SPI);
    gpio_set_function(spi->config.copi_pin, GPIO_FUNC_SPI);
    gpio_set_function(spi->config.sck_pin, GPIO_FUNC_SPI);

    gpio_init(spi->config.cs_pin);
    if (!spi->config.cs_active_high) {
        gpio_set_outover(spi->config.cs_pin, GPIO_OVERRIDE_INVERT);
    } else {
        gpio_set_outover(spi->config.cs_pin, GPIO_OVERRIDE_NORMAL);
    }
    gpio_put(spi->config.cs_pin, false);
    gpio_set_dir(spi->config.cs_pin, true);

    spi_set_format(spi->spi, 8, (spi->config.mode & 2) ? 1 : 0, (spi->config.mode & 1) ? 1 : 0, SPI_MSB_FIRST);

    // Reset SPI to force mode change
    hw_clear_bits(&spi_get_hw(spi->spi)->cr1, SPI_SSPCR1_SSE_BITS);
    hw_set_bits(&spi_get_hw(spi->spi)->cr1, SPI_SSPCR1_SSE_BITS);

    return spi;
}

void spi_controller_write(spi_controller_t* spi, uint8_t* buffer, uint32_t length) {
    gpio_put(spi->config.cs_pin, true);
    spi_write_blocking(spi->spi, buffer, length);
    gpio_put(spi->config.cs_pin, false);
}

int spi_controller_register_transaction(spi_controller_t* spi, uint8_t* buffer, uint32_t length, int num_retries) {
    bool was_found = false;
    int num_attempts = 0;
    gpio_put(spi->config.cs_pin, true);
    spi_write_blocking(spi->spi, buffer, 1);
    uint8_t expected_reg = buffer[0];
    for (int i = 0; i < num_retries; i++) {
        num_attempts++;
        spi_read_blocking(spi->spi, 0, buffer, 1);
        SEGGER_RTT_printf(0, "Buffer: %d\n", buffer[0]);
        if (buffer[0] == expected_reg) {
            was_found = true;
            break;
        }
    }

    if (!was_found) {
        num_attempts = 0;
        goto exit;
    }

    spi_read_blocking(spi->spi, 0, buffer + 1, length - 1);

exit:
    gpio_put(spi->config.cs_pin, false);
    return num_attempts;
}
