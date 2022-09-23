#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/sync.h"
#include "pico/multicore.h"
#include "pio_spi.h"
#include "SEGGER_RTT.h"
#include <stdint.h>

// SPI Defines
// Any pins can be used, but SCK must be 1 after COPI
#define PIN_CS   5
#define PIN_SCK  3
#define PIN_COPI 2
#define PIN_CIPO 4

#define PIN_DBG  15

uint8_t* current_values = NULL;
size_t data_length;
spin_lock_t* spin_lock = NULL;

extern uint8_t* get_current_values(size_t* data_length);

static pio_spi_t* spi;
static volatile uint8_t dma_buf[256];

static void __time_critical_func(transaction_started)(void* ctx) {
    (void)ctx;
    pio_spi_provide_read_buffer(spi, dma_buf, 255);
    current_values = get_current_values(&data_length);
}

static void __time_critical_func(data_request)(void* ctx, uint8_t reg) {
    (void)ctx;
    (void)reg;
    //return NULL;
    uint8_t* ptr = current_values;
    if (current_values == NULL) {
        return;
    }
    if (reg == 2) {
        ptr = current_values + data_length;
    }
    pio_spi_provide_write_buffer(spi, (volatile uint8_t*)ptr, data_length);
}

static void __time_critical_func(transaction_ended)(void* ctx, uint8_t num_bytes_read, uint8_t num_bytes_written) {
    (void)ctx;
    (void)num_bytes_read;
    (void)num_bytes_written;
    SEGGER_RTT_printf(0, "%d %d\n", num_bytes_read, num_bytes_written);
    // buffer[0],
    // buffer[1],
    // buffer[2],
    // buffer[3],
    // buffer[4],
    // buffer[5],
    // buffer[6],
    // buffer[7]);
}

extern void core1_main(void);

int main()
{
    SEGGER_RTT_printf(0, "Program Start!\n");

    int spin_lock_num = spin_lock_claim_unused(true);
    spin_lock = spin_lock_init(spin_lock_num);

    multicore_reset_core1();
    multicore_launch_core1(core1_main);

    sleep_ms(500);

    pio_spi_config_t config = {
        .callback_ctx = NULL,
        .transaction_ended = transaction_ended,
        .transaction_started = transaction_started,
        .data_request = data_request,
        .cipo_pin = PIN_CIPO,
        .copi_pin = PIN_COPI,
        .cs_pin = PIN_CS,
        .dbg_pin = PIN_DBG,
        .sck_pin = PIN_SCK,
        .pio_idx = 0,
    };

    spi = pio_spi_init(&config);
    pio_spi_start(spi);

    // Run a heartbeat
    const uint LED_PIN = PICO_DEFAULT_LED_PIN;
    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);

    bool state = false;

    while (1) {
        state = !state;
        // if (state) {
        //     spi = pio_spi_init(&config);
        //     pio_spi_start(spi);
        // } else {
        //     pio_spi_stop(spi);
        //     pio_spi_free(spi);
        // }
        gpio_put(LED_PIN, state);

        sleep_ms(500);
    }
}
