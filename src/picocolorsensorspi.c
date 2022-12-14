#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/sync.h"
#include "pico/multicore.h"
#include "pio_spi.h"
#include "SEGGER_RTT.h"
#include "spi_controller.h"
#include <stdint.h>
#include <string.h>

// SPI Defines
// Any pins can be used, but SCK must be 1 after COPI
#define PIN_CS   5
#define PIN_SCK  3
#define PIN_COPI 2
#define PIN_CIPO 4

#define PIN_DBG  15

uint8_t blank_buffer[64];

uint8_t* current_values = blank_buffer;
uint8_t data_length;
spin_lock_t* spin_lock = NULL;

extern uint8_t* get_current_values(uint8_t* data_length);

static pio_spi_t* spi;
static volatile uint8_t dma_buf[256];
static volatile uint8_t write_buf[] = {1, 2, 3, 4, 5, 6};

static dma_channel_hw_t* write_dma;

static void __time_critical_func(transaction_started)(void* ctx) {
    (void)ctx;
    //pio_spi_provide_write_buffer(spi, write_buf, sizeof(write_buf));
    current_values = get_current_values(&data_length);
}

static void __time_critical_func(data_request)(void* ctx, uint8_t reg) {
    (void)ctx;
    uint8_t* ptr = current_values;
    uintptr_t offset;
    if (reg == 1) {
        offset = 0;
    } else if (reg == 2) {
        offset = 18;
    } else {
        return;
    }
    pio_spi_provide_write_buffer_direct(write_dma, (volatile uint8_t*)ptr + offset, 18);
}

static void __time_critical_func(transaction_ended)(void* ctx, uint8_t num_bytes_read, uint8_t num_bytes_written, uint32_t num_bits_transacted) {
    (void)ctx;
    (void)num_bytes_read;
    (void)num_bytes_written;
    (void)num_bits_transacted;
    pio_spi_provide_read_buffer(spi, dma_buf, 255);
    pio_spi_provide_write_buffer_length(spi, 18);

    // SEGGER_RTT_printf(0, "%d %d %d\n", num_bytes_read, num_bytes_written, num_bits_transacted);
    SEGGER_RTT_printf(0,"%2x %2x %2x %2x %2x %2x %2x %2x\n",
        dma_buf[0],
        dma_buf[1],
        dma_buf[2],
        dma_buf[3],
        dma_buf[4],
        dma_buf[5],
        dma_buf[6],
        dma_buf[7]);
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
        .cs_active_high = false,
        .trigger_on_falling = false,
        .default_write_value = 42
    };

    spi = pio_spi_init(&config);
    write_dma = pio_spi_get_dma_write_channel(spi);
    pio_spi_provide_read_buffer(spi, dma_buf, 255);
    pio_spi_start(spi);

    spi_controller_config_t cconfig = {
        .cipo_pin = 19,
        .copi_pin = 16,
        .sck_pin = 18,
        .cs_pin = 12,
        .cs_active_high = false,
        .frequency = 20000000,
        .mode = 3,
        .spi_idx = 0,
    };
    spi_controller_t* controller = spi_controller_initialize(&cconfig);

    // Run a heartbeat
    const uint LED_PIN = PICO_DEFAULT_LED_PIN;
    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);

    bool state = false;

    uint8_t cbuffer[8] = {1, 2, 3, 4, 5, 6, 7, 8};

    while (1) {
        state = !state;
        // if (state) {
        //     spi = pio_spi_init(&config);125
        //     pio_spi_start(spi);
        // } else {
        //     pio_spi_stop(spi);
        //     pio_spi_free(spi);
        // }
        gpio_put(LED_PIN, state);
        spi_controller_write(controller, cbuffer, 8);
        // memset(cbuffer, 0xFF, sizeof(cbuffer));
        // cbuffer[0] = 2;
        // int num_attempts = spi_controller_register_transaction(controller, cbuffer, 8, 4);
        // SEGGER_RTT_printf(0, "Attempts: %d\n", num_attempts);

        sleep_ms(500);
    }
}
