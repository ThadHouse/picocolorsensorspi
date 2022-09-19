#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/sync.h"
#include "pico/multicore.h"
#include "stretched_spi.h"
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

static void __time_critical_func(transaction_started)(void* ctx) {
    (void)ctx;
    current_values = get_current_values(&data_length);
}

static const volatile uint8_t* __time_critical_func(data_request)(void* ctx, uint32_t reg, uint32_t* length) {
    (void)ctx;
    uint8_t* ptr = current_values;
    if (current_values == NULL) {
        *length = 0;
        return NULL;
    }
    if (reg == 2) {
        ptr = current_values + data_length;
    }
    *length = data_length;
    return (const volatile uint8_t*)ptr;
}

static void __time_critical_func(transaction_ended)(void* ctx, const uint8_t* buffer, uint32_t buffer_len) {
    (void)ctx;
    (void)buffer;
    (void)buffer_len;
    // SEGGER_RTT_printf(0, "%x %x %x %x %x %x %x %x\n",
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

    stretched_spi_config_t config = {
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

    const stretched_spi_t* spi = stretched_spi_init(&config);
    (void)spi;
    //stretched_spi_start(spi);

    // Run a heartbeat
    const uint LED_PIN = PICO_DEFAULT_LED_PIN;
    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);

    bool state = false;

    while (1) {
        state = !state;
        gpio_put(LED_PIN, state);

        sleep_ms(500);
    }
}
