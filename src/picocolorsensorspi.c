#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/spi.h"
#include "hardware/pio.h"
#include "hardware/gpio.h"
#include "hardware/dma.h"
#include "hardware/sync.h"
#include "pico/multicore.h"
#include "stretched_spi.pio.h"
#include "SEGGER_RTT.h"

// SPI Defines
// Any pins can be used, but SCK must be 1 after MOSI
#define PIN_CS   1
#define PIN_SCK  3
#define PIN_MOSI 2
#define PIN_MISO 4

#define PIN_DBG  15

static void setup_write_sm(PIO pio, uint *sm, uint* offset) {
    *offset = pio_add_program(pio, &spi_write_loop_program);
    *sm = pio_claim_unused_sm(pio, true);
    pio_sm_config c = spi_write_loop_program_get_default_config(*offset);
    sm_config_set_in_pins(&c, PIN_MOSI);
    sm_config_set_out_pins(&c, PIN_MISO, 1);
    pio_gpio_init(pio, PIN_MISO);
    pio_sm_set_consecutive_pindirs(pio, *sm, PIN_MISO, 1, true);

    sm_config_set_out_shift(
        &c,
        false, // Shift-to-right = false (i.e. shift to left)
        true,  // Autopush enabled
        8      // Autopush threshold = 8
    );
    pio_sm_init(pio, *sm, *offset, &c);
}

static void setup_read_sm(PIO pio, uint *sm, uint* offset, irq_handler_t prepare_data_handler) {
    *offset = pio_add_program(pio, &spi_loop_program);
    *sm = pio_claim_unused_sm(pio, true);
    pio_sm_config c = spi_loop_program_get_default_config(*offset);
    sm_config_set_in_pins(&c, PIN_MOSI);

    sm_config_set_in_shift(
        &c,
        false, // Shift-to-right = false (i.e. shift to left)
        true,  // Autopush enabled
        8      // Autopush threshold = 8
    );

    pio_sm_init(pio, *sm, *offset, &c);

    pio_set_irq0_source_enabled(pio, pis_interrupt0, true);
	irq_set_exclusive_handler(PIO0_IRQ_0, prepare_data_handler);
	irq_set_enabled(PIO0_IRQ_0, true);
}

static void configure_read_dma(PIO pio, uint sm, uint* channel) {
    uint dma_channel = dma_claim_unused_channel(true);

    dma_channel_config channel_config = dma_channel_get_default_config(dma_channel);
    channel_config_set_transfer_data_size(&channel_config, DMA_SIZE_8);
    channel_config_set_dreq(&channel_config, pio_get_dreq(pio, sm, false));
    //channel_config_set_irq_quiet(&channel_config, true);
    channel_config_set_read_increment(&channel_config, false);
    channel_config_set_write_increment(&channel_config, true);

    dma_channel_configure(
        dma_channel,
        &channel_config,
        NULL,    		// dst
        &pio->rxf[sm], 	 // src
        1,  					// transfer count
        false
    );

    *channel = dma_channel;
}

static void configure_write_dma(PIO pio, uint sm, uint* channel) {
    uint dma_channel = dma_claim_unused_channel(true);

    dma_channel_config channel_config = dma_channel_get_default_config(dma_channel);
    channel_config_set_transfer_data_size(&channel_config, DMA_SIZE_8);
    channel_config_set_dreq(&channel_config, pio_get_dreq(pio, sm, true));
    //channel_config_set_irq_quiet(&channel_config, true);
    channel_config_set_read_increment(&channel_config, true);
    channel_config_set_write_increment(&channel_config, false);

    dma_channel_configure(
        dma_channel,
        &channel_config,
        &pio->txf[sm],    		// dst
        NULL, 	 // src
        1,  					// transfer count
        false
    );

    *channel = dma_channel;
}

typedef void(*transaction_started_func)(void* ctx);
typedef volatile uint8_t*(*data_request_function)(void* ctx, uint reg, uint* length);
typedef void(*transaction_ended_func)(void* ctx, const uint8_t* buffer, uint buffer_len);

typedef struct stretched_spi_t {
    PIO pio;
    uint sm_write;
    uint offset_write;
    uint channel_write;
    uint sm_read;
    uint offset_read;
    uint channel_read;
    void* context;
    transaction_started_func transaction_started;
    data_request_function data_request;
    transaction_ended_func transaction_ended;
    volatile uint8_t read_buffer[256];
} stretched_spi_t;

static stretched_spi_t stretched_spi;

static void __time_critical_func(cs_change_irq)(uint gpio, uint32_t events) {
    if (gpio != PIN_CS) return;

    if (events & GPIO_IRQ_EDGE_FALL) {
        gpio_put(PIN_DBG, true);
        gpio_set_dir(PIN_MISO, GPIO_OUT); // Set MISO to output
        gpio_put(PIN_MISO, false);
        pio_sm_exec(stretched_spi.pio, stretched_spi.sm_write, pio_encode_jmp(stretched_spi.offset_write));
        pio_sm_exec(stretched_spi.pio, stretched_spi.sm_read, pio_encode_jmp(stretched_spi.offset_read));
        pio_enable_sm_mask_in_sync(stretched_spi.pio, (1u << stretched_spi.sm_write) | (1u << stretched_spi.sm_read));

        dma_channel_abort(stretched_spi.channel_read);
        dma_channel_transfer_to_buffer_now(stretched_spi.channel_read, stretched_spi.read_buffer, 256);

        if (stretched_spi.transaction_started) stretched_spi.transaction_started(stretched_spi.context);

        gpio_put(PIN_DBG, false);
    } else if (events & GPIO_IRQ_EDGE_RISE) {
        gpio_set_dir(PIN_MISO, false);
        pio_sm_set_enabled(stretched_spi.pio, stretched_spi.sm_read, false);
        pio_sm_set_enabled(stretched_spi.pio, stretched_spi.sm_write, false);
        pio_sm_restart(stretched_spi.pio, stretched_spi.sm_read);
        pio_sm_restart(stretched_spi.pio, stretched_spi.sm_write);
        pio_interrupt_clear(stretched_spi.pio, 0);
        pio_interrupt_clear(stretched_spi.pio, 7);
        pio_sm_clear_fifos(stretched_spi.pio, stretched_spi.sm_read);
        pio_sm_clear_fifos(stretched_spi.pio, stretched_spi.sm_write);

        if (stretched_spi.transaction_ended) stretched_spi.transaction_ended(stretched_spi.context, (const uint8_t*)stretched_spi.read_buffer, 256);
    }
}

void __time_critical_func(data_request_isr)(void) {
    gpio_put(PIN_DBG, true);
    uint8_t reg = stretched_spi.read_buffer[0];
    pio_interrupt_clear(stretched_spi.pio, 0);
    dma_channel_abort(stretched_spi.channel_write);
    if (stretched_spi.data_request) {
        uint buf_len = 0;
        volatile uint8_t* buf = stretched_spi.data_request(stretched_spi.context, reg, &buf_len);
        if (buf_len > 0) {
            dma_channel_transfer_from_buffer_now(stretched_spi.channel_write, buf, buf_len);
        }
    }
    gpio_put(PIN_DBG, false);
}

static void __time_critical_func(transaction_started)(void* ctx) {
    (void)ctx;
}

uint8_t ret_data[] = {1, 2, 3, 4};

spin_lock_t* spin_lock = NULL;

extern unsigned int* get_current_values();

static volatile uint8_t* __time_critical_func(data_request)(void* ctx, uint reg, uint* length) {
    (void)ctx;
    (void)reg;
    unsigned int* buf = get_current_values();
    *length = 4;
    (void)buf;
    return (volatile uint8_t*)ret_data;
}

static void __time_critical_func(transaction_ended)(void* ctx, const uint8_t* buffer, uint buffer_len) {
    (void)ctx;
    (void)buffer;
    (void)buffer_len;
}

extern void core1_main(void);

int main()
{
    SEGGER_RTT_printf(0, "Program Start!\n");

    int spin_lock_num = spin_lock_claim_unused(true);
    spin_lock = spin_lock_init(spin_lock_num);

    multicore_reset_core1();
    multicore_launch_core1(core1_main);

    gpio_init(PIN_CS);
    gpio_init(PIN_SCK);
    gpio_init(PIN_MOSI);
    gpio_init(PIN_MISO);
    gpio_init(PIN_DBG);

    gpio_set_dir(PIN_CS, GPIO_IN);
    gpio_set_dir(PIN_SCK, GPIO_IN);
    gpio_set_dir(PIN_MOSI, GPIO_IN);
    gpio_set_dir(PIN_MISO, GPIO_OUT);
    gpio_set_pulls(PIN_CS, false, false);
    gpio_set_pulls(PIN_SCK, false, false);
    gpio_set_pulls(PIN_MOSI, false, false);
    gpio_set_pulls(PIN_MISO, false, false);
    gpio_set_dir(PIN_DBG, GPIO_OUT);
    gpio_set_pulls(PIN_DBG, false, false);

    stretched_spi.pio = pio0;
    stretched_spi.context = NULL;
    stretched_spi.transaction_started = transaction_started;
    stretched_spi.transaction_ended = transaction_ended;
    stretched_spi.data_request = data_request;

    setup_write_sm(stretched_spi.pio, &stretched_spi.sm_write, &stretched_spi.offset_write);
    configure_write_dma(stretched_spi.pio, stretched_spi.sm_write, &stretched_spi.channel_write);

    setup_read_sm(stretched_spi.pio, &stretched_spi.sm_read, &stretched_spi.offset_read, data_request_isr);
    configure_read_dma(stretched_spi.pio, stretched_spi.sm_read, &stretched_spi.channel_read);

    gpio_set_irq_enabled_with_callback(PIN_CS, GPIO_IRQ_EDGE_FALL | GPIO_IRQ_EDGE_RISE, true, cs_change_irq);

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
