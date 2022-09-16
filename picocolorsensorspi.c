#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/spi.h"
#include "hardware/pio.h"
#include "hardware/gpio.h"
#include "hardware/dma.h"
#include "spi.pio.h"
#include "SEGGER_RTT.h"

void setup_output(void) {
        PIO pioblink = pio1;
        int pin = 20;

    uint bl = pio_add_program(pioblink, &pio_output_program);
    uint bls = pio_claim_unused_sm(pioblink, true);
    pio_sm_config c = pio_output_program_get_default_config(bl);
    sm_config_set_out_pins(&c, pin, 1);
    pio_gpio_init(pioblink, pin);
    pio_sm_set_consecutive_pindirs(pioblink, bls, pin, 1, true);
    pio_sm_init(pioblink, bls, bl, &c);
    pio_sm_set_enabled(pioblink, bls, true);

    while (true) {
        // Blink
        pio_sm_put_blocking(pioblink, bls, 1);
        sleep_ms(10);
        // Blonk
        pio_sm_put_blocking(pioblink, bls, 0);
        sleep_ms(10);
    }
}

// SPI Defines
// We are going to use SPI 0, and allocate it to the following GPIO pins
// Pins can be changed, see the GPIO function select table in the datasheet for information on GPIO assignments
#define PIN_CS   1
#define PIN_SCK  3
#define PIN_MOSI 2
#define PIN_MISO 4

static char event_str[128];

void gpio_event_string(char *buf, uint32_t events);

static const PIO pio = pio0;
static uint sm = 0;
static uint offset = 0;

static uint sm_write = 0;
static uint offset_write = 0;

static void setup_write_sm(void) {
    offset_write = pio_add_program(pio, &spi_write_loop_program);
    sm_write = pio_claim_unused_sm(pio, true);
    pio_sm_config c = spi_write_loop_program_get_default_config(offset_write);
    SEGGER_RTT_printf(0, "Configuring SM %d at offset %d\n", sm_write, offset_write);
    sm_config_set_out_pins(&c, PIN_MISO, 1);
    pio_gpio_init(pio, PIN_MISO);
    pio_gpio_init(pio, 5);
    pio_sm_set_consecutive_pindirs(pio, sm_write, PIN_MISO, 1, true);

    sm_config_set_out_shift(
        &c,
        false, // Shift-to-right = false (i.e. shift to left)
        true,  // Autopush enabled
        8      // Autopush threshold = 8
    );
    pio_sm_init(pio, sm_write, offset_write, &c);
    //pio_sm_set_enabled(pioblink, sm_write, true);
}

uint write_channel = 0;
void configure_read_dma(void) {
    write_channel = dma_claim_unused_channel(true);

    dma_channel_config channel_config = dma_channel_get_default_config(write_channel);
    channel_config_set_transfer_data_size(&channel_config, DMA_SIZE_8);
    channel_config_set_dreq(&channel_config, pio_get_dreq(pio, sm, false));
    //channel_config_set_irq_quiet(&channel_config, true);
    channel_config_set_read_increment(&channel_config, false);
    channel_config_set_write_increment(&channel_config, true);

    dma_channel_configure(
        write_channel,
        &channel_config,
        NULL,    		// dst
        &pio->rxf[sm], 	 // src
        1,  					// transfer count
        false
    );
}

uint8_t write_buffer[256];

static void __time_critical_func(cs_change_irq)(uint gpio, uint32_t events) {
    gpio_put(5, false);
    if (events & GPIO_IRQ_EDGE_FALL) {

        //SEGGER_RTT_printf(0, "SPI FALL! %d\n", offset_write);
        //gpio_set_dir(PIN_MISO, GPIO_OUT);
        //gpio_put(PIN_MISO, true);
        pio_sm_exec(pio, sm_write, pio_encode_jmp(offset_write));
        pio_sm_set_enabled(pio, sm_write, true);

        pio_sm_exec(pio, sm, pio_encode_jmp(offset));
        pio_sm_set_enabled(pio, sm, true);

        dma_channel_abort(write_channel);
        dma_channel_transfer_to_buffer_now(write_channel, write_buffer, 256);

    } else if (events & GPIO_IRQ_EDGE_RISE) {
        //SEGGER_RTT_printf(0, "SPI RISE!\n");
        pio_sm_set_enabled(pio, sm, false);
        pio_sm_set_enabled(pio, sm_write, false);
        //gpio_set_dir(PIN_MISO, false);
        pio_sm_restart(pio, sm);
        pio_interrupt_clear(pio, 0);
        pio_interrupt_clear(pio, 1);
        pio_sm_clear_fifos(pio, sm);
        pio_sm_clear_fifos(pio, sm_write);

        SEGGER_RTT_printf(0, "SPI RISE %x %x %x %x %x %x %x\n", write_buffer[0],
            write_buffer[1],
            write_buffer[2],
            write_buffer[3],
            write_buffer[4],
            write_buffer[5],
            write_buffer[6],
            write_buffer[7]);
    }
}

// // bit plane content dma channel
// #define DMA_CHANNEL 0
// // chain channel for configuring main dma channel to output from disjoint 8 word fragments of memory
// #define DMA_CB_CHANNEL 1

// #define DMA_CHANNEL_MASK (1u << DMA_CHANNEL)
// #define DMA_CB_CHANNEL_MASK (1u << DMA_CB_CHANNEL)
// #define DMA_CHANNELS_MASK (DMA_CHANNEL_MASK | DMA_CB_CHANNEL_MASK)

void __time_critical_func(dma_isr)(void) {
    SEGGER_RTT_printf(0, "DMA IRQ\n");
}

uint channel = 0;

void configure_write_dma(void) {
    channel = dma_claim_unused_channel(true);

    dma_channel_config channel_config = dma_channel_get_default_config(channel);
    channel_config_set_transfer_data_size(&channel_config, DMA_SIZE_8);
    channel_config_set_dreq(&channel_config, pio_get_dreq(pio, sm_write, true));
    //channel_config_set_irq_quiet(&channel_config, true);
    channel_config_set_read_increment(&channel_config, true);
    channel_config_set_write_increment(&channel_config, false);

    dma_channel_configure(
        channel,
        &channel_config,
        &pio->txf[sm_write],    		// dst
        NULL, 	 // src
        1,  					// transfer count
        false
    );
}

uint8_t data[] = {0xAB, 0xBC, 0xCD, 0xDE};

void __time_critical_func(data_isr)(void) {
    //uint32_t w = pio_sm_get(pio, sm);
    uint8_t w = write_buffer[0];
    //SEGGER_RTT_printf(0, "PIO ISR! %x\n", w);
    pio_interrupt_clear(pio, 0);
    gpio_put(15, true);
    dma_channel_abort(channel);
    dma_channel_transfer_from_buffer_now(channel, data, 4);
    //dma_channel_start(channel);
    // pio_sm_put_blocking(pio, sm_write, 0xFF000000);
    // pio_sm_put_blocking(pio, sm_write, 0);
    // pio_sm_put_blocking(pio, sm_write, 0xFE000000);
    // pio_sm_put_blocking(pio, sm_write, 0);
    gpio_put(15, false);
}

void start_writing_isr(void) {
    gpio_put(15, true);
    pio_interrupt_clear(pio, 1);
    gpio_put(15, false);
}

int main()
{
    SEGGER_RTT_printf(0, "Program Start!\n");

   // setup_output();




    const uint LED_PIN = PICO_DEFAULT_LED_PIN;
    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);

    gpio_init(PIN_CS);
    gpio_init(PIN_SCK);
    gpio_init(PIN_MOSI);
    gpio_init(PIN_MISO);
    gpio_init(5);

    gpio_init(15);

    gpio_set_dir(PIN_CS, GPIO_IN);
    gpio_set_dir(PIN_SCK, GPIO_IN);
    gpio_set_dir(PIN_MOSI, GPIO_IN);
    gpio_set_dir(PIN_MISO, GPIO_OUT);
    gpio_set_dir(5, GPIO_OUT);

    gpio_set_dir(15, GPIO_OUT);

    gpio_set_dir(PIN_CS, GPIO_IN);
    gpio_set_pulls(PIN_CS, false, false);

    setup_write_sm();

    configure_write_dma();

    offset = pio_add_program(pio, &spi_loop_program);
    sm = pio_claim_unused_sm(pio, true);
    pio_sm_config c = spi_loop_program_get_default_config(offset);
    sm_config_set_in_pins(&c, PIN_MOSI);

    // sm_config_set_out_pins(&c, PIN_MISO, 1);
    // pio_gpio_init(pio, PIN_MISO);
    // pio_sm_set_consecutive_pindirs(pio, sm, PIN_MISO, 1, true);

    // sm_config_set_out_pins(&c, PIN_MISO, 1);
    // pio_gpio_init(pio, PIN_MISO);
    // pio_sm_set_consecutive_pindirs(pio, sm, PIN_MISO, 1, true);

    sm_config_set_in_shift(
        &c,
        false, // Shift-to-right = false (i.e. shift to left)
        true,  // Autopush enabled
        8      // Autopush threshold = 8
    );

    // sm_config_set_out_shift(
    //     &c,
    //     false, // Shift-to-right = false (i.e. shift to left)
    //     true,  // Autopush enabled
    //     8      // Autopush threshold = 8
    // );

    pio_sm_init(pio, sm, offset, &c);

    pio_set_irq0_source_enabled(pio, pis_interrupt0, true);
    pio_set_irq1_source_enabled(pio, pis_interrupt1, true);
	irq_set_exclusive_handler(PIO0_IRQ_0, data_isr);
	irq_set_enabled(PIO0_IRQ_0, true);
    irq_set_exclusive_handler(PIO0_IRQ_1, start_writing_isr);
	irq_set_enabled(PIO0_IRQ_1, true);

    pio_sm_set_enabled(pio, sm, false);
    //gpio_set_dir(PIN_MISO, false);
    pio_sm_restart(pio, sm);
    pio_interrupt_clear(pio, 0);
    //pio_sm_set_enabled(pio, sm, true);

    configure_read_dma();

    gpio_set_irq_enabled_with_callback(PIN_CS, GPIO_IRQ_EDGE_FALL | GPIO_IRQ_EDGE_RISE, true, cs_change_irq);

    bool state = false;

    while (1) {
        uint8_t ipc = pio_sm_get_pc (pio, sm);
        uint8_t ipc2 = pio_sm_get_pc (pio, sm_write);
        SEGGER_RTT_printf(0, "Instruction %d\t%d %d %d\n", ipc - offset, ipc2 - offset_write, ipc2, offset);
        // gpio_put(PIN_SCK, state);
        state = !state;
        gpio_put(LED_PIN, state);

        sleep_ms(500);
    }
}

static const char *gpio_irq_str[] = {
        "LEVEL_LOW",  // 0x1
        "LEVEL_HIGH", // 0x2
        "EDGE_FALL",  // 0x4
        "EDGE_RISE"   // 0x8
};

void gpio_event_string(char *buf, uint32_t events) {
    for (uint i = 0; i < 4; i++) {
        uint mask = (1 << i);
        if (events & mask) {
            // Copy this event string into the user string
            const char *event_str = gpio_irq_str[i];
            while (*event_str != '\0') {
                *buf++ = *event_str++;
            }
            events &= ~mask;

            // If more events add ", "
            if (events) {
                *buf++ = ',';
                *buf++ = ' ';
            }
        }
    }
    *buf++ = '\0';
}
