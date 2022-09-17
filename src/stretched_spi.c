#include "stretched_spi.h"
#include "stretched_spi.pio.h"
#include "hardware/pio.h"
#include "hardware/gpio.h"
#include "hardware/dma.h"
#include "SEGGER_RTT.h"

typedef struct stretched_spi_t {
    bool allocated;
    PIO pio;
    uint sm_write;
    uint offset_write;
    uint sm_dirs;
    uint offset_dirs;
    uint channel_write;
    uint sm_read;
    uint offset_read;
    uint channel_read;
    stretched_spi_config_t config;
    volatile uint8_t read_buffer[256];
} stretched_spi_t;


stretched_spi_t stretched_spi[2];

static void setup_dirs_sm(PIO pio, int cipo_pin, int cs_pin, uint *sm, uint* offset) {
    *offset = pio_add_program(pio, &spi_dirs_loop_program);
    *sm = pio_claim_unused_sm(pio, true);
    pio_sm_config c = spi_dirs_loop_program_get_default_config(*offset);
    sm_config_set_out_pins(&c, cipo_pin, 1);
    sm_config_set_in_pins(&c, cs_pin);
    gpio_set_oeover(cipo_pin, GPIO_OVERRIDE_INVERT);

    pio_sm_init(pio, *sm, *offset, &c);
    pio_sm_set_enabled(pio, *sm, true);
}

static void setup_write_sm(PIO pio, int cipo_pin, int copi_pin, uint *sm, uint* offset) {
    *offset = pio_add_program(pio, &spi_write_loop_program);
    *sm = pio_claim_unused_sm(pio, true);
    pio_sm_config c = spi_write_loop_program_get_default_config(*offset);
    sm_config_set_in_pins(&c, copi_pin);
    sm_config_set_out_pins(&c, cipo_pin, 1);
    pio_gpio_init(pio, cipo_pin);

    sm_config_set_out_shift(
        &c,
        false, // Shift-to-right = false (i.e. shift to left)
        true,  // Autopush enabled
        8      // Autopush threshold = 8
    );
    pio_sm_init(pio, *sm, *offset, &c);
}

static void setup_read_sm(PIO pio, int copi_pin, uint *sm, uint* offset, irq_handler_t prepare_data_handler) {
    *offset = pio_add_program(pio, &spi_loop_program);
    *sm = pio_claim_unused_sm(pio, true);
    pio_sm_config c = spi_loop_program_get_default_config(*offset);
    sm_config_set_in_pins(&c, copi_pin);

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

static void __time_critical_func(cs_change_irq)(uint gpio, uint32_t events, stretched_spi_t* spi) {
    if (gpio != (uint)spi->config.cs_pin) return;

    if (events & GPIO_IRQ_EDGE_FALL) {
        gpio_put(spi->config.dbg_pin, true);
        pio_sm_exec(spi->pio, spi->sm_write, pio_encode_jmp(spi->offset_write));
        pio_sm_exec(spi->pio, spi->sm_read, pio_encode_jmp(spi->offset_read));
        pio_enable_sm_mask_in_sync(spi->pio, (1u << spi->sm_write) | (1u << spi->sm_read));

        dma_channel_abort(spi->channel_read);
        dma_channel_transfer_to_buffer_now(spi->channel_read, spi->read_buffer, 256);

        if (spi->config.transaction_started) spi->config.transaction_started(spi->config.callback_ctx);

        gpio_put(spi->config.dbg_pin, false);
    } else if (events & GPIO_IRQ_EDGE_RISE) {
        pio_sm_set_enabled(spi->pio, spi->sm_read, false);
        pio_sm_set_enabled(spi->pio, spi->sm_write, false);
        pio_sm_restart(spi->pio, spi->sm_read);
        pio_sm_restart(spi->pio, spi->sm_write);
        pio_interrupt_clear(spi->pio, 0);
        pio_interrupt_clear(spi->pio, 7);
        pio_sm_clear_fifos(spi->pio, spi->sm_read);
        pio_sm_clear_fifos(spi->pio, spi->sm_write);

        if (spi->config.transaction_ended) spi->config.transaction_ended(spi->config.callback_ctx, (const uint8_t*)spi->read_buffer, 256);
    }
}

static void __time_critical_func(cs_change_irq_0)(uint gpio, uint32_t events) {
    cs_change_irq(gpio, events, &stretched_spi[0]);
}

static void __time_critical_func(cs_change_irq_1)(uint gpio, uint32_t events) {
    cs_change_irq(gpio, events, &stretched_spi[1]);
}

static void __time_critical_func(data_request_isr)(stretched_spi_t* spi) {
    gpio_put(spi->config.dbg_pin, true);
    uint8_t reg = spi->read_buffer[0];
    pio_interrupt_clear(spi->pio, 0);
    dma_channel_abort(spi->channel_write);
    if (spi->config.data_request) {
        uint32_t buf_len = 0;
        volatile uint8_t* buf = spi->config.data_request(spi->config.callback_ctx, reg, &buf_len);
        if (buf_len > 0) {
            dma_channel_transfer_from_buffer_now(spi->channel_write, buf, buf_len);
        }
    }
    gpio_put(spi->config.dbg_pin, false);
}

static void __time_critical_func(data_request_isr_0)(void) {
    data_request_isr(&stretched_spi[0]);
}

static void __time_critical_func(data_request_isr_1)(void) {
    data_request_isr(&stretched_spi[1]);
}

const stretched_spi_t* stretched_spi_init(const stretched_spi_config_t* config) {
    assert(config);
    assert(config->pio_idx == 0 || config->pio_idx == 1);
    stretched_spi_t* spi = &stretched_spi[config->pio_idx];
    assert(!spi->allocated);
    spi->allocated = true;

    spi->config = *config;
    spi->pio = config->pio_idx == 0 ? pio0 : pio1;

    gpio_init(config->dbg_pin);
    gpio_set_dir(config->dbg_pin, GPIO_OUT);
    gpio_set_pulls(config->dbg_pin, false, false);

    gpio_init(config->cs_pin);
    gpio_init(config->sck_pin);
    gpio_init(config->copi_pin);
    gpio_init(config->cipo_pin);

    gpio_set_dir(config->cs_pin, GPIO_IN);
    gpio_set_dir(config->sck_pin, GPIO_IN);
    gpio_set_dir(config->copi_pin, GPIO_IN);
    gpio_set_dir(config->cipo_pin, GPIO_IN);
    gpio_set_pulls(config->cs_pin, false, false);
    gpio_set_pulls(config->sck_pin, false, false);
    gpio_set_pulls(config->copi_pin, false, false);
    gpio_set_pulls(config->cipo_pin, false, false);

    setup_write_sm(spi->pio, spi->config.cipo_pin, spi->config.copi_pin, &spi->sm_write, &spi->offset_write);
    configure_write_dma(spi->pio, spi->sm_write, &spi->channel_write);

    setup_read_sm(spi->pio, spi->config.copi_pin, &spi->sm_read, &spi->offset_read, config->pio_idx == 0 ? data_request_isr_0 : data_request_isr_1);
    configure_read_dma(spi->pio, spi->sm_read, &spi->channel_read);

    setup_dirs_sm(spi->pio, spi->config.cipo_pin, spi->config.cs_pin, &spi->sm_dirs, &spi->offset_dirs);

    gpio_set_irq_enabled_with_callback(config->cs_pin, GPIO_IRQ_EDGE_FALL | GPIO_IRQ_EDGE_RISE, true, config->pio_idx == 0 ? cs_change_irq_0 : cs_change_irq_1);

    // gpio_set_dir(spi->config.cipo_pin, false);
    // pio_sm_set_enabled(spi->pio, spi->sm_read, false);
    // pio_sm_set_enabled(spi->pio, spi->sm_write, false);
    // pio_sm_restart(spi->pio, spi->sm_read);
    // pio_sm_restart(spi->pio, spi->sm_write);
    // pio_interrupt_clear(spi->pio, 0);
    // pio_interrupt_clear(spi->pio, 7);
    // pio_sm_clear_fifos(spi->pio, spi->sm_read);
    // pio_sm_clear_fifos(spi->pio, spi->sm_write);

    return spi;
}

void stretched_spi_free(const stretched_spi_t* spi) {
    assert(spi);
    assert(spi->allocated);
    assert(spi->config.pio_idx == 0 || spi->config.pio_idx == 1);
    assert(spi == &stretched_spi[spi->config.pio_idx]);
    stretched_spi[spi->config.pio_idx].allocated = false;
}

// void stretched_spi_start(const stretched_spi_t* spi) {
//     assert(spi);
//     assert(spi->allocated);
//     assert(spi->config.pio_idx == 0 || spi->config.pio_idx == 1);
//     assert(spi == &stretched_spi[spi->config.pio_idx]);


//     gpio_set_irq_enabled(spi->config.cs_pin, GPIO_IRQ_EDGE_FALL | GPIO_IRQ_EDGE_RISE, true);


// }

// void stretched_spi_stop(const stretched_spi_t* spi) {
//     assert(spi);
//     assert(spi->allocated);
//     assert(spi->config.pio_idx == 0 || spi->config.pio_idx == 1);
//     assert(spi == &stretched_spi[spi->config.pio_idx]);

//     gpio_set_irq_enabled(spi->config.cs_pin, GPIO_IRQ_EDGE_FALL | GPIO_IRQ_EDGE_RISE, false);
//     gpio_set_dir(spi->config.cipo_pin, false);
//     pio_sm_set_enabled(spi->pio, spi->sm_read, false);
//     pio_sm_set_enabled(spi->pio, spi->sm_write, false);
//     pio_sm_restart(spi->pio, spi->sm_read);
//     pio_sm_restart(spi->pio, spi->sm_write);
//     pio_interrupt_clear(spi->pio, 0);
//     pio_interrupt_clear(spi->pio, 7);
//     pio_sm_clear_fifos(spi->pio, spi->sm_read);
//     pio_sm_clear_fifos(spi->pio, spi->sm_write);
// }
