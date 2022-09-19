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
    uint sm_cs;
    uint offset_cs;
    uint sm_initial;
    uint offset_initial;
    uint channel_write;
    uint sm_read;
    uint offset_read;
    uint channel_read;
    uint32_t startstop_mask;
    stretched_spi_config_t config;
    volatile uint8_t read_buffer[256];
} stretched_spi_t;


stretched_spi_t stretched_spi[2];

static void setup_cs_sm(PIO pio, int cipo_pin, int cs_pin, uint* sm, uint* offset) {
    *offset = pio_add_program(pio, &spi_cs_loop_program);
    *sm = pio_claim_unused_sm(pio, true);
    pio_sm_config c = spi_cs_loop_program_get_default_config(*offset);
    sm_config_set_in_pins(&c, cs_pin);
    sm_config_set_set_pins(&c, cipo_pin, 1);
    pio_sm_set_consecutive_pindirs(pio, *sm, cipo_pin, 1, false);

    pio_sm_init(pio, *sm, *offset, &c);
}

static void setup_read_initial_sm(PIO pio, int copi_pin, uint *sm, uint* offset) {
    *offset = pio_add_program(pio, &spi_initial_loop_program);
    *sm = pio_claim_unused_sm(pio, true);
    pio_sm_config c = spi_initial_loop_program_get_default_config(*offset);
    sm_config_set_in_pins(&c, copi_pin);

    sm_config_set_in_shift(
        &c,
        false, // Shift-to-right = false (i.e. shift to left)
        true,  // Autopush enabled
        8      // Autopush threshold = 8
    );

    pio_sm_init(pio, *sm, *offset, &c);
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
        false,  // Autopush enabled
        8      // Autopush threshold = 8
    );
    pio_sm_init(pio, *sm, *offset, &c);
}

static void setup_read_sm(PIO pio, int copi_pin, uint *sm, uint* offset) {
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

inline static int safe_fifo_rx_wait_for_finish(pio_hw_t *pio, uint sm, uint chan) {
    int wooble = 0;
    while (!pio_sm_is_rx_fifo_empty(pio, sm)) {
        wooble++;
        if (wooble > 1000) {
            //check_pio_debug("stuck dma");
            SEGGER_RTT_printf(0, "stuck dma channel %d rem %08x %d @ %d\n", chan, (uint)dma_hw->ch[chan].transfer_count, sm, (int)pio->sm[sm].addr);
            //__breakpoint();
            return 1;
        }
    }
    return 0;
}

static void __time_critical_func(prepare_for_next)(stretched_spi_t* spi) {
    pio_set_sm_mask_enabled(spi->pio, spi->startstop_mask, false); // Stop state machines
    pio_restart_sm_mask(spi->pio, spi->startstop_mask); // Restart state machines to known states
    hw_set_bits(&spi->pio->irq, 0xFF); // Clear IRQs
    safe_fifo_rx_wait_for_finish(spi->pio, spi->sm_read, spi->channel_read);
    pio_sm_clear_fifos(spi->pio, spi->sm_read);
    pio_sm_clear_fifos(spi->pio, spi->sm_write);
    pio_sm_clear_fifos(spi->pio, spi->sm_initial);

    pio_sm_exec(spi->pio, spi->sm_write, pio_encode_jmp(spi->offset_write));
    pio_sm_exec(spi->pio, spi->sm_read, pio_encode_jmp(spi->offset_read));
    pio_sm_exec(spi->pio, spi->sm_initial, pio_encode_jmp(spi->offset_initial));

    dma_channel_abort(spi->channel_read);
    dma_channel_abort(spi->channel_write);
}

static void __time_critical_func(data_request_isr)(stretched_spi_t* spi) {
    gpio_put(spi->config.dbg_pin, true);
    uint8_t reg = pio_sm_get(spi->pio, spi->sm_initial);
    pio_interrupt_clear(spi->pio, 0);
    if (spi->config.data_request) {
        uint32_t buf_len = 0;
        const volatile uint8_t* buf = spi->config.data_request(spi->config.callback_ctx, reg, &buf_len);
        if (buf) {
            dma_channel_transfer_from_buffer_now(spi->channel_write, buf, buf_len);
        }
    }
    gpio_put(spi->config.dbg_pin, false);
}

static void __time_critical_func(pio_irq)(stretched_spi_t* spi) {
    io_rw_32 irqs = spi->pio->irq;
    //SEGGER_RTT_printf(0, "PIO IRQ %x\n", irqs);
    if (irqs & (1u << 0)) {
        //SEGGER_RTT_printf(0, "PIO IRQ  DSR\n");
        while (pio_sm_is_rx_fifo_empty(spi->pio, spi->sm_initial)) tight_loop_contents();
        data_request_isr(spi);
    }
    if (irqs & (1u << 1)) {
        //SEGGER_RTT_printf(0, "PIO IRQ CS Falling\n");
        pio_interrupt_clear(spi->pio, 1);
        gpio_put(spi->config.dbg_pin, true);

        if (spi->config.transaction_started) {
            uint32_t buf_len;
            const volatile uint8_t* buf = spi->config.transaction_started(spi->config.callback_ctx, &buf_len);
            if (buf) {
                dma_channel_transfer_from_buffer_now(spi->channel_write, buf, buf_len);
            }
        }

        pio_enable_sm_mask_in_sync(spi->pio, (1u << spi->sm_write) | (1u << spi->sm_read) | (1u << spi->sm_initial));

        gpio_put(spi->config.dbg_pin, false);
    }
    if (irqs & (1u << 2)) {
        //SEGGER_RTT_printf(0, "PIO IRQ CS Rising\n");
        prepare_for_next(spi);

        if (spi->config.transaction_ended) spi->config.transaction_ended(spi->config.callback_ctx, (const uint8_t*)spi->read_buffer, 256);

        dma_channel_transfer_to_buffer_now(spi->channel_read, spi->read_buffer, 256);
    }
}

static void __time_critical_func(pio_irq_0)(void) {
    pio_irq(&stretched_spi[0]);
}

static void __time_critical_func(pio_irq_1)(void) {
    pio_irq(&stretched_spi[1]);
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

    setup_read_sm(spi->pio, spi->config.copi_pin, &spi->sm_read, &spi->offset_read);
    configure_read_dma(spi->pio, spi->sm_read, &spi->channel_read);

    setup_read_initial_sm(spi->pio, spi->config.copi_pin, &spi->sm_initial, &spi->offset_initial);

    setup_cs_sm(spi->pio, spi->config.cipo_pin, spi->config.cs_pin, &spi->sm_cs, &spi->offset_cs);

    pio_set_irq0_source_enabled(spi->pio, pis_interrupt0, true);
    pio_set_irq0_source_enabled(spi->pio, pis_interrupt1, true);
    pio_set_irq0_source_enabled(spi->pio, pis_interrupt2, true);

    if (config->pio_idx == 0) {
        irq_set_exclusive_handler(PIO0_IRQ_0, pio_irq_0);
        irq_set_enabled(PIO0_IRQ_0, true);
    } else {
        irq_set_exclusive_handler(PIO1_IRQ_0, pio_irq_1);
        irq_set_enabled(PIO1_IRQ_0, true);
    }

    spi->startstop_mask = (1u << spi->sm_write) | (1u << spi->sm_read) | (1u << spi->sm_initial);

    prepare_for_next(spi);

    pio_sm_set_enabled(spi->pio, spi->sm_cs, true);

    return spi;
}

void stretched_spi_free(const stretched_spi_t* spi) {
    assert(spi);
    assert(spi->allocated);
    assert(spi->config.pio_idx == 0 || spi->config.pio_idx == 1);
    assert(spi == &stretched_spi[spi->config.pio_idx]);
    stretched_spi[spi->config.pio_idx].allocated = false;
}
