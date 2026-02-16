#include <stdio.h>
#include "pico/stdlib.h"
#include <hardware/uart.h>
#include <hardware/pwm.h>

#include "bus.pio.h"

#define BUSPIO pio0
static uint write_sm;
static uint read_sm;

static __force_inline void handle_write(uint32_t data_and_addr) {
    uint32_t addr = data_and_addr & 0xff;
    uint32_t data = data_and_addr >> 8;

    switch (addr)
    {
    case 0x00:
        uart_putc(uart0, (char)data);
        break;
    
    default:
        break;
    }
}

static __force_inline void handle_read(uint32_t addr) {
    switch (addr)
    {
    case 0x00:
        uint32_t c = 0;
        if (uart_is_readable(uart0)) {
            c = uart_getc(uart0);
        }
        pio_sm_put(BUSPIO, read_sm, c);
        break;
    
    default:
        break;
    }
}

int main() {
    stdio_init_all();

    write_sm = pio_claim_unused_sm(BUSPIO, true);
    read_sm = pio_claim_unused_sm(BUSPIO, true);

    // Setup clock
    uint CLK = 19;
    uint RST = 20;
    gpio_init(RST);
    gpio_put(RST, 1);
    gpio_set_pulls(RST, true, false);
    gpio_set_dir(RST, GPIO_OUT);

    gpio_set_function(CLK, GPIO_FUNC_PWM);
    uint slice_num = pwm_gpio_to_slice_num(CLK);
    pwm_set_wrap(slice_num, 1);
    pwm_set_chan_level(slice_num, PWM_CHAN_B, 1);
    pwm_set_clkdiv(slice_num, 75);  // 1 MHz clock
    pwm_set_enabled(slice_num, true);

    gpio_put(RST, 0);

    {
        uint write_offset = pio_add_program(BUSPIO, &rv4028_write_program);
        uint read_offset = pio_add_program(BUSPIO, &rv4028_read_program);

        rv4028_program_init(pio0, write_sm, read_sm, write_offset, read_offset);
    }

    printf("RV4028 IO Module initialized, starting clock\n");

    gpio_set_dir(RST, GPIO_IN);

    while (true) {
        if (!pio_sm_is_rx_fifo_empty(BUSPIO, write_sm)) {
            handle_write(pio_sm_get(BUSPIO, write_sm));
            continue;
        }
        if (!pio_sm_is_rx_fifo_empty(BUSPIO, read_sm)) {
            handle_read(pio_sm_get(BUSPIO, read_sm));
            continue;
        }
    }

    return 0;
}
