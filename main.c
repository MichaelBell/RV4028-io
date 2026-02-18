#include <stdio.h>
#include "pico/stdlib.h"
#include <hardware/uart.h>
#include <hardware/pwm.h>
#include <hardware/clocks.h>

#include "bus.pio.h"

#define BUSPIO pio0
static uint write_sm;
static uint read_sm;

#define UART0_DATA   0x00
#define UART0_FLAGS  0x02
#define UART1_DATA   0x08
#define UART1_FLAGS  0x0A

#define WS2812_CTRL  0x80
#define WS2812_R     0x82
#define WS2812_G     0x84
#define WS2812_B     0x86

#define FLASH_DATA   0x90
#define FLASH_ADDR0  0x92
#define FLASH_ADDR1  0x94
#define FLASH_ADDR2  0x96

#define FLASH_BASE   0x10200000
static uint8_t* flash_ptr = (uint8_t*)FLASH_BASE;

static __force_inline void handle_write(uint32_t data_and_addr) {
    uintptr_t new_flash_ptr;

    uint32_t addr = data_and_addr & 0xff;
    uint32_t data = data_and_addr >> 8;

    switch (addr)
    {
    case UART0_DATA:
        uart0_hw->dr = data;
        break;
    
    case UART1_DATA:
        uart1_hw->dr = data;
        break;

    case FLASH_ADDR0:
        new_flash_ptr = (uintptr_t)flash_ptr & 0xffffff00;
        new_flash_ptr |= data;
        flash_ptr = (uint8_t*)new_flash_ptr;
        break;

    case FLASH_ADDR1:
        new_flash_ptr = (uintptr_t)flash_ptr & 0xffff00ff;
        new_flash_ptr |= data << 8;
        flash_ptr = (uint8_t*)new_flash_ptr;
        break;
        
    case FLASH_ADDR2:
        new_flash_ptr = (uintptr_t)flash_ptr & 0xfff0ffff;
        new_flash_ptr |= (data & 0xf) << 16;
        flash_ptr = (uint8_t*)new_flash_ptr;
        break;
        
    default:
        break;
    }
}

static __force_inline void handle_read(uint32_t addr) {
    uint32_t data = 0;

    switch (addr)
    {
    case UART0_DATA:
        data = uart0_hw->dr;
        break;
    
    case UART0_FLAGS:
        data = uart0_hw->fr;
        break;
    
    case UART1_DATA:
        data = uart1_hw->dr;
        break;
    
    case UART1_FLAGS:
        data = uart1_hw->fr;
        break;
    
    case FLASH_DATA:
        //printf("Flash read from %08x: %02x\n", (uintptr_t)flash_ptr, *flash_ptr);
        data = *flash_ptr++;
        break;

    case FLASH_ADDR0:
        data = (uint32_t)flash_ptr;
        break;

    case FLASH_ADDR1:
        data = (uint32_t)flash_ptr >> 8;
        break;
        
    case FLASH_ADDR2:
        data = ((uint32_t)flash_ptr >> 16) & 0xF;
        break;

    default:
        break;
    }

    pio_sm_put(BUSPIO, read_sm, data);
}

int main() {
    set_sys_clock_khz(200000, true);

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
    pwm_set_clkdiv(slice_num, 10);  // 10 MHz clock
    pwm_set_enabled(slice_num, true);

    gpio_put(RST, 0);

    {
        uint write_offset = pio_add_program(BUSPIO, &rv4028_write_program);
        uint read_offset = pio_add_program(BUSPIO, &rv4028_read_program);

        rv4028_program_init(pio0, write_sm, read_sm, write_offset, read_offset);
    }

    printf("\n\n=== RV4028 IO Module:  Init complete.  Resetting RV4028. ===\n");

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
