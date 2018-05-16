/* Author, Copyright: Oleg Borodin <onborodin@gmail.com> 2018 */

#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/spi.h>

#include <stdlib.h>
#include <stdio.h>

#include <st7735.h>

#define USE_SPI 2

#if (USE_SPI == 1)

#define LCD_SPI         SPI1
#define LCD_SPI_PORT    GPIOA
#define LCD_SDA_PIN     GPIO7
#define LCD_SCL_PIN     GPIO5
#define LCD_CS_PIN      GPIO4

#define LCD_A0_PORT     GPIOA
#define LCD_A0_PIN      GPIO6

#define LCD_RESET_PORT  GPIOA
#define LCD_RESET_PIN   GPIO3

#elif (USE_SPI == 2)

#define LCD_SPI         SPI2
#define LCD_SPI_PORT    GPIOB

#define LCD_SDA_PIN     GPIO15
#define LCD_SCL_PIN     GPIO13
#define LCD_CS_PIN      GPIO12

#define LCD_A0_PORT     GPIOB
#define LCD_A0_PIN      GPIO1

//#define LCD_RESET_PORT  GPIOB
//#define LCD_RESET_PIN   GPIO2

#else
#error Please define USE_SPI=1 or USE_SPI=2
#endif


void lcd_spi_setup(void) {

    gpio_set_mode(LCD_SPI_PORT, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, LCD_CS_PIN);
    gpio_set_mode(LCD_SPI_PORT, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, LCD_SCL_PIN);
    gpio_set_mode(LCD_SPI_PORT, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, LCD_SDA_PIN);

    gpio_set_mode(LCD_A0_PORT, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, LCD_A0_PIN);
#if (USE_SPI == 1)
    gpio_set_mode(LCD_RESET_PORT, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, LCD_RESET_PIN);
#endif
    spi_reset(LCD_SPI);
    spi_disable(LCD_SPI);

    spi_init_master(LCD_SPI,
        SPI_CR1_BAUDRATE_FPCLK_DIV_2,
        SPI_CR1_CPOL_CLK_TO_1_WHEN_IDLE,
        SPI_CR1_CPHA_CLK_TRANSITION_2,
        SPI_CR1_DFF_8BIT,
        SPI_CR1_MSBFIRST);

    spi_disable_software_slave_management(LCD_SPI);
    spi_set_nss_high(LCD_SPI);
    spi_set_master_mode(LCD_SPI);
    spi_enable_ss_output(LCD_SPI);
    spi_disable_crc(LCD_SPI);
    spi_enable(LCD_SPI);
}


inline void _delay_ms (uint16_t ms) {
    for (volatile int i = 0; i < ms * 800; i++)
        __asm__("nop");
}

inline void _delay (uint16_t ms) {
    for (volatile int i = 0; i < ms; i++)
        __asm__("nop");
}


inline void spi_wait_busy(uint32_t spi) {
    while ((SPI_SR(spi) & SPI_SR_BSY));
    _delay(1);
}

inline void lcd_write_byte(uint8_t c) {
    spi_wait_busy(LCD_SPI);
    gpio_set(LCD_A0_PORT, LCD_A0_PIN);
    spi_send(LCD_SPI, c);
}

inline void lcd_write_word(uint16_t w) {
    spi_wait_busy(LCD_SPI);

    gpio_set(LCD_A0_PORT, LCD_A0_PIN);

    spi_send(LCD_SPI, (uint8_t)(w >> 8));
    spi_send(LCD_SPI, (uint8_t)(w & 0xff));
}

inline void lcd_write_command(uint8_t c) {
    spi_wait_busy(LCD_SPI);
    gpio_clear(LCD_A0_PORT, LCD_A0_PIN);
    spi_send(LCD_SPI, c);
}

void spi_write_word_array(uint16_t w, uint16_t n) {
    spi_wait_busy(LCD_SPI);
    gpio_set(LCD_A0_PORT, LCD_A0_PIN);

    while (n--) {
        spi_send(LCD_SPI, (uint8_t)(w >> 8));
        spi_send(LCD_SPI, (uint8_t)(w & 0xff));
    }
}

void lcd_hard_reset(void) {
#if (USE_SPI == 1)
    gpio_set(LCD_RESET_PORT, LCD_RESET_PIN);
    _delay_ms(100);
    gpio_clear(LCD_RESET_PORT, LCD_RESET_PIN);
    _delay_ms(100);
    gpio_set(LCD_RESET_PORT, LCD_RESET_PIN);
#endif
}

void lcd_setup(void) {

    lcd_hard_reset();
    _delay_ms(500);

    /* 1: Software reset, 0 args, w/delay */
    lcd_write_command(LCD_SWRESET);
    /* 150 ms delay */
    _delay_ms(150);

    /* 2: Out of sleep mode, 0 args, w/delay */
    lcd_write_command(LCD_SLPOUT);
    /* 500 ms delay */
    _delay_ms(500);

    /* 3: Frame rate ctrl - normal mode, 3 args: */
    lcd_write_command(LCD_FRMCTR1);
    /* Rate = fosc/(1x2+40) * (LINE+2C+2D) */
    lcd_write_byte(0x01);
    lcd_write_byte(0x2C);
    lcd_write_byte(0x2D);

    /* 4: Frame rate control - idle mode, 3 args: */
    lcd_write_command(LCD_FRMCTR2);
    /* Rate = fosc/(1x2+40) * (LINE+2C+2D) */
    lcd_write_byte(0x01);
    lcd_write_byte(0x2C);
    lcd_write_byte(0x2D);

    /* 5: Frame rate ctrl - partial mode, 6 args: */
    lcd_write_command(LCD_FRMCTR3);
    /* Dot inversion mode */
    lcd_write_byte(0x01);
    lcd_write_byte(0x2C);
    lcd_write_byte(0x2D);
    /* Line inversion mode */
    lcd_write_byte(0x01);
    lcd_write_byte(0x2C);
    lcd_write_byte(0x2D);

    /* 6: Display inversion ctrl, 1 arg, no delay: */
    lcd_write_command(LCD_INVCTR);
    /* No inversion */
    lcd_write_byte(0x07);

    /* 7: Power control, 3 args, no delay: */
    lcd_write_command(LCD_PWCTR1);
    lcd_write_byte(0xA2);
    /* -4.6V */
    lcd_write_byte(0x02);
    /* AUTO mode */
    lcd_write_byte(0x84);

    /* 8: Power control, 1 arg, no delay: */
    lcd_write_command(LCD_PWCTR2);
    /* VGH25 = 2.4C VGSEL = -10 VGH = 3 * AVDD */
    lcd_write_byte(0xC5);

    /* 9: Power control, 2 args, no delay: */
    lcd_write_command(LCD_PWCTR3);
    /* Opamp current small */
    lcd_write_byte(0x0A);
    /* Boost frequency */
    lcd_write_byte(0x00);

    /* 10: Power control, 2 args, no delay: */
    lcd_write_command(LCD_PWCTR4);
    /* BCLK/2, Opamp current small & Medium low */
    lcd_write_byte(0x8A);
    lcd_write_byte(0x2A);

    /*  11: Power control, 2 args, no delay: */
    lcd_write_command(LCD_PWCTR5);
    lcd_write_byte(0x8A);
    lcd_write_byte(0xEE);

    /* 12: Power control, 1 arg, no delay: */
    lcd_write_command(LCD_VMCTR1);
    lcd_write_byte(0x0E);

    /* 13: Don't invert display, no args, no delay */
    lcd_write_command(LCD_INVOFF);

    /* 14: Memory access control (directions), 1 arg: */
    lcd_write_command(LCD_MADCTL);
    /* row addr/col addr, bottom to top refresh, RGB order */
    lcd_write_byte(/* 0xC0 */ LCD_MADCTL_MX  | LCD_MADCTL_MY | LCD_MADCTL_BGR | LCD_MADCTL_ML | LCD_MADCTL_MH);

    /* 15: Set color mode, 1 arg + delay: */
    lcd_write_command(LCD_COLMOD);
    /* 16-bit color 5-6-5 color format */
    lcd_write_byte(0x05);
    /* 10 ms delay */

    _delay_ms(10);

    /* 1: Column addr set, 4 args, no delay */
    lcd_write_command(LCD_CASET);
    /* XSTART = 0 */
    lcd_write_byte(0x00);
    lcd_write_byte(0x00);
    /* XEND = 127 */
    lcd_write_byte(0x00);
    lcd_write_byte(LCD_TFTWIDTH /* 0x7F */);

    /* 2: Row addr set, 4 args, no delay: */
    lcd_write_command(LCD_RASET);
    /* XSTART = 0 */
    lcd_write_byte(0x00);
    lcd_write_byte(0x00);
    /* XEND = 127 */
    lcd_write_byte(0x00);
    lcd_write_byte(LCD_TFTHEIGHT /* 0x7F */);

    /* 1: Magical unicorn dust, 16 args, no delay: */
    lcd_write_command(LCD_GMCTRP1);
    lcd_write_byte(0x02);
    lcd_write_byte(0x1c);
    lcd_write_byte(0x07);
    lcd_write_byte(0x12);
    lcd_write_byte(0x37);
    lcd_write_byte(0x32);
    lcd_write_byte(0x29);
    lcd_write_byte(0x2d);
    lcd_write_byte(0x29);
    lcd_write_byte(0x25);
    lcd_write_byte(0x2B);
    lcd_write_byte(0x39);
    lcd_write_byte(0x00);
    lcd_write_byte(0x01);
    lcd_write_byte(0x03);
    lcd_write_byte(0x10);

    /* 2: Sparkles and rainbows, 16 args, no delay: */
    lcd_write_command(LCD_GMCTRN1);
    lcd_write_byte(0x03);
    lcd_write_byte(0x1d);
    lcd_write_byte(0x07);
    lcd_write_byte(0x06);
    lcd_write_byte(0x2E);
    lcd_write_byte(0x2C);
    lcd_write_byte(0x29);
    lcd_write_byte(0x2D);
    lcd_write_byte(0x2E);
    lcd_write_byte(0x2E);
    lcd_write_byte(0x37);
    lcd_write_byte(0x3F);
    lcd_write_byte(0x00);
    lcd_write_byte(0x00);
    lcd_write_byte(0x02);
    lcd_write_byte(0x10);

    /*  3: Normal display on, no args, w/delay */
    lcd_write_command(LCD_NORON);
    _delay_ms(10);

    /*  4: Main screen turn on, no args w/delay */
    lcd_write_command(LCD_DISPON);
    _delay_ms(100);
}

void lcd_write_rect(uint8_t x, uint8_t y, uint8_t w, uint8_t h, uint32_t color) {
    uint16_t c = lcd_rgb2c(color);
    lcd_addr_window(x, y, (x + w), (y + h));
    spi_write_word_array(c, (w + 1) * (h + 1));
}

#define LCD_DELTA_X     0
#define LCD_DELTA_Y     0

void lcd_addr_window(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1) {

    lcd_write_command(LCD_CASET);

    lcd_write_word(x0 + LCD_DELTA_X);
    lcd_write_word(x1 + LCD_DELTA_X);

    lcd_write_command(LCD_RASET);

    lcd_write_word(y0 + LCD_DELTA_Y);
    lcd_write_word(y1 + LCD_DELTA_Y);

    lcd_write_command(LCD_RAMWR);
}

void lcd_draw_pixel(uint8_t x, uint8_t y, uint32_t color) {
    lcd_addr_window(x, y, x, y);
    uint16_t c = lcd_rgb2c(color);
    lcd_write_word(c);
}

#define swap(a, b) { int16_t t = a; a = b; b = t; }

void lcd_draw_line(int16_t x0, int16_t y0, int16_t x1, int16_t y1, uint32_t color) {
    int16_t steep = abs(y1 - y0) > abs(x1 - x0);

    if (steep) {
        swap(x0, y0);
        swap(x1, y1);
    }

    if (x0 > x1) {
        swap(x0, x1);
        swap(y0, y1);
    }

    int16_t dx, dy;
    dx = x1 - x0;
    dy = abs(y1 - y0);

    int16_t err = dx / 2;
    int16_t ystep;

    if (y0 < y1)
        ystep = 1;
    else
        ystep = -1;

    while (x0 <= x1) {
        if (steep) {
            lcd_draw_pixel(y0, x0, color);
        } else {
            lcd_draw_pixel(x0, y0, color);
        }
        err -= dy;
        if (err < 0) {
            y0 += ystep;
            err += dx;
        }
        x0++;
    }
}

void lcd_draw_vline(int16_t x, int16_t y, int16_t l, uint32_t color) {
    lcd_write_rect(x, y, (l - 1), 0, color);
}

void lcd_draw_hline(int16_t x, int16_t y, int16_t l, uint32_t color) {
    lcd_write_rect(x, y, 0, (l - 1), color);
}

void lcd_draw_rect(uint16_t x1, uint16_t y1, uint16_t w, uint16_t h, uint32_t color) {
    lcd_draw_hline(x1, y1, w, color);
    lcd_draw_vline(x1, y1, h, color);
    lcd_draw_hline((x1 + h) - 0, y1, w, color);
    lcd_draw_vline(x1, (y1 + w) - 0, h, color);
}

uint16_t lcd_rgb2c(uint32_t c) {
#if 0
    return (uint16_t) 
        (((c & 0xF80000) >> 19) |
         ((c & 0x00FC00 ) >> 5) |
         ((c & 0x0000F8) << 8));
#else
    return (uint16_t) 
        (((c & 0xF80000) >> 11) |
         ((c & 0x00FC00 ) >> 5) |
         ((c & 0x0000F8) >> 3));
#endif
}

void lcd_clear(void) {
    lcd_write_rect(0, 0, LCD_TFTWIDTH, LCD_TFTHEIGHT, LCD_BLACK);
}

void lcd_draw_char(uint16_t xbase, uint16_t ybase, font_t *font, uint8_t c) {
    if (c < font->start || c > (font->start + font->length))
        c = ' ';
    c = c - font->start;
    lcd_addr_window(xbase, ybase, xbase + font->height - 1, ybase + font->width - 1);
    for (uint8_t w = font->width; w > 0; w--) {
        for (uint8_t h = font->height; h > 0; h--) {
            if ((font->bitmap[(c) * font->height + (h - 1)]) & (1 << (w - 1)))
                lcd_write_word(0xffff);
            else
                lcd_write_word(0x0000);
        }
    }
}

/* EOF */
