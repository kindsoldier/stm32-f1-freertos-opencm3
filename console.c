
/* Author, Copyright: Oleg Borodin <onborodin@gmail.com> 2018 */

#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>

#include <st7735.h>
#include <console.h>
#include <font8x14.h>

font_t basefont = {
    .width = 8,
    .height = 14,
    .start = 32,
    .length = 0x5F,
    .bitmap = basefont_bitmap
};

console_t console = {
    .width = CONSOLE_WIDTH,
    .height = CONSOLE_HEIGHT,
    .line = 0,
    .row = 0,
    .xmax = 127,
    .ymax = 127,
    .xshift = 0,
    .yshift = 0,
    .font = &basefont,
    .buffer_len = CONSOLE_WIDTH * CONSOLE_HEIGHT + 1
};

void _console_setup(console_t *console, uint16_t xmax, uint16_t ymax, font_t *font) {
    console->xmax = xmax;
    console->ymax = ymax;
    console->font = font;
    console->width = ymax/font->width;
    console->height = ymax/font->height;
    console->buffer_len = console->width * console->height + 1;
    console->buffer = malloc(console->buffer_len);
}

void console_setup(void) {
    _console_setup(&console, LCD_TFTWIDTH, LCD_TFTHEIGHT, &basefont);
}

void console_render_char(console_t *console, uint8_t line, uint8_t row) {
    uint8_t c = console->buffer[(console->width * line) + row];
    lcd_draw_char(
        (console->xmax - (console->font->height * (line + 1))) + console->yshift,
        (console->font->width * row) + console->xshift,
        console->font, c);
}


void console_render(console_t *console) {
#if 0
    for (uint8_t line = console->height; line > 0; --line) {
        for (uint8_t row = console->width; row > 0; --row) {
            console_render_char(console, line - 1, row - 1);
        }
    }
#endif
    for (uint8_t line = 0; line < console->height; line++) {
        for (uint8_t row = 0; row < console->width; row++) {
            console_render_char(console, line, row);
        }
    }
}

void console_shift(console_t *console) {
    uint16_t i = 0;
    uint16_t pos = console->width * (console->line - 1) + console->row;
    uint16_t end = console->width * console->height;

    while (i < (end - console->width)) {
        console->buffer[i] = console->buffer[i + console->width];
        i++;
    }
    while (i < end) {
        console->buffer[i] = ' ';
        i++;
    }

    if (console->line > 0) 
        console->line--;
    else 
        console->line = 0;
}

void console_putc(console_t *console, uint8_t c) {

    if ((console->row + 1) > console->width) {
        console->line++;
        console->row = 0;
    }

    if (console->line >= console->height) {
        console_shift(console);
        console_render(console);
    }
    console->buffer[(console->line * console->width) + console->row] = c;
    console_render_char(console, console->line, console->row);
    console->row++;
}

int console_puts(console_t *console, uint8_t *str) {
    uint8_t i = 0;
    while (str[i] != 0) {
        console_putc(console, str[i]);
        i++;
    }
    return i;
}

void console_render_xychar(console_t *console, uint8_t line, uint8_t row, uint8_t c) {
    lcd_draw_char(
        (console->xmax - (console->font->height * (line + 1))) + console->yshift,
        (console->font->width * row) + console->xshift,
        console->font, c);
}

void console_xyputc(console_t *console, uint16_t line, uint16_t row, uint8_t c) {
    if (row < console->width && line < console->height) {
        console_render_xychar(console, line, row, c);
    }
}

int console_xyputs(console_t *console, uint16_t line, uint16_t row, uint8_t *str) {
    uint8_t i = 0;
    while (str[i] != 0 && row < console->width && line < console->height) {
        console_render_xychar(console, line, row, str[i]);
        i++;
        row++;
    }
    return i;
}

/* EOF */
