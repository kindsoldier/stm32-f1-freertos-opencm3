

#include <st7735.h>

#ifndef CONSOLE_H_ITU
#define CONSOLE_H_ITU

typedef struct console {
    uint8_t width;
    uint8_t height;
    uint8_t line;
    uint8_t row;
    uint16_t xmax;
    uint16_t ymax;
    uint16_t xshift;
    uint16_t yshift;
    font_t *font;
    uint8_t *buffer;
    uint16_t buffer_len;
} console_t;

#define CONSOLE_WIDTH   (LCD_TFTHEIGHT/8)
#define CONSOLE_HEIGHT  (LCD_TFTWIDTH/14)

extern console_t console;

void _console_setup(console_t *console, uint16_t xmax, uint16_t ymax, font_t *font);
void console_setup(void);
void console_render_char(console_t *console, uint8_t line, uint8_t row);
void console_render(console_t *console);
void console_shift(console_t *console);
void console_putc(console_t *console, uint8_t c);
int console_puts(console_t *console, uint8_t *str);
void console_render_xychar(console_t *console, uint8_t line, uint8_t row, uint8_t c);
void console_xyputc(console_t *console, uint16_t line, uint16_t row, uint8_t c);
int console_xyputs(console_t *console, uint16_t line, uint16_t row, uint8_t *str);

#endif
