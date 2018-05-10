
/* Author, Copyright: Oleg Borodin <onborodin@gmail.com> 2018 */

#include <libopencm3/cm3/nvic.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/stm32/timer.h>
#include <libopencm3/stm32/i2c.h>
#include <libopencm3/stm32/spi.h>
#include <libopencm3/stm32/rtc.h>
#include <libopencm3/cm3/systick.h>
#include <libopencm3/stm32/adc.h>
#include <libopencm3/stm32/dma.h>
#include <libopencm3/stm32/exti.h>


#include <stdlib.h>
#include <stdio.h>
#include <errno.h>
#include <wctype.h>
#include <ctype.h>
#include <locale.h>
#include <wchar.h>
#include <time.h>

#include <FreeRTOS.h>
#include <task.h>
#include <queue.h>

#include <st7735.h>
#include <console.h>


volatile QueueHandle_t usart_q;
volatile QueueHandle_t console_q;

uint32_t sp;

#define CONSOLE_STR_LEN 16

typedef struct console_message_t {
    uint8_t row;
    uint8_t col;
    uint8_t str[CONSOLE_STR_LEN + 1];
} console_message_t;


static void clock_setup(void) {
    rcc_clock_setup_in_hse_8mhz_out_72mhz();
    rcc_periph_clock_enable(RCC_GPIOA);
    rcc_periph_clock_enable(RCC_GPIOB);
    rcc_periph_clock_enable(RCC_AFIO);
    rcc_periph_clock_enable(RCC_USART1);
    rcc_periph_clock_enable(RCC_SPI2);
}

static void usart_setup(void) {

    usart_disable(USART1);

    gpio_set_mode(GPIOA, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, GPIO_USART1_TX);
    gpio_set_mode(GPIOA, GPIO_MODE_INPUT, GPIO_CNF_INPUT_FLOAT, GPIO_USART1_RX);

    usart_set_baudrate(USART1, 115200);
    usart_set_databits(USART1, 8);
    usart_set_stopbits(USART1, USART_STOPBITS_1);
    usart_set_parity(USART1, USART_PARITY_NONE);
    usart_set_flow_control(USART1, USART_FLOWCONTROL_NONE);
    usart_set_mode(USART1, USART_MODE_TX_RX);

    usart_enable(USART1);
}

void usart_puts(uint8_t * str) {
    uint16_t i = 0;
    while (str[i] != 0) {
        usart_send_blocking(USART1, str[i]);
    }
}

void usart_putc(uint8_t c) {
    usart_send_blocking(USART1, c);
}

static void usart_task(void *args __attribute__ ((unused))) {
    uint8_t c;
    while (1) {
        if (xQueueReceive(usart_q, &c, 10) == pdPASS) {
            while (!usart_get_flag(USART1, USART_SR_TXE))
                taskYIELD();
            usart_putc(c);
        } else {
            taskYIELD();
        }
    }
}

static void console_task(void *args __attribute__ ((unused))) {
    console_message_t msg;
    while (1) {
        if (xQueueReceive(console_q, &msg, 10) == pdPASS) {
            console_xyputs(&console, msg.row, msg.col, msg.str);
        } else {
            taskYIELD();
        }
    }
}


static void log_task(void *args __attribute__ ((unused))) {

    uint32_t i = 0;
    console_message_t msg;
    while (1) {
        msg.row = 8;
        msg.col = 0;
        snprintf(msg.str, CONSOLE_STR_LEN, "0x%08X", i);
        xQueueSend(console_q, &msg, portMAX_DELAY);
        //taskYIELD();
        //vTaskDelay(pdMS_TO_TICKS(10));
        i++;
    }
}

int main(void) {

    clock_setup();
    usart_setup();

    lcd_spi_setup();
    console_setup();

    lcd_setup();
    lcd_clear();

    console_xyputs(&console, 0, 0, "FreeRTOS STM32");
    console_xyputs(&console, 1, 0, "READY>");

#define UART_QUEUE_LEN      1024
#define CONSOLE_QUEUE_LEN   8

    usart_q = xQueueCreate(UART_QUEUE_LEN, sizeof(uint8_t));
    console_q = xQueueCreate(CONSOLE_QUEUE_LEN, sizeof(console_message_t));


    xTaskCreate(usart_task, "UART", configMINIMAL_STACK_SIZE, NULL, configMAX_PRIORITIES - 1, NULL);
    xTaskCreate(log_task, "LOG", configMINIMAL_STACK_SIZE, NULL, configMAX_PRIORITIES - 2, NULL);
    xTaskCreate(console_task, "CONSOLE", configMINIMAL_STACK_SIZE, NULL, configMAX_PRIORITIES - 8, NULL);

    vTaskStartScheduler();
}

/* EOF */
