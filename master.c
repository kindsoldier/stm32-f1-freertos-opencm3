
/* Author, Copyright: Oleg Borodin <onborodin@gmail.com> 2018 */

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/usart.h>

#include <FreeRTOS.h>
#include <task.h>
#include <queue.h>

#include <stdlib.h>
#include <stdio.h>
#include <strings.h>

#include <strings.h>

volatile QueueHandle_t usart_txq;


static void clock_setup(void) {
    rcc_clock_setup_in_hse_8mhz_out_72mhz();
    rcc_periph_clock_enable(RCC_GPIOA);
    rcc_periph_clock_enable(RCC_AFIO);
    rcc_periph_clock_enable(RCC_USART1);
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

//extern size_t __heap_size;

static void olleh_task(void *args __attribute__ ((unused))) {
    while (1) {
        printf("Dlrow, Olleh!\r\n");
        printf("HS=%d\r\n", &__heap_size);
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}


static void usart_task(void *args __attribute__ ((unused))) {
    uint8_t c;
    while (1) {
        if (xQueueReceive(usart_txq, &c, 10) == pdPASS) {
            while (!usart_get_flag(USART1, USART_SR_TXE))
                taskYIELD();
            usart_putc(c);
        } else {
            taskYIELD();
        }
    }
}

static void hello_task(void *args __attribute__ ((unused))) {
    while (1) {
        uint8_t str[] = "Hello, World!\r\n";
        uint16_t i = 0;
        while (str[i] != 0) {
            xQueueSend(usart_txq, &str[i], portMAX_DELAY);
            i++;
        }
        //taskYIELD();
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

int main(void) {

    clock_setup();
    usart_setup();

    usart_txq = xQueueCreate(1024, sizeof(uint8_t));

    xTaskCreate(usart_task, "UART", configMINIMAL_STACK_SIZE, NULL, configMAX_PRIORITIES - 1, NULL);
    xTaskCreate(hello_task, "HELLO", configMINIMAL_STACK_SIZE, NULL, configMAX_PRIORITIES - 2, NULL);
    xTaskCreate(olleh_task, "OLLEH", configMINIMAL_STACK_SIZE, NULL, configMAX_PRIORITIES - 3, NULL);

    vTaskStartScheduler();

    uint32_t i = 0;
    while (1) {
        i++;
    }
    return 0;
}

/* EOF */
