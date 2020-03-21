
/* Author, Copyright: Oleg Borodin <onborodin@gmail.com> 2018 */

#include <libopencm3/cm3/nvic.h>
#include <libopencm3/cm3/scb.h>
#include <libopencm3/cm3/systick.h>

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/stm32/timer.h>
#include <libopencm3/stm32/i2c.h>
#include <libopencm3/stm32/spi.h>
#include <libopencm3/stm32/rtc.h>
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
#include <string.h>

#include <FreeRTOS.h>
#include <task.h>
#include <queue.h>
#include <timers.h>

#include <st7735.h>
#include <console.h>

#include <i2creg.h>
#include <mpu6050.h>

#include <bh1750.h>


volatile QueueHandle_t usart_q;
volatile QueueHandle_t console_q;

uint32_t sp;

#define CONSOLE_STR_LEN 18
#define STR_LEN 16

typedef struct console_message_t {
    uint8_t row;
    uint8_t col;
    uint8_t str[CONSOLE_STR_LEN + 1];
} console_message_t;

void delay(uint32_t n) {
    for (volatile int i = 0; i < n; i++)
        __asm__("nop");
}

static void clock_setup(void) {
    rcc_clock_setup_in_hse_8mhz_out_72mhz();
    rcc_periph_clock_enable(RCC_GPIOA);
    rcc_periph_clock_enable(RCC_GPIOB);
    rcc_periph_clock_enable(RCC_GPIOC);

    rcc_periph_clock_enable(RCC_AFIO);
    rcc_periph_clock_enable(RCC_USART1);
    rcc_periph_clock_enable(RCC_SPI2);

    rcc_periph_clock_enable(RCC_DMA1);
    rcc_periph_clock_enable(RCC_ADC1);

    rcc_periph_clock_enable(RCC_I2C1);
}


/* USART1 */
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
    usart_enable_rx_interrupt(USART1);

    nvic_enable_irq(NVIC_USART1_IRQ);
    usart_enable(USART1);
}

inline bool usart_recv_is_ready(uint32_t usart) {
    return (USART_SR(usart) & USART_SR_RXNE);
}

inline bool usart_rx_int_is_enable(uint32_t usart) {
    return (USART_CR1(USART1) & USART_CR1_RXNEIE);
}


void usart1_isr(void) {
    uint8_t data = 0;

    BaseType_t xHigherPriorityTaskWoken;
    xHigherPriorityTaskWoken = pdTRUE;

    if (usart_rx_int_is_enable(USART1) && usart_recv_is_ready(USART1)) {
        data = usart_recv_blocking(USART1);
        xQueueSendFromISR(usart_q, &data, &xHigherPriorityTaskWoken);
    }
    portEND_SWITCHING_ISR(xHigherPriorityTaskWoken);
    //portYIELD();
}


static void usart_puts(uint8_t * str) {
    uint16_t i = 0;
    while (str[i] != 0) {
        usart_send_blocking(USART1, str[i]);
    }
}

static void usart_putc(uint8_t c) {
    usart_send_blocking(USART1, c);
}

/* DMA1 */
volatile uint16_t adc_res[4];

static void dma_setup(void) {
    dma_disable_channel(DMA1, DMA_CHANNEL1);

    dma_enable_circular_mode(DMA1, DMA_CHANNEL1);
    dma_enable_memory_increment_mode(DMA1, DMA_CHANNEL1);
    dma_set_peripheral_size(DMA1, DMA_CHANNEL1, DMA_CCR_PSIZE_16BIT);
    dma_set_memory_size(DMA1, DMA_CHANNEL1, DMA_CCR_MSIZE_16BIT);

    dma_set_read_from_peripheral(DMA1, DMA_CHANNEL1);
    dma_set_peripheral_address(DMA1, DMA_CHANNEL1, (uint32_t) & ADC_DR(ADC1));

    dma_set_memory_address(DMA1, DMA_CHANNEL1, (uint32_t) & adc_res);
    dma_set_number_of_data(DMA1, DMA_CHANNEL1, 2);

    dma_enable_transfer_complete_interrupt(DMA1, DMA_CHANNEL1);
    dma_enable_channel(DMA1, DMA_CHANNEL1);
}


/* ADC1 */
static void adc_setup(void) {
    static uint8_t channel_seq[16];

    gpio_set_mode(GPIOA, GPIO_MODE_INPUT, GPIO_CNF_INPUT_ANALOG, GPIO2);
    gpio_set_mode(GPIOA, GPIO_MODE_INPUT, GPIO_CNF_INPUT_ANALOG, GPIO1);

    adc_power_off(ADC1);

    adc_enable_scan_mode(ADC1);
    adc_set_continuous_conversion_mode(ADC1);
    adc_disable_discontinuous_mode_regular(ADC1);

    adc_enable_external_trigger_regular(ADC1, ADC_CR2_EXTSEL_SWSTART);
    adc_set_right_aligned(ADC1);
    adc_set_sample_time_on_all_channels(ADC1, ADC_SMPR_SMP_7DOT5CYC);

    adc_power_on(ADC1);
    delay(10);

    adc_reset_calibration(ADC1);
    adc_calibrate(ADC1);
    adc_enable_temperature_sensor();

    channel_seq[0] = 16;
    channel_seq[1] = 0;
    channel_seq[2] = 1;
    adc_set_regular_sequence(ADC1, 1, channel_seq);

    adc_enable_dma(ADC1);
    delay(100);
    adc_start_conversion_regular(ADC1);
    nvic_enable_irq(NVIC_DMA1_CHANNEL1_IRQ);
}

void dma1_channel1_isr(void) {
    dma_clear_interrupt_flags(DMA1, DMA_CHANNEL1, DMA_IFCR_CGIF1);
    portEND_SWITCHING_ISR(pdFALSE);
}

static int16_t get_mcu_temp(void) {
    float V_25 = 1.45;
    float Slope = 4.3e-3;
    float Vref = 1.78;
    float V_sense = adc_res[0] / 4096.0 * Vref;
    float temp = (V_25 - V_sense) / Slope + 25.0;
    return (int16_t) temp;
}


/* I2C1 */

static void i2c_setup(void) {
    gpio_set_mode(GPIOB, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_ALTFN_OPENDRAIN, GPIO_I2C1_SCL | GPIO_I2C1_SDA);

    i2c_peripheral_disable(I2C1);
    i2c_reset(I2C1);
    i2c_set_speed(I2C1, i2c_speed_sm_100k, I2C_CR2_FREQ_36MHZ);
    i2c_peripheral_enable(I2C1);
}

/* TASKs */

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


void print_stats(void) {
#define MAX_TASK_COUNT  6
    volatile UBaseType_t task_count = MAX_TASK_COUNT;
    TaskStatus_t *status_array = pvPortMalloc(task_count * sizeof(TaskStatus_t));

    if (status_array != NULL) {

        uint32_t total_time, stat_as_percentage;
        task_count = uxTaskGetSystemState(status_array, task_count, &total_time);

        total_time /= 100UL;
        if (total_time > 0) {
            uint16_t row = 3;
            for (UBaseType_t x = 0; x < task_count; x++) {
                stat_as_percentage = status_array[x].ulRunTimeCounter / total_time;

                if (stat_as_percentage > 0UL) {
                    #define CONSOLE_MAX_STAT_ROW 8
                    if (row < CONSOLE_MAX_STAT_ROW) {
                        console_message_t msg;
                        msg.row = row;
                        msg.col = 0;
                        snprintf(msg.str, CONSOLE_STR_LEN, "%-4s %3d %3u",
                                 status_array[x].pcTaskName, stat_as_percentage, status_array[x].usStackHighWaterMark);
                        xQueueSend(console_q, &msg, portMAX_DELAY);
                    }
                    row++;
                }
            }
        }
    }
    vPortFree(status_array);
}

static void counter_task(void *args __attribute__ ((unused))) {
    uint32_t i = 0;
    console_message_t msg;
    while (1) {
        msg.row = 8;
        msg.col = 0;
        snprintf(msg.str, CONSOLE_STR_LEN, "0x%08X", i);
        xQueueSend(console_q, &msg, portMAX_DELAY);
        print_stats();
        vTaskDelay(pdMS_TO_TICKS(100));
        i++;
    }
}

static void temp_task(void *args __attribute__ ((unused))) {
    console_message_t msg;
    while (1) {
        msg.row = 2;
        msg.col = 0;

        //snprintf(msg.str, CONSOLE_STR_LEN, "Lx %6lu", bh_read(I2C1, BH1750_ADDR, BH1750_ONE_TIME_HR_MODE));
        snprintf(msg.str, CONSOLE_STR_LEN, "Tu %6d", get_mcu_temp());

    
        xQueueSend(console_q, &msg, portMAX_DELAY);
        vTaskDelay(pdMS_TO_TICKS(250));
    }
}

#define TIMER_ID 1

TimerHandle_t timer;

const unsigned portBASE_TYPE timer_id = TIMER_ID;
unsigned int uiAutoReloadTimerPeriod = 1 / portTICK_PERIOD_MS;

void timer_cb(TimerHandle_t xTimer) {
    volatile unsigned portBASE_TYPE *timer_id;
    timer_id = pvTimerGetTimerID(xTimer);
    switch (*timer_id) {
        case TIMER_ID:
            //xTimerStart(timer, 0);
            break;
    }
}


/* MAIN */

#define _IRQ2NVIC_PRIOR(x)       ((x) << 4)
#define UART_QUEUE_LEN      1024
#define CONSOLE_QUEUE_LEN   8

xTaskHandle usart_task_h, counter_task_h, console_task_h;

void reset_handler();

int main(void) {
    clock_setup();
    usart_setup();
    dma_setup();
    adc_setup();

    scb_set_priority_grouping(SCB_AIRCR_PRIGROUP_GROUP16_NOSUB);
    nvic_set_priority(NVIC_SYSTICK_IRQ, _IRQ2NVIC_PRIOR(15));
    nvic_set_priority(NVIC_USART1_IRQ, configMAX_SYSCALL_INTERRUPT_PRIORITY + _IRQ2NVIC_PRIOR(1));
    nvic_set_priority(NVIC_USART3_IRQ, configMAX_SYSCALL_INTERRUPT_PRIORITY + _IRQ2NVIC_PRIOR(2));


    lcd_spi_setup();
    console_setup();

    lcd_setup();
    lcd_clear();

    //i2c_setup();
    //bh_setup(I2C1, BH1750_ADDR);

    console_xyputs(&console, 0, 0, "FreeRTOS STM32");
    console_xyputs(&console, 1, 0, "READY>");

    delay(100);

    usart_q = xQueueCreate(UART_QUEUE_LEN, sizeof(uint8_t));
    console_q = xQueueCreate(CONSOLE_QUEUE_LEN, sizeof(console_message_t));

    xTaskCreate(usart_task, "UAR", configMINIMAL_STACK_SIZE, NULL, configMAX_PRIORITIES - 1, &usart_task_h);
    xTaskCreate(counter_task, "LOG", configMINIMAL_STACK_SIZE, NULL, configMAX_PRIORITIES - 2, &counter_task_h);
    xTaskCreate(console_task, "CON", configMINIMAL_STACK_SIZE, NULL, configMAX_PRIORITIES - 2, &console_task_h);
    xTaskCreate(temp_task, "TMP", configMINIMAL_STACK_SIZE, NULL, configMAX_PRIORITIES - 2, NULL);
    timer = xTimerCreate("TMR1", 20 / portTICK_PERIOD_MS, pdTRUE, (void*) &timer_id, timer_cb);
    xTimerReset(timer, 0);

    vTaskStartScheduler();

    reset_handler();
}

/* EOF */
