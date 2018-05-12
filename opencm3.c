/* Author, Copyright: Oleg Borodin <onborodin@gmail.com> 2018 */

#include <FreeRTOS.h>
#include <task.h>

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/cm3/nvic.h>


extern void vPortSVCHandler(void) __attribute__ ((naked));
extern void xPortPendSVHandler(void) __attribute__ ((naked));
extern void xPortSysTickHandler(void);

__attribute__((used)) void sv_call_handler(void) {
    vPortSVCHandler();
}

__attribute__((used)) void pend_sv_handler(void) {
    xPortPendSVHandler();
}

__attribute__((used)) void sys_tick_handler(void) {
    xPortSysTickHandler();
}

/* EOF */
