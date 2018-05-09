
#include <stdlib.h>

__attribute__((always_inline)) static inline uint32_t __get_LR(void) { 
    register uint32_t result;
    __asm volatile ("mov %0, lp\n" : "=r" (result)); 
    return result;
}

__attribute__((always_inline)) static inline uint32_t __get_SP(void) { 
    register uint32_t result;
    __asm volatile ("mov %0, sp\n" : "=r" (result)); 
    return result;
}

/* EOF */
