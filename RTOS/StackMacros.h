/*
    FreeRTOS V9.0.0 - Copyright (C) 2016 Real Time Engineers Ltd.
    All rights reserved
*/

#ifndef STACK_MACROS_H
#define STACK_MACROS_H

/*
 * Call the stack overflow hook function if the stack of the task being swapped
 * out is currently overflowed, or looks like it might have overflowed in the
 * past.
 *
 * Setting configCHECK_FOR_STACK_OVERFLOW to 1 will cause the macro to check
 * the current stack state only - comparing the current top of stack value to
 * the stack limit.  Setting configCHECK_FOR_STACK_OVERFLOW to greater than 1
 * will also cause the last few stack bytes to be checked to ensure the value
 * to which the bytes were set when the task was created have not been
 * overwritten.  Note this second test does not guarantee that an overflowed
 * stack will always be recognised.
 */

/*-----------------------------------------------------------*/

#if ((configCHECK_FOR_STACK_OVERFLOW == 1) && (portSTACK_GROWTH < 0))

    /* Only the current stack state is to be checked. */
    #define taskCHECK_FOR_STACK_OVERFLOW()                                                              \
    {                                                                                                   \
        /* Is the currently saved stack pointer within the stack limit? */                              \
        if(pxCurrentTCB->pxTopOfStack <= pxCurrentTCB->pxStack)                                         \
        {                                                                                               \
            vApplicationStackOverflowHook((TaskHandle_t) pxCurrentTCB, pxCurrentTCB->pcTaskName);       \
        }                                                                                               \
    }

#endif /* configCHECK_FOR_STACK_OVERFLOW == 1 */

/*-----------------------------------------------------------*/

#if ((configCHECK_FOR_STACK_OVERFLOW == 1) && (portSTACK_GROWTH > 0))

    /* Only the current stack state is to be checked. */
    #define taskCHECK_FOR_STACK_OVERFLOW()                                                              \
    {                                                                                                   \
        /* Is the currently saved stack pointer within the stack limit? */                              \
        if(pxCurrentTCB->pxTopOfStack >= pxCurrentTCB->pxEndOfStack)                                    \
        {                                                                                               \
            vApplicationStackOverflowHook((TaskHandle_t) pxCurrentTCB, pxCurrentTCB->pcTaskName);       \
        }                                                                                               \
    }

#endif /* configCHECK_FOR_STACK_OVERFLOW == 1 */

/*-----------------------------------------------------------*/

#if ((configCHECK_FOR_STACK_OVERFLOW > 1) && (portSTACK_GROWTH < 0))

    #define taskCHECK_FOR_STACK_OVERFLOW()                                                              \
    {                                                                                                   \
        const uint32_t * const pulStack = (uint32_t *) pxCurrentTCB->pxStack;                           \
        const uint32_t ulCheckValue = (uint32_t) 0xa5a5a5a5;                                            \
                                                                                                        \
        if((pulStack[ 0 ] != ulCheckValue) ||                                                           \
            (pulStack[ 1 ] != ulCheckValue) ||                                                          \
            (pulStack[ 2 ] != ulCheckValue) ||                                                          \
            (pulStack[ 3 ] != ulCheckValue))                                                            \
        {                                                                                               \
            vApplicationStackOverflowHook((TaskHandle_t) pxCurrentTCB, pxCurrentTCB->pcTaskName);       \
        }                                                                                               \
    }

#endif /* #if (configCHECK_FOR_STACK_OVERFLOW > 1) */

/*-----------------------------------------------------------*/

#if ((configCHECK_FOR_STACK_OVERFLOW > 1) && (portSTACK_GROWTH > 0))

    #define taskCHECK_FOR_STACK_OVERFLOW()                                                                                              \
    {                                                                                                                                   \
    int8_t *pcEndOfStack = (int8_t *) pxCurrentTCB->pxEndOfStack;                                                                     \
    static const uint8_t ucExpectedStackBytes[] = { tskSTACK_FILL_BYTE, tskSTACK_FILL_BYTE, tskSTACK_FILL_BYTE, tskSTACK_FILL_BYTE,     \
                                                    tskSTACK_FILL_BYTE, tskSTACK_FILL_BYTE, tskSTACK_FILL_BYTE, tskSTACK_FILL_BYTE,     \
                                                    tskSTACK_FILL_BYTE, tskSTACK_FILL_BYTE, tskSTACK_FILL_BYTE, tskSTACK_FILL_BYTE,     \
                                                    tskSTACK_FILL_BYTE, tskSTACK_FILL_BYTE, tskSTACK_FILL_BYTE, tskSTACK_FILL_BYTE,     \
                                                    tskSTACK_FILL_BYTE, tskSTACK_FILL_BYTE, tskSTACK_FILL_BYTE, tskSTACK_FILL_BYTE };   \
                                                                                                                                        \
        pcEndOfStack -= sizeof(ucExpectedStackBytes);                                                                                 \
                                                                                                                                        \
        /* Has the extremity of the task stack ever been written over? */                                                               \
        if(memcmp((void *) pcEndOfStack, (void *) ucExpectedStackBytes, sizeof(ucExpectedStackBytes)) != 0)                   \
        {                                                                                                                               \
            vApplicationStackOverflowHook((TaskHandle_t) pxCurrentTCB, pxCurrentTCB->pcTaskName);                                   \
        }                                                                                                                               \
    }

#endif /* #if (configCHECK_FOR_STACK_OVERFLOW > 1) */

/*-----------------------------------------------------------*/

/* Remove stack overflow macro if not being used. */
#ifndef taskCHECK_FOR_STACK_OVERFLOW
    #define taskCHECK_FOR_STACK_OVERFLOW()
#endif

#endif /* STACK_MACROS_H */

