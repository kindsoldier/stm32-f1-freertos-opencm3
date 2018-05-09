/*
    FreeRTOS V9.0.0 - Copyright (C) 2016 Real Time Engineers Ltd.
    All rights reserved
*/


/*
 * Implementation of pvPortMalloc() and vPortFree() that relies on the
 * compilers own malloc() and free() implementations.
 *
 * This file can only be used if the linker is configured to to generate
 * a heap memory area.
 *
 * See heap_1.c, heap_2.c and heap_4.c for alternative implementations, and the
 * memory management pages of http://www.FreeRTOS.org for more information.
 */

#include <stdlib.h>

/* Defining MPU_WRAPPERS_INCLUDED_FROM_API_FILE prevents task.h from redefining
all the API functions to use the MPU wrappers.  That should only be done when
task.h is included from an application file. */
#define MPU_WRAPPERS_INCLUDED_FROM_API_FILE

#include <FreeRTOS.h>
#include <task.h>

#undef MPU_WRAPPERS_INCLUDED_FROM_API_FILE

#if (configSUPPORT_DYNAMIC_ALLOCATION == 0)
    #error This file must not be used if configSUPPORT_DYNAMIC_ALLOCATION is 0
#endif


/*-----------------------------------------------------------*/

void *pvPortMalloc(size_t xWantedSize) {
    void *pvReturn;

    vTaskSuspendAll();
    {
        pvReturn = malloc(xWantedSize);
        traceMALLOC(pvReturn, xWantedSize);
    }
    (void)xTaskResumeAll();

#if (configUSE_MALLOC_FAILED_HOOK == 1)
    {
        if (pvReturn == NULL) {
            extern void vApplicationMallocFailedHook(void);
            vApplicationMallocFailedHook();
        }
    }
#endif

    return pvReturn;
}

/*-----------------------------------------------------------*/

void vPortFree(void *pv) {
    if (pv) {
        vTaskSuspendAll();
        {
            free(pv);
            traceFREE(pv, 0);
        }
        (void)xTaskResumeAll();
    }
}
