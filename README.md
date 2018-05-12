
# FreeRTOS STM32 Lab

Mix FreeRTOS + OpenCM3 + Newlib

Released
- RTOS interrupt glue code for opencm3
- work with newlib malloc()
- ST7735 based console
- ADC with DMA
- USART loopback

### Memory map

```
------- __stack_start
 |
 |  global_mallock_reserv
 |----  __global_stack_end
 |  <------------------------------------- may be memory reserv
 |----------------- __global_heap_end
------- __freertos_heap_end
 |
----
 |  task2_stack                 RAM MEMORY
 |  task2_control_block
----
 |  some_memory_allocation_for_variables
----
 |  task1_stack
 |  task1_control_block
----
 |
------- __heap_start ---------------------------
 |
 |                              FLASH MEMORY
 | 
-------    --------------------------------------
```

No guarantees. This code is given only as an sample.

Also can see http://wiki.unix7.org/mcu/start

![](http://wiki.unix7.org/_media/mcu/img_20180422_144519-480.jpg)

