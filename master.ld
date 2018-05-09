
/* Author, Copyright: Oleg Borodin <onborodin@gmail.com> 2018 */
 
MEMORY {
    FLASH (rx) : ORIGIN = 0x08000000, LENGTH = 64K
    SRAM (rwx) : ORIGIN = 0x20000000, LENGTH = 20K
}

/* Enforce emmition of the vector table. */
EXTERN(vector_table)

/* Define the entry point of the output file. */
ENTRY(main)

SECTIONS {
    __text_start = .;
    .text : {
        KEEP(*(.vectors))    /* Vector table */
        *(.text*)      /* Program code */
        . = ALIGN(4);
        *(.rodata*)    /* Read-only data */
        . = ALIGN(4);
    } >FLASH

    /* C++ Static constructors/destructors, also used for __attribute__
     * ((constructor)) and the likes */

    .preinit_array : {
        . = ALIGN(4);
        __preinit_array_start = .;
        KEEP (*(.preinit_array))
        __preinit_array_end = .;
    } >FLASH

    .init_array : {
        . = ALIGN(4);
        __init_array_start = .;
        KEEP (*(SORT(.init_array.*)))
        KEEP (*(.init_array))
        __init_array_end = .;
    } >FLASH

    .fini_array : {
        . = ALIGN(4);
        __fini_array_start = .;
        KEEP (*(.fini_array))
        KEEP (*(SORT(.fini_array.*)))
        __fini_array_end = .;
    } >FLASH

    /*
     * Another section used by C++ stuff, appears when using newlib with
     * 64bit (long long) printf support
     */

    .ARM.extab : {
        *(.ARM.extab*)
    } >FLASH

    .ARM.exidx : {
        __exidx_start = .;
        *(.ARM.exidx*)
        __exidx_end = .;
    } >FLASH

    . = ALIGN(4);
    __text_end = .;

    .data : {
        __data_start = .;
        _sdata = .;
        _data = .;
        *(.data*)               /* Read-write initialized data */
        . = ALIGN(4);
        __data_end = . ;
        _edata = . ;

    } >SRAM AT >FLASH

    _data_loadaddr = LOADADDR(.data);

    .bss : {
        __bss_start = .;
        __bss_start__ = .;
        *(.bss*)                /* Read-write zero initialized data */
        *(COMMON)
        . = ALIGN(4);
        __bss_end = .;
        __bss_end__ = .;
        _ebss = .;

    } >SRAM

    /*
     * The .eh_frame section appears to be used for C++ exception handling.
     * You may need to fix this if you're using C++.
     */

    /* /DISCARD/ : { *(.eh_frame) } */

    . = ALIGN(4);
    __end = .;
}

PROVIDE(__heap = __end );
PROVIDE(__heap_start = __end);
PROVIDE(__heap_size = LENGTH(SRAM) - 3048);
PROVIDE(__heap_end = ORIGIN(SRAM) + __heap_size);
PROVIDE(_eheap = __heap_end );
PROVIDE(_stack = ORIGIN(SRAM) + LENGTH(SRAM));



/* EOF */




