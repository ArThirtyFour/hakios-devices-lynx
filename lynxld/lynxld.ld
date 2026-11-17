ENTRY(_start)
SECTIONS { . = 0x80010000; .text : { *(.text.boot) *(.text .text.*) } .rodata : { *(.rodata .rodata.*) } .data : { *(.data .data.*) } .bss : { __bss_start = .; *(.bss .bss.*) *(COMMON) __bss_end = .; } /DISCARD/ : { *(.comment) *(.eh_frame*) } }
