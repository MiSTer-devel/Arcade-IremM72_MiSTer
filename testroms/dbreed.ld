OUTPUT_FORMAT("elf32-i386")
OUTPUT_ARCH(i386)
ENTRY(main)

SECTIONS
{
    . = 0x0;

    .text 0x00000 : {
        KEEP(*(.vectors .vectors.*));
        . = ALIGN(4);
        *(.entry.text);
        *(.text);
        *(.text.*);
        *(.entry.text);
        . = ALIGN(16);
    }

    .data 0x0000 : AT(0x88000) {
        data_start = . ;
        *(.rodata .rodata.*);
        *(.data .data.*);
        data_end = . ;

        . = ALIGN(4);
        bss_start = . ;
        *(.bss .bss.*);
        *(COMMON);
        bss_end = . ;
        . = ALIGN(16);
    }

    .reset 0xfff0 : AT(0x3fff0) {
        *(.reset.text);
    }

    /* Throw away everything else. */
    /DISCARD/ : { *(*) }
}
