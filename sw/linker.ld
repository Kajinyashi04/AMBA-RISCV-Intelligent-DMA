OUTPUT_ARCH("riscv")
ENTRY(_start)

SECTIONS
{
    /* Đặt code bắt đầu chính xác tại địa chỉ 0x00000000 (Khớp với SOC_TOP) */
    . = 0x00000000;
    .text : {
        *(.text.init)
        *(.text*)
    }
    .data : { *(.data*) }
    .bss : { *(.bss*) }
}