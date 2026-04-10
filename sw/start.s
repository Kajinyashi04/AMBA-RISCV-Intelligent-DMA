.section .text.init
.global _start

_start:
    /* Khởi tạo Stack Pointer (Trỏ tới cuối RAM 4KB = 0x1000) */
    li sp, 0x1000
    
    /* Nhảy vào hàm main() trong code C của bạn */
    jal main

    /* Nếu thoát khỏi main, rơi vào vòng lặp vô tận để chống treo chip */
end_loop:
    j end_loop
