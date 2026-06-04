#define DMA_BASE_ADDR 0x40000000


#define DMA_SRC   (*(volatile unsigned int*)(DMA_BASE_ADDR + 0x00))
#define DMA_DST   (*(volatile unsigned int*)(DMA_BASE_ADDR + 0x04))
#define DMA_LEN   (*(volatile unsigned int*)(DMA_BASE_ADDR + 0x08))
#define DMA_CTRL  (*(volatile unsigned int*)(DMA_BASE_ADDR + 0x0C))

void main() {

    unsigned int *ram_source = (unsigned int *)0x00000100;
    ram_source[0] = 0x0AFF32C8; 
    ram_source[1] = 0x11223344;
    

    DMA_SRC = 0x00000100; // Sorce address 
    DMA_DST = 0x00000200; // Destination address
    DMA_LEN = 8;          // Xử lý 8 bytes (2 từ 32-bit)
    

    DMA_CTRL = 1;

   
    while (DMA_CTRL != 2) {
    }

    while(1) {
    }
}