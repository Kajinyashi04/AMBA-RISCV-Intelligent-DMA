// Định nghĩa địa chỉ phần cứng (Trùng khớp 100% với file bus_arbiter.v)
#define DMA_BASE_ADDR 0x40000000

// Từ khóa 'volatile' cực kỳ quan trọng! Nó cấm bộ biên dịch tối ưu hóa đoạn code này,
// ép CPU phải thực sự xuất tín hiệu điện ra Bus mỗi khi gán giá trị.
#define DMA_SRC   (*(volatile unsigned int*)(DMA_BASE_ADDR + 0x00))
#define DMA_DST   (*(volatile unsigned int*)(DMA_BASE_ADDR + 0x04))
#define DMA_LEN   (*(volatile unsigned int*)(DMA_BASE_ADDR + 0x08))
#define DMA_CTRL  (*(volatile unsigned int*)(DMA_BASE_ADDR + 0x0C))

void main() {
    // 1. Tạo một vài dữ liệu giả lập trong RAM (Giả sử là các pixel ảnh)
    // CPU ghi trực tiếp vào RAM (địa chỉ 0x0000_0100)
    unsigned int *ram_source = (unsigned int *)0x00000100;
    ram_source[0] = 0x0AFF32C8; // Pixel ngẫu nhiên
    ram_source[1] = 0x11223344;
    
    // 2. GIAI ĐOẠN 2: CPU Cấu hình DMA (Ghi vào thanh ghi)
    DMA_SRC = 0x00000100; // Địa chỉ gốc
    DMA_DST = 0x00000200; // Địa chỉ đích (Nơi chứa ảnh kết quả)
    DMA_LEN = 8;          // Xử lý 8 bytes (2 từ 32-bit)
    
    // Bấm nút START (Kích hoạt LSB = 1)
    DMA_CTRL = 1;

    // 3. GIAI ĐOẠN 3: CPU Offloading (Chờ đợi)
    // CPU bị kẹt ở vòng lặp này cho đến khi DMA báo DONE (giá trị 2'b10 tức là số 2)
    while (DMA_CTRL != 2) {
        // CPU nhàn rỗi (Trong lúc này DMA đang lấy dữ liệu ảnh, so sánh ngưỡng và ghi lại RAM)
    }

    // 4. Hoàn thành! CPU có thể đọc kết quả từ 0x00000200 nếu muốn
    while(1) {
        // Vòng lặp vô tận để chip không bị treo khi hết chương trình
    }
}