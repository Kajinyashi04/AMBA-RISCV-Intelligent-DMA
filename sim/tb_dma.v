`timescale 1ns/1ps

module tb_dma();

    // 1. Khai báo tín hiệu
    reg clk;
    reg rst_n;

    // Tín hiệu AXI-Lite (CPU mô phỏng)
    reg [31:0] s_axi_awaddr;
    reg s_axi_awvalid;
    wire s_axi_awready;
    reg [31:0] s_axi_wdata;
    reg s_axi_wvalid;
    wire s_axi_wready;
    wire [1:0] s_axi_bresp;
    wire s_axi_bvalid;
    reg s_axi_bready;

    // Tín hiệu AXI (RAM mô phỏng)
    wire [31:0] m_axi_araddr;
    wire m_axi_arvalid;
    reg m_axi_arready;
    reg [31:0] m_axi_rdata;
    reg m_axi_rvalid;
    
    wire [31:0] m_axi_awaddr;
    wire m_axi_awvalid;
    reg m_axi_awready;
    wire [31:0] m_axi_wdata;
    wire m_axi_wvalid;
    reg m_axi_wready;
    reg m_axi_bvalid;

    // Kết nối với DMA Controller của bạn
    dma_controller uut (
        .clk(clk), .rst_n(rst_n),
        
        // Slave ports
        .s_axi_awaddr(s_axi_awaddr), .s_axi_awvalid(s_axi_awvalid), .s_axi_awready(s_axi_awready),
        .s_axi_wdata(s_axi_wdata), .s_axi_wvalid(s_axi_wvalid), .s_axi_wready(s_axi_wready),
        .s_axi_bresp(s_axi_bresp), .s_axi_bvalid(s_axi_bvalid), .s_axi_bready(s_axi_bready),
        
        .s_axi_araddr(32'h0), .s_axi_arvalid(1'b0), .s_axi_rready(1'b1), // Bỏ qua phần Read Status cho gọn
        
        // Master ports
        .m_axi_araddr(m_axi_araddr), .m_axi_arvalid(m_axi_arvalid), .m_axi_arready(m_axi_arready),
        .m_axi_rdata(m_axi_rdata), .m_axi_rvalid(m_axi_rvalid), .m_axi_rready(),
        
        .m_axi_awaddr(m_axi_awaddr), .m_axi_awvalid(m_axi_awvalid), .m_axi_awready(m_axi_awready),
        .m_axi_wdata(m_axi_wdata), .m_axi_wvalid(m_axi_wvalid), .m_axi_wready(m_axi_wready),
        .m_axi_bvalid(m_axi_bvalid), .m_axi_bready()
    );

    // 2. Tạo Xung nhịp (Clock) - 10ns
    always #5 clk = ~clk;

    // 3. Khối mô phỏng RAM (Đơn giản hóa)
    reg [31:0] fake_ram [0:1023]; // Bộ nhớ 4KB
    
    always @(posedge clk) begin
        // Phản hồi Đọc
        m_axi_arready <= 1'b1;
        if (m_axi_arvalid && m_axi_arready) begin
            m_axi_rvalid <= 1'b1;
            // Trả về dữ liệu từ RAM. Chú ý: Dịch địa chỉ đi 2 bit (chia 4) vì mỗi ô RAM chứa 4 byte
            m_axi_rdata <= fake_ram[m_axi_araddr >> 2]; 
        end else begin
            m_axi_rvalid <= 1'b0;
        end

        // Phản hồi Ghi
        m_axi_awready <= 1'b1;
        m_axi_wready  <= 1'b1;
        m_axi_bvalid  <= 1'b0;
        if (m_axi_wvalid && m_axi_wready) begin
            fake_ram[m_axi_awaddr >> 2] <= m_axi_wdata; // Ghi dữ liệu vào RAM
            m_axi_bvalid <= 1'b1;
            $display("THỜI GIAN %0t: RAM nhận được dữ liệu GHI: %h tại địa chỉ %h", $time, m_axi_wdata, m_axi_awaddr);
        end
    end

    // 4. Kịch bản chạy Test (CPU)
    initial begin
        // Khởi tạo file xuất waveform để xem trên GTKWave
        $dumpfile("dma_wave.vcd");
        $dumpvars(0, tb_dma);

        // Khởi tạo giá trị
        clk = 0; rst_n = 0;
        s_axi_awvalid = 0; s_axi_wvalid = 0; s_axi_bready = 1;
        
        // Tạo "ảnh gốc" trong RAM (Source Addr = 0x100)
        // Ô thứ 1: Pixel 1=200(Trắng), Pixel 2=50(Đen), Pixel 3=255(Trắng), Pixel 4=10(Đen)
        fake_ram[32'h100 >> 2] = 32'h0A_FF_32_C8; 

        #20 rst_n = 1; // Bỏ Reset
        #20;

        $display("--- BẮT ĐẦU TEST DMA ---");

        // CPU Cấu hình Source = 0x100 (Ghi vào ngăn 0x00)
        s_axi_awaddr = 32'h00; s_axi_wdata = 32'h100;
        s_axi_awvalid = 1; s_axi_wvalid = 1;
        #10 s_axi_awvalid = 0; s_axi_wvalid = 0; #10;

        // CPU Cấu hình Dest = 0x200 (Ghi vào ngăn 0x04)
        s_axi_awaddr = 32'h04; s_axi_wdata = 32'h200;
        s_axi_awvalid = 1; s_axi_wvalid = 1;
        #10 s_axi_awvalid = 0; s_axi_wvalid = 0; #10;

        // CPU Cấu hình Length = 4 bytes (Ghi vào ngăn 0x08)
        s_axi_awaddr = 32'h08; s_axi_wdata = 32'd4;
        s_axi_awvalid = 1; s_axi_wvalid = 1;
        #10 s_axi_awvalid = 0; s_axi_wvalid = 0; #10;

        // CPU Bấm nút START (Ghi 1 vào ngăn 0x0C)
        s_axi_awaddr = 32'h0C; s_axi_wdata = 32'd1;
        s_axi_awvalid = 1; s_axi_wvalid = 1;
        #10 s_axi_awvalid = 0; s_axi_wvalid = 0;

        // Chờ DMA chạy xong
        #200;
        
        $display("--- KẾT THÚC TEST ---");
        $display("Dữ liệu GỐC tại 0x100 : %h", fake_ram[32'h100 >> 2]);
        $display("Dữ liệu ĐÍCH tại 0x200: %h", fake_ram[32'h200 >> 2]);
        
        $finish;
    end
endmodule