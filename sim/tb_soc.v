`timescale 1ns/1ps

module tb_soc();
    reg clk;
    reg rst_n;
    wire debug_done;

    // Gọi con chip SoC ra
    soc_top uut (
        .clk(clk),
        .rst_n(rst_n),
        .debug_done(debug_done)
    );

    // Tạo xung nhịp 10ns
    always #5 clk = ~clk;

    initial begin
        $dumpfile("soc_wave.vcd");
        $dumpvars(0, tb_soc);
        $readmemh("../sw/firmware.hex", uut.sram_memory);
        // Bật nguồn
        clk = 0; rst_n = 0;
        #20 rst_n = 1;

        $display("--- BẬT NGUỒN SOC ---");
        $display("Lenh dau tien tai RAM: %h", uut.sram_memory[0]); // Chèn dòng này
        $display("CPU dang doc lenh tu RAM va cau hinh DMA...");

        // Chờ CPU và DMA làm việc. (Giới hạn tối đa 5000 clock để tránh bị treo vô tận)
        fork
            begin
                wait(debug_done == 1'b1);
                #50;
                $display("--- DMA DA BAO CAO HOAN THANH! ---");
                // Kiểm tra dữ liệu ảnh tại đích 0x200
                $display("Du lieu dich tai RAM 0x200: %h", uut.sram_memory[32'h200 >> 2]);
                $finish;
            end
            begin
                #50000;
                $display("!!! LOI: HET THOI GIAN MO PHONG (TIMEOUT) !!!");
                $display("Co the CPU hoac DMA da bi treo tren Bus.");
                $finish;
            end
        join
    end
endmodule