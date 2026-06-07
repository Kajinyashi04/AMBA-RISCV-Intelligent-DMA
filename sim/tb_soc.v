`timescale 1ns/1ps

module tb_soc();
    reg clk;
    reg rst_n;
    wire debug_done;

    soc_top uut (
        .clk(clk),
        .rst_n(rst_n),
        .debug_done(debug_done)
    );

    always #5 clk = ~clk;

    reg [31:0] expected_data;
    integer i;

    initial begin
        $dumpfile("soc_wave.vcd");
        $dumpvars(0, tb_soc);

        $readmemh("../sw/firmware.hex", uut.sram_memory);
        
        clk = 0; rst_n = 0;
        #20 rst_n = 1;

        // VP_CPU_01: Verify successful memory preloading
        $display("--------------------------------------------------");
        $display("[STATUS VP_CPU_01]: RAM First Instruction: %h", uut.sram_memory[0]);
        $display("--------------------------------------------------");

        fork
            begin
                // Wait for DMA completed status via CPU MMIO register check (VP_CPU_02)
                wait(debug_done == 1'b1);
                #50;
                
                $display("\n--------------------------------------------------");
                $display("--- DMA ACCELERATION TASK COMPLETED! ---");
                $display("--------------------------------------------------");

                // =============================================================
                // VP_DMA_01: AUTOMATIC SELF-CHECKING MATHEMATICAL CHECKER
                // =============================================================
                // 1. Fetch raw data from address 0x100
                // 2. Perform the thresholding math locally in the testbench
                expected_data[7:0]   = (uut.sram_memory[32'h100 >> 2][7:0]   > 8'd128) ? 8'hFF : 8'h00;
                expected_data[15:8]  = (uut.sram_memory[32'h100 >> 2][15:8]  > 8'd128) ? 8'hFF : 8'h00;
                expected_data[23:16] = (uut.sram_memory[32'h100 >> 2][23:16] > 8'd128) ? 8'hFF : 8'h00;
                expected_data[31:24] = (uut.sram_memory[32'h100 >> 2][31:24] > 8'd128) ? 8'hFF : 8'h00;

                // 3. Compare with the actual data written by the DMA to address 0x200
                if (uut.sram_memory[32'h200 >> 2] === expected_data) begin
                    $display("[VERIFICATION SUCCESS]: VP_DMA_01 PASSED!");
                    $display("Raw Input at 0x100: %h", uut.sram_memory[32'h100 >> 2]);
                    $display("Expected Output   : %h", expected_data);
                    $display("Actual HW Output  : %h", uut.sram_memory[32'h200 >> 2]);
                end else begin
                    $display("[VERIFICATION FAILED]: VP_DMA_01 FAILED! Data mismatch!");
                    $display("Expected: %h | Actual: %h", expected_data, uut.sram_memory[32'h200 >> 2]);
                end
                $display("--------------------------------------------------\n");
                $finish;
            end
            begin
                // Watchdog Timer (Timeout prevents infinite simulation hangs)
                #50000;
                $display("\n--------------------------------------------------");
                $display("[TIMEOUT ERROR]: Simulation exceeded limit! Deadlock detected.");
                $display("--------------------------------------------------");
                $finish;
            end
        join
    end

    // =========================================================================
    // SYSTEM-LEVEL ASSERTIONS (V-PLAN COMPLIANCE)
    // =========================================================================

    // 1. VP_ARB_02: Mutual Exclusion Check (No dual-grant)
    // Both CPU (m0) and DMA (m1) must NEVER be granted the bus simultaneously.
    always @(posedge clk) begin
        if (rst_n && uut.arbiter.m0_ready && uut.arbiter.m1_ready) begin
            $display("\n[ASSERTION ERROR] VP_ARB_02 FAILED: Mutual exclusion violated!");
            $display("Both CPU (m0_ready) and DMA (m1_ready) are active at the same time!");
            $finish; // Stop simulation immediately
        end
    end

// =========================================================================
    // 2. VP_PROT_01: Handshake Protocol Hold Check (ĐÃ SỬA LỖI TIMING)
    // Khi CPU_VALID đã bật lên 1 và READY đang bằng 0, 
    // thì Địa chỉ (ADDR) của chu kỳ này phải Y HỆT chu kỳ trước.
    // =========================================================================
    reg        last_cpu_valid;
    reg [31:0] last_cpu_addr;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            last_cpu_valid <= 1'b0;
            last_cpu_addr  <= 32'h0;
        end else begin
            // 1. Chỉ thực hiện kiểm tra nếu chu kỳ TRƯỚC ĐÓ valid đã bằng 1 và ready bằng 0
            if (last_cpu_valid && !uut.cpu_ready) begin
                if (last_cpu_addr != uut.cpu_addr) begin
                    $display("\n[ASSERTION ERROR] VP_PROT_01 FAILED: CPU changed address from %h to %h while waiting for ready!", last_cpu_addr, uut.cpu_addr);
                    $finish; // Dừng mô phỏng vì vi phạm giao thức Bus
                end
            end
            
            // 2. Lưu lại trạng thái của chu kỳ này để làm mốc so sánh cho chu kỳ sau
            last_cpu_valid <= uut.cpu_valid;
            last_cpu_addr  <= uut.cpu_addr;
        end
    end

    // 3. VP_DMA_02: Burst/Multi-word Transfer Tracker
    // Tracks each successful write operation executed by the DMA on the bus.
    integer word_counter;
    initial word_count_init;
    task word_count_init;
        word_counter = 0;
    endtask
    
    always @(posedge clk) begin
        if (rst_n && uut.dma_inst.state == 2'd2 && uut.dma_inst.m_axi_awready) begin
            word_counter <= word_counter + 1;
            $display("[TRACKER VP_DMA_02]: DMA successfully transferred Word %0d to Destination: %h", word_counter, uut.dma_inst.m_axi_awaddr);
        end
    end

    // 4. VP_DMA_03: Zero-Length Safety Check
    // Ensures the DMA FSM never attempts to read from the bus if the configured length is 0.
    always @(posedge clk) begin
        if (rst_n && uut.dma_inst.state == 2'd1 && uut.dma_inst.count == 0) begin
            $display("\n[ASSERTION ERROR] VP_DMA_03 FAILED: DMA entered active READ state with zero transfer length!");
            $finish;
        end
    end

endmodule