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

    initial begin
        $dumpfile("soc_wave.vcd");
        $dumpvars(0, tb_soc);
        $readmemh("../sw/firmware.hex", uut.sram_memory);
        
        clk = 0; rst_n = 0;
        #20 rst_n = 1;

        $display("---------------------------------------");
        $display("RAM src:%h", uut.sram_memory[0]);
        $display("---------------------------------------");

        fork
            begin
                wait(debug_done == 1'b1);
                #50;
                $display("---------------------------------------");
                $display("RAM des: 0x200: %h", uut.sram_memory[32'h200 >> 2]);
                $display("---------------------------------------");
                $finish;
            end
            begin
                #50000;
                $display("---------");
                $display("Timeout");
                $display("---------");
                $finish;
            end
        join
    end
endmodule