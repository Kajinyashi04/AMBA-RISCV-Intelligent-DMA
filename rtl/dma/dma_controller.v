module dma_controller (
    input wire clk,
    input wire rst_n,

    // AXI_Lite slave interface
    input wire [31:0] s_axi_awaddr, // Address for write operations
    input wire        s_axi_awvalid, // Write address valid
    output wire        s_axi_awready, // Write address ready
    input wire [31:0] s_axi_wdata, // Data for write operations
    // input wire [3:0]  s_axi_wstrb,
    input wire        s_axi_wvalid, // Write data valid
    output wire        s_axi_wready, // Write data ready
    output wire [1:0]  s_axi_bvalid, // Write response valid
    input wire [1:0]  s_axi_bready, // Write response ready

    // Local interface to DMA engine
    output reg [31:0] dma_src_addr, // Source address for DMA transfer
    output reg [31:0] dma_dst_addr, // Destination address for DMA transfer
    output reg [15:0] dma_length,   // Length of data to transfer
    output reg        dma_start,    // Signal to start DMA transfer
);

    // Registers for control and status
    // 0x00: Source Address
    // 0x04: Destination Address
    // 0x08: Length
    // 0x0C: Control (bit 0: start)

    assign s_axi_awready = 1'b1; // Always ready to accept write address
    assign s_axi_wready = 1'b1; // Always ready to accept write data
    assign s_axi_bvalid = 2'b11; // Always valid write response

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            dma_src_addr <= 32'b0;
            dma_dst_addr <= 32'b0;
            dma_length <= 16'b0;
            dma_start <= 1'b0;
        end else begin
            if (s_axi_awvalid && s_axi_wvalid) begin
                case (s_axi_awaddr[3:0])
                    4'h0: dma_src_addr <= s_axi_wdata; // Write source address
                    4'h4: dma_dst_addr <= s_axi_wdata; // Write destination address
                    4'h8: dma_length <= s_axi_wdata[15:0]; // Write length
                    4'hC: dma_start <= s_axi_wdata[0]; // Write control (start)
                    default: ; // Ignore other addresses
                endcase
            end else begin
                dma_start <= 1'b0; // Clear start signal after one cycle
            end
        end
    end

endmodule