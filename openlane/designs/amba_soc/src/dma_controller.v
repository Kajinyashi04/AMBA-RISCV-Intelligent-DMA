module dma_controller (
    input  wire        clk,
    input  wire        rst_n,

    // -- Slave Interface (CPU -> DMA) --
    input  wire [31:0] s_axi_awaddr,
    input  wire        s_axi_awvalid,
    output wire        s_axi_awready,
    input  wire [31:0] s_axi_wdata,
    input  wire        s_axi_wvalid,
    output wire        s_axi_wready,
    output wire [31:0] s_axi_rdata,
    input  wire [31:0] s_axi_araddr,

    // -- Master Interface (DMA -> RAM) --
    output reg  [31:0] m_axi_araddr,
    output wire        m_axi_arvalid,
    input  wire        m_axi_arready,
    input  wire [31:0] m_axi_rdata,
    
    output reg  [31:0] m_axi_awaddr,
    output wire        m_axi_awvalid,
    input  wire        m_axi_awready,
    output wire [31:0] m_axi_wdata,
    output wire        m_axi_wvalid
);

    // Thanh ghi cấu hình
    reg [31:0] reg_src, reg_dst, reg_len;
    reg [1:0]  reg_status; // 0:Idle, 1:Busy, 2:Done
    reg        dma_start_pulse;

    // Logic Slave: CPU ghi vào ngăn kéo
    assign s_axi_awready = 1'b1;
    assign s_axi_wready  = 1'b1;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            reg_src <= 0; reg_dst <= 0; reg_len <= 0; dma_start_pulse <= 0;
        end else if (s_axi_awvalid && s_axi_wvalid) begin
            case (s_axi_awaddr[3:0])
                4'h0: reg_src <= s_axi_wdata;
                4'h4: reg_dst <= s_axi_wdata;
                4'h8: reg_len <= s_axi_wdata;
                4'hC: dma_start_pulse <= s_axi_wdata[0];
            endcase
        end else begin
            dma_start_pulse <= 1'b0;
        end
    end
    assign s_axi_rdata = (s_axi_araddr[3:0] == 4'hC) ? {30'b0, reg_status} : 32'h0;

    // Máy trạng thái DMA
    localparam IDLE=0, READ=1, WRITE=2;
    reg [1:0] state;
    reg [31:0] count;
    reg [31:0] data_buf;

    // Logic xử lý ảnh (Intelligence)
    wire [31:0] processed = {
        (m_axi_rdata[31:24] > 8'd128) ? 8'hFF : 8'h00,
        (m_axi_rdata[23:16] > 8'd128) ? 8'hFF : 8'h00,
        (m_axi_rdata[15:8]  > 8'd128) ? 8'hFF : 8'h00,
        (m_axi_rdata[7:0]   > 8'd128) ? 8'hFF : 8'h00
    };

    // Điều khiển Master
    assign m_axi_arvalid = (state == READ);
    assign m_axi_awvalid = (state == WRITE);
    assign m_axi_wvalid  = (state == WRITE);
    assign m_axi_wdata   = data_buf;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state <= IDLE; reg_status <= 0;
        end else begin
            case (state)
                IDLE: if (dma_start_pulse) begin
                    m_axi_araddr <= reg_src;
                    m_axi_awaddr <= reg_dst;
                    count <= reg_len;
                    state <= READ;
                    reg_status <= 2'b01; // Busy
                end
                READ: if (m_axi_arready) begin
                    data_buf <= processed; // Xử lý ảnh ngay khi đọc
                    state <= WRITE;
                end
                WRITE: if (m_axi_awready) begin
                    if (count <= 4) begin
                        state <= IDLE;
                        reg_status <= 2'b10; // Done!
                    end else begin
                        m_axi_araddr <= m_axi_araddr + 4;
                        m_axi_awaddr <= m_axi_awaddr + 4;
                        count <= count - 4;
                        state <= READ;
                    end
                end
            endcase
        end
    end
endmodule