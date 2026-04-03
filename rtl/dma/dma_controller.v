module dma_controller #(
    parameter AXI_ADDR_WIDTH = 32,
    parameter AXI_DATA_WIDTH = 32
) (
    input  wire                  clk,
    input  wire                  rst_n,

    // -- AXI-Lite Slave Interface (CPU -> DMA) --
    input  wire [AXI_ADDR_WIDTH-1:0] s_axi_awaddr,
    input  wire                      s_axi_awvalid,
    output wire                      s_axi_awready,
    input  wire [AXI_DATA_WIDTH-1:0] s_axi_wdata,
    input  wire                      s_axi_wvalid,
    output wire                      s_axi_wready,
    output wire [1:0]                s_axi_bresp,
    output wire                      s_axi_bvalid,
    input  wire                      s_axi_bready,
    input  wire [AXI_ADDR_WIDTH-1:0] s_axi_araddr,
    input  wire                      s_axi_arvalid,
    output wire                      s_axi_arready,
    output wire [AXI_DATA_WIDTH-1:0] s_axi_rdata,
    output wire [1:0]                s_axi_rresp,
    output wire                      s_axi_rvalid,
    input  wire                      s_axi_rready,

    // -- AXI4 Master Interface (DMA -> RAM) --
    output wire [AXI_ADDR_WIDTH-1:0] m_axi_araddr,
    output wire                      m_axi_arvalid,
    input  wire                      m_axi_arready,
    input  wire [AXI_DATA_WIDTH-1:0] m_axi_rdata,
    input  wire [1:0]                m_axi_rresp,
    input  wire                      m_axi_rvalid,
    output wire                      m_axi_rready,
    output wire [AXI_ADDR_WIDTH-1:0] m_axi_awaddr,
    output wire                      m_axi_awvalid,
    input  wire                      m_axi_awready,
    output wire [AXI_DATA_WIDTH-1:0] m_axi_wdata,
    output wire                      m_axi_wvalid,
    input  wire                      m_axi_wready,
    input  wire [1:0]                m_axi_bresp,
    input  wire                      m_axi_bvalid,
    output wire                      m_axi_bready
);

//================================================================
// 1. Thanh ghi nội bộ (Internal Registers)
//================================================================
reg [31:0] reg_src_addr;
reg [31:0] reg_dst_addr;
reg [31:0] reg_len;
reg        reg_start;
reg [1:0]  reg_status; // 00: idle, 01: busy, 10: done

//================================================================
// 2. Logic cho Giao diện Slave (CPU cấu hình DMA)
//================================================================
reg s_axi_awready_reg, s_axi_wready_reg, s_axi_arready_reg, s_axi_rvalid_reg;
reg s_axi_bvalid_reg;

// -- Logic Ghi (Write) --
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        reg_src_addr <= 32'h0;
        reg_dst_addr <= 32'h0;
        reg_len      <= 32'h0;
        reg_start    <= 1'b0;
        reg_status   <= 2'b00;
    end else begin
        // CPU ghi lệnh vào thanh ghi
        if (s_axi_awvalid && s_axi_wvalid) begin
            case (s_axi_awaddr[3:0])
                4'h0: reg_src_addr <= s_axi_wdata;
                4'h4: reg_dst_addr <= s_axi_wdata;
                4'h8: reg_len      <= s_axi_wdata;
                4'hC: begin
                    reg_start <= s_axi_wdata[0];
                    if (s_axi_wdata[0]) reg_status <= 2'b01; // Set busy
                end
                default: ;
            endcase
        end else begin
            reg_start <= 1'b0; // Start chỉ có hiệu lực 1 chu kỳ
        end
        // Khi DMA làm xong (FSM_DONE), reset lại cờ status
        if(reg_status == 2'b10 && s_axi_bready) reg_status <= 2'b00;
    end
end

assign s_axi_awready = !s_axi_bvalid_reg || s_axi_bready;
assign s_axi_wready  = !s_axi_bvalid_reg || s_axi_bready;
assign s_axi_bvalid  = s_axi_bvalid_reg;
assign s_axi_bresp   = 2'b00; // OK

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) s_axi_bvalid_reg <= 1'b0;
    else if (s_axi_awvalid && s_axi_wvalid && !s_axi_bvalid_reg) s_axi_bvalid_reg <= 1'b1;
    else if (s_axi_bready) s_axi_bvalid_reg <= 1'b0;
end

// -- Logic Đọc (Read - để CPU kiểm tra trạng thái) --
assign s_axi_arready = 1'b1;
assign s_axi_rvalid  = 1'b1;
assign s_axi_rresp   = 2'b00; // OK
assign s_axi_rdata   = (s_axi_araddr[3:0] == 4'hC) ? {30'b0, reg_status} : 32'h0;

//================================================================
// 3. Logic cho Giao diện Master (DMA tự đi làm việc)
//================================================================

// --- Định nghĩa Máy Trạng thái (State Machine) ---
localparam FSM_IDLE       = 3'd0;
localparam FSM_READ_ADDR  = 3'd1;
localparam FSM_READ_DATA  = 3'd2;
localparam FSM_WRITE_ADDR = 3'd3;
localparam FSM_WRITE_DATA = 3'd4;
localparam FSM_WRITE_RESP = 3'd5;

reg [2:0] state, next_state;
reg [31:0] current_src, current_dst, bytes_left;
reg [31:0] data_buffer; // Thanh ghi tạm chứa dữ liệu đã được xử lý

// --- Logic xử lý ảnh (Image Thresholding) ---
wire [31:0] processed_data;
assign processed_data[7:0]   = (m_axi_rdata[7:0]   > 8'd128) ? 8'hFF : 8'h00;
assign processed_data[15:8]  = (m_axi_rdata[15:8]  > 8'd128) ? 8'hFF : 8'h00;
assign processed_data[23:16] = (m_axi_rdata[23:16] > 8'd128) ? 8'hFF : 8'h00;
assign processed_data[31:24] = (m_axi_rdata[31:24] > 8'd128) ? 8'hFF : 8'h00;

// --- Gán tín hiệu điều khiển Master ---
assign m_axi_arvalid = (state == FSM_READ_ADDR);
assign m_axi_araddr  = current_src;
assign m_axi_rready  = (state == FSM_READ_DATA);

assign m_axi_awvalid = (state == FSM_WRITE_ADDR);
assign m_axi_awaddr  = current_dst;
assign m_axi_wvalid  = (state == FSM_WRITE_DATA);
assign m_axi_wdata   = data_buffer;
assign m_axi_bready  = (state == FSM_WRITE_RESP);

// --- Cập nhật trạng thái (Phần Tuần tự) ---
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        state <= FSM_IDLE;
    end else begin
        state <= next_state;
    end
end

// --- Logic chuyển trạng thái (Phần Tổ hợp) ---
always @(*) begin
    next_state = state;
    case (state)
        FSM_IDLE:
            if (reg_start) begin
                current_src <= reg_src_addr;
                current_dst <= reg_dst_addr;
                bytes_left  <= reg_len;
                next_state  = FSM_READ_ADDR;
            end
        FSM_READ_ADDR:
            if (m_axi_arready) next_state = FSM_READ_DATA;
        FSM_READ_DATA:
            if (m_axi_rvalid) begin
                data_buffer <= processed_data; // Lưu dữ liệu đã qua xử lý
                next_state  = FSM_WRITE_ADDR;
            end
        FSM_WRITE_ADDR:
            if (m_axi_awready) next_state = FSM_WRITE_DATA;
        FSM_WRITE_DATA:
            if (m_axi_wready) next_state = FSM_WRITE_RESP;
        FSM_WRITE_RESP:
            if (m_axi_bvalid) begin
                if (bytes_left <= 4) begin
                    reg_status <= 2'b10; // Đặt cờ báo xong
                    next_state = FSM_IDLE;
                end else begin
                    current_src <= current_src + 4;
                    current_dst <= current_dst + 4;
                    bytes_left  <= bytes_left - 4;
                    next_state  = FSM_READ_ADDR;
                end
            end
        default:
            next_state = FSM_IDLE;
    endcase
end

endmodule