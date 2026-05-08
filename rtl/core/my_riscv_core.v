module my_riscv_core #(
    parameter PROGADDR_RESET = 32'h0000_0000
)(
    input  wire        clk,
    input  wire        resetn,
    
    // Giao tiếp với Bus (Chuẩn Native)
    output reg         mem_valid,
    input  wire        mem_ready,
    output reg  [31:0] mem_addr,
    output reg  [31:0] mem_wdata,
    output reg  [3:0]  mem_wstrb,
    input  wire [31:0] mem_rdata
);

    // ==========================================
    // 1. KHAI BÁO DÂY ĐIỆN VÀ BIẾN NỘI BỘ
    // ==========================================
    reg [31:0] pc;
    reg [31:0] instruction;
    
    // Các dây từ Decoder
    wire [6:0]  opcode;
    wire [2:0]  funct3;
    wire [6:0]  funct7;
    wire [4:0]  rs1_addr, rs2_addr, rd_addr;
    wire [31:0] imm;
    
    // Các dây từ Control Unit
    wire        reg_write, alu_src, mem_write, mem_read, branch, jump;
    wire [1:0]  result_src;
    wire [3:0]  alu_ctrl;
    
    // Các dây của Register File và ALU
    wire [31:0] rs1_data, rs2_data;
    wire[31:0] alu_result;
    wire        zero;

    // ==========================================
    // 2. LẮP RÁP CÁC LINH KIỆN (INSTANTIATION)
    // ==========================================

    decoder u_decoder (
        .instr(instruction),
        .opcode(opcode), .funct3(funct3), .funct7(funct7),
        .rs1_addr(rs1_addr), .rs2_addr(rs2_addr), .rd_addr(rd_addr),
        .imm(imm)
    );

    control_unit u_control (
        .opcode(opcode), .funct3(funct3), .funct7_5(funct7[5]),
        .reg_write(reg_write), .alu_src(alu_src),
        .mem_write(mem_write), .mem_read(mem_read),
        .result_src(result_src), .branch(branch), .jump(jump),
        .alu_ctrl(alu_ctrl)
    );

    reg        rf_we; // Cầu dao bật/tắt ghi thanh ghi
    reg [31:0] rf_wdata; // Dữ liệu ghi vào thanh ghi

    reg_file u_reg_file (
        .clk(clk),
        .we(rf_we),
        .rs1_addr(rs1_addr), .rs2_addr(rs2_addr), .rd_addr(rd_addr),
        .rd_data(rf_wdata),
        .rs1_data(rs1_data), .rs2_data(rs2_data)
    );

    // MUX chọn đầu vào B cho ALU
    wire [31:0] alu_b = alu_src ? imm : rs2_data;

    alu u_alu (
        .a(rs1_data), .b(alu_b), .alu_ctrl(alu_ctrl),
        .result(alu_result), .zero(zero)
    );
// ==========================================
    // LOGIC RẼ NHÁNH (BRANCH) ĐẦY ĐỦ
    // ==========================================
    reg take_branch;
    always @(*) begin
        case(funct3)
            3'b000: take_branch = zero;           // BEQ (Bằng nhau)
            3'b001: take_branch = !zero;          // BNE (Khác nhau)
            3'b100: take_branch = alu_result[0];  // BLT (Nhỏ hơn)
            3'b101: take_branch = !alu_result[0]; // BGE (Lớn/Bằng)
            3'b110: take_branch = alu_result[0];  // BLTU
            3'b111: take_branch = !alu_result[0]; // BGEU
            default: take_branch = 0;
        endcase
    end

    // ==========================================
    // 3. MÁY TRẠNG THÁI ĐIỀU KHIỂN LUỒNG (FSM)
    // ==========================================
    localparam S_FETCH    = 2'd0; // Đi lấy lệnh
    localparam S_EXECUTE  = 2'd1; // Giải mã và tính toán
    localparam S_MEM_WAIT = 2'd2; // Chờ RAM trả lời (nếu có lệnh Load/Store)

    reg [1:0] state;

    always @(posedge clk or negedge resetn) begin
        if (!resetn) begin
            pc <= PROGADDR_RESET;
            state <= S_FETCH;
            mem_valid <= 0;
            mem_wstrb <= 0;
            rf_we <= 0;
        end else begin
            rf_we <= 0; // Mặc định tắt ghi thanh ghi để an toàn

            case (state)
                S_FETCH: begin
                    mem_addr  <= pc;
                    mem_valid <= 1;
                    mem_wstrb <= 4'b0000; // 0 = Đọc
                    
                    if (mem_ready && mem_valid) begin
                        instruction <= mem_rdata; // Bắt lấy lệnh từ RAM
                        mem_valid   <= 0;         // Cắt yêu cầu Bus
                        state       <= S_EXECUTE; // Chuyển sang xử lý
                    end
                end

                S_EXECUTE: begin
                    // Nếu là lệnh truy cập RAM (LW, SW)
                    if (mem_read || mem_write) begin
                        mem_addr  <= alu_result; // ALU tính ra địa chỉ RAM
                        mem_valid <= 1;
                        mem_wdata <= rs2_data;   // Ghi dữ liệu từ rs2
                        mem_wstrb <= mem_write ? 4'b1111 : 4'b0000;
                        state     <= S_MEM_WAIT;
                    end 
                    // Nếu là các lệnh khác (Tính toán, Nhảy...)
                    else begin
                        // 1. Cập nhật Register File
                        if (reg_write) begin
                            rf_we <= 1;
                            // Chọn kết quả từ ALU hoặc PC+4 (cho lệnh JAL)
                            rf_wdata <= (result_src == 2'b10) ? (pc + 4) : alu_result;
                        end
                        
                        // 2. Cập nhật Bộ đếm PC
                        if ((branch && take_branch) || jump) begin
                            pc <= pc + imm; // Nhảy
                        end else begin
                            pc <= pc + 4;   // Đi tiếp
                        end
                        
                        state <= S_FETCH; // Quay lại đọc lệnh mới
                    end
                end

                S_MEM_WAIT: begin
                    if (mem_ready && mem_valid) begin
                        mem_valid <= 0;
                        
                        // Nếu là lệnh LW (Load), phải ghi dữ liệu lấy từ RAM vào thanh ghi
                        if (mem_read && reg_write) begin
                            rf_we <= 1;
                            rf_wdata <= mem_rdata; 
                        end
                        
                        pc <= pc + 4; // Lệnh Load/Store không bao giờ rẽ nhánh
                        state <= S_FETCH;
                    end
                end
            endcase
        end
    end

endmodule