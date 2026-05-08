module control_unit (
    // --- INPUT (Từ khối Decoder truyền sang) ---
    input  wire[6:0] opcode,
    input  wire [2:0] funct3,
    input  wire       funct7_5,   // Bit thứ 30 của lệnh (Dùng để phân biệt ADD và SUB)

    // --- OUTPUT (Các công tắc điều khiển hệ thống) ---
    output reg        reg_write,  // 1 = Ghi vào thanh ghi
    output reg        alu_src,    // 0 = ALU đọc rs2, 1 = ALU đọc Immediate
    output reg        mem_write,  // 1 = Ghi ra RAM/Bus
    output reg        mem_read,   // 1 = Đọc từ RAM/Bus
    output reg  [1:0] result_src, // MUX chọn kết quả: 00=ALU, 01=RAM, 10=PC+4 (Cho lệnh Nhảy)
    output reg        branch,     // 1 = Lệnh rẽ nhánh (BEQ, BNE...)
    output reg        jump,       // 1 = Lệnh nhảy vô điều kiện (JAL, JALR)
    output reg  [3:0] alu_ctrl    // Lệnh cho ALU (0000=ADD, 1000=SUB...)
);

    reg [1:0] alu_op; // Tín hiệu trung gian

    // =========================================================
    // 1. MAIN DECODER (Bộ giải mã chính - Nhìn vào Opcode)
    // =========================================================
    always @(*) begin
        // Mặc định tắt hết công tắc cho an toàn
        reg_write = 0; alu_src = 0; mem_write = 0; mem_read = 0; 
        result_src = 2'b00; branch = 0; jump = 0; alu_op = 2'b00;

        case (opcode)
            7'b0110011: begin // R-Type (ADD, SUB, AND...)
                reg_write = 1; alu_src = 0; alu_op = 2'b10;
            end
            7'b0010011: begin // I-Type (ADDI, ORI...)
                reg_write = 1; alu_src = 1; alu_op = 2'b10;
            end
            7'b0000011: begin // Load (LW - Đọc RAM)
                reg_write = 1; alu_src = 1; mem_read = 1; result_src = 2'b01; alu_op = 2'b00;
            end
            7'b0100011: begin // Store (SW - Ghi RAM)
                alu_src = 1; mem_write = 1; alu_op = 2'b00;
            end
            7'b1100011: begin // Branch (BEQ, BNE - Rẽ nhánh)
                branch = 1; alu_src = 0; alu_op = 2'b01;
            end
            7'b1101111: begin // JAL (Jump - Nhảy)
                reg_write = 1; jump = 1; result_src = 2'b10;
            end
            7'b0110111: begin // LUI (Load Upper Immediate)
                reg_write = 1; alu_src = 1; alu_op = 2'b11; // Có thể tùy biến thêm
            end
        endcase
    end

// =========================================================
    // 2. ALU DECODER (Hỗ trợ full RV32I)
    // =========================================================
    always @(*) begin
        case (alu_op)
            2'b00: alu_ctrl = 4'b0000; // Load/Store -> CỘNG
            2'b01: alu_ctrl = 4'b1000; // Branch -> TRỪ (Để so sánh)
            2'b10: begin // Lệnh R-Type / I-Type
                case (funct3)
                    3'b000: alu_ctrl = (funct7_5 && opcode == 7'b0110011) ? 4'b1000 : 4'b0000; // SUB/ADD
                    3'b001: alu_ctrl = 4'b0001; // SLL
                    3'b010: alu_ctrl = 4'b0010; // SLT
                    3'b011: alu_ctrl = 4'b0011; // SLTU
                    3'b100: alu_ctrl = 4'b0100; // XOR
                    3'b101: alu_ctrl = funct7_5 ? 4'b1101 : 4'b0101; // SRA/SRL
                    3'b110: alu_ctrl = 4'b0110; // OR
                    3'b111: alu_ctrl = 4'b0111; // AND
                endcase
            end
            default: alu_ctrl = 4'b0000;
        endcase
    end

endmodule