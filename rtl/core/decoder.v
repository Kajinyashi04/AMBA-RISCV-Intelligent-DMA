module decoder (
    input  wire [31:0] instr,      // Lệnh 32-bit lấy từ RAM về

    output wire [6:0]  opcode,     // Phân loại nhóm lệnh (7 bit cuối)
    output wire [2:0]  funct3,     // Phân loại chi tiết lệnh (3 bit)
    output wire [6:0]  funct7,     // Bổ sung cho funct3 (7 bit)
    
    output wire [4:0]  rs1_addr,   // Địa chỉ thanh ghi nguồn 1
    output wire [4:0]  rs2_addr,   // Địa chỉ thanh ghi nguồn 2
    output wire [4:0]  rd_addr,    // Địa chỉ thanh ghi đích
    
    output wire [31:0] imm         // Giá trị tức thời (Số gài sẵn trong lệnh)
);

    assign opcode   = instr[6:0];
    assign rd_addr  = instr[11:7];
    assign funct3   = instr[14:12];
    assign rs1_addr = instr[19:15];
    assign rs2_addr = instr[24:20];
    assign funct7   = instr[31:25];

       
    reg [31:0] imm_out;
    always @(*) begin
        case (opcode)
            // Instruction I-Type (imm[11:0]|rs1|funct3|rd|opcode)
            7'b0010011, 7'b0000011: 
                imm_out = { {20{instr[31]}}, instr[31:20] }; // Sign-extend 12 bits thành 32 bits

            // Instruction S-Type (imm[11:5]|rs2|rs1|funct3|imm[4:0]|opcode)
            7'b0100011: 
                imm_out = { {20{instr[31]}}, instr[31:25], instr[11:7] };

            // Instruction B-Type (imm[12|10:5]|rs2|rs1|funct3|imm[4:1|11]|opcode)
            7'b1100011: 
                imm_out = { {20{instr[31]}}, instr[7], instr[30:25], instr[11:8], 1'b0 };

            // Instruction J-Type (imm[20|10:1|11|19:12]|rd|opcode)
            7'b1101111: 
                imm_out = { {12{instr[31]}}, instr[19:12], instr[20], instr[30:21], 1'b0 };

            // Instruction U-Type (imm[31:12]|rd|opcode)
            7'b0110111, 7'b0010111: 
                imm_out = { instr[31:12], 12'b0 };

            default: imm_out = 32'b0;
        endcase
    end
    
    assign imm = imm_out;

endmodule