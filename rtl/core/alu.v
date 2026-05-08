module alu (
    input  wire [31:0] a,          
    input  wire [31:0] b,          
    input  wire [3:0]  alu_ctrl,   
    output reg  [31:0] result,     
    output wire        zero        
);

    // ALU Control Codes
    localparam ALU_ADD  = 4'b0000;
    localparam ALU_SUB  = 4'b1000;
    localparam ALU_SLL  = 4'b0001; // Shift Left Logical
    localparam ALU_SLT  = 4'b0010; // Set Less Than (Có dấu)
    localparam ALU_SLTU = 4'b0011; // Set Less Than Unsigned (Không dấu)
    localparam ALU_XOR  = 4'b0100;
    localparam ALU_SRL  = 4'b0101; // Shift Right Logical
    localparam ALU_SRA  = 4'b1101; // Shift Right Arithmetic
    localparam ALU_OR   = 4'b0110;
    localparam ALU_AND  = 4'b0111;

    assign zero = (result == 32'd0);

    always @(*) begin
        case (alu_ctrl)
            ALU_ADD:  result = a + b; // Addition
            ALU_SUB:  result = a - b; // Subtraction
            ALU_SLL:  result = a << b[4:0];  // Shift left logical
            ALU_SLT:  result = ($signed(a) < $signed(b)) ? 32'd1 : 32'd0; // Set Less Than (Có dấu)
            ALU_SLTU: result = (a < b) ? 32'd1 : 32'd0;                   // Set Less Than Unsigned (Không dấu)
            ALU_XOR:  result = a ^ b; // Exclusive OR
            ALU_SRL:  result = a >> b[4:0]; // Shift Right Logical
            ALU_SRA:  result = $signed(a) >>> b[4:0]; // Shift Right Arithmetic
            ALU_OR:   result = a | b; // OR
            ALU_AND:  result = a & b; // AND
            default:  result = 32'd0; 
        endcase
    end

endmodule