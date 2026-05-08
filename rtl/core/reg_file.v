module reg_file (
    input wire clk,                 
    input wire we,                 // Write Enable

    input wire [4:0] rs1_addr,     // Địa chỉ thanh ghi nguồn 1
    input wire [4:0] rs2_addr,     // Địa chỉ thanh ghi nguồn 2
    input wire [4:0] rd_addr,      // Địa chỉ thanh ghi đích

    input wire [31:0] rd_data,     // Dữ liệu cần ghi vào thanh ghi đích    
    output wire [31:0] rs1_data,   // Dữ liệu đọc từ thanh ghi nguồn 1
    output wire [31:0] rs2_data    // Dữ liệu đọc từ thanh ghi nguồn 2
);
    reg [31:0] reg_array [31:0]; // 32 thanh ghi, mỗi thanh ghi 32 bit

    // Đọc dữ liệu từ thanh ghi nguồn
    assign rs1_data = (rs1_addr != 5'b00000) ? reg_array[rs1_addr] : 32'b0; // Thanh ghi x0 luôn trả về 0
    assign rs2_data = (rs2_addr != 5'b00000) ? reg_array[rs2_addr] : 32'b0; // Thanh ghi x0 luôn trả về 0

    // Ghi dữ liệu vào thanh ghi đích
    always @(posedge clk) begin
        if (we && rd_addr != 5'b00000) begin // Chỉ ghi khi Write Enable được kích hoạt và không phải thanh ghi x0
            reg_array[rd_addr] <= rd_data;
        end
    end

    integer i;
    initial begin
        for (i = 0; i < 32; i = i + 1) begin
            reg_array[i] = 32'b0; // Khởi tạo tất cả thanh ghi về 0
        end
    end
endmodule