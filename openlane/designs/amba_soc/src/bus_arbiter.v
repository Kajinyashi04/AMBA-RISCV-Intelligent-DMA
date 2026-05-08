module bus_arbiter (
    // Clock and Reset
    input wire clk,
    input wire rst_n,

    //MASTER 0: CPU
    input  wire [31:0] m0_addr,
    input  wire        m0_valid,
    input  wire        m0_we,      
    input  wire [31:0] m0_wdata,
    output wire [31:0] m0_rdata,
    output wire        m0_ready,   

    //MASTER 1: DMA
    input  wire [31:0] m1_addr,
    input  wire        m1_valid,
    input  wire        m1_we,
    input  wire [31:0] m1_wdata,
    output wire [31:0] m1_rdata,
    output wire        m1_ready,

    //SLAVE 0: RAM
    output wire [31:0] s0_addr,
    output wire        s0_valid,
    output wire        s0_we,
    output wire [31:0] s0_wdata,
    input  wire [31:0] s0_rdata,
    input  wire        s0_ready,

    //SLAVE 1: DMA SLAVE
    output wire [31:0] s1_addr,
    output wire        s1_valid,
    output wire        s1_we,
    output wire [31:0] s1_wdata,
    input  wire [31:0] s1_rdata,
    input  wire        s1_ready
);

    // 1. ARBITRATION
    // Rules:
    // If DMA (M1) requests -> Grant M1 immediately (Highest Priority)
    // Else if CPU (M0) requests -> Grant M0 (Only if M1 is not requesting)
    wire grant_m1 = m1_valid;                 
    wire grant_m0 = m0_valid && !m1_valid;    

    // 2. MULTIPLEXER (choose which master gets to use the bus)
    wire [31:0] bus_addr   = grant_m1 ? m1_addr   : (grant_m0 ? m0_addr   : 32'h0);
    wire        bus_we     = grant_m1 ? m1_we     : (grant_m0 ? m0_we     : 1'b0);
    wire [31:0] bus_wdata  = grant_m1 ? m1_wdata  : (grant_m0 ? m0_wdata  : 32'h0);
    wire        bus_valid  = grant_m1 | grant_m0; // Enable bus if any master is requesting

    // 3. ADDRESS DECODING 
    // If address >= 0x4000_0000 -> select Slave 1 (DMA Config)
    // Else -> select Slave 0 (RAM)
    wire sel_s1 = (bus_addr >= 32'h4000_0000); // S1 (DMA Config)
    wire sel_s0 = !sel_s1;                     // S0 (RAM)

    // Send address and data to both slaves (but only the selected slave will respond)
    assign s0_valid = bus_valid && sel_s0;
    assign s1_valid = bus_valid && sel_s1;

    // Send address and data to both slaves (but only the selected slave will respond)
    assign s0_addr  = bus_addr;
    assign s0_we    = bus_we;
    assign s0_wdata = bus_wdata;

    assign s1_addr  = bus_addr;
    assign s1_we    = bus_we;
    assign s1_wdata = bus_wdata;


    // RETURN PATH 
    // Take data and ready from the selected slave and route it back to the correct master
    wire [31:0] bus_rdata = sel_s1 ? s1_rdata : s0_rdata;
    wire        bus_ready = sel_s1 ? s1_ready : s0_ready;

    // Return data to the correct master
    assign m1_rdata = bus_rdata;
    assign m0_rdata = bus_rdata;

    // Return ready to the correct master
    assign m1_ready = bus_ready && grant_m1;
    assign m0_ready = bus_ready && grant_m0;

endmodule