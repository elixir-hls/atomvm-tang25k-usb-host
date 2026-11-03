// usb_bufs.v — packet buffers between the CPU (sys clock, 32-bit words) and the USB
// transaction engine (ULPI clock, bytes). 512 bytes each way, little-endian byte order
// (byte n = word n/4, bits 8*(n%4) +: 8).
//   TX: CPU writes, engine reads.   RX: engine writes, CPU reads.
// Four byte lanes per buffer so the two sides can use different widths. Synchronous reads
// on both sides (1 cycle).
module usb_bufs (
    // sys side
    input  wire        sys_clk,
    input  wire [3:0]  tx_we,          // byte enables (any: the whole word)
    input  wire [6:0]  tx_waddr,       // word address
    input  wire [31:0] tx_wdata,
    input  wire [6:0]  rx_raddr,
    output wire [31:0] rx_rdata,

    // ULPI side
    input  wire        ulpi_clk,
    input  wire [8:0]  tx_raddr,       // byte address
    output wire [7:0]  tx_rbyte,
    input  wire        rx_we,
    input  wire [8:0]  rx_waddr,       // byte address
    input  wire [7:0]  rx_wbyte
);
    // TX: one 32-bit wide block RAM, written a whole word at a time (the CPU only writes words:
    // usb_xfer.c); a block RAM per byte lane took 3 more of the 56
    reg [31:0] tx [0:127];
    reg [7:0] rx0 [0:127], rx1 [0:127], rx2 [0:127], rx3 [0:127];

    // TX: sys writes
    always @(posedge sys_clk) begin
        if (|tx_we) tx[tx_waddr] <= tx_wdata;
    end
    // TX: ULPI reads
    reg [31:0] t = 32'h0;
    reg [1:0] tlane = 2'd0;
    always @(posedge ulpi_clk) begin
        t <= tx[tx_raddr[8:2]];
        tlane <= tx_raddr[1:0];
    end
    assign tx_rbyte = (tlane == 2'd0) ? t[7:0] : (tlane == 2'd1) ? t[15:8] : (tlane == 2'd2) ? t[23:16] : t[31:24];

    // RX: ULPI writes
    always @(posedge ulpi_clk) begin
        if (rx_we && rx_waddr[1:0] == 2'd0) rx0[rx_waddr[8:2]] <= rx_wbyte;
        if (rx_we && rx_waddr[1:0] == 2'd1) rx1[rx_waddr[8:2]] <= rx_wbyte;
        if (rx_we && rx_waddr[1:0] == 2'd2) rx2[rx_waddr[8:2]] <= rx_wbyte;
        if (rx_we && rx_waddr[1:0] == 2'd3) rx3[rx_waddr[8:2]] <= rx_wbyte;
    end
    // RX: sys reads
    reg [31:0] r = 32'h0;
    always @(posedge sys_clk) r <= {rx3[rx_raddr], rx2[rx_raddr], rx1[rx_raddr], rx0[rx_raddr]};
    assign rx_rdata = r;
endmodule
