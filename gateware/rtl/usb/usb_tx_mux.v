// usb_tx_mux.v — shares ulpi_link's transmit port between packet sources.
// Priority (at packet boundaries only): 0 = chirp, 1 = SOF, 2 = transactions.
// Once a source's first byte (TXCMD) is taken, it owns the port until tx_done / tx_abort.
module usb_tx_mux (
    input  wire       clk,
    input  wire       rst,
    input  wire [2:0] s_valid,
    input  wire [23:0] s_data,        // {src2, src1, src0}
    input  wire [2:0] s_last,
    output wire [2:0] s_ack,
    output wire [2:0] s_done,
    output wire [2:0] s_abort,

    output wire       tx_valid,
    output wire [7:0] tx_data,
    output wire       tx_last,
    input  wire       tx_ack,
    input  wire       tx_done,
    input  wire       tx_abort
);
    reg  [1:0] owner = 2'd3;          // 3 = none
    wire [1:0] pick = s_valid[0] ? 2'd0 : s_valid[1] ? 2'd1 : s_valid[2] ? 2'd2 : 2'd3;
    wire [1:0] sel  = (owner != 2'd3) ? owner : pick;
    wire       none = (sel == 2'd3);

    assign tx_valid = !none && s_valid[sel];
    assign tx_data  = none ? 8'h00 : s_data[sel*8 +: 8];
    assign tx_last  = !none && s_last[sel];
    assign s_ack    = none ? 3'b000 : ({2'b00, tx_ack} << sel);
    assign s_done   = (owner == 2'd3) ? 3'b000 : ({2'b00, tx_done} << owner);
    assign s_abort  = (owner == 2'd3) ? 3'b000 : ({2'b00, tx_abort} << owner);

    always @(posedge clk) begin
        if (rst) owner <= 2'd3;
        else if (tx_done || tx_abort) owner <= 2'd3;
        else if (tx_ack && owner == 2'd3) owner <= pick;
    end
endmodule
