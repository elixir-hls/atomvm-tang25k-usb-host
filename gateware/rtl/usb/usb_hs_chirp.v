// usb_hs_chirp.v — host side of the USB 2.0 high-speed detection handshake (§7.1.7.5).
//
// While the host drives reset (SE0) a high-speed device answers with chirp K (1-7 ms). The
// host must start its own K-J chirp sequence within 100 us after the device's chirp ends,
// and keep alternating K/J (each 40-60 us) until 100-500 us before the end of reset.
// Polling from software is too coarse for the 100 us window (measured on the original
// iCESugar design), so the firing is done here.
//
//   arm      : level from software. While 1, a device chirp K that lasted >= K_MIN cycles and
//              then ended fires the host chirp. Software drops it ~400 us before the end of
//              reset; the chirp then stops after the current K-J pair.
//   linestate: LineState of the last RX CMD (ulpi_link rxcmd[1:0]); K = 2'b10.
//   seg_len  : length of one K or J phase in bytes = ULPI clocks (PHY holds NXT during the
//              chirp), e.g. 3000 = 50 us.
//   min_pairs: at least this many K-J pairs are sent even if arm drops earlier (>= 3).
//
// The chirp is a TX packet on ulpi_link's transmit port: TXCMD 0x40 (NOPID), then 0x00 (K)
// or 0xFF (J) bytes, ended by STP. The PHY must be in chirp mode (FuncCtrl OpMode = 10,
// written by software before reset), where data bits go to the line without bit stuffing /
// NRZI: 0x00 = K, 0xFF = J (measured on the USB3300 in the original project).
//
// The "K lasted >= K_MIN" filter: RX CMD LineState returns a wrong value about once in
// 1449 samples (measured), so a single K is not trusted. Default 31 us.
module usb_hs_chirp #(
    parameter K_MIN = 1860               // 31 us at 60 MHz
) (
    input  wire        clk,
    input  wire        rst,
    input  wire        arm,
    input  wire [1:0]  linestate,
    input  wire [15:0] seg_len,
    input  wire [2:0]  min_pairs,

    // TX source (ulpi_link transmit port, through the TX mux)
    output wire        tx_valid,
    output wire [7:0]  tx_data,
    output wire        tx_last,
    input  wire        tx_ack,
    input  wire        tx_done,
    input  wire        tx_abort,

    output reg  [7:0]  fires = 8'd0,     // host chirps started (rolling)
    output reg         k_seen = 1'b0,    // a device chirp K passed the filter (cleared by arm 0->1)
    output reg         aborted = 1'b0,   // sticky: the PHY took the bus during a chirp
    output wire        busy
);
    localparam S_IDLE = 2'd0, S_CMD = 2'd1, S_KJ = 2'd2, S_WAIT = 2'd3;
    reg [1:0]  st = S_IDLE;
    reg [15:0] k_run = 16'd0;            // cycles the line has been K
    reg [15:0] seg_cnt = 16'd0;          // bytes sent in the current phase
    reg        phase_j = 1'b0;           // 0 = K phase, 1 = J phase
    reg [2:0]  pairs = 3'd0;             // completed K-J pairs (saturating)
    reg        arm_d = 1'b0;

    wire is_k     = (linestate == 2'b10);
    wire seg_end  = (seg_cnt == seg_len - 16'd1);
    // the last byte: end of a J phase, enough pairs, and software has dropped arm
    // (4-bit sum: pairs saturates at 7, and 7 + 1 must not wrap to 0 — it did on hardware,
    //  where a 15 ms reset gives ~150 pairs, and the chirp never ended)
    wire last_pair = ({1'b0, pairs} + 4'd1 >= {1'b0, min_pairs}) && !arm;
    assign tx_valid = (st == S_CMD) || (st == S_KJ);
    assign tx_data  = (st == S_CMD) ? 8'h40 : (phase_j ? 8'hFF : 8'h00);
    assign tx_last  = (st == S_KJ) && phase_j && seg_end && last_pair;
    assign busy     = (st != S_IDLE);

    always @(posedge clk) begin
        arm_d <= arm;
        if (rst) begin
            st <= S_IDLE;
            k_run <= 16'd0;
            k_seen <= 1'b0;
        end else begin
            if (arm && !arm_d) begin
                k_seen  <= 1'b0;
                aborted <= 1'b0;
            end
            case (st)
            S_IDLE: begin
                if (is_k) begin
                    if (k_run != 16'hFFFF) k_run <= k_run + 16'd1;
                    if (k_run >= K_MIN) k_seen <= 1'b1;
                end else begin
                    k_run <= 16'd0;
                    // device chirp K ended (line left K after a long K): fire
                    if (arm && k_run >= K_MIN) begin
                        st      <= S_CMD;
                        fires   <= fires + 8'd1;
                        seg_cnt <= 16'd0;
                        phase_j <= 1'b0;
                        pairs   <= 3'd0;
                    end
                end
            end
            S_CMD: if (tx_ack) st <= S_KJ;            // TXCMD taken, data bytes follow
            S_KJ: begin
                if (tx_abort) begin
                    aborted <= 1'b1;
                    st      <= S_IDLE;
                end else if (tx_ack) begin
                    if (seg_end) begin
                        seg_cnt <= 16'd0;
                        phase_j <= ~phase_j;
                        if (phase_j) begin
                            if (pairs != 3'd7) pairs <= pairs + 3'd1;
                            if (last_pair) st <= S_WAIT;
                        end
                    end else begin
                        seg_cnt <= seg_cnt + 16'd1;
                    end
                end
            end
            S_WAIT: if (tx_done || tx_abort) st <= S_IDLE;   // STP sent
            endcase
        end
    end
endmodule
