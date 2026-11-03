// ulpi_link.v — ULPI link layer for the USB3300 (ULPI_CLK = 60 MHz from the PHY's CLKOUT).
//
// Everything here runs on the ULPI clock. All bus timing is relative to the rising edge
// (USB3300 DS Table 6.2: PHY input setup 5.0 ns / hold 0 ns, PHY output delay 2.0-5.0 ns).
//
// Pad timing (checked by STA with set_input_delay / set_output_delay, see gateware/usbh.py):
//   * DATA in, DIR, NXT are registered once at the pad (data_q / dir_q / nxt_q). These three
//     describe the same bus cycle and are what the RX side and the register-read use.
//   * DATA out and STP are registered (data_o / stp_o) — IOB output registers.
//   * The TX byte advance uses the RAW nxt_i: when the PHY shows NXT at an edge, the byte on
//     the bus during that cycle is consumed and the next byte must be in data_o right after
//     that edge (ULPI 1.1 §3.8.2), so data_o's D input depends combinationally on nxt_i.
//   * Output enable: drive_en & ~dir_i & ~dir_q. Raw DIR releases the bus in the same cycle
//     the PHY takes it; dir_q keeps the turnaround cycle undriven when the PHY gives it back
//     (finding carried over from usb-rtl-sdr rtl/ulpi_pads.v "F3").
//   No negedge sampling (the original iCE40 design needed it; here the STA decides).
//
// Ports to the rest of the USB host (same clock):
//   * Register port: reg_req (pulse) with reg_we/reg_addr/reg_wdata; reg_done (pulse) with
//     reg_rdata. A register access interrupted by the PHY (DIR asserted) is retried.
//   * TX packet port: tx_valid + tx_data (first byte = TXCMD 0x40|PID, then payload) with
//     tx_last on the final byte. tx_ack is combinational: high in a cycle whose rising edge
//     takes tx_data into data_o, so the source (first-word-fall-through) advances on that
//     same edge. tx_done / tx_abort pulse at
//     the end (abort = PHY took the bus before the packet was accepted).
//   * RX: rxcmd (last RX CMD byte) + rxcmd_valid pulse; rx_valid/rx_data per received USB byte;
//     rx_end pulses when a received packet ends, rx_err is set if an RxError was seen in it.
module ulpi_link #(
    parameter POST_STP = 18         // S_STP cycles: next TXCMD on the bus POST_STP+1 clocks after STP
) (
    input  wire       clk,
    input  wire       rst,

    // pads (I/O buffers are outside; data_oe drives their output enable)
    input  wire       dir_i,
    input  wire       nxt_i,
    input  wire [7:0] data_i,
    output reg  [7:0] data_o = 8'h00,
    output wire       data_oe,
    output reg        stp_o = 1'b0,

    // register access
    input  wire       reg_req,
    input  wire       reg_we,
    input  wire [5:0] reg_addr,
    input  wire [7:0] reg_wdata,
    output reg        reg_done = 1'b0,
    output reg  [7:0] reg_rdata = 8'h00,

    // packet transmit
    input  wire       tx_valid,
    input  wire [7:0] tx_data,
    input  wire       tx_last,
    output wire       tx_ack,
    output reg        tx_done = 1'b0,
    output reg        tx_abort = 1'b0,

    // receive
    output reg  [7:0] rxcmd = 8'h00,
    output reg        rxcmd_valid = 1'b0,
    output reg        rx_active = 1'b0,
    output reg  [7:0] rx_data = 8'h00,
    output reg        rx_valid = 1'b0,
    output reg        rx_end = 1'b0,
    output reg        rx_err = 1'b0,

    output wire       busy,
    // debug: {st[2:0], reg_pend, drive_en, dir_q, nxt_q, rx_active, wr_nxt2, rx_pkts[6:0]}
    //   wr_nxt2 = NXT was high in the data cycle of the last register write
    //   rx_pkts = received USB packets (rx_active rises), rolling
    output wire [15:0] dbg
);
    // ---- pad input registers (same bus cycle) ----
    reg [7:0] data_q = 8'h00;
    reg       dir_q = 1'b1, nxt_q = 1'b0, dir_qq = 1'b1;
    always @(posedge clk) begin
        data_q <= data_i;
        dir_q  <= dir_i;
        nxt_q  <= nxt_i;
        dir_qq <= dir_q;
    end

    // ---- transmit side ----
    localparam S_IDLE = 3'd0, S_REG_CMD = 3'd1, S_REG_DATA = 3'd2, S_PKT = 3'd3,
               S_STP = 3'd4, S_RD_WAIT = 3'd5;
    reg [2:0] st = S_IDLE;
    reg       drive_en = 1'b0;
    // pending register access (kept until done, so an aborted access is retried)
    reg       reg_pend = 1'b0, reg_pend_we = 1'b0;
    reg [5:0] reg_pend_addr = 6'd0;
    reg [7:0] reg_pend_wdata = 8'h00;
    reg       pkt_last = 1'b0;      // the byte in data_o is the packet's last
    reg [7:0] pkt_wait = 8'd0;      // S_PKT: clocks without NXT
    reg       rd_turn = 1'b0;       // register read: turnaround seen, data comes next
    reg [4:0] wait_cnt = 5'd0;      // S_STP: post-STP gap; S_RD_WAIT: turnaround timeout

    reg       wr_nxt2 = 1'b0;
    reg [6:0] rx_pkts = 7'd0;
    reg       rx_active_d = 1'b0;
    always @(posedge clk) begin
        rx_active_d <= rx_active;
        if (rx_active && !rx_active_d) rx_pkts <= rx_pkts + 7'd1;
    end
    assign dbg = {st, reg_pend, drive_en, dir_q, nxt_q, rx_active, wr_nxt2, rx_pkts};

    assign data_oe = drive_en & ~dir_i & ~dir_q;
    assign busy = (st != S_IDLE) | reg_pend;

    // the link may start only when it owns the bus and no turnaround is pending
    wire bus_free = ~dir_i & ~dir_q;

    assign tx_ack = rst ? 1'b0 :
                    (st == S_IDLE) ? (bus_free & ~reg_pend & tx_valid) :
                    (st == S_PKT)  ? (~dir_i & nxt_i & ~pkt_last) : 1'b0;

    always @(posedge clk) begin
        reg_done <= 1'b0;
        tx_done  <= 1'b0;
        tx_abort <= 1'b0;
        stp_o    <= 1'b0;
        if (reg_req && !reg_pend) begin
            reg_pend       <= 1'b1;
            reg_pend_we    <= reg_we;
            reg_pend_addr  <= reg_addr;
            reg_pend_wdata <= reg_wdata;
        end
        if (rst) begin
            st       <= S_IDLE;
            drive_en <= 1'b0;
            data_o   <= 8'h00;
            reg_pend <= 1'b0;
        end else case (st)
        S_IDLE: begin
            data_o   <= 8'h00;          // NOOP while the link owns the bus
            drive_en <= 1'b1;
            if (bus_free) begin
                if (reg_pend) begin
                    data_o <= {reg_pend_we ? 2'b10 : 2'b11, reg_pend_addr};
                    st     <= S_REG_CMD;
                end else if (tx_valid) begin
                    data_o   <= tx_data;      // TXCMD
                    pkt_last <= tx_last;
                    pkt_wait <= 8'd0;
                    st       <= S_PKT;
                end
            end
        end
        S_REG_CMD: begin
            if (dir_i) begin
                st <= S_IDLE;                 // PHY took the bus: retry from idle
            end else if (nxt_i) begin
                if (reg_pend_we) begin
                    data_o <= reg_pend_wdata;
                    st     <= S_REG_DATA;
                end else begin
                    // NOOP until the PHY turns the bus around (data_oe drops with the raw DIR).
                    // Never float the bus here: a PHY that did not take the read would decode
                    // a floating bus as a TX command (found on hardware).
                    data_o   <= 8'h00;
                    rd_turn  <= 1'b0;
                    wait_cnt <= 5'd0;
                    st       <= S_RD_WAIT;
                end
            end
        end
        S_REG_DATA: begin
            if (dir_i) begin
                st <= S_IDLE;
            end else if (nxt_i) begin
                wr_nxt2 <= 1'b1;              // (bring-up: the USB3300 accepts the data with NXT)
                data_o <= 8'h00;
                stp_o  <= 1'b1;
                reg_pend <= 1'b0;
                reg_done <= 1'b1;
                wait_cnt <= 5'd0;
                st     <= S_STP;
            end
        end
        S_PKT: begin
            // A TXCMD the PHY never accepts (no NXT, no DIR) used to hold the link here for
            // good, every transaction and SOF with it. After 255 clocks without NXT the
            // packet is given up: STP, tx_abort (the transaction reports an abort and its
            // owner retries). Seen with a PHY knocked out by glitches on its RESET pin: then
            // it answers nothing until a proper reset and the link at least keeps cycling
            // instead of hanging.
            pkt_wait <= nxt_i ? 8'd0 : pkt_wait + 8'd1;
            if (dir_i) begin
                tx_abort <= 1'b1;
                st       <= S_IDLE;
            end else if (!nxt_i && pkt_wait == 8'd255) begin
                data_o   <= 8'h00;
                stp_o    <= 1'b1;
                tx_abort <= 1'b1;
                wait_cnt <= 5'd0;
                st       <= S_STP;
            end else if (nxt_i) begin
                if (pkt_last) begin
                    data_o  <= 8'h00;
                    stp_o   <= 1'b1;
                    tx_done <= 1'b1;
                    wait_cnt <= 5'd0;
                    st      <= S_STP;
                end else begin
                    data_o   <= tx_data;
                    pkt_last <= tx_last;
                end
            end
        end
        S_STP: begin
            // STP is on the bus in the first cycle, NOOP after it. Nothing new starts for
            // POST_STP cycles, for two reasons (both found on hardware):
            //  * the PHY can still show the last byte's NXT right after STP; a TXCMD started
            //    there takes that stale NXT as its accept (the original waits too,
            //    ulpi_tx_link.v done_ph);
            //  * HS host Transmit-Transmit (ULPI 1.1 Table 10): the TXCMD of a second packet
            //    (DATA after a SETUP/OUT token) goes out 15..24 clocks after STP of the first
            //    = the 88..192 bit-time inter-packet gap on the wire (USB 2.0 §7.1.18.2). With
            //    a 5-clock gap the device never answered the SETUP. POST_STP = 18 puts the
            //    TXCMD at 19 clocks, mid-window (the original used ~17: ulpi_reg_link.v
            //    TT_START_CLK).
            // The gap is not cut short by DIR: an RX CMD in it just delays S_IDLE's start.
            wait_cnt <= wait_cnt + 5'd1;
            if (wait_cnt == POST_STP - 1) st <= S_IDLE;
        end
        S_RD_WAIT: begin
            // registered view: dir_q rising = turnaround cycle (nxt_q set there means a USB
            // receive took the bus instead: the read was aborted and is retried later)
            if (!rd_turn && !dir_i && !dir_q) begin
                wait_cnt <= wait_cnt + 5'd1;
                if (wait_cnt == 5'd15) st <= S_IDLE;   // never turned around: retry
            end
            if (dir_q && !dir_qq) begin
                if (nxt_q) st <= S_IDLE;
                else       rd_turn <= 1'b1;
            end else if (rd_turn && dir_q && dir_qq) begin
                reg_rdata <= data_q;
                reg_pend  <= 1'b0;
                reg_done  <= 1'b1;
                st        <= S_IDLE;
            end else if (!dir_q && !dir_i && rd_turn) begin
                st <= S_IDLE;                 // bus returned without data: retry
            end
        end
        default: st <= S_IDLE;
        endcase
    end

    // ---- receive side (registered view) ----
    // dir_q && dir_qq: PHY drives a byte this cycle; nxt_q = USB data, else RX CMD.
    // The single byte returned by a register read is not an RX CMD (excluded by rd_turn).
    wire phy_byte = dir_q & dir_qq;
    wire rd_slot  = (st == S_RD_WAIT) & rd_turn;
    always @(posedge clk) begin
        rxcmd_valid <= 1'b0;
        rx_valid    <= 1'b0;
        rx_end      <= 1'b0;
        if (rst) begin
            rx_active <= 1'b0;
            rx_err    <= 1'b0;
        end else begin
            if (dir_q && !dir_qq && nxt_q) begin
                rx_active <= 1'b1;            // turnaround with NXT: RxActive
                rx_err    <= 1'b0;
            end
            if (phy_byte && !rd_slot) begin
                if (nxt_q) begin
                    rx_data  <= data_q;
                    rx_valid <= 1'b1;
                end else begin
                    rxcmd       <= data_q;
                    rxcmd_valid <= 1'b1;
                    if (data_q[5:4] == 2'b11) rx_err <= 1'b1;           // RxError
                    if (rx_active && data_q[4] == 1'b0 && data_q[5:4] != 2'b11) begin
                        rx_active <= 1'b0;                              // RxActive dropped
                        rx_end    <= 1'b1;
                    end else if (!rx_active && data_q[4]) begin
                        rx_active <= 1'b1;
                        rx_err    <= 1'b0;
                    end
                end
            end
            if (rx_active && !dir_q && dir_qq) begin
                rx_active <= 1'b0;            // PHY released the bus: packet over
                rx_end    <= 1'b1;
            end
        end
    end
endmodule
