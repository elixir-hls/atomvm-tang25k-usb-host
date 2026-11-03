// usb_bulk_in.v — continuous bulk IN (the RTL-SDR's IQ endpoint) in hardware (ULPI clock).
//
// While `en`, it keeps asking the transaction arbiter for IN transactions on addr/endp. Each
// answer (from usb_xact, which already ACKed a good packet) is sorted here:
//   DATAx with the expected toggle  -> packet committed: its bank is handed to the reader,
//                                      toggle flips, the other bank takes the next packet
//   DATAx with the other toggle     -> a repeat of a packet we already have (our ACK was
//                                      lost): dropped, counted in `dups`
//   NAK                             -> counted, asked again
//   anything else (timeout, CRC...) -> counted in `errs`, asked again (no ACK was sent)
// Payload bytes arrive on the RX write port (the arbiter routes them here only for this
// engine's transactions) into one of two 512-byte banks. A new IN is asked for only when the
// bank to fill is free, so a full output FIFO stops the requests: the device then holds the
// data (it NAKs) instead of us dropping it.
//
// Output: a byte stream (valid/ready), packets back to back in order.
// `init_toggle` is loaded on en 0->1 (DATA0 after SET_CONFIGURATION).
// `nak_gap`: after a NAK, wait this many clocks before asking again (0 = at once). Fewer
// IN/NAK pairs on the bus while the device has no data (RF noise experiments).
module usb_bulk_in (
    input  wire        clk,
    input  wire        rst,
    input  wire        en,
    input  wire        init_toggle,
    input  wire [6:0]  addr,
    input  wire [3:0]  endp,
    input  wire [15:0] nak_gap,

    // to the transaction arbiter
    output wire        xreq,
    input  wire        xgo,          // our transaction has started
    input  wire        xdone,
    input  wire [7:0]  xpid,
    input  wire [5:0]  xflags,
    input  wire [9:0]  xlen,
    input  wire        rx_we,
    input  wire [8:0]  rx_waddr,
    input  wire [7:0]  rx_wbyte,

    // byte stream out
    output reg         m_valid = 1'b0,
    output reg  [7:0]  m_data = 8'h00,
    input  wire        m_ready,

    output reg  [31:0] pkts = 32'd0,     // committed packets
    output reg  [31:0] naks = 32'd0,
    output reg  [15:0] errs = 16'd0,
    output reg  [15:0] dups = 16'd0,
    output reg         toggle = 1'b0
);
    reg [7:0] bank [0:1023];             // {bank, byte}
    reg       wbank = 1'b0;              // bank being filled
    reg [1:0] full = 2'b00;
    reg [9:0] blen0 = 10'd0, blen1 = 10'd0;
    reg       inflight = 1'b0;
    reg       en_d = 1'b0;

    reg [15:0] hold = 16'd0;
    assign xreq = en && !inflight && !full[wbank] && hold == 16'd0;

    always @(posedge clk) begin
        if (rx_we) bank[{wbank, rx_waddr}] <= rx_wbyte;
    end

    // ---- reader: 1 byte per 2 clocks (far above the 480 kB/s needed) ----
    reg       rbank = 1'b0;
    reg [9:0] ridx = 10'd0;
    reg       rd_pend = 1'b0;
    wire [9:0] rlen = rbank ? blen1 : blen0;
    reg [7:0] rq = 8'h00;
    always @(posedge clk) rq <= bank[{rbank, ridx[8:0]}];

    wire commit = xdone && inflight && xflags == 6'd0 &&
                  (xpid == 8'hC3 || xpid == 8'h4B) && ((xpid == 8'h4B) == toggle);
    wire release_bank = full[rbank] && !rd_pend && !m_valid && ridx == rlen;

    always @(posedge clk) begin
        en_d <= en;
        if (rst) begin
            inflight <= 1'b0; full <= 2'b00; wbank <= 1'b0; rbank <= 1'b0;
            ridx <= 10'd0; rd_pend <= 1'b0; m_valid <= 1'b0;
        end else begin
            if (en && !en_d) toggle <= init_toggle;
            if (hold != 16'd0) hold <= hold - 16'd1;
            if (xgo) inflight <= 1'b1;
            if (xdone && inflight) begin
                inflight <= 1'b0;
                if (commit) begin
                    pkts <= pkts + 32'd1;
                    toggle <= ~toggle;
                    if (xlen != 10'd0) begin
                        if (wbank) blen1 <= xlen; else blen0 <= xlen;
                        wbank <= ~wbank;
                    end
                end else if (xflags == 6'd0 && (xpid == 8'hC3 || xpid == 8'h4B)) begin
                    dups <= dups + 16'd1;
                end else if (xflags == 6'd0 && xpid == 8'h5A) begin
                    naks <= naks + 32'd1;
                    hold <= nak_gap;
                end else begin
                    errs <= errs + 16'd1;
                end
            end
            // reader
            if (m_valid && m_ready) m_valid <= 1'b0;
            if (rd_pend) begin
                m_data  <= rq;
                m_valid <= 1'b1;
                rd_pend <= 1'b0;
                ridx    <= ridx + 10'd1;
            end else if (full[rbank] && ridx != rlen && (!m_valid || m_ready)) begin
                rd_pend <= 1'b1;               // rq shows bank[ridx] next cycle
            end
            // full flags: set by a commit, cleared when the reader is done
            full <= (full | ((commit && xlen != 10'd0) ? (2'b01 << wbank) : 2'b00))
                    & ~(release_bank ? (2'b01 << rbank) : 2'b00);
            if (release_bank) begin
                rbank <= ~rbank;
                ridx  <= 10'd0;
            end
        end
    end
endmodule
