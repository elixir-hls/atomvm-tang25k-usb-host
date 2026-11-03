// usb_xact.v — runs one USB high-speed transaction (ULPI clock).
//
//   kind 0 SETUP : token SETUP, DATA0 + payload (len bytes from the TX buffer), wait handshake
//   kind 1 OUT   : token OUT,  DATA0/1 + payload,                                wait handshake
//   kind 2 IN    : token IN, receive DATAx + payload into the RX buffer; CRC16 good -> ACK
//   kind 3 PING  : token PING, wait handshake
//
// Split transactions (a full / low-speed device behind a high-speed hub, USB 2.0 §8.4.2.2,
// §11.17-11.21): with split[0] set a SPLIT token goes out first:
//   [TXCMD 0x48] [SC, hub addr] [S, port] [crc5, ET, E]       (CRC5 over the 19 field bits)
//   split = {nohs[22], et[21:20], e[19], s[18], port[17:11], hub[10:4], sc[1], en[0]}
//   start-split  (sc 0): SETUP/OUT send their data, IN sends only the token; the hub answers
//                         ACK / NAK (nohs = 1, isochronous OUT: no handshake, done after the data)
//   complete-split (sc 1): SETUP/OUT send no data; IN takes the data packet without an ACK
//                         (the hub already ACKed the device); handshakes NYET / NAK / STALL / ERR
//
// Software decides the sequence (control transfer stages, retries on NAK, data toggles);
// everything with a deadline is here: CRC5/CRC16, the handshake within the bus turnaround,
// the response timeout, and keeping the whole transaction inside one microframe (it starts
// only between GUARD_LO and GUARD_HI ULPI clocks after the SOF, so it never collides with
// the next SOF in the TX mux).
//
// Packets on ULPI: TXCMD = 0x40 | PID[3:0] (the PHY sends the PID byte), then the bytes.
//   token: {endp[0], addr}, {crc5, endp[3:1]}        data: payload, crc16 lo, crc16 hi
// CRC16: poly 0x8005 reflected (0xA001), init 0xFFFF, sent inverted, low byte first.
// RX check: running the same CRC over payload + received CRC leaves 0xB001.
//
// Result (valid at done): pid = received PID (handshake or DATAx; 0 on timeout),
// rx_len = payload bytes received (IN), flags {pid_err, overflow, rx_err, crc_err, abort, timeout}.
module usb_xact #(
    parameter TIMEOUT  = 400,     // ULPI clocks from the end of our packet to the start of RX
    parameter GUARD_LO = 64,      // earliest start after the SOF (SOF is on the wire)
    parameter GUARD_HI = 6000     // latest start (7500 = 125 us microframe; 512 B IN + margin)
) (
    input  wire        clk,
    input  wire        rst,

    input  wire        start,
    input  wire [1:0]  kind,
    input  wire [6:0]  addr,
    input  wire [3:0]  endp,
    input  wire        data1,       // OUT/SETUP: send DATA1 (else DATA0)
    input  wire [9:0]  len,         // OUT/SETUP payload bytes (0..512)
    input  wire [22:0] split,       // see above; split[0] = 0: no SPLIT token

    input  wire        sof_en,
    input  wire [15:0] uf_timer,

    // TX source
    output wire        tx_valid,
    output wire [7:0]  tx_data,
    output wire        tx_last,
    input  wire        tx_ack,
    input  wire        tx_done,
    input  wire        tx_abort,

    // RX (from ulpi_link)
    input  wire        rx_active,
    input  wire        rx_valid,
    input  wire [7:0]  rx_data,
    input  wire        rx_end,
    input  wire        rx_err,

    // buffers
    output wire [8:0]  buf_tx_raddr,
    input  wire [7:0]  buf_tx_rbyte,
    output reg         buf_rx_we = 1'b0,
    output reg  [8:0]  buf_rx_waddr = 9'd0,
    output reg  [7:0]  buf_rx_wbyte = 8'h00,

    output wire        busy,
    output reg         done = 1'b0,
    output reg  [7:0]  pid = 8'h00,
    output reg  [9:0]  rx_len = 10'd0,
    output reg  [5:0]  flags = 6'd0,
    output reg  [15:0] count = 16'd0      // transactions completed (rolling)
);
    localparam K_SETUP = 2'd0, K_OUT = 2'd1, K_IN = 2'd2, K_PING = 2'd3;
    localparam S_IDLE = 3'd0, S_WAITSLOT = 3'd1, S_TOK = 3'd2, S_DATA = 3'd3,
               S_WAITRX = 3'd4, S_ACK = 3'd5, S_FIN = 3'd6, S_SPLIT = 3'd7;
    localparam F_TIMEOUT = 0, F_ABORT = 1, F_CRC = 2, F_RXERR = 3, F_OVF = 4, F_PIDERR = 5;

    function [4:0] crc5(input [10:0] d);
        integer i; reg [4:0] c; reg fb;
        begin
            c = 5'b11111;
            for (i = 0; i < 11; i = i + 1) begin
                fb = c[4] ^ d[i]; c = {c[3:0], 1'b0};
                if (fb) c = c ^ 5'b00101;
            end
            crc5 = ~{c[0], c[1], c[2], c[3], c[4]};
        end
    endfunction
    function [4:0] crc5_19(input [18:0] d);
        integer i; reg [4:0] c; reg fb;
        begin
            c = 5'b11111;
            for (i = 0; i < 19; i = i + 1) begin
                fb = c[4] ^ d[i]; c = {c[3:0], 1'b0};
                if (fb) c = c ^ 5'b00101;
            end
            crc5_19 = ~{c[0], c[1], c[2], c[3], c[4]};
        end
    endfunction
    function [15:0] crc16_byte(input [15:0] c, input [7:0] b);
        integer i; reg [15:0] r;
        begin
            r = c ^ {8'h00, b};
            for (i = 0; i < 8; i = i + 1) r = r[0] ? ((r >> 1) ^ 16'hA001) : (r >> 1);
            crc16_byte = r;
        end
    endfunction

    reg [2:0]  st = S_IDLE;
    reg [1:0]  k = 2'd0;
    reg [6:0]  a = 7'd0;
    reg [3:0]  e = 4'd0;
    reg        d1 = 1'b0;
    reg [9:0]  n = 10'd0;          // OUT payload length
    reg [22:0] sp = 23'd0;         // split fields of this transaction
    reg [1:0]  tidx = 2'd0;        // token byte index
    reg [9:0]  didx = 10'd0;       // data packet byte index (0 = TXCMD)
    reg [15:0] crc = 16'hFFFF;
    reg        sent_last = 1'b0;   // the packet's last byte has been taken
    reg [11:0] tmo = 12'd0;
    reg        rx_seen = 1'b0;
    reg [10:0] rx_n = 11'd0;       // bytes received incl. PID
    reg [15:0] rx_crc = 16'hFFFF;

    wire [7:0] tok_pid = (k == K_SETUP) ? 8'h2D : (k == K_OUT) ? 8'hE1 :
                         (k == K_IN)    ? 8'h69 : 8'hB4;
    wire [7:0] dat_pid = d1 ? 8'h4B : 8'hC3;
    wire [4:0] tcrc = crc5({e, a});
    wire       sp_en = sp[0], sp_sc = sp[1], sp_nohs = sp[22];
    wire [18:0] sp_bits = {sp[21:20], sp[19], sp[18], sp[17:11], sp_sc, sp[10:4]};
    wire [4:0] scrc = crc5_19(sp_bits);
    wire in_split = (st == S_SPLIT) && !sent_last;

    wire in_tok  = (st == S_TOK)  && !sent_last;
    wire in_data = (st == S_DATA) && !sent_last;
    wire in_ack  = (st == S_ACK)  && !sent_last;
    assign tx_valid = in_split | in_tok | in_data | in_ack;
    assign tx_data  = in_split ? ((tidx == 2'd0) ? 8'h48 :
                                  (tidx == 2'd1) ? sp_bits[7:0] :
                                  (tidx == 2'd2) ? sp_bits[15:8] : {scrc, sp_bits[18:16]}) :
                      in_tok  ? ((tidx == 2'd0) ? {4'h4, tok_pid[3:0]} :
                                 (tidx == 2'd1) ? {e[0], a} : {tcrc, e[3:1]}) :
                      in_data ? ((didx == 10'd0) ? {4'h4, dat_pid[3:0]} :
                                 (didx <= n) ? buf_tx_rbyte :
                                 (didx == n + 10'd1) ? ~crc[7:0] : ~crc[15:8]) :
                      in_ack  ? 8'h42 : 8'h00;
    assign tx_last  = in_split ? (tidx == 2'd3) : in_tok ? (tidx == 2'd2) : in_data ? (didx == n + 10'd2) : in_ack;
    // payload byte shown in the next cycle (synchronous buffer read, look-ahead on tx_ack)
    wire [9:0] didx_next = (in_data && tx_ack) ? didx + 10'd1 : didx;
    assign buf_tx_raddr = didx_next[8:0] - 9'd1;
    assign busy = (st != S_IDLE);

    wire slot_ok = !sof_en || (uf_timer >= GUARD_LO && uf_timer < GUARD_HI);
    wire pid_ok  = (rx_n != 11'd0) && (pid[7:4] == ~pid[3:0]);
    wire is_data = (pid == 8'hC3) || (pid == 8'h4B) || (pid == 8'h87) || (pid == 8'h0F);

    always @(posedge clk) begin
        done <= 1'b0;
        buf_rx_we <= 1'b0;
        if (rst) begin
            st <= S_IDLE;
        end else case (st)
        S_IDLE: if (start) begin
            k <= kind; a <= addr; e <= endp; d1 <= data1;
            n <= (len > 10'd512) ? 10'd512 : len;
            sp <= split;
            flags <= 6'd0; pid <= 8'h00; rx_len <= 10'd0;
            st <= S_WAITSLOT;
        end
        S_WAITSLOT: if (slot_ok) begin
            tidx <= 2'd0; sent_last <= 1'b0;
            st <= sp_en ? S_SPLIT : S_TOK;
        end
        S_SPLIT: begin
            if (tx_ack) begin
                if (tidx == 2'd3) sent_last <= 1'b1;
                tidx <= tidx + 2'd1;
            end
            if (tx_abort) begin
                flags[F_ABORT] <= 1'b1; st <= S_FIN;
            end else if (tx_done) begin
                tidx <= 2'd0; sent_last <= 1'b0;
                st <= S_TOK;
            end
        end
        S_TOK: begin
            if (tx_ack) begin
                if (tidx == 2'd2) sent_last <= 1'b1;
                tidx <= tidx + 2'd1;
            end
            if (tx_abort) begin
                flags[F_ABORT] <= 1'b1; st <= S_FIN;
            end else if (tx_done) begin
                sent_last <= 1'b0;
                if ((k == K_SETUP || k == K_OUT) && !(sp_en && sp_sc)) begin
                    didx <= 10'd0; crc <= 16'hFFFF;
                    st <= S_DATA;
                end else begin
                    tmo <= 12'd0; rx_seen <= 1'b0; rx_n <= 11'd0; rx_crc <= 16'hFFFF;
                    st <= S_WAITRX;
                end
            end
        end
        S_DATA: begin
            if (tx_ack) begin
                if (didx >= 10'd1 && didx <= n) crc <= crc16_byte(crc, buf_tx_rbyte);
                if (didx == n + 10'd2) sent_last <= 1'b1;
                didx <= didx + 10'd1;
            end
            if (tx_abort) begin
                flags[F_ABORT] <= 1'b1; st <= S_FIN;
            end else if (tx_done) begin
                sent_last <= 1'b0;
                tmo <= 12'd0; rx_seen <= 1'b0; rx_n <= 11'd0; rx_crc <= 16'hFFFF;
                st <= (sp_en && sp_nohs) ? S_FIN : S_WAITRX;   // isochronous: no handshake
            end
        end
        S_WAITRX: begin
            // handshake (SETUP/OUT/PING) or data packet (IN)
            if (rx_active || rx_valid) rx_seen <= 1'b1;
            if (!rx_seen && !rx_active && !rx_valid) begin
                tmo <= tmo + 12'd1;
                if (tmo == TIMEOUT) begin
                    flags[F_TIMEOUT] <= 1'b1; st <= S_FIN;
                end
            end
            if (rx_valid) begin
                rx_n <= rx_n + 11'd1;
                if (rx_n == 11'd0) begin
                    pid <= rx_data;
                end else begin
                    rx_crc <= crc16_byte(rx_crc, rx_data);
                    if (rx_n <= 11'd512) begin
                        buf_rx_we    <= 1'b1;
                        buf_rx_waddr <= rx_n[8:0] - 9'd1;
                        buf_rx_wbyte <= rx_data;
                    end else if (rx_n > 11'd514) begin
                        flags[F_OVF] <= 1'b1;
                    end
                end
            end
            if (rx_end) begin
                if (rx_err) flags[F_RXERR] <= 1'b1;
                st <= S_FIN;
                if (k == K_IN) begin
                    rx_len <= (rx_n >= 11'd3) ? (rx_n[9:0] - 10'd3) : 10'd0;
                    if (!pid_ok) flags[F_PIDERR] <= 1'b1;
                    else if (is_data) begin
                        if (rx_n < 11'd3 || rx_crc != 16'hB001 || rx_err) begin
                            flags[F_CRC] <= 1'b1;        // no ACK: the device will resend
                        end else if (!(sp_en && sp_sc)) begin
                            sent_last <= 1'b0;
                            st <= S_ACK;               // (complete-split: no ACK to the hub)
                        end
                    end
                end else if (!pid_ok || rx_n != 11'd1) begin
                    flags[F_PIDERR] <= 1'b1;
                end
            end
        end
        S_ACK: begin
            if (tx_ack) sent_last <= 1'b1;
            if (tx_abort) begin
                flags[F_ABORT] <= 1'b1; st <= S_FIN;
            end else if (tx_done) st <= S_FIN;
        end
        S_FIN: begin
            done  <= 1'b1;
            count <= count + 16'd1;
            st    <= S_IDLE;
        end
        default: st <= S_IDLE;
        endcase
    end
endmodule
