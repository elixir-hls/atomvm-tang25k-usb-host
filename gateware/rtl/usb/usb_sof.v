// usb_sof.v — high-speed SOF generator (one SOF per 125 us microframe).
//
// SOF token on ULPI: TXCMD 0x45 (TX CMD 01 + PID[3:0] = 0101), then two bytes
//   byte 1 = frame[7:0], byte 2 = {crc5, frame[10:8]}
// (the PHY makes the PID byte from the TXCMD). The frame number counts USB frames: it
// advances every 8 microframes (USB 2.0 §8.4.3.1). CRC5 = x^5 + x^2 + 1 over the 11 frame
// bits, LSB first, seed 11111, sent inverted (§8.3.5.1).
//
//   en         : level. The first SOF goes out right away on 0->1 (after reset the host must
//                start SOFs immediately, or a HS device suspends after 3 ms of idle).
//   frame_load : pulse — set the frame number to frame_init (and the microframe to 0).
//   uframe_cyc : microframe length in ULPI clocks (7500 = 125 us at 60 MHz).
//   sof_pulse  : one cycle when an SOF has been requested (for the transaction scheduler).
module usb_sof (
    input  wire        clk,
    input  wire        rst,
    input  wire        en,
    input  wire        frame_load,
    input  wire [10:0] frame_init,
    input  wire [15:0] uframe_cyc,

    output wire        tx_valid,
    output wire [7:0]  tx_data,
    output wire        tx_last,
    input  wire        tx_ack,
    input  wire        tx_done,
    input  wire        tx_abort,

    output reg  [10:0] frame = 11'd0,
    output reg  [2:0]  uframe = 3'd0,
    output reg         sof_pulse = 1'b0,
    output reg  [15:0] uf_timer = 16'd0,   // ULPI clocks since the start of this microframe
    output reg  [15:0] sent = 16'd0,       // SOFs sent (rolling)
    output reg  [15:0] missed = 16'd0      // microframes whose SOF could not be sent in time
);
    function [4:0] crc5(input [10:0] d);
        integer i;
        reg [4:0] c;
        reg fb;
        begin
            c = 5'b11111;
            for (i = 0; i < 11; i = i + 1) begin
                fb = c[4] ^ d[i];
                c  = {c[3:0], 1'b0};
                if (fb) c = c ^ 5'b00101;
            end
            crc5 = ~{c[0], c[1], c[2], c[3], c[4]};
        end
    endfunction

    reg        pend = 1'b0;           // an SOF is waiting to be sent
    reg        active = 1'b0;         // our packet is on the link
    reg [1:0]  idx = 2'd0;            // 0 = TXCMD, 1 = frame lo, 2 = crc|frame hi
    reg        en_d = 1'b0;
    wire [4:0] crc = crc5(frame);

    assign tx_valid = pend | active;
    assign tx_data  = (idx == 2'd0) ? 8'h45 : (idx == 2'd1) ? frame[7:0] : {crc, frame[10:8]};
    assign tx_last  = (idx == 2'd2);

    always @(posedge clk) begin
        sof_pulse <= 1'b0;
        en_d <= en;
        if (rst || !en) begin
            pend <= 1'b0;
            active <= 1'b0;
            idx <= 2'd0;
            uf_timer <= 16'd0;
            if (frame_load) begin
                frame <= frame_init;
                uframe <= 3'd0;
            end
        end else begin
            if (frame_load) begin
                frame <= frame_init;
                uframe <= 3'd0;
            end
            // microframe timer; the first SOF right after enable
            if (!en_d || uf_timer == uframe_cyc - 16'd1) begin
                uf_timer <= 16'd0;
                if (en_d) begin
                    uframe <= uframe + 3'd1;
                    if (uframe == 3'd7) frame <= frame + 11'd1;
                end
                if (pend || active) missed <= missed + 16'd1;
                else begin
                    pend <= 1'b1;
                    sof_pulse <= 1'b1;
                end
            end else begin
                uf_timer <= uf_timer + 16'd1;
            end
            if (tx_ack) begin
                pend   <= 1'b0;
                active <= 1'b1;
                idx    <= idx + 2'd1;
            end
            if (active && (tx_done || tx_abort)) begin
                active <= 1'b0;
                idx    <= 2'd0;
                if (tx_done) sent <= sent + 16'd1;
                else pend <= 1'b1;              // aborted by the PHY: send again
            end
        end
    end
endmodule
