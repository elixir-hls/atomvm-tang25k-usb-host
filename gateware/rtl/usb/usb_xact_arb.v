// usb_xact_arb.v — shares usb_xact between the isochronous OUT engine, the CPU (one command at a
// time) and the bulk IN engine (ULPI clock), in that order of priority; the bulk engine keeps the
// bus busy otherwise. The owner of the running transaction gets its done pulse and its received bytes
// (CPU -> the RX buffer the CPU reads, bulk -> the bulk engine's banks).
module usb_xact_arb (
    input  wire        clk,
    input  wire        rst,

    // CPU command (cpu_start: one-cycle pulse, fields stable while pending)
    input  wire        cpu_start,
    input  wire [1:0]  cpu_kind,
    input  wire [6:0]  cpu_addr,
    input  wire [3:0]  cpu_endp,
    input  wire        cpu_data1,
    input  wire [9:0]  cpu_len,
    input  wire [22:0] cpu_split,
    output wire        cpu_done,

    // isochronous OUT engine (OUT, DATA0, payload from its own buffer: x_from_iso)
    input  wire        iso_req,
    input  wire [6:0]  iso_addr,
    input  wire [3:0]  iso_endp,
    input  wire [9:0]  iso_len,
    input  wire [22:0] iso_split,
    output wire        iso_go,
    output wire        iso_done,
    output wire        x_from_iso,

    // bulk IN engine
    input  wire        bulk_req,
    input  wire [6:0]  bulk_addr,
    input  wire [3:0]  bulk_endp,
    output wire        bulk_go,
    output wire        bulk_done,

    // to / from usb_xact
    output wire        x_start,
    output wire [1:0]  x_kind,
    output wire [6:0]  x_addr,
    output wire [3:0]  x_endp,
    output wire        x_data1,
    output wire [9:0]  x_len,
    output wire [22:0] x_split,
    input  wire        x_busy,
    input  wire        x_done,
    input  wire        x_rx_we,
    output wire        cpu_rx_we,
    output wire        bulk_rx_we
);
    localparam O_NONE = 2'd0, O_CPU = 2'd1, O_BULK = 2'd2, O_ISO = 2'd3;
    reg [1:0] owner = O_NONE;
    reg       cpu_pend = 1'b0;
    reg [1:0] k = 2'd0;
    reg [6:0] a = 7'd0;
    reg [3:0] e = 4'd0;
    reg       d1 = 1'b0;
    reg [9:0] n = 10'd0;
    reg [22:0] sp = 23'd0;

    wire idle    = (owner == O_NONE) && !x_busy;
    wire go_iso  = idle && iso_req;
    wire go_cpu  = idle && !iso_req && cpu_pend;
    wire go_bulk = idle && !iso_req && !cpu_pend && !cpu_start && bulk_req;

    assign x_start = go_iso | go_cpu | go_bulk;
    assign x_kind  = go_iso ? 2'd1 : go_cpu ? k  : 2'd2;      // iso: OUT, bulk: IN
    assign x_addr  = go_iso ? iso_addr : go_cpu ? a  : bulk_addr;
    assign x_endp  = go_iso ? iso_endp : go_cpu ? e  : bulk_endp;
    assign x_data1 = go_cpu ? d1 : 1'b0;
    assign x_len   = go_iso ? iso_len : go_cpu ? n  : 10'd0;
    assign x_split = go_iso ? iso_split : go_cpu ? sp : 23'd0;
    assign bulk_go = go_bulk;
    assign iso_go  = go_iso;
    assign iso_done   = x_done && owner == O_ISO;
    assign x_from_iso = owner == O_ISO;
    assign cpu_done   = x_done && owner == O_CPU;
    assign bulk_done  = x_done && owner == O_BULK;
    assign cpu_rx_we  = x_rx_we && owner == O_CPU;
    assign bulk_rx_we = x_rx_we && owner == O_BULK;

    always @(posedge clk) begin
        if (rst) begin
            owner <= O_NONE;
            cpu_pend <= 1'b0;
        end else begin
            if (cpu_start) begin
                cpu_pend <= 1'b1;
                k <= cpu_kind; a <= cpu_addr; e <= cpu_endp; d1 <= cpu_data1; n <= cpu_len;
                sp <= cpu_split;
            end
            if (go_iso) begin
                owner <= O_ISO;
            end else if (go_cpu) begin
                owner <= O_CPU;
                cpu_pend <= 1'b0;
            end else if (go_bulk) begin
                owner <= O_BULK;
            end
            if (x_done) owner <= O_NONE;
        end
    end
endmodule
