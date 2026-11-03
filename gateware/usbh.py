"""
(From usb-rtl-sdr-tv 42056a9, gateware/usbh.py; the RTL is in gateware/rtl/usb/; used by
hls_usb_host.py.)

USB host (ULPI, USB3300) for the LiteX SoC.
  S1: ULPI link + PHY register access.  S2: HS chirp (auto fire) + SOF generator.
  S3: transaction engine (SETUP/OUT/IN/PING) with packet buffers on the bus.
  S5: bulk IN engine (continuous IN on the IQ endpoint) -> `iq` byte stream in the sys domain.

Clock domains
  sys  : CPU / CSRs.
  ulpi : 60 MHz from the PHY's CLKOUT (pads.clk, a GCLK pin). It stops while the PHY is
         in reset, so nothing on the sys side waits on it without a timeout in software.

CSRs (all in sys)
  ctrl      : phy_reset (USB3300 RESET pin, active high).
  reg_cmd   : write starts one PHY register access: addr[5:0], we[8], wdata[23:16].
  reg_stat  : rdata[7:0], done[8] (1 = the last access finished), rxcmd[23:16] (last RX CMD).
  ulpi_freq : ULPI clock frequency in Hz (LiteX FreqMeter) — proves CLKOUT reaches the pin.
  chirp_ctrl: arm[0] (fire the host chirp when a device chirp K ends), min_pairs[6:4],
              seg_len[31:16] (one K/J phase in ULPI clocks, 3000 = 50 us).
  chirp_stat: fires[7:0] (rolling), k_seen[8], aborted[9], busy[10], linestate[13:12].
  sof_ctrl  : en[0] (SOF every 125 us; the first one right away).
  sof_frame : write = set the frame number (applied in the ulpi domain).
  sof_stat  : frame[10:0], sent[31:16] (rolling).
  xfer_cmd  : write = run one transaction: kind[1:0] (0 SETUP, 1 OUT, 2 IN, 3 PING), data1[2],
              endp[7:4], addr[14:8], len[25:16] (OUT/SETUP payload bytes from the TX buffer).
  xfer_stat : done[0], pid[15:8], flags[21:16] {pid_err, overflow, rx_err, crc_err, abort,
              timeout}, rx_len[31:22] (IN payload bytes in the RX buffer).

  bulk_ctrl : en[0], init_toggle[1] (1 = DATA1 expected first), endp[7:4], addr[14:8],
              nak_gap[31:16] (ULPI clocks to wait after a NAK before the next IN; 0 = at once).
              The RTL-SDR fills a packet every ~1 ms, so without a gap ~1 M IN/NAK pairs per
              second run on the bus (a possible source of RF noise).
  bulk_pkts / bulk_naks : rolling counters; bulk_err : errs[15:0], dups[31:16].
  link_dbg  : ulpi_link levels for bring-up (see rtl/ulpi_link.v `dbg`).

Stream: `self.iq` (sys domain, 8-bit LiteX stream) carries the payload of every good bulk
packet, in order. Nothing consumed -> the FIFO fills -> the engine stops asking.

Packet buffers (bus slave "usbh_buf", uncached): TX at +0x000 (512 B, write), RX at +0x800
(512 B, read). Little-endian bytes in 32-bit words.

Timing constraints: the ULPI pads get set_input_delay / set_output_delay from the USB3300
datasheet (Table 6.2): PHY output delay 2.0-5.0 ns, PHY input setup 5.0 ns / hold 0 ns, with
+-0.5 ns allowed for clock/data skew on the wiring. The STA then shows whether posedge
capture meets setup/hold on this board (docs/progress.md, S1).
"""
import os

from migen import Constant, ClockDomain, ClockSignal, ResetSignal, Instance, Signal, If, Replicate, Cat, ClockDomainsRenamer
from migen.genlib.cdc import MultiReg, PulseSynchronizer, BusSynchronizer
from migen.genlib.resetsync import AsyncResetSynchronizer

from litex.gen import LiteXModule
from litex.build.io import Tristate
from litex.soc.interconnect.csr import CSRStorage, CSRStatus, CSRField
from litex.soc.interconnect import wishbone, stream
from litex.soc.cores.freqmeter import FreqMeter

RTL_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "rtl", "usb")

class CmdHandshake(LiteXModule):
    """sys -> ulpi command handshake with a Gray-coded sequence number.

    Each CSR write (`issue`) advances a 4-bit Gray counter (one bit changes per command, so a
    plain 2-FF synchroniser cannot tear it). The ulpi side starts a command when the counter
    differs from the last one it started and nothing is in flight; at `finish` it reports the
    started number back. `done` (sys) = reported == issued. If the ulpi clock is stopped,
    commands pile up and only the newest runs when it comes back; `done` stays 0 until then
    (a toggle would read "done" again after an even number of lost commands).
    The sequence registers are reset-less: a ulpi reset does not re-run an old command.
    The counter advances 3 sys cycles after the CSR write, so the command fields (synchronised
    separately) have settled before the ulpi side can see the new number.
    """
    def __init__(self, issue, finish):
        self.start = Signal()          # ulpi: one-cycle start pulse
        self.done  = Signal()          # sys: the newest command has finished
        seq = Signal(4)
        gray = Signal(4)
        issue_d = Signal(3)
        self.sync += [
            issue_d.eq(Cat(issue, issue_d[:-1])),
            If(issue_d[-1], seq.eq(seq + 1)),
        ]
        self.comb += gray.eq(seq ^ (seq >> 1))
        req_u = Signal(4)
        self.specials += MultiReg(gray, req_u, "ulpi")
        last = Signal(4, reset_less=True)
        ack = Signal(4, reset_less=True)
        inflight, finish_d = Signal(), Signal()
        self.comb += self.start.eq((req_u != last) & ~inflight)
        self.sync.ulpi += [
            finish_d.eq(finish),
            If(self.start, last.eq(req_u), inflight.eq(1)),
            If(finish_d, ack.eq(last), inflight.eq(0)),   # results latched one cycle earlier
        ]
        ack_s = Signal(4)
        self.specials += MultiReg(ack, ack_s)
        # not done while a write is still on its way to the counter
        self.comb += self.done.eq((ack_s == gray) & ~issue & (issue_d == 0))


# USB3300 DS Table 6.2 + skew allowance (ns)
PHY_TCO_MIN, PHY_TCO_MAX = 2.0, 5.0
PHY_TSU, PHY_TH = 5.0, 0.0
SKEW = 0.5


class USBHost(LiteXModule):
    def __init__(self, platform, pads, sys_clk_freq):
        # ---- ulpi clock domain ----
        self.cd_ulpi = ClockDomain()
        self.comb += self.cd_ulpi.clk.eq(pads.clk)
        platform.add_period_constraint(pads.clk, 1e9 / 60e6, name="ulpi_clk")

        # ---- CSRs ----
        self.ctrl = CSRStorage(fields=[
            CSRField("phy_reset", size=1, description="USB3300 RESET pin (1 = in reset)."),
        ])
        self.reg_cmd = CSRStorage(fields=[
            CSRField("addr",  size=6, offset=0),
            CSRField("we",    size=1, offset=8),
            CSRField("wdata", size=8, offset=16),
        ])
        self.reg_stat = CSRStatus(fields=[
            CSRField("rdata", size=8, offset=0),
            CSRField("done",  size=1, offset=8),
            CSRField("rxcmd", size=8, offset=16),
        ])
        self.ulpi_freq = FreqMeter(period=int(sys_clk_freq), clk=self.cd_ulpi.clk)

        self.comb += pads.rst.eq(self.ctrl.fields.phy_reset)
        self.specials += AsyncResetSynchronizer(self.cd_ulpi,
            ResetSignal("sys") | self.ctrl.fields.phy_reset)

        # ---- register access handshake ----
        addr_u, we_u, wdata_u = Signal(6), Signal(), Signal(8)
        self.specials += [
            MultiReg(self.reg_cmd.fields.addr, addr_u, "ulpi"),
            MultiReg(self.reg_cmd.fields.we, we_u, "ulpi"),
            MultiReg(self.reg_cmd.fields.wdata, wdata_u, "ulpi"),
        ]
        reg_done, reg_rdata = Signal(), Signal(8)
        self.reg_hs = CmdHandshake(self.reg_cmd.re, reg_done)
        reg_req = self.reg_hs.start
        rdata_l = Signal(8)
        self.sync.ulpi += If(reg_done, rdata_l.eq(reg_rdata))
        rdata_s = Signal(8)
        self.specials += MultiReg(rdata_l, rdata_s)
        rxcmd, rxcmd_s = Signal(8), Signal(8)
        self.specials += MultiReg(rxcmd, rxcmd_s)
        self.comb += [
            self.reg_stat.fields.rdata.eq(rdata_s),
            self.reg_stat.fields.done.eq(self.reg_hs.done),
            self.reg_stat.fields.rxcmd.eq(rxcmd_s),
        ]

        # ---- chirp / SOF CSRs ----
        self.chirp_ctrl = CSRStorage(fields=[
            CSRField("arm",       size=1,  offset=0),
            CSRField("min_pairs", size=3,  offset=4,  reset=3),
            CSRField("seg_len",   size=16, offset=16, reset=3000),
        ])
        self.chirp_stat = CSRStatus(fields=[
            CSRField("fires",     size=8, offset=0),
            CSRField("k_seen",    size=1, offset=8),
            CSRField("aborted",   size=1, offset=9),
            CSRField("busy",      size=1, offset=10),
            CSRField("linestate", size=2, offset=12),
        ])
        self.sof_ctrl  = CSRStorage(fields=[CSRField("en", size=1)])
        self.sof_frame = CSRStorage(11)
        self.sof_stat  = CSRStatus(fields=[
            CSRField("frame", size=11, offset=0),
            CSRField("sent",  size=16, offset=16),
        ])
        arm_u, minp_u, seg_u, sof_en_u = Signal(), Signal(3), Signal(16), Signal()
        frame_init_u = Signal(11)
        self.specials += [
            MultiReg(self.chirp_ctrl.fields.arm, arm_u, "ulpi"),
            MultiReg(self.chirp_ctrl.fields.min_pairs, minp_u, "ulpi"),
            MultiReg(self.chirp_ctrl.fields.seg_len, seg_u, "ulpi"),
            MultiReg(self.sof_ctrl.fields.en, sof_en_u, "ulpi"),
            MultiReg(self.sof_frame.storage, frame_init_u, "ulpi"),
        ]
        # frame load: delay the write strobe so frame_init_u has settled when it lands
        self.frame_ps = PulseSynchronizer("sys", "ulpi")
        frame_we_d = Signal(4)
        self.sync += frame_we_d.eq(Cat(self.sof_frame.re, frame_we_d[:-1]))
        self.comb += self.frame_ps.i.eq(frame_we_d[-1])
        frame_u, sent_u = Signal(11), Signal(16)
        fires_u, k_seen_u, aborted_u, cbusy_u = Signal(8), Signal(), Signal(), Signal()
        self.frame_sync = BusSynchronizer(11, "ulpi", "sys")
        self.sent_sync  = BusSynchronizer(16, "ulpi", "sys")
        self.fires_sync = BusSynchronizer(8, "ulpi", "sys")
        flags_s = Signal(3)
        self.specials += MultiReg(Cat(k_seen_u, aborted_u, cbusy_u), flags_s)
        self.comb += [
            self.frame_sync.i.eq(frame_u), self.sent_sync.i.eq(sent_u),
            self.fires_sync.i.eq(fires_u),
            self.sof_stat.fields.frame.eq(self.frame_sync.o),
            self.sof_stat.fields.sent.eq(self.sent_sync.o),
            self.chirp_stat.fields.fires.eq(self.fires_sync.o),
            self.chirp_stat.fields.k_seen.eq(flags_s[0]),
            self.chirp_stat.fields.aborted.eq(flags_s[1]),
            self.chirp_stat.fields.busy.eq(flags_s[2]),
            self.chirp_stat.fields.linestate.eq(rxcmd_s[0:2]),
        ]

        # ---- transaction CSRs ----
        self.xfer_cmd = CSRStorage(fields=[
            CSRField("kind",  size=2,  offset=0),
            CSRField("data1", size=1,  offset=2),
            CSRField("endp",  size=4,  offset=4),
            CSRField("addr",  size=7,  offset=8),
            CSRField("len",   size=10, offset=16),
        ])
        self.xfer_stat = CSRStatus(fields=[
            CSRField("done",   size=1,  offset=0),
            CSRField("pid",    size=8,  offset=8),
            CSRField("flags",  size=6,  offset=16),
            CSRField("rx_len", size=10, offset=22),
        ])
        # split token fields of the next CPU transaction (usb_xact.v); written before xfer_cmd
        self.xfer_split = CSRStorage(23)
        xcmd_u = Signal(32)
        self.specials += MultiReg(self.xfer_cmd.storage, xcmd_u, "ulpi")
        xsplit_u = Signal(23)
        self.specials += MultiReg(self.xfer_split.storage, xsplit_u, "ulpi")
        x_done, x_pid, x_flags, x_rxlen = Signal(), Signal(8), Signal(6), Signal(10)
        cpu_done_ = Signal()
        self.xfer_hs = CmdHandshake(self.xfer_cmd.re, cpu_done_)
        x_start = self.xfer_hs.start
        xres_l = Signal(24)
        self.sync.ulpi += If(cpu_done_, xres_l.eq(Cat(x_pid, x_flags, x_rxlen)))
        xres_s = Signal(24)
        self.specials += MultiReg(xres_l, xres_s)
        self.comb += [
            self.xfer_stat.fields.done.eq(self.xfer_hs.done),
            self.xfer_stat.fields.pid.eq(xres_s[0:8]),
            self.xfer_stat.fields.flags.eq(xres_s[8:14]),
            self.xfer_stat.fields.rx_len.eq(xres_s[14:24]),
        ]

        # ---- bulk IN CSRs ----
        self.bulk_ctrl = CSRStorage(fields=[
            CSRField("en",          size=1, offset=0),
            CSRField("init_toggle", size=1, offset=1),
            CSRField("endp",        size=4, offset=4, reset=1),
            CSRField("addr",        size=7, offset=8, reset=1),
            CSRField("nak_gap",     size=16, offset=16),
        ])
        self.bulk_pkts = CSRStatus(32)
        self.bulk_naks = CSRStatus(32)
        self.bulk_err  = CSRStatus(fields=[
            CSRField("errs", size=16, offset=0),
            CSRField("dups", size=16, offset=16),
        ])
        bctl_u = Signal(32)
        self.specials += MultiReg(self.bulk_ctrl.storage, bctl_u, "ulpi")
        b_pkts, b_naks, b_errs, b_dups = Signal(32), Signal(32), Signal(16), Signal(16)
        self.bulk_pkts_sync = BusSynchronizer(32, "ulpi", "sys")
        self.bulk_naks_sync = BusSynchronizer(32, "ulpi", "sys")
        self.bulk_err_sync  = BusSynchronizer(32, "ulpi", "sys")
        self.comb += [
            self.bulk_pkts_sync.i.eq(b_pkts), self.bulk_pkts.status.eq(self.bulk_pkts_sync.o),
            self.bulk_naks_sync.i.eq(b_naks), self.bulk_naks.status.eq(self.bulk_naks_sync.o),
            self.bulk_err_sync.i.eq(Cat(b_errs, b_dups)), self.bulk_err.status.eq(self.bulk_err_sync.o),
        ]

        # ---- IQ stream: bulk engine (ulpi) -> async FIFO -> sys ----
        self.iq = stream.Endpoint([("data", 8)])
        iq_fifo = stream.AsyncFIFO([("data", 8)], depth=1024, buffered=True)
        self.iq_fifo = ClockDomainsRenamer({"write": "ulpi", "read": "sys"})(iq_fifo)
        self.comb += self.iq_fifo.source.connect(self.iq)

        # ---- packet buffers on the bus ----
        self.bus = wishbone.Interface(data_width=32, address_width=32, addressing="word")
        b_tx_we, b_ack = Signal(4), Signal()
        b_rx_rdata = Signal(32)
        bt_raddr, bt_rbyte = Signal(9), Signal(8)
        br_we, br_waddr, br_wbyte = Signal(), Signal(9), Signal(8)
        cpu_rx_we, bulk_rx_we = Signal(), Signal()
        req = self.bus.cyc & self.bus.stb
        self.sync += b_ack.eq(req & ~b_ack)
        self.comb += [
            self.bus.ack.eq(b_ack),
            self.bus.dat_r.eq(b_rx_rdata),
            If(req & self.bus.we & ~b_ack & ~self.bus.adr[9], b_tx_we.eq(self.bus.sel)),
        ]
        self.specials += Instance("usb_bufs",
            i_sys_clk=ClockSignal("sys"),
            i_tx_we=b_tx_we, i_tx_waddr=self.bus.adr[0:7], i_tx_wdata=self.bus.dat_w,
            i_rx_raddr=self.bus.adr[0:7], o_rx_rdata=b_rx_rdata,
            i_ulpi_clk=ClockSignal("ulpi"),
            i_tx_raddr=bt_raddr, o_tx_rbyte=bt_rbyte,
            i_rx_we=cpu_rx_we, i_rx_waddr=br_waddr, i_rx_wbyte=br_wbyte)

        # ---- TX sources: 0 = chirp, 1 = SOF, 2 = transactions ----
        tx_valid, tx_data, tx_last = Signal(), Signal(8), Signal()
        tx_ack, tx_done, tx_abort = Signal(), Signal(), Signal()
        s_ack, s_done, s_abort = Signal(3), Signal(3), Signal(3)
        c_valid, c_data, c_last = Signal(), Signal(8), Signal()
        f_valid, f_data, f_last = Signal(), Signal(8), Signal()
        x_valid, x_data, x_last = Signal(), Signal(8), Signal()
        uf_timer = Signal(16)
        uframe_u, sof_pulse_u = Signal(3), Signal()
        rx_active, rx_valid, rx_data, rx_end, rx_err = Signal(), Signal(), Signal(8), Signal(), Signal()
        self.specials += Instance("usb_tx_mux",
            i_clk=ClockSignal("ulpi"), i_rst=ResetSignal("ulpi"),
            i_s_valid=Cat(c_valid, f_valid, x_valid), i_s_data=Cat(c_data, f_data, x_data),
            i_s_last=Cat(c_last, f_last, x_last),
            o_s_ack=s_ack, o_s_done=s_done, o_s_abort=s_abort,
            o_tx_valid=tx_valid, o_tx_data=tx_data, o_tx_last=tx_last,
            i_tx_ack=tx_ack, i_tx_done=tx_done, i_tx_abort=tx_abort)
        self.specials += Instance("usb_hs_chirp",
            i_clk=ClockSignal("ulpi"), i_rst=ResetSignal("ulpi"),
            i_arm=arm_u, i_linestate=rxcmd[0:2], i_seg_len=seg_u, i_min_pairs=minp_u,
            o_tx_valid=c_valid, o_tx_data=c_data, o_tx_last=c_last,
            i_tx_ack=s_ack[0], i_tx_done=s_done[0], i_tx_abort=s_abort[0],
            o_fires=fires_u, o_k_seen=k_seen_u, o_aborted=aborted_u, o_busy=cbusy_u)
        self.specials += Instance("usb_sof",
            i_clk=ClockSignal("ulpi"), i_rst=ResetSignal("ulpi"),
            i_en=sof_en_u, i_frame_load=self.frame_ps.o, i_frame_init=frame_init_u,
            i_uframe_cyc=7500,
            o_tx_valid=f_valid, o_tx_data=f_data, o_tx_last=f_last,
            i_tx_ack=s_ack[1], i_tx_done=s_done[1], i_tx_abort=s_abort[1],
            o_frame=frame_u, o_sent=sent_u, o_uf_timer=uf_timer,
            o_uframe=uframe_u, o_sof_pulse=sof_pulse_u)
        xa_start, xa_kind, xa_addr, xa_endp, xa_d1, xa_len, xa_split = (
            Signal(), Signal(2), Signal(7), Signal(4), Signal(), Signal(10), Signal(23))
        x_busy, cpu_done, bulk_req, bulk_go, bulk_done = (
            Signal(), Signal(), Signal(), Signal(), Signal())
        # the arbiter's isochronous OUT port (the USB speaker of the FM radio) is not used
        iso_go, iso_done, x_from_iso = Signal(), Signal(), Signal()
        self.specials += Instance("usb_xact_arb",
            i_clk=ClockSignal("ulpi"), i_rst=ResetSignal("ulpi"),
            i_cpu_start=x_start, i_cpu_kind=xcmd_u[0:2], i_cpu_addr=xcmd_u[8:15],
            i_cpu_endp=xcmd_u[4:8], i_cpu_data1=xcmd_u[2], i_cpu_len=xcmd_u[16:26],
            i_cpu_split=xsplit_u,
            o_cpu_done=cpu_done_,
            i_iso_req=0, i_iso_addr=Constant(0, 7), i_iso_endp=Constant(0, 4), i_iso_len=Constant(0, 10),
            i_iso_split=Constant(0, 23), o_iso_go=iso_go, o_iso_done=iso_done, o_x_from_iso=x_from_iso,
            i_bulk_req=bulk_req, i_bulk_addr=bctl_u[8:15], i_bulk_endp=bctl_u[4:8],
            o_bulk_go=bulk_go, o_bulk_done=bulk_done,
            o_x_start=xa_start, o_x_kind=xa_kind, o_x_addr=xa_addr, o_x_endp=xa_endp,
            o_x_data1=xa_d1, o_x_len=xa_len, o_x_split=xa_split,
            i_x_busy=x_busy, i_x_done=x_done, i_x_rx_we=br_we,
            o_cpu_rx_we=cpu_rx_we, o_bulk_rx_we=bulk_rx_we)
        self.specials += Instance("usb_bulk_in",
            i_clk=ClockSignal("ulpi"), i_rst=ResetSignal("ulpi"),
            i_en=bctl_u[0], i_init_toggle=bctl_u[1], i_addr=bctl_u[8:15], i_endp=bctl_u[4:8],
            i_nak_gap=bctl_u[16:32],
            o_xreq=bulk_req, i_xgo=bulk_go, i_xdone=bulk_done,
            i_xpid=x_pid, i_xflags=x_flags, i_xlen=x_rxlen,
            i_rx_we=bulk_rx_we, i_rx_waddr=br_waddr, i_rx_wbyte=br_wbyte,
            o_m_valid=iq_fifo.sink.valid, o_m_data=iq_fifo.sink.data, i_m_ready=iq_fifo.sink.ready,
            o_pkts=b_pkts, o_naks=b_naks, o_errs=b_errs, o_dups=b_dups)
        self.specials += Instance("usb_xact",
            i_clk=ClockSignal("ulpi"), i_rst=ResetSignal("ulpi"),
            i_start=xa_start, i_kind=xa_kind, i_data1=xa_d1, i_endp=xa_endp,
            i_addr=xa_addr, i_len=xa_len, i_split=xa_split, o_busy=x_busy,
            i_sof_en=sof_en_u, i_uf_timer=uf_timer,
            o_tx_valid=x_valid, o_tx_data=x_data, o_tx_last=x_last,
            i_tx_ack=s_ack[2], i_tx_done=s_done[2], i_tx_abort=s_abort[2],
            i_rx_active=rx_active, i_rx_valid=rx_valid, i_rx_data=rx_data,
            i_rx_end=rx_end, i_rx_err=rx_err,
            o_buf_tx_raddr=bt_raddr, i_buf_tx_rbyte=bt_rbyte,
            o_buf_rx_we=br_we, o_buf_rx_waddr=br_waddr, o_buf_rx_wbyte=br_wbyte,
            o_done=x_done, o_pid=x_pid, o_rx_len=x_rxlen, o_flags=x_flags)
        for f in ("usb_tx_mux.v", "usb_hs_chirp.v", "usb_sof.v", "usb_bufs.v", "usb_xact.v",
                  "usb_xact_arb.v", "usb_bulk_in.v"):
            platform.add_source(os.path.join(RTL_DIR, f))

        # ---- ULPI link ----
        dbg_u = Signal(16)
        self.link_dbg = CSRStatus(16, description="ulpi_link debug (levels, not coherent): "
            "{st[2:0], reg_pend, drive_en, dir, nxt, rx_active, wr_nxt2, rx_pkts[6:0]}.")
        self.specials += MultiReg(dbg_u, self.link_dbg.status)
        data_i, data_o, data_oe = Signal(8), Signal(8), Signal()
        self.specials += Tristate(pads.data, data_o, Replicate(data_oe, 8), data_i)
        self.specials += Instance("ulpi_link",
            i_clk=ClockSignal("ulpi"), i_rst=ResetSignal("ulpi"),
            i_dir_i=pads.dir, i_nxt_i=pads.nxt, i_data_i=data_i,
            o_data_o=data_o, o_data_oe=data_oe, o_stp_o=pads.stp,
            i_reg_req=reg_req, i_reg_we=we_u, i_reg_addr=addr_u, i_reg_wdata=wdata_u,
            o_reg_done=reg_done, o_reg_rdata=reg_rdata,
            i_tx_valid=tx_valid, i_tx_data=tx_data, i_tx_last=tx_last,
            o_tx_ack=tx_ack, o_tx_done=tx_done, o_tx_abort=tx_abort,
            o_rxcmd=rxcmd,
            o_rx_active=rx_active, o_rx_valid=rx_valid, o_rx_data=rx_data,
            o_rx_end=rx_end, o_rx_err=rx_err, o_dbg=dbg_u,
        )
        platform.add_source(os.path.join(RTL_DIR, "ulpi_link.v"))

        # ---- pad timing ----
        clk = "ulpi_clk"
        inputs = "{ulpi_dir ulpi_nxt ulpi_data[*]}"
        outputs = "{ulpi_stp ulpi_data[*]}"
        platform.extra_sdc = getattr(platform, "extra_sdc", []) + [
            # sys <-> ulpi only meet through the synchronisers above
            f"set_clock_groups -asynchronous -group [get_clocks {{{clk}}}]",
            f"set_input_delay -clock {clk} -max {PHY_TCO_MAX + SKEW:.3f} [get_ports {inputs}]",
            f"set_input_delay -clock {clk} -min {PHY_TCO_MIN - SKEW:.3f} [get_ports {inputs}]",
            f"set_output_delay -clock {clk} -max {PHY_TSU + SKEW:.3f} [get_ports {outputs}]",
            f"set_output_delay -clock {clk} -min {-(PHY_TH + SKEW):.3f} [get_ports {outputs}]",
            # list every ULPI pad path in the timing report (the default top-25 hides them)
            f"report_timing -setup -max_paths 40 -from [get_ports {inputs}]",
            f"report_timing -hold -max_paths 40 -from [get_ports {inputs}]",
            f"report_timing -setup -max_paths 40 -to [get_ports {outputs}]",
            f"report_timing -hold -max_paths 40 -to [get_ports {outputs}]",
            "report_timing -setup -max_paths 40 -from [get_ports {ulpi_nxt}]",
            "report_timing -hold -max_paths 40 -from [get_ports {ulpi_nxt}]",
        ]
        # experiment hook: USBH_IOB_OPTS="oreg_in_iob=0,..." sets Gowin IOB packing options
        for kv in filter(None, os.environ.get("USBH_IOB_OPTS", "").split(",")):
            k, v = kv.split("=")
            platform.toolchain.options[k] = int(v)

