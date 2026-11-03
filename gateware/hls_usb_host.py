"""
USB 2.0 high-speed host on a USB3300 ULPI PHY for elixir-hls / atomvm-tang25k-hls (EHLS.USBHost on the
Elixir side, EHLS.Blocks):

  usb_bulk_in  source: the payload bytes of a continuous bulk IN (the endpoint and device
               address set by the CPU, bulk_ctrl), one byte per word
  usb_host     peripheral: the host alone (control / bulk transactions from the CPU)

Both share one host per SoC (soc.usbh, CSRs usbh_*, packet buffers usbh_buf); the C side is
src/usb/ (ULPI registers, bus reset and the HS handshake, transactions, enumeration, hubs).

Default pins: PMOD J4 / J5 of the Sipeed Tang Primer 25K Dock, as in usb-rtl-sdr-tang25k;
an application moves them with use EHLS, blocks: [{EHLS.USBHost, pins: [stp: "j4:0", ...]}]
(configure). The PHY's RESET may be left unconnected (the PHY module pulls it down).
"""
from litex.build.generic_platform import IOStandard, Pins, Subsignal
from litex.gen import LiteXModule
from litex.soc.integration.soc import SoCRegion
from litex.soc.interconnect import stream

from usbh import USBHost

# signal -> LiteX connector pins (Dock: J4 / J5; the timing-critical ones in one bank,
# CLK = C11 GCLKT_13)
PINS = {
    "clk":  "j4:3",                                      # J4-5  C11
    # DATA0..3 = J5-9, 10, 11, 12 (E11 E10 A11 A10); DATA4..7 = J4-6, 8, 10, 12 (C10 B10 D10 G10)
    "data": "j5:1 j5:5 j5:0 j5:4 j4:7 j4:6 j4:5 j4:4",
    "dir":  "j4:2",                                      # J4-7  B11
    "nxt":  "j4:1",                                      # J4-9  D11
    "stp":  "j4:0",                                      # J4-11 G11
    "rst":  "j5:3",                                      # J5-5  L5
}


def configure(options):
    """options: {"pins": {signal: pins}} over PINS."""
    for k, v in options.items():
        if k != "pins":
            raise SystemExit(f"hls_usb_host: unknown option {k}")
        for sig, pins in v.items():
            if sig not in PINS:
                raise SystemExit(f"hls_usb_host: unknown pin {sig} (one of {', '.join(PINS)})")
            PINS[sig] = pins


def host(soc):
    """The SoC's USB host (made on first use)."""
    if not hasattr(soc, "usbh"):
        platform = soc.platform
        platform.add_extension([
            ("ulpi", 0, *[Subsignal(s, Pins(PINS[s])) for s in
                          ("clk", "data", "dir", "nxt", "stp", "rst")], IOStandard("LVCMOS33")),
        ])
        soc.usbh = USBHost(platform, platform.request("ulpi"), soc.sys_clk_freq)
        soc.bus.add_slave("usbh_buf", soc.usbh.bus,
                          SoCRegion(origin=0x90000000, size=0x1000, cached=False))
    return soc.usbh


class UsbBulkIn(LiteXModule):
    """Source: the payload bytes of the USB host's continuous bulk IN, one byte per word
    (bits 7..0). The CPU sets the device up and starts the engine; while the pipeline is
    stopped the bytes are dropped, so the engine never waits on stale data."""
    source_width = 8            # the platform's link after it needs only the byte
    source_signed = False

    def __init__(self, soc, enable):
        self.source = stream.Endpoint([("data", 32)])
        iq = host(soc).iq
        self.comb += [
            self.source.valid.eq(iq.valid & enable),
            self.source.data.eq(iq.data),
            iq.ready.eq(self.source.ready | ~enable),
        ]


BLOCKS = {"usb_bulk_in": UsbBulkIn}
PERIPHERALS = {"usb_host": host}
