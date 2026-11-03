# atomvm-tang25k-usb-host

A USB 2.0 high-speed host on a USB3300 ULPI PHY for
[atomvm-tang25k-hls](https://github.com/elixir-hls/atomvm-tang25k-hls) (with [elixir-hls](https://github.com/elixir-hls/elixir-hls)): a block
package (`EHLS.Blocks`) whose bulk IN engine feeds a pipeline directly, so a device's data
stream never passes through the CPU.

```elixir
use EHLS, blocks: [EHLS.USBHost]

defpipeline iq do
  usb_bulk_in() |> ... 
end
```

| | Kind | |
|---|---|---|
| `usb_bulk_in()` | source | The payload of a continuous bulk IN, one byte per word; the device driver sets up the device and the engine (endpoint, address, data toggle) |
| `:usb_host` | peripheral | The host alone, for the CPU's control / bulk transfers |

Supported for now: what the FM radio ([atomvm-fm-radio](https://github.com/elixir-hls/atomvm-fm-radio))
needs. In a pipeline that is bulk IN (`usb_bulk_in()`) only: there are no blocks for bulk
OUT, interrupt or isochronous transfers. From the CPU, the C library below does control
transfers and single transactions, enough to enumerate and set up a device.

Device drivers are written in Elixir on `:usb_host` (lib/usb_host.erl, NIFs in
src/usb_host_nif.c): bus reset, enumeration (directly or behind a USB 2.0 hub), control
transfers, the bulk IN engine's start / stop / counters, the host's state for diagnosis.
[atomvm-tang25k-rtlsdr](https://github.com/elixir-hls/atomvm-tang25k-rtlsdr) is one. How
to write one: [docs/driver-guide.md](docs/driver-guide.md) ([日本語](docs/driver-guide.ja.md)).

Underneath is the C library (src/usb/, built into AtomVM when the SoC has the host), which a
driver in C can use directly: ULPI register access (`ulpi.h`), bus reset with the high-speed
handshake and SOF, the bulk IN engine (`usbh.h`), transactions and control transfers with
retries (`usb_xfer.h`), enumeration with split transactions for full-speed devices behind a
hub (`usb_enum.h`).

The hardware (gateware/usbh.py, gateware/rtl/usb/): the ULPI link layer, SOF, the HS
chirp, transactions, the bulk IN engine (two 512-byte banks, NAK pacing), the transaction
arbiter. The link gives a packet up when the PHY does not take its TXCMD (255 clocks without
NXT) instead of hanging.

## Pins

Default: PMOD J4 / J5 of the Sipeed Tang Primer 25K Dock (the timing-critical ones in one
bank; CLK on a clock-capable pin):

| Signal | Pin | Dock |
|---|---|---|
| `clk` | j4:3 | J4-5 (C11) |
| `data` | j5:1 j5:5 j5:0 j5:4 j4:7 j4:6 j4:5 j4:4 | DATA0..3 J5-9..12, DATA4..7 J4-6, 8, 10, 12 |
| `dir` | j4:2 | J4-7 (B11) |
| `nxt` | j4:1 | J4-9 (D11) |
| `stp` | j4:0 | J4-11 (G11) |
| `rst` | j5:3 | J5-5 (L5) |

Other pins: `use EHLS, blocks: [{EHLS.USBHost, pins: [stp: "j4:0", ...]}]`. The PHY's RESET can
be left unconnected (USB3300 modules pull it down); a RESET wire picking up a neighbouring
signal knocks the PHY out.

From usb-rtl-sdr-tang25k and usb-rtl-sdr-tv.

## License

Apache-2.0 (see [LICENSE](LICENSE)).
