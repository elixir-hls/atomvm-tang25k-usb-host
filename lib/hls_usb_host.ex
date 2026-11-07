defmodule EHLS.USBHost do
  @moduledoc """
  USB 2.0 high-speed host (USB3300 ULPI PHY) for elixir-hls / atomvm-tang25k-hls (`EHLS.Blocks`; FPGA
  side: gateware/hls_usb_host.py):

      use EHLS, blocks: [EHLS.USBHost]
      use EHLS, blocks: [{EHLS.USBHost, pins: [stp: "j4:0"]}]   # other pins than the defaults

    * `usb_bulk_in()`: source, the payload of a continuous bulk IN, one byte per word (a
      device driver, e.g. atomvm-tang25k-rtlsdr, sets up the device and starts the engine);
    * peripheral `:usb_host`: the host for the CPU's transfers alone.

  Supported for now: what atomvm-fm-radio needs. In a pipeline only bulk IN (no blocks for
  bulk OUT, interrupt or isochronous transfers); from the CPU, control transfers and single
  transactions (the C library in src/usb/).
  """
  @behaviour EHLS.Blocks

  @impl true
  def soc, do: :litex

  @impl true
  def blocks, do: %{usb_bulk_in: {:source, 0}}

  @impl true
  def types, do: %{usb_bulk_in: {nil, :u8}}

  @impl true
  def peripherals, do: [:usb_host]

  @impl true
  def gateware, do: "hls_usb_host"
end
