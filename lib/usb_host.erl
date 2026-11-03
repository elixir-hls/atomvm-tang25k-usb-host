%% The USB host for device drivers written in Elixir or Erlang: stubs of the NIFs in
%% src/usb_host_nif.c, over the C library in src/usb/ (docs/driver-guide.md). Built into
%% AtomVM when the SoC has the USB host (use EHLS, blocks: [EHLS.USBHost]).
-module(usb_host).
-export([bus_reset/0, phy_reset/0, enumerate/0, ctrl_in/3, ctrl_out/3,
         bulk_in_start/3, bulk_in_stop/0, bulk_in_stats/0, debug/0]).

%% VBUS on, wait for a device, reset the bus with the high-speed handshake, start the SOFs.
-spec bus_reset() -> ok | {error, integer()}.
bus_reset() -> erlang:nif_error(undefined).

%% A reset pulse to the PHY (it sometimes does not answer after power-up); then bus_reset/0.
-spec phy_reset() -> ok.
phy_reset() -> erlang:nif_error(undefined).

%% Descriptors, address, configuration, directly or behind a USB 2.0 hub. Addr: the
%% high-speed device's address; DeviceDescriptor: its 18 bytes. Step: the step that failed.
-spec enumerate() -> {ok, Addr :: 1..127, DeviceDescriptor :: binary()} | {error, Step :: integer()}.
enumerate() -> erlang:nif_error(undefined).

%% A control transfer from the device on endpoint 0: Setup is the 8-byte SETUP packet, Len
%% the bytes asked for (at most 256); Data what came (NAKs and errors are retried).
-spec ctrl_in(Addr :: 0..127, Setup :: <<_:64>>, Len :: 0..256) ->
    {ok, Data :: binary()} | {error, {Err :: integer(), Stage :: integer(), Pid :: integer()}}.
ctrl_in(_Addr, _Setup, _Len) -> erlang:nif_error(undefined).

%% A control transfer to the device on endpoint 0 (Data at most 256 bytes).
-spec ctrl_out(Addr :: 0..127, Setup :: <<_:64>>, Data :: binary()) ->
    ok | {error, {Err :: integer(), Stage :: integer(), Pid :: integer()}}.
ctrl_out(_Addr, _Setup, _Data) -> erlang:nif_error(undefined).

%% The bulk IN engine on: a continuous bulk IN from Endpoint of Addr into the pipeline's
%% usb_bulk_in(); Data1First: the data toggle the device starts with.
-spec bulk_in_start(Addr :: 1..127, Endpoint :: 1..15, Data1First :: boolean()) -> ok.
bulk_in_start(_Addr, _Endpoint, _Data1First) -> erlang:nif_error(undefined).

-spec bulk_in_stop() -> ok.
bulk_in_stop() -> erlang:nif_error(undefined).

%% Rolling counters of the bulk IN engine (below 2^27; Errors: errs[15:0]).
-spec bulk_in_stats() -> {Packets :: integer(), Naks :: integer(), Errors :: integer()}.
bulk_in_stats() -> erlang:nif_error(undefined).

%% The host's state for diagnosing a stopped stream: the ULPI link levels, sof_stat,
%% xfer_stat >> 5, the PHY's registers 0x04 | 0x07 << 8 | 0x0A << 16 (0xFF each when a read
%% fails), the last RX CMD.
-spec debug() -> {integer(), integer(), integer(), integer(), integer()}.
debug() -> erlang:nif_error(undefined).
