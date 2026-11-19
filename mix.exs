defmodule AtomVMTang25KUSBHost.MixProject do
  use Mix.Project

  def project do
    [
      app: :atomvm_tang25k_usb_host,
      version: "0.1.0",
      elixir: "~> 1.18",
      elixirc_paths: ["lib"],
      deps: deps(),
      description: "A USB 2.0 high-speed host on a USB3300 ULPI PHY for Elixir HLS on the Tang Primer 25K: a bulk IN engine feeding an FPGA pipeline",
      package: [
        licenses: ["Apache-2.0"],
        links: %{"GitHub" => "https://github.com/elixir-hls/atomvm-tang25k-usb-host"},
        files: ~w(csr_headers.json docs gateware lib src mix.exs README.md LICENSE),
        exclude_patterns: [~r/__pycache__/, ~r/\.pyc$/]
      ]
    ]
  end

  def application, do: []

  defp deps do
    [
      {:elixir_hls, "~> 0.1"}
    ]
  end
end
