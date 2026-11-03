/* From usb-rtl-sdr-tv 42056a9 (apps/sdr/src). */
/* ULPI PHY (USB3300) access through the usbh CSRs (gateware/usbh.py). */
#pragma once
#include <stdint.h>

void ulpi_phy_reset(int on);
uint32_t ulpi_clk_hz(void);
/* return 0 on success, -1 on timeout (no ULPI clock / PHY not answering) */
int ulpi_read(uint8_t addr, uint8_t *val);
int ulpi_write(uint8_t addr, uint8_t val);
uint8_t ulpi_last_rxcmd(void);
/* ulpi_link debug levels: {st[2:0], reg_pend, drive_en, dir, nxt, rx_active, wr_nxt2, rx_pkts[6:0]} */
uint32_t ulpi_link_dbg(void);
