/* Built only when the SoC has the USB host (usbh_csr.h, scripts/gen_csr_header.py). */
#if __has_include("usbh_csr.h")
/* From usb-rtl-sdr-tv 42056a9 (apps/sdr/src). */
#include <zephyr/kernel.h>
#include <zephyr/sys/sys_io.h>
#include "usbh_csr.h"
#include "ulpi.h"

#define REG_STAT_DONE BIT(8)
#define TIMEOUT_US    1000

void ulpi_phy_reset(int on)
{
	sys_write32(on ? 1u : 0u, CSR_USBH_CTRL_ADDR);
}

uint32_t ulpi_clk_hz(void)
{
	return sys_read32(CSR_USBH_ULPI_FREQ_VALUE_ADDR);
}

uint32_t ulpi_link_dbg(void)
{
	return sys_read32(CSR_USBH_LINK_DBG_ADDR);
}

uint8_t ulpi_last_rxcmd(void)
{
	return (sys_read32(CSR_USBH_REG_STAT_ADDR) >> 16) & 0xff;
}

static int ulpi_access(uint8_t addr, int we, uint8_t wdata, uint8_t *rdata)
{
	sys_write32((addr & 0x3fu) | (we ? BIT(8) : 0u) | ((uint32_t)wdata << 16),
		    CSR_USBH_REG_CMD_ADDR);
	for (int i = 0; i < TIMEOUT_US; i++) {
		uint32_t st = sys_read32(CSR_USBH_REG_STAT_ADDR);

		if (st & REG_STAT_DONE) {
			if (rdata) {
				*rdata = st & 0xff;
			}
			return 0;
		}
		k_busy_wait(1);
	}
	return -1;
}

int ulpi_read(uint8_t addr, uint8_t *val)
{
	return ulpi_access(addr, 0, 0, val);
}

int ulpi_write(uint8_t addr, uint8_t val)
{
	return ulpi_access(addr, 1, val, NULL);
}
#endif
