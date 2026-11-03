/* Built only when the SoC has the USB host (usbh_csr.h, scripts/gen_csr_header.py). */
#if __has_include("usbh_csr.h")
/* From usb-rtl-sdr-tv 42056a9 (apps/sdr/src). */
/*
 * USB host bring-up sequence (S2). Same steps as usb-rtl-sdr firmware/hs_min.c 0-5:
 *   0. stop SOF (a HS device returns to FS only after >3 ms of bus idle)
 *   1. VBUS on (OTG Control: DrvVbus + D+/D- pull-downs), FS termination
 *   2. look for an FS attach (LineState J) after >= 4 ms, then debounce 200 ms (TATTDB)
 *   3. arm the HW chirp, drive SE0 for 15 ms (bus reset); the device answers with chirp K
 *      and the HW fires the host K-J chirp; disarm 400 us before the end of reset
 *   4. HS termination and SOF right away (nothing in between: 3 ms idle = suspend)
 *   5. recovery 12 ms (TRSTRCY >= 10 ms) with SOF running
 */
#include <zephyr/kernel.h>
#include <zephyr/sys/sys_io.h>
#include "usbh_csr.h"
#include "ulpi.h"
#include "usbh.h"

#define PHY_FUNC_CTRL      0x04
#define PHY_OTG_CTRL       0x0a
#define PHY_USB_INT_STATUS 0x13
#define PHY_DEBUG          0x15

#define FUNC_ATTACH 0x45 /* XcvrSel=01 (FS) TermSelect=1 OpMode=00 SuspendM=1 */
#define FUNC_SE0    0x50 /* XcvrSel=00 TermSelect=0 OpMode=10 (chirp): drives SE0 */
#define FUNC_HS     0x40 /* XcvrSel=00 TermSelect=0 OpMode=00 SuspendM=1 */
#define OTG_DRVVBUS 0x26 /* DrvVbus + DmPulldown + DpPulldown */
#define OTG_NOVBUS  0x06 /* pull-downs only: VBUS off */
#define VBUS_OFF_MS 300

#define CHIRP_SEG   3000 /* one K/J phase, ULPI clocks (50 us) */
#define CHIRP_PAIRS 3

#define ATTACH_MS_MAX      100
#define QUIET_MS_MIN       4
#define ATTACH_DEBOUNCE_MS 200
#define RESET_US           15000
#define CHIRP_STOP_US      400
#define RECOVERY_MS        12

void usbh_sof_enable(int en, uint16_t frame)
{
	if (en) {
		sys_write32(frame & 0x7ff, CSR_USBH_SOF_FRAME_ADDR);
		k_busy_wait(2); /* the frame load crosses to the ULPI clock */
	}
	sys_write32(en ? 1u : 0u, CSR_USBH_SOF_CTRL_ADDR);
}

uint16_t usbh_sof_frame(void)
{
	return sys_read32(CSR_USBH_SOF_STAT_ADDR) & 0x7ff;
}

uint16_t usbh_sof_sent(void)
{
	return sys_read32(CSR_USBH_SOF_STAT_ADDR) >> 16;
}

uint32_t usbh_chirp_stat(void)
{
	return sys_read32(CSR_USBH_CHIRP_STAT_ADDR);
}

static void chirp_arm(int arm)
{
	sys_write32((arm ? 1u : 0u) | (CHIRP_PAIRS << 4) | ((uint32_t)CHIRP_SEG << 16),
		    CSR_USBH_CHIRP_CTRL_ADDR);
}

static int bus_reset_once(struct usbh_reset_report *r)
{
	uint8_t v = 0;
	uint32_t ms, fires0;
	uint16_t sent0;

	*r = (struct usbh_reset_report){0};

	/* 0. quiet bus */
	usbh_sof_enable(0, 0);
	chirp_arm(0);

	/* 1. VBUS on, FS termination */
	if (ulpi_write(PHY_FUNC_CTRL, FUNC_ATTACH) || ulpi_write(PHY_OTG_CTRL, OTG_DRVVBUS) ||
	    ulpi_read(PHY_OTG_CTRL, &r->otg)) {
		return r->err = -1; /* PHY not answering */
	}
	if (!(r->otg & 0x20)) {
		return r->err = -2; /* DrvVbus did not stick */
	}

	/* 2. FS attach (J) — observed, not required: a device left in HS shows SE0 here */
	for (ms = 0; ms < ATTACH_MS_MAX; ms++) {
		if (ulpi_read(PHY_DEBUG, &v) == 0) {
			r->linestate = v & 3;
			if (r->linestate == 1 && ms >= QUIET_MS_MIN) {
				break;
			}
		}
		k_msleep(1);
	}
	r->attach_ms = ms;
	k_msleep(ATTACH_DEBOUNCE_MS);

	/* 3. bus reset with the HS handshake */
	fires0 = usbh_chirp_stat() & 0xff;
	chirp_arm(1);
	ulpi_write(PHY_FUNC_CTRL, FUNC_SE0);
	k_busy_wait(RESET_US - CHIRP_STOP_US);
	chirp_arm(0);
	k_busy_wait(CHIRP_STOP_US);

	/* 4. HS + SOF immediately */
	ulpi_write(PHY_FUNC_CTRL, FUNC_HS);
	usbh_sof_enable(1, 0);

	/* 5. recovery with SOF running */
	sent0 = usbh_sof_sent();
	k_msleep(RECOVERY_MS);
	r->chirp_fires = (usbh_chirp_stat() & 0xff) - fires0;
	r->k_seen = (usbh_chirp_stat() >> 8) & 1;
	ulpi_read(PHY_FUNC_CTRL, &r->func_hs);
	ulpi_read(PHY_USB_INT_STATUS, &r->usb_int_status);
	r->sof_sent = usbh_sof_sent() - sent0;

	if (r->chirp_fires == 0) {
		return r->err = -3; /* no device chirp K (FS-only device, or none attached) */
	}
	if (r->func_hs != FUNC_HS) {
		return r->err = -4;
	}
	if (r->sof_sent < RECOVERY_MS * 8 - 8) {
		return r->err = -5; /* SOF not running */
	}
	return 0;
}

/*
 * A device (seen with a self-powered hub) can stop answering the reset with a chirp; it comes
 * back once VBUS has been off: then the reset is tried once more after VBUS_OFF_MS without VBUS.
 */
int usbh_bus_reset(struct usbh_reset_report *r)
{
	if (bus_reset_once(r) != -3) {
		return r->err;
	}
	usbh_sof_enable(0, 0);
	ulpi_write(PHY_OTG_CTRL, OTG_NOVBUS);
	k_msleep(VBUS_OFF_MS);
	return bus_reset_once(r);
}

void usbh_vbus_off(int ms)
{
	usbh_sof_enable(0, 0);
	ulpi_write(PHY_OTG_CTRL, OTG_NOVBUS);
	k_msleep(ms);
}

/* bulk_ctrl: en[0], init_toggle[1], endp[7:4], addr[14:8], nak_gap[31:16] (gateware/usbh.py);
 * the fields of the last start, kept for the stop */
static uint32_t bulk_cfg;

void usbh_bulk_in_start(uint8_t addr, uint8_t ep, int data1_first)
{
	bulk_cfg = (data1_first ? BIT(1) : 0u) | ((uint32_t)(ep & 0xf) << 4) |
		   ((uint32_t)(addr & 0x7f) << 8) | ((uint32_t)USBH_BULK_NAK_GAP << 16);
	/* the fields first, then en (the engine takes them when en rises) */
	sys_write32(bulk_cfg, CSR_USBH_BULK_CTRL_ADDR);
	sys_write32(bulk_cfg | BIT(0), CSR_USBH_BULK_CTRL_ADDR);
}

void usbh_bulk_in_stop(void)
{
	sys_write32(bulk_cfg, CSR_USBH_BULK_CTRL_ADDR);
}

void usbh_bulk_in_stats(struct usbh_bulk_in_stats *s)
{
	s->packets = sys_read32(CSR_USBH_BULK_PKTS_ADDR);
	s->naks = sys_read32(CSR_USBH_BULK_NAKS_ADDR);
	s->errors = sys_read32(CSR_USBH_BULK_ERR_ADDR);
}

void usbh_debug(struct usbh_debug *d)
{
	d->link = sys_read32(CSR_USBH_LINK_DBG_ADDR);
	d->sof = sys_read32(CSR_USBH_SOF_STAT_ADDR);
	d->xfer = sys_read32(CSR_USBH_XFER_STAT_ADDR);
}
#endif
