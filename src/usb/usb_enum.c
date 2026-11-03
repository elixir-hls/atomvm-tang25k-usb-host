/* Built only when the SoC has the USB host (usbh_csr.h, scripts/gen_csr_header.py). */
#if __has_include("usbh_csr.h")
/* From usb-rtl-sdr-tv 42056a9 (apps/sdr/src). */
/*
 * Standard enumeration (USB 2.0 §9.1.2), same order as usb-rtl-sdr firmware/hs_min.c step 6:
 *   1 GET_DESCRIPTOR(DEVICE, 64) at address 0
 *   2 SET_ADDRESS(addr), then 2 ms (TDSETADDR) before the next request
 *   3 GET_DESCRIPTOR(DEVICE, 18) at the new address
 *   4 GET_DESCRIPTOR(CONFIGURATION, 9)
 *   5 GET_DESCRIPTOR(CONFIGURATION, wTotalLength)
 *   6 SET_CONFIGURATION(1)
 * If that is a hub (class 9): power its ports (11..13), then for each connected port a reset
 * (14..15) and the same steps for the device there: 21..26 for the first high-speed one
 * (address 2), 31..36 for the first full / low-speed one (address 3, split transactions).
 */
#include <errno.h>
#include <string.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include "usb_enum.h"

static void setup_pkt(uint8_t s[8], uint8_t bm, uint8_t req, uint16_t val, uint16_t idx,
		      uint16_t len)
{
	s[0] = bm; s[1] = req;
	s[2] = val & 0xff; s[3] = val >> 8;
	s[4] = idx & 0xff; s[5] = idx >> 8;
	s[6] = len & 0xff; s[7] = len >> 8;
}

uint8_t usb_dev_addr = USB_HUB_ADDR;
static uint8_t fs_cfg_buf[USB_FS_CFG_MAX];

int usb_enumerate_at(uint8_t addr, int step_base, uint8_t *dev, uint8_t *cfg, int cfg_max,
		     int *cfg_len, struct usb_enum_info *e)
{
	uint8_t s[8], tmp[64];
	int n;

	e->step = step_base + 1;
	setup_pkt(s, 0x80, 6, 0x0100, 0, 64);
	n = usbh_ctrl_in(0, s, tmp, 64, &e->st);
	if (n < 8) {
		return e->err = n < 0 ? n : -EPROTO;
	}

	e->step = step_base + 2;
	setup_pkt(s, 0x00, 5, addr, 0, 0);
	/* no error check here: when only the status stage got lost, the device already moved to
	 * `addr` and the retries at address 0 time out; step 3 tells whether it answers there */
	usbh_ctrl_out(0, s, NULL, 0, &e->st);
	k_msleep(10);                       /* TDSETADDR 2 ms, with margin (behind a hub) */

	e->step = step_base + 3;
	setup_pkt(s, 0x80, 6, 0x0100, 0, 18);
	n = usbh_ctrl_in(addr, s, dev, 18, &e->st);
	if (n != 18) {
		return e->err = n < 0 ? n : -EPROTO;
	}

	e->step = step_base + 4;
	setup_pkt(s, 0x80, 6, 0x0200, 0, 9);
	n = usbh_ctrl_in(addr, s, cfg, 9, &e->st);
	if (n != 9) {
		return e->err = n < 0 ? n : -EPROTO;
	}

	e->step = step_base + 5;
	uint16_t total = MIN(cfg[2] | (cfg[3] << 8), cfg_max);

	setup_pkt(s, 0x80, 6, 0x0200, 0, total);
	n = usbh_ctrl_in(addr, s, cfg, total, &e->st);
	if (n < 9) {
		return e->err = n < 0 ? n : -EPROTO;
	}
	*cfg_len = n;

	e->step = step_base + 6;
	setup_pkt(s, 0x00, 9, cfg[5], 0, 0);
	n = usbh_ctrl_out(addr, s, NULL, 0, &e->st);
	if (n < 0) {
		return e->err = n;
	}
	return 0;
}

int usb_fs_attach(struct usb_enum_info *e, int port)
{
	int speed = usb_hub_reset_port(USB_HUB_ADDR, port, e);

	if (speed < 0 || speed == 2) {
		return speed < 0 ? speed : -ENOTSUP;
	}
	/* both address 0 and the new one go through the hub's TT from the start */
	usb_route_set(0, 8, USB_HUB_ADDR, port, speed == 1);
	usb_route_set(USB_FS_ADDR, 8, USB_HUB_ADDR, port, speed == 1);
	if (usb_enumerate_at(USB_FS_ADDR, 30, e->fs_dev, e->fs_cfg, USB_FS_CFG_MAX, &e->fs_cfg_len,
			     e)) {
		usb_route_set(0, 0, 0, 0, 0);
		return e->err;
	}
	usb_route_set(0, 0, 0, 0, 0);
	usb_route_set(USB_FS_ADDR, e->fs_dev[7], USB_HUB_ADDR, port, speed == 1);
	e->fs_port = port;
	e->fs_low_speed = speed == 1;
	return 0;
}

int usb_enumerate(struct usb_enum_info *e)
{
	int nports, conn, hs_done = 0;

	memset(e, 0, sizeof(*e));
	e->fs_cfg = fs_cfg_buf;
	usb_dev_addr = USB_HUB_ADDR;
	for (int a = 0; a <= USB_FS_ADDR; a++) {
		usb_route_set(a, 0, 0, 0, 0);
	}
	if (usb_enumerate_at(USB_HUB_ADDR, 0, e->dev, e->cfg, sizeof(e->cfg), &e->cfg_len, e)) {
		return e->err;
	}
	usb_route_set(USB_HUB_ADDR, e->dev[7], 0, 0, 0);
	if (e->dev[4] != 0x09) {             /* not a hub: the device itself */
		e->step = 0;
		return 0;
	}
	e->hub_vid = e->dev[8] | (e->dev[9] << 8);
	e->hub_pid = e->dev[10] | (e->dev[11] << 8);
	nports = usb_hub_power(USB_HUB_ADDR, e);
	if (nports < 0) {
		return e->err = nports;
	}
	e->nports = nports;
	/* wait (up to 1 s) for the devices to connect */
	e->step = 13;
	for (int ms = 0, prev = -1; ms <= 1000; ms += 100) {
		conn = usb_hub_connected(USB_HUB_ADDR, nports, e);
		if (conn < 0) {
			return e->err = conn;
		}
		if (conn && conn == prev) {
			break;                      /* the same ports twice in a row: settled */
		}
		prev = conn;
		k_msleep(100);
	}
	printk("usb: hub %04x:%04x, %d ports, connected 0x%02x\n", e->hub_vid, e->hub_pid, nports,
	       conn);
	for (int p = 1; p <= nports; p++) {
		int speed;

		if (!(conn & BIT(p)) || (hs_done && e->fs_port)) {
			continue;
		}
		speed = usb_hub_reset_port(USB_HUB_ADDR, p, e);
		printk("usb: hub port %d: %s\n", p, speed == 2 ? "high speed" : speed == 1 ? "low speed" :
		       speed == 0 ? "full speed" : "reset failed");
		if (speed < 0) {
			continue;
		}
		if (speed == 2 && !hs_done) {
			usb_route_set(0, 0, 0, 0, 0);
			if (usb_enumerate_at(USB_HS_ADDR, 20, e->dev, e->cfg, sizeof(e->cfg),
					     &e->cfg_len, e) == 0) {
				usb_route_set(USB_HS_ADDR, e->dev[7], 0, 0, 0);
				e->hub_port = p;
				hs_done = 1;
			}
		} else if (speed < 2 && !e->fs_port) {
			/* both address 0 and the new one go through the hub's TT from the start */
			usb_route_set(0, 8, USB_HUB_ADDR, p, speed == 1);
			usb_route_set(USB_FS_ADDR, 8, USB_HUB_ADDR, p, speed == 1);
			if (usb_enumerate_at(USB_FS_ADDR, 30, e->fs_dev, e->fs_cfg, USB_FS_CFG_MAX,
					     &e->fs_cfg_len, e) == 0) {
				usb_route_set(USB_FS_ADDR, e->fs_dev[7], USB_HUB_ADDR, p, speed == 1);
				e->fs_port = p;
				e->fs_low_speed = speed == 1;
			} else {
				printk("usb: port %d device failed at step %d (stage %u, PID 0x%02x, "
				       "flags 0x%02x, NAKs %u, errors %u): %d\n", p, e->step, e->st.stage,
				       e->st.last_pid, e->st.last_flags, e->st.naks, e->st.errors, e->err);
			}
		}
	}
	usb_route_set(0, 0, 0, 0, 0);
	if (!hs_done) {
		return e->err = -ENODEV;            /* no high-speed device (no RTL-SDR) */
	}
	usb_dev_addr = USB_HS_ADDR;
	e->err = 0;
	e->step = 0;
	return 0;
}
#endif
