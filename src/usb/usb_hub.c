/* Built only when the SoC has the USB host (usbh_csr.h, scripts/gen_csr_header.py). */
#if __has_include("usbh_csr.h")
/* From usb-rtl-sdr-tv 42056a9 (apps/sdr/src). */
/*
 * USB 2.0 hub between the host and the devices (USB 2.0 §11.24). A high-speed device behind it
 * is reached directly by its address (the hub repeats the packets); a full / low-speed one
 * through the hub's transaction translator with split transactions (usb_xfer.c routes).
 *
 *   11 GET_DESCRIPTOR(HUB): number of ports, power-on to power-good time
 *   12 SET_PORT_FEATURE(PORT_POWER) on every port, wait power-good + 100 ms (connect debounce)
 *   13 GET_PORT_STATUS: which ports have a connection
 *   14 SET_PORT_FEATURE(PORT_RESET), wait for C_PORT_RESET (the hub does the HS handshake),
 *      clear C_PORT_RESET / C_PORT_CONNECTION
 *   15 the port must be enabled; its speed comes from the status (high / low / full)
 * Then 50 ms reset recovery; the device answers at address 0.
 */
#include <errno.h>
#include <zephyr/kernel.h>
#include "usb_enum.h"

#define REQ_GET_STATUS     0
#define REQ_CLEAR_FEATURE  1
#define REQ_SET_FEATURE    3
#define REQ_GET_DESCRIPTOR 6

#define PORT_RESET         4
#define PORT_POWER         8
#define C_PORT_CONNECTION  16
#define C_PORT_RESET       20

#define PS_CONNECTION      BIT(0)
#define PS_ENABLE          BIT(1)
#define PS_RESET           BIT(4)
#define PS_LOW_SPEED       BIT(9)
#define PS_HIGH_SPEED      BIT(10)
#define PC_RESET           BIT(4)

static void setup(uint8_t s[8], uint8_t bm, uint8_t req, uint16_t val, uint16_t idx, uint16_t len)
{
	s[0] = bm; s[1] = req;
	s[2] = val & 0xff; s[3] = val >> 8;
	s[4] = idx & 0xff; s[5] = idx >> 8;
	s[6] = len & 0xff; s[7] = len >> 8;
}

static int port_feature(uint8_t hub, int set, uint16_t feature, int port, struct usb_enum_info *e)
{
	uint8_t s[8];

	setup(s, 0x23, set ? REQ_SET_FEATURE : REQ_CLEAR_FEATURE, feature, port, 0);
	return usbh_ctrl_out(hub, s, NULL, 0, &e->st);
}

/* wPortStatus | wPortChange << 16, or < 0 */
static int port_status(uint8_t hub, int port, struct usb_enum_info *e)
{
	uint8_t s[8], d[4];
	int n;

	setup(s, 0xa3, REQ_GET_STATUS, 0, port, 4);
	n = usbh_ctrl_in(hub, s, d, 4, &e->st);
	if (n != 4) {
		return n < 0 ? n : -EPROTO;
	}
	return d[0] | (d[1] << 8) | (d[2] << 16) | (d[3] << 24);
}

int usb_hub_reset_port(uint8_t hub, int port, struct usb_enum_info *e)
{
	int st = 0;

	e->step = 14;
	if (port_feature(hub, 1, PORT_RESET, port, e) < 0) {
		return -EIO;
	}
	for (int ms = 0; ms < 200; ms += 10) {
		k_msleep(10);
		st = port_status(hub, port, e);
		if (st < 0) {
			return st;
		}
		if (!(st & PS_RESET) && ((st >> 16) & PC_RESET)) {
			break;
		}
	}
	port_feature(hub, 0, C_PORT_RESET, port, e);
	port_feature(hub, 0, C_PORT_CONNECTION, port, e);
	e->step = 15;
	if (!(st & PS_ENABLE)) {
		return -ENODEV;
	}
	k_msleep(50);                               /* TRSTRCY (10 ms) with margin */
	return (st & PS_HIGH_SPEED) ? 2 : (st & PS_LOW_SPEED) ? 1 : 0;
}

int usb_hub_power(uint8_t hub, struct usb_enum_info *e)
{
	uint8_t s[8], d[16];
	int n, nports, pwr_ms;

	e->step = 11;
	setup(s, 0xa0, REQ_GET_DESCRIPTOR, 0x2900, 0, sizeof(d));
	n = usbh_ctrl_in(hub, s, d, sizeof(d), &e->st);
	if (n < 7) {
		return n < 0 ? n : -EPROTO;
	}
	nports = MIN(d[2], 15);
	pwr_ms = d[5] * 2;

	e->step = 12;
	for (int p = 1; p <= nports; p++) {
		if (port_feature(hub, 1, PORT_POWER, p, e) < 0) {
			return -EIO;
		}
	}
	k_msleep(pwr_ms + 100);
	return nports;
}

int usb_hub_connected(uint8_t hub, int nports, struct usb_enum_info *e)
{
	int map = 0;

	e->step = 13;
	for (int p = 1; p <= nports; p++) {
		int st = port_status(hub, p, e);

		if (st < 0) {
			return st;
		}
		if (st & PS_CONNECTION) {
			map |= BIT(p);
		}
	}
	return map;
}
#endif
