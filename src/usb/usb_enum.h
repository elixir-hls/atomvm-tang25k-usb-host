/* From usb-rtl-sdr-tv 42056a9 (apps/sdr/src). */
/* Enumeration of what is on the root port (S3): descriptors, address, configuration.
 * Directly attached: the device gets address 1. Behind a USB 2.0 (high-speed) hub, the hub gets
 * address 1 and every connected port is reset and enumerated (usb_hub.c): the first high-speed
 * device (the RTL-SDR) gets address 2, the first full / low-speed one (the USB speaker) address
 * 3, reached through the hub's transaction translator with split transactions.
 * USB_DEV_ADDR is the address of the device the radio talks to. */
#pragma once
#include <stdint.h>
#include "usb_xfer.h"

#define USB_HUB_ADDR 1
#define USB_HS_ADDR  2
#define USB_FS_ADDR  3
extern uint8_t usb_dev_addr;
#define USB_DEV_ADDR usb_dev_addr

#define USB_FS_CFG_MAX 1024

struct usb_enum_info {
	int err;                  /* 0 = configured */
	int step;                 /* that failed: 1..6 first device, 11..15 hub, 21..26 HS device,
				   * 31..36 FS device (see usb_enum.c) */
	struct usb_ctrl_stats st; /* of the failing (or last) control transfer */
	uint8_t dev[18];          /* device descriptor (the RTL-SDR) */
	uint8_t cfg[64];          /* its configuration descriptor (first 64 bytes) */
	int cfg_len;
	int hub_port;             /* 0 = attached directly, else the hub port (1..) */
	int nports;               /* of the hub (0 = no hub) */
	uint16_t hub_vid, hub_pid;
	/* full / low-speed device behind the hub (fs_port 0 = none) */
	int fs_port, fs_low_speed;
	uint8_t fs_dev[18];
	uint8_t *fs_cfg;          /* USB_FS_CFG_MAX bytes, static */
	int fs_cfg_len;
};

/* enumerate what is on the root port (see above) */
int usb_enumerate(struct usb_enum_info *e);
/* enumerate the device answering at address 0 and give it `addr` (steps 1..6 + step_base);
 * dev[18], cfg[cfg_max] receive the descriptors */
int usb_enumerate_at(uint8_t addr, int step_base, uint8_t *dev, uint8_t *cfg, int cfg_max,
		     int *cfg_len, struct usb_enum_info *e);

/* reset hub port `port` and enumerate the full / low-speed device there at USB_FS_ADDR
 * (fills e->fs_*); 0 = done */
int usb_fs_attach(struct usb_enum_info *e, int port);
/* hub on `hub_addr` (configured): power the ports; returns the number of ports or < 0 */
int usb_hub_power(uint8_t hub_addr, struct usb_enum_info *e);
/* bitmap of the ports with a connection (bit p = port p), or < 0 */
int usb_hub_connected(uint8_t hub_addr, int nports, struct usb_enum_info *e);
/* reset one port; returns its speed (0 full, 1 low, 2 high) or < 0 */
int usb_hub_reset_port(uint8_t hub_addr, int port, struct usb_enum_info *e);
