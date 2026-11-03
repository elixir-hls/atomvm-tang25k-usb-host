/* From usb-rtl-sdr-tv 42056a9 (apps/sdr/src). */
/* USB transactions and control transfers on the usbh transaction engine (S3). */
#pragma once
#include <stddef.h>
#include <stdint.h>

#define USB_PID_ACK   0xd2
#define USB_PID_NAK   0x5a
#define USB_PID_STALL 0x1e
#define USB_PID_NYET  0x96
#define USB_PID_DATA0 0xc3
#define USB_PID_DATA1 0x4b
#define USB_PID_ERR   0x3c

enum usb_kind { USB_SETUP = 0, USB_OUT = 1, USB_IN = 2, USB_PING = 3 };

/* flags of one transaction (xfer_stat) */
#define XF_TIMEOUT BIT(0)
#define XF_ABORT   BIT(1)
#define XF_CRC     BIT(2)
#define XF_RXERR   BIT(3)
#define XF_OVF     BIT(4)
#define XF_PIDERR  BIT(5)

struct usb_xact_result {
	uint8_t pid;    /* received PID (0 = none) */
	uint8_t flags;  /* XF_* */
	uint16_t len;   /* IN payload bytes */
};

/* one transaction; OUT/SETUP data from `out`, IN data to `in` (up to in_max bytes) */
int usb_xact(enum usb_kind kind, uint8_t addr, uint8_t endp, int data1, const void *out,
	     size_t len, void *in, size_t in_max, struct usb_xact_result *r);

struct usb_ctrl_stats {
	uint32_t naks;       /* NAKs seen (retried) */
	uint32_t errors;     /* timeouts / CRC errors retried */
	uint8_t last_pid;    /* PID of the failing step, for diagnostics */
	uint8_t last_flags;  /* XF_* of the failing step */
	uint8_t stage;       /* 1 = SETUP, 2 = DATA, 3 = STATUS */
};

/*
 * Per-address route: EP0 max packet size, and for a full / low-speed device behind a
 * high-speed hub the hub address and port (transactions then go as split transactions).
 * Address 0 has a route too (set while that device is being enumerated).
 */
void usb_route_set(uint8_t addr, uint8_t mps0, uint8_t hub, uint8_t port, int low_speed);
/* one transaction with explicit SPLIT fields (bring-up) */
int usb_xact_raw(enum usb_kind kind, uint8_t addr, uint8_t endp, int data1, const void *out,
		 size_t len, void *in, size_t in_max, struct usb_xact_result *r, uint32_t split);
/* SPLIT token fields for the CSR (usb_xact.v); 0 = no split */
uint32_t usb_split_bits(uint8_t hub, uint8_t port, int sc, int s, int e, int et, int nohs);

/* control transfers on endpoint 0 (max packet from the route, default 64). Return bytes
 * transferred or < 0. */
int usbh_ctrl_in(uint8_t addr, const uint8_t setup[8], void *buf, size_t len,
		struct usb_ctrl_stats *st);
int usbh_ctrl_out(uint8_t addr, const uint8_t setup[8], const void *buf, size_t len,
		 struct usb_ctrl_stats *st);
