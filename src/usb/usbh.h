/* From usb-rtl-sdr-tv 42056a9 (apps/sdr/src). */
/* USB host control (S2: bus reset with HS handshake, SOF). */
#pragma once
#include <stdint.h>

struct usbh_reset_report {
	int err;                /* 0 = reached HS with SOF running */
	uint8_t otg;            /* OTG Control read-back (DrvVbus = bit 5) */
	uint8_t linestate;      /* LineState before reset (1 = J = FS device attached) */
	uint32_t attach_ms;     /* when J was seen (ms after VBUS on), or the limit */
	uint8_t chirp_fires;    /* host chirps fired during this reset */
	uint8_t k_seen;         /* device chirp K detected */
	uint8_t func_hs;        /* Function Control read-back after reset (0x40 = HS) */
	uint8_t usb_int_status; /* PHY USB Interrupt Status after recovery */
	uint16_t sof_sent;      /* SOFs sent during the recovery time */
};

void usbh_sof_enable(int en, uint16_t frame);
uint16_t usbh_sof_frame(void);
uint16_t usbh_sof_sent(void);
uint32_t usbh_chirp_stat(void);
int usbh_bus_reset(struct usbh_reset_report *r);
/* VBUS off for ms (the device and a bus-powered hub lose power; the next bus reset turns it on) */
void usbh_vbus_off(int ms);

/*
 * The bulk IN engine (the usb_bulk_in pipeline block): a continuous bulk IN from endpoint
 * `ep` of device `addr`, its payload going into the pipeline (one byte per word), never
 * through the CPU. data1_first: the data toggle the device starts with (0 = DATA0).
 * After a NAK the engine waits USBH_BULK_NAK_GAP ULPI clocks before the next IN.
 */
#define USBH_BULK_NAK_GAP 6000
void usbh_bulk_in_start(uint8_t addr, uint8_t ep, int data1_first);
void usbh_bulk_in_stop(void);

/* the bulk IN engine's rolling counters */
struct usbh_bulk_in_stats {
	uint32_t packets;       /* data packets received */
	uint32_t naks;          /* NAKs from the device */
	uint32_t errors;        /* errs[15:0], duplicates[31:16] */
};
void usbh_bulk_in_stats(struct usbh_bulk_in_stats *s);

/* the host's state, for diagnosing a stopped stream (gateware/usbh.py) */
struct usbh_debug {
	uint32_t link;          /* link_dbg: ulpi_link levels (rtl/ulpi_link.v `dbg`) */
	uint32_t sof;           /* sof_stat: frame[10:0], SOFs sent[31:16] */
	uint32_t xfer;          /* xfer_stat: done, pid, flags, rx_len */
};
void usbh_debug(struct usbh_debug *d);
