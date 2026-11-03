/* Built only when the SoC has the USB host (usbh_csr.h, scripts/gen_csr_header.py). */
#if __has_include("usbh_csr.h")
/* From usb-rtl-sdr-tv 42056a9 (apps/sdr/src). */
/*
 * USB transactions (usbh transaction engine) and control transfers (S3).
 * The engine runs one token + data + handshake with all the deadlines in hardware; the
 * sequencing, data toggles and NAK retries are here.
 */
#include <errno.h>
#include <string.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/sys_io.h>
#include "usbh_csr.h"
#include "usb_xfer.h"

#define BUF_TX ((volatile uint32_t *)(MEM_USBH_BUF_BASE + 0x000))
#define BUF_RX ((volatile uint32_t *)(MEM_USBH_BUF_BASE + 0x800))
#define EP0_MPS_DEFAULT 64
#define SPLIT_TRIES    3000   /* complete-split NYETs before giving up (~100 ms) */

struct route {
	uint8_t mps0;
	uint8_t hub, port, ls;     /* hub = 0: no split */
};
static struct route routes[128];

void usb_route_set(uint8_t addr, uint8_t mps0, uint8_t hub, uint8_t port, int low_speed)
{
	routes[addr & 0x7f] = (struct route){ mps0, hub, port, low_speed ? 1 : 0 };
}

static int ep0_mps(uint8_t addr)
{
	return routes[addr & 0x7f].mps0 ? routes[addr & 0x7f].mps0 : EP0_MPS_DEFAULT;
}

uint32_t usb_split_bits(uint8_t hub, uint8_t port, int sc, int s, int e, int et, int nohs)
{
	return 1u | (sc ? BIT(1) : 0) | ((uint32_t)(hub & 0x7f) << 4) | ((uint32_t)(port & 0x7f) << 11) |
	       (s ? BIT(18) : 0) | (e ? BIT(19) : 0) | ((uint32_t)(et & 3) << 20) | (nohs ? BIT(22) : 0);
}
#define XACT_WAIT_US   1000   /* one transaction: < 2 microframes */
#define NAK_LIMIT_MS   500    /* control transfer: give up after NAKs for this long */
#define ERR_RETRIES    3

int usb_xact_raw(enum usb_kind kind, uint8_t addr, uint8_t endp, int data1, const void *out,
		    size_t len, void *in, size_t in_max, struct usb_xact_result *r, uint32_t split)
{
	if (len > 512) {
		return -EINVAL;
	}
	if (out && len) {
		const uint8_t *p = out;

		for (size_t i = 0; i < len; i += 4) {
			uint32_t w = 0;

			for (size_t b = 0; b < 4 && i + b < len; b++) {
				w |= (uint32_t)p[i + b] << (8 * b);
			}
			BUF_TX[i / 4] = w;
		}
	}
	sys_write32(split, CSR_USBH_XFER_SPLIT_ADDR);
	sys_write32((kind & 3) | (data1 ? BIT(2) : 0) | ((endp & 0xf) << 4) |
		    ((uint32_t)(addr & 0x7f) << 8) | ((uint32_t)len << 16),
		    CSR_USBH_XFER_CMD_ADDR);
	for (int t = 0; t < XACT_WAIT_US; t++) {
		uint32_t st = sys_read32(CSR_USBH_XFER_STAT_ADDR);

		if (st & 1) {
			r->pid = (st >> 8) & 0xff;
			r->flags = (st >> 16) & 0x3f;
			r->len = st >> 22;
			if (kind == USB_IN && in && r->len) {
				uint8_t *q = in;
				size_t n = MIN((size_t)r->len, in_max);

				for (size_t i = 0; i < n; i += 4) {
					uint32_t w = BUF_RX[i / 4];

					for (size_t b = 0; b < 4 && i + b < n; b++) {
						q[i + b] = w >> (8 * b);
					}
				}
			}
			return 0;
		}
		k_busy_wait(1);
	}
	return -ETIMEDOUT; /* engine did not finish (no ULPI clock?) */
}

/*
 * One control / bulk / interrupt transaction to a device behind a high-speed hub (USB 2.0
 * §11.17): start-split (the hub's TT takes it: ACK, or NAK when its buffer is busy), then
 * complete-splits until the TT has the device's answer (NYET = not yet). The result looks like
 * the direct transaction's: ACK / NAK / STALL / DATAx (ERR from the TT comes back as a timeout,
 * so it is retried like one).
 */
static int xact_split(enum usb_kind kind, const struct route *rt, uint8_t addr, uint8_t endp,
		      int data1, const void *out, size_t len, void *in, size_t in_max,
		      struct usb_xact_result *r, int et)
{
	int err = usb_xact_raw(kind, addr, endp, data1, out, len, NULL, 0, r,
			   usb_split_bits(rt->hub, rt->port, 0, rt->ls, 0, et, 0));

	if (err || r->flags || r->pid != USB_PID_ACK) {
		return err;                         /* NAK / error: the caller retries */
	}
	for (int i = 0; i < SPLIT_TRIES; i++) {
		err = usb_xact_raw(kind, addr, endp, data1, NULL, 0, in, in_max, r,
			       usb_split_bits(rt->hub, rt->port, 1, rt->ls, 0, et, 0));
		if (err) {
			return err;
		}
		if (r->flags || r->pid != USB_PID_NYET) {
			if (r->pid == USB_PID_ERR) {
				r->flags |= XF_TIMEOUT;
			}
			return 0;
		}
		k_busy_wait(30);
	}
	r->flags |= XF_TIMEOUT;
	return 0;
}

int usb_xact(enum usb_kind kind, uint8_t addr, uint8_t endp, int data1, const void *out,
	     size_t len, void *in, size_t in_max, struct usb_xact_result *r)
{
	const struct route *rt = &routes[addr & 0x7f];

	if (rt->hub) {
		return xact_split(kind, rt, addr, endp, data1, out, len, in, in_max, r,
				  endp ? 2 : 0);   /* ET: control on EP0, else bulk */
	}
	return usb_xact_raw(kind, addr, endp, data1, out, len, in, in_max, r, 0);
}

/* repeat one transaction until it is answered (NAK: retry for NAK_LIMIT_MS) */
static int xact_retry(enum usb_kind kind, uint8_t addr, int data1, const void *out, size_t len,
		      void *in, size_t in_max, struct usb_xact_result *r, struct usb_ctrl_stats *st)
{
	int64_t end = k_uptime_get() + NAK_LIMIT_MS;
	int errs = 0;

	for (;;) {
		int err = usb_xact(kind, addr, 0, data1, out, len, in, in_max, r);

		if (err) {
			return err;
		}
		st->last_pid = r->pid;
		st->last_flags = r->flags;
		if (r->flags & (XF_TIMEOUT | XF_CRC | XF_RXERR | XF_PIDERR | XF_ABORT)) {
			st->errors++;
			if (++errs > ERR_RETRIES) {
				return -EIO;
			}
			continue;
		}
		if (r->pid == USB_PID_NAK) {
			st->naks++;
			if (k_uptime_get() > end) {
				return -EAGAIN;
			}
			continue;
		}
		if (r->pid == USB_PID_STALL) {
			return -EPIPE;
		}
		return 0;
	}
}

static int setup_stage(uint8_t addr, const uint8_t setup[8], struct usb_ctrl_stats *st)
{
	struct usb_xact_result r;
	int err;

	st->stage = 1;
	err = xact_retry(USB_SETUP, addr, 0, setup, 8, NULL, 0, &r, st);
	if (!err && r.pid != USB_PID_ACK) {
		err = -EPROTO;
	}
	return err;
}

static int ctrl_in_once(uint8_t addr, const uint8_t setup[8], void *buf, size_t len,
			struct usb_ctrl_stats *st)
{
	struct usb_xact_result r;
	uint8_t *p = buf;
	size_t got = 0;
	int toggle = 1, err;

	memset(st, 0, sizeof(*st));
	err = setup_stage(addr, setup, st);
	if (err) {
		return err;
	}
	st->stage = 2;
	while (got < len) {
		err = xact_retry(USB_IN, addr, 0, NULL, 0, p + got, len - got, &r, st);
		if (err == -EPIPE && got > 0) {
			break;      /* no ZLP after a full last packet (Philips DSS330): STALL ends it */
		}
		if (err) {
			return err;
		}
		if (r.pid != (toggle ? USB_PID_DATA1 : USB_PID_DATA0)) {
			continue; /* repeated packet (our ACK was lost): already have it */
		}
		toggle ^= 1;
		got += MIN((size_t)r.len, len - got);
		if (r.len < ep0_mps(addr)) {
			break; /* short packet ends the data stage */
		}
	}
	st->stage = 3; /* status: OUT ZLP DATA1 */
	err = xact_retry(USB_OUT, addr, 1, NULL, 0, NULL, 0, &r, st);
	if (!err && r.pid != USB_PID_ACK && r.pid != USB_PID_NYET) {
		err = -EPROTO;
	}
	return err ? err : (int)got;
}

static int ctrl_out_once(uint8_t addr, const uint8_t setup[8], const void *buf, size_t len,
			 struct usb_ctrl_stats *st)
{
	struct usb_xact_result r;
	const uint8_t *p = buf;
	size_t sent = 0;
	int toggle = 1, err;

	memset(st, 0, sizeof(*st));
	err = setup_stage(addr, setup, st);
	if (err) {
		return err;
	}
	st->stage = 2;
	while (sent < len) {
		size_t n = MIN(len - sent, (size_t)ep0_mps(addr));

		err = xact_retry(USB_OUT, addr, toggle, p + sent, n, NULL, 0, &r, st);
		if (err) {
			return err;
		}
		/* ACK or NYET: accepted (NYET = no room for the next one yet; PING before it) */
		if (r.pid != USB_PID_ACK && r.pid != USB_PID_NYET) {
			return -EPROTO;
		}
		if (r.pid == USB_PID_NYET) {
			do {
				err = xact_retry(USB_PING, addr, 0, NULL, 0, NULL, 0, &r, st);
			} while (!err && r.pid != USB_PID_ACK);
			if (err) {
				return err;
			}
		}
		toggle ^= 1;
		sent += n;
	}
	st->stage = 3; /* status: IN ZLP DATA1 */
	err = xact_retry(USB_IN, addr, 0, NULL, 0, NULL, 0, &r, st);
	if (!err && r.pid != USB_PID_DATA1) {
		err = -EPROTO;
	}
	return err ? err : (int)sent;
}

/*
 * Through a hub (FE1.1s), about one enumeration in four saw a STALL in the data stage of a
 * standard request, or a status stage that timed out, and the request worked on the next try.
 * A STALL on EP0 ends with the next SETUP, so the whole transfer is repeated (CTRL_TRIES times
 * in all) after a STALL or an error (-EIO: timeouts / CRC errors past the retries).
 */
#define CTRL_TRIES 3

int usbh_ctrl_in(uint8_t addr, const uint8_t setup[8], void *buf, size_t len,
		struct usb_ctrl_stats *st)
{
	int n = -EPIPE;

	for (int i = 0; i < CTRL_TRIES && (n == -EPIPE || n == -EIO); i++) {
		n = ctrl_in_once(addr, setup, buf, len, st);
	}
	return n;
}

int usbh_ctrl_out(uint8_t addr, const uint8_t setup[8], const void *buf, size_t len,
		 struct usb_ctrl_stats *st)
{
	int n = -EPIPE;

	for (int i = 0; i < CTRL_TRIES && (n == -EPIPE || n == -EIO); i++) {
		n = ctrl_out_once(addr, setup, buf, len, st);
	}
	return n;
}
#endif
