/* Built only when the SoC has the USB host (usbh_csr.h, written from its csr.json). */
#if __has_include("usbh_csr.h")
/*
 * The USB host for Elixir: NIFs over the C library in src/usb/ (lib/usb_host.erl has the
 * stubs and the documentation). A device driver written in Elixir sets its device up with
 * these and starts the bulk IN engine, whose payload goes into the pipeline's usb_bulk_in().
 *
 *   usb_host:bus_reset() -> ok | {error, Err}
 *   usb_host:phy_reset() -> ok
 *   usb_host:enumerate() -> {ok, Addr, DeviceDescriptor} | {error, Step}
 *   usb_host:ctrl_in(Addr, Setup, Len) -> {ok, Data} | {error, {Err, Stage, Pid}}
 *   usb_host:ctrl_out(Addr, Setup, Data) -> ok | {error, {Err, Stage, Pid}}
 *   usb_host:bulk_in_start(Addr, Endpoint, Data1First) -> ok
 *   usb_host:bulk_in_stop() -> ok
 *   usb_host:bulk_in_stats() -> {Packets, Naks, Errors}
 *   usb_host:debug() -> {LinkDbg, SofStat, XferStat, Phy, RxCmd}
 */
#include <stdint.h>
#include <string.h>

#include <defaultatoms.h>
#include <memory.h>
#include <nifs.h>
#include <portnifloader.h>
#include <term.h>

#include <zephyr/kernel.h>

#include "ulpi.h"
#include "usb_enum.h"
#include "usb_xfer.h"
#include "usbh.h"

#define CTRL_MAX 256                /* the longest control transfer data stage taken */

static struct usb_enum_info enum_info;

static term error_tuple(Context *ctx, term reason)
{
    term t = term_alloc_tuple(2, &ctx->heap);
    term_put_tuple_element(t, 0, ERROR_ATOM);
    term_put_tuple_element(t, 1, reason);
    return t;
}

/* {error, {Err, Stage, Pid}} of a failed control transfer */
static term ctrl_error(Context *ctx, int err, const struct usb_ctrl_stats *st)
{
    if (UNLIKELY(memory_ensure_free(ctx, TUPLE_SIZE(3) + TUPLE_SIZE(2)) != MEMORY_GC_OK)) {
        RAISE_ERROR(OUT_OF_MEMORY_ATOM);
    }
    term r = term_alloc_tuple(3, &ctx->heap);
    term_put_tuple_element(r, 0, term_from_int(err));
    term_put_tuple_element(r, 1, term_from_int(st->stage));
    term_put_tuple_element(r, 2, term_from_int(st->last_pid));
    return error_tuple(ctx, r);
}

static term nif_bus_reset(Context *ctx, int argc, term argv[])
{
    UNUSED(argc);
    UNUSED(argv);
    struct usbh_reset_report rr;
    int err = usbh_bus_reset(&rr);
    printk("usb_host: bus reset %d (otg 0x%02x)\n", err, rr.otg);
    if (err == 0) {
        return OK_ATOM;
    }
    if (UNLIKELY(memory_ensure_free(ctx, TUPLE_SIZE(2)) != MEMORY_GC_OK)) {
        RAISE_ERROR(OUT_OF_MEMORY_ATOM);
    }
    return error_tuple(ctx, term_from_int(err));
}

/* the PHY sometimes does not answer after power-up: a reset pulse brings it back */
static term nif_phy_reset(Context *ctx, int argc, term argv[])
{
    UNUSED(ctx);
    UNUSED(argc);
    UNUSED(argv);
    ulpi_phy_reset(1);
    k_msleep(1);
    ulpi_phy_reset(0);
    k_msleep(10);
    return OK_ATOM;
}

static term nif_enumerate(Context *ctx, int argc, term argv[])
{
    UNUSED(argc);
    UNUSED(argv);
    if (usb_enumerate(&enum_info)) {
        if (UNLIKELY(memory_ensure_free(ctx, TUPLE_SIZE(2)) != MEMORY_GC_OK)) {
            RAISE_ERROR(OUT_OF_MEMORY_ATOM);
        }
        return error_tuple(ctx, term_from_int(enum_info.step));
    }
    if (UNLIKELY(memory_ensure_free(ctx, TUPLE_SIZE(3) + term_binary_heap_size(sizeof(enum_info.dev))) != MEMORY_GC_OK)) {
        RAISE_ERROR(OUT_OF_MEMORY_ATOM);
    }
    term dev = term_create_uninitialized_binary(sizeof(enum_info.dev), &ctx->heap, ctx->global);
    memcpy((void *) term_binary_data(dev), enum_info.dev, sizeof(enum_info.dev));
    term t = term_alloc_tuple(3, &ctx->heap);
    term_put_tuple_element(t, 0, OK_ATOM);
    term_put_tuple_element(t, 1, term_from_int(USB_DEV_ADDR));
    term_put_tuple_element(t, 2, dev);
    return t;
}

static term nif_ctrl_in(Context *ctx, int argc, term argv[])
{
    UNUSED(argc);
    VALIDATE_VALUE(argv[0], term_is_integer);
    VALIDATE_VALUE(argv[1], term_is_binary);
    VALIDATE_VALUE(argv[2], term_is_integer);
    if (term_binary_size(argv[1]) != 8 || term_to_int(argv[2]) < 0 || term_to_int(argv[2]) > CTRL_MAX) {
        RAISE_ERROR(BADARG_ATOM);
    }
    uint8_t setup[8], buf[CTRL_MAX];
    memcpy(setup, term_binary_data(argv[1]), 8);
    struct usb_ctrl_stats st;
    int r = usbh_ctrl_in((uint8_t) term_to_int(argv[0]), setup, buf, (size_t) term_to_int(argv[2]), &st);
    if (r < 0) {
        return ctrl_error(ctx, r, &st);
    }
    if (UNLIKELY(memory_ensure_free(ctx, TUPLE_SIZE(2) + term_binary_heap_size(r)) != MEMORY_GC_OK)) {
        RAISE_ERROR(OUT_OF_MEMORY_ATOM);
    }
    term data = term_create_uninitialized_binary(r, &ctx->heap, ctx->global);
    memcpy((void *) term_binary_data(data), buf, r);
    term t = term_alloc_tuple(2, &ctx->heap);
    term_put_tuple_element(t, 0, OK_ATOM);
    term_put_tuple_element(t, 1, data);
    return t;
}

static term nif_ctrl_out(Context *ctx, int argc, term argv[])
{
    UNUSED(argc);
    VALIDATE_VALUE(argv[0], term_is_integer);
    VALIDATE_VALUE(argv[1], term_is_binary);
    VALIDATE_VALUE(argv[2], term_is_binary);
    size_t n = term_binary_size(argv[2]);
    if (term_binary_size(argv[1]) != 8 || n > CTRL_MAX) {
        RAISE_ERROR(BADARG_ATOM);
    }
    uint8_t setup[8], buf[CTRL_MAX];
    memcpy(setup, term_binary_data(argv[1]), 8);
    memcpy(buf, term_binary_data(argv[2]), n);
    struct usb_ctrl_stats st;
    int r = usbh_ctrl_out((uint8_t) term_to_int(argv[0]), setup, buf, n, &st);
    if (r != (int) n) {
        return ctrl_error(ctx, r, &st);
    }
    return OK_ATOM;
}

static term nif_bulk_in_start(Context *ctx, int argc, term argv[])
{
    UNUSED(ctx);
    UNUSED(argc);
    VALIDATE_VALUE(argv[0], term_is_integer);
    VALIDATE_VALUE(argv[1], term_is_integer);
    usbh_bulk_in_start((uint8_t) term_to_int(argv[0]), (uint8_t) term_to_int(argv[1]),
                       argv[2] == TRUE_ATOM);
    return OK_ATOM;
}

static term nif_bulk_in_stop(Context *ctx, int argc, term argv[])
{
    UNUSED(ctx);
    UNUSED(argc);
    UNUSED(argv);
    usbh_bulk_in_stop();
    return OK_ATOM;
}

static term nif_bulk_in_stats(Context *ctx, int argc, term argv[])
{
    UNUSED(argc);
    UNUSED(argv);
    if (UNLIKELY(memory_ensure_free(ctx, TUPLE_SIZE(3)) != MEMORY_GC_OK)) {
        RAISE_ERROR(OUT_OF_MEMORY_ATOM);
    }
    /* rolling counters: kept below 2^27 so they stay small integers */
    struct usbh_bulk_in_stats st;
    usbh_bulk_in_stats(&st);
    term t = term_alloc_tuple(3, &ctx->heap);
    term_put_tuple_element(t, 0, term_from_int(st.packets & 0x7ffffff));
    term_put_tuple_element(t, 1, term_from_int(st.naks & 0x7ffffff));
    term_put_tuple_element(t, 2, term_from_int(st.errors & 0xffff));
    return t;
}

/* {LinkDbg, SofStat, XferStat, Phy, RxCmd}: link_dbg (16 bits), sof_stat (low 27), xfer_stat >> 5,
 * the PHY's Function Control (0x04) | Interface Control (0x07) << 8 | OTG Control (0x0A) << 16
 * (0xFF each when a read fails: a PHY that does not answer), the last RX CMD */
static term nif_debug(Context *ctx, int argc, term argv[])
{
    UNUSED(argc);
    UNUSED(argv);
    if (UNLIKELY(memory_ensure_free(ctx, TUPLE_SIZE(5)) != MEMORY_GC_OK)) {
        RAISE_ERROR(OUT_OF_MEMORY_ATOM);
    }
    static const uint8_t regs[3] = { 0x04, 0x07, 0x0a };
    uint32_t phy = 0;
    for (int i = 0; i < 3; i++) {
        uint8_t v = 0xff;
        if (ulpi_read(regs[i], &v) != 0) {
            v = 0xff;
        }
        phy |= (uint32_t) v << (8 * i);
    }
    struct usbh_debug d;
    usbh_debug(&d);
    term t = term_alloc_tuple(5, &ctx->heap);
    term_put_tuple_element(t, 0, term_from_int(d.link & 0xffff));
    term_put_tuple_element(t, 1, term_from_int(d.sof & 0x7ffffff));
    term_put_tuple_element(t, 2, term_from_int(d.xfer >> 5));
    term_put_tuple_element(t, 3, term_from_int(phy));
    term_put_tuple_element(t, 4, term_from_int(ulpi_last_rxcmd()));
    return t;
}

#define NIF(f) { .base.type = NIFFunctionType, .nif_ptr = f }
static const struct Nif bus_reset_nif = NIF(nif_bus_reset);
static const struct Nif phy_reset_nif = NIF(nif_phy_reset);
static const struct Nif enumerate_nif = NIF(nif_enumerate);
static const struct Nif ctrl_in_nif = NIF(nif_ctrl_in);
static const struct Nif ctrl_out_nif = NIF(nif_ctrl_out);
static const struct Nif bulk_in_start_nif = NIF(nif_bulk_in_start);
static const struct Nif bulk_in_stop_nif = NIF(nif_bulk_in_stop);
static const struct Nif bulk_in_stats_nif = NIF(nif_bulk_in_stats);
static const struct Nif debug_nif = NIF(nif_debug);

static const struct Nif *usb_host_get_nif(const char *nifname)
{
    static const struct {
        const char *name;
        const struct Nif *nif;
    } nifs[] = {
        { "usb_host:bus_reset/0", &bus_reset_nif },
        { "usb_host:phy_reset/0", &phy_reset_nif },
        { "usb_host:enumerate/0", &enumerate_nif },
        { "usb_host:ctrl_in/3", &ctrl_in_nif },
        { "usb_host:ctrl_out/3", &ctrl_out_nif },
        { "usb_host:bulk_in_start/3", &bulk_in_start_nif },
        { "usb_host:bulk_in_stop/0", &bulk_in_stop_nif },
        { "usb_host:bulk_in_stats/0", &bulk_in_stats_nif },
        { "usb_host:debug/0", &debug_nif },
    };
    for (size_t i = 0; i < sizeof(nifs) / sizeof(nifs[0]); i++) {
        if (strcmp(nifs[i].name, nifname) == 0) {
            return nifs[i].nif;
        }
    }
    return NULL;
}

REGISTER_NIF_COLLECTION(usb_host, NULL, NULL, usb_host_get_nif)
#endif
