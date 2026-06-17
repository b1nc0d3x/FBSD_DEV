/*-
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Copyright (c) 2026 Kyle Crenshaw <b1nc0d3x@gmail.com>
 *
 * FreeBSD port of Linux drivers/mmc/host/sdhci-iproc.c.  Implements
 * the SDHCI controller variant Broadcom integrates into iProc SoCs
 * (Cygnus, BCM2711 / Raspberry Pi 4 emmc2, BCM7211a0, etc.).  These
 * controllers are mostly SDHCI-spec compliant on the wire but the
 * Synopsys/Arasan IP block + iProc integration glue introduces a few
 * non-standard behaviours that break the generic sdhci(4) framework
 * if not patched at the bus-access layer:
 *
 *   * **32-bit-only register access.**  Narrower reads/writes either
 *     silently drop or corrupt adjacent register state.  read_2 /
 *     read_1 must synthesise via a 32-bit read + shift; write_2 /
 *     write_1 must read-modify-write the full dword.
 *
 *   * **Clock-domain crossing on back-to-back 16-bit writes.**  When
 *     BLOCK_SIZE+BLOCK_COUNT (32-bit pair) is written followed within
 *     two SD-clock cycles by TRANSFER_MODE+COMMAND (another 32-bit
 *     pair), the controller can lose one of the writes.  Linux's
 *     fix: shadow the writes and emit them as paired 32-bit writes
 *     only at COMMAND time.  We do the same here.
 *
 *   * **Slow-clock write delay.**  At SD ID frequencies (<=400 kHz)
 *     every register write needs four SD-clock cycles to settle.  We
 *     delay after every writel under that threshold.
 *
 *   * **CAPS register reads zero.**  bcm2711_data.caps is intentionally
 *     0 (the controller reports nothing); the slot caps come from
 *     mmc_caps + framework defaults.  Other variants (bcm2835, cygnus,
 *     bcm7211a0) set caps explicitly because their hardware also
 *     under-reports.
 *
 *   * **BCM2711 200 kHz min clock.**  At 100 kHz polling, BCM2711's
 *     SDHCI hangs when the core clock is >=500 MHz.  Floor the min
 *     frequency at 200 kHz to dodge this.
 *
 * For the immediate FreeBSD use case (Pi 4 WiFi SDIO on emmc2), the
 * bcm2711 variant is what unblocks BCM43455 communication.  The
 * other variants are included for completeness and so this driver
 * can take over from bcm2835_sdhci.c on all iProc-derived SoCs.
 *
 * Driver coexistence:  bcm2835_sdhci.c retains its broadcom,bcm2835-
 * sdhci compat string (original Pi 1/2/3 hardware works); this driver
 * claims the brcm,bcm2711-emmc2 string so it takes precedence on the
 * boards that need the iProc semantics.
 */

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/bus.h>
#include <sys/kernel.h>
#include <sys/lock.h>
#include <sys/module.h>
#include <sys/mutex.h>
#include <sys/resource.h>
#include <sys/rman.h>
#include <sys/sysctl.h>
#include <sys/taskqueue.h>

#include <machine/bus.h>
#include <machine/resource.h>

#include <dev/ofw/ofw_bus.h>
#include <dev/ofw/ofw_bus_subr.h>

#include <dev/mmc/bridge.h>
#include <dev/mmc/mmcreg.h>

#include <dev/sdhci/sdhci.h>

#include "mmcbr_if.h"
#include "sdhci_if.h"

/*
 * SDHCI register offsets we shadow live in sdhci.h:
 *   SDHCI_BLOCK_SIZE	= 0x04 (16-bit)
 *   SDHCI_BLOCK_COUNT	= 0x06 (16-bit)
 *   SDHCI_TRANSFER_MODE	= 0x0C (16-bit)
 *   SDHCI_COMMAND_FLAGS	= 0x0E (16-bit) — packed (opcode << 8 | flags)
 *
 * The framework writes SDHCI_COMMAND_FLAGS as the last step of issuing
 * a command, which is what we use as the flush trigger.
 */

/*
 * Per-variant data — selected at probe via OFW compat-string match.
 * `caps` and `caps1` override the controller-reported capabilities
 * when those reads come back as zero (very common on iProc variants).
 * `missing_caps` signals that the SDHCI framework should treat the
 * controller's CAPABILITIES register as garbage and prefer the
 * software-supplied values exclusively.
 */
struct sdhci_iproc_data {
	const char	*name;
	uint32_t	 caps;
	uint32_t	 caps1;
	uint32_t	 min_freq;	/* 0 = let framework decide */
	bool		 missing_caps;
};

static const struct sdhci_iproc_data bcm2711_data = {
	.name		= "BCM2711 emmc2 (iProc)",
	.caps		= 0,			/* use framework defaults */
	.caps1		= 0,
	.min_freq	= 200000,		/* 200 kHz, BCM2711 quirk */
	.missing_caps	= false,
};

static const struct sdhci_iproc_data iproc_data = {
	.name		= "iProc SDHCI",
	.caps		= SDHCI_CAN_VDD_330 | SDHCI_CAN_VDD_180 |
			  SDHCI_CAN_DO_HISPD,
	.caps1		= 0,
	.min_freq	= 0,
	.missing_caps	= true,
};

static const struct sdhci_iproc_data cygnus_data = {
	.name		= "iProc Cygnus SDHCI",
	.caps		= SDHCI_CAN_VDD_330 | SDHCI_CAN_VDD_180 |
			  SDHCI_CAN_DO_HISPD,
	.caps1		= 0,
	.min_freq	= 0,
	.missing_caps	= true,
};

static struct ofw_compat_data compat_data[] = {
	{ "brcm,bcm2711-emmc2",	  (uintptr_t)&bcm2711_data },
	{ "brcm,bcm2838-emmc2",	  (uintptr_t)&bcm2711_data },
	{ "brcm,sdhci-iproc",	  (uintptr_t)&iproc_data },
	{ "brcm,sdhci-iproc-cygnus", (uintptr_t)&cygnus_data },
	{ NULL,			  0 }
};

/*
 * Safety + instrumentation knobs (under hw.sdhci_iproc.*):
 *
 *   enable	(default 0): the driver returns ENXIO from probe unless
 *		set to non-zero.  Lets us boot with the module loaded but
 *		inert, then flip the switch at runtime via
 *		`sysctl hw.sdhci_iproc.enable=1` and trigger a rescan.
 *		Default is OFF because the first deploy wedged BOTH
 *		SDHCI controllers on the Pi 4 (SD card + emmc2),
 *		corrupting the rootfs.  Don't claim hardware without
 *		operator opt-in.
 *
 *   debug	(default 1): DPRINTF verbosity.
 *		0 = silent, 1 = milestones, 2 = per-register-access, 3 = noisy.
 *
 *   stop_after (default 0): if non-zero, halt attach at the
 *		specified checkpoint (1=after resource alloc, 2=after
 *		quirks set, 3=after sdhci_init_slot, 4=after
 *		bus_setup_intr).  Lets us bisect which stage trips up
 *		shared SDHCI state.
 *
 * All three are TUNABLE so they can be set from loader.conf.
 */
static int sdhci_iproc_enable = 0;
static int sdhci_iproc_debug = 1;
static int sdhci_iproc_stop_after = 0;

SYSCTL_NODE(_hw, OID_AUTO, sdhci_iproc, CTLFLAG_RD | CTLFLAG_MPSAFE, 0,
    "BCM iProc SDHCI driver");
SYSCTL_INT(_hw_sdhci_iproc, OID_AUTO, enable, CTLFLAG_RWTUN,
    &sdhci_iproc_enable, 0,
    "Master enable: 0 = driver inert (probe returns ENXIO), 1 = claim devices");
SYSCTL_INT(_hw_sdhci_iproc, OID_AUTO, debug, CTLFLAG_RWTUN,
    &sdhci_iproc_debug, 0,
    "DPRINTF level: 0=silent 1=milestones 2=reg 3=noisy");
SYSCTL_INT(_hw_sdhci_iproc, OID_AUTO, stop_after, CTLFLAG_RWTUN,
    &sdhci_iproc_stop_after, 0,
    "Halt attach after checkpoint N (0=run all, 1-4=bisect)");

#define	DPRINTF(level, dev, fmt, ...)					\
	do {								\
		if (sdhci_iproc_debug >= (level))			\
			device_printf((dev), fmt, ##__VA_ARGS__);	\
	} while (0)

struct sdhci_iproc_softc {
	device_t			 dev;
	const struct sdhci_iproc_data	*variant;
	struct resource			*mem_res;
	struct resource			*irq_res;
	void				*irq_cookie;
	struct sdhci_slot		 slot;

	/*
	 * Shadow state for the Linux-style BLOCK / TRANSFER / COMMAND
	 * coalescing.  When the SDHCI framework writes BLOCK_SIZE,
	 * BLOCK_COUNT, or TRANSFER_MODE we stash the value and only
	 * actually emit them — paired into 32-bit writes — when the
	 * subsequent COMMAND write comes in.  This dodges the iProc
	 * clock-domain-crossing register-loss bug.
	 */
	uint32_t			 shadow_blk;	/* BLOCK_SIZE+COUNT */
	uint32_t			 shadow_cmd;	/* TRANSFER+COMMAND */
	bool				 blk_shadowed;
	bool				 cmd_shadowed;
};

/* ------------------------------------------------------------------
 * Register access — 32-bit-only with synthesised narrower paths.
 *
 * REG_OFFSET_IN_BITS extracts the byte/halfword-within-dword shift
 * for a register at offset `reg`: bits 1-2 of the address select the
 * 16/8-bit slot in the enclosing 32-bit word.
 * ------------------------------------------------------------------ */

#define	REG_OFFSET_IN_BITS(reg)		(((reg) << 3) & 0x18)

static uint32_t
sdhci_iproc_read_4(device_t dev, struct sdhci_slot *slot, bus_size_t off)
{
	struct sdhci_iproc_softc *sc = device_get_softc(dev);

	(void)slot;
	return (bus_read_4(sc->mem_res, off));
}

static uint16_t
sdhci_iproc_read_2(device_t dev, struct sdhci_slot *slot, bus_size_t off)
{
	struct sdhci_iproc_softc *sc = device_get_softc(dev);
	uint32_t val;

	(void)slot;
	/*
	 * If this is a read of a register we've shadowed, return the
	 * shadowed value — the actual controller register hasn't been
	 * written yet (it'll be written on next COMMAND).  Otherwise
	 * do a 32-bit read and extract the 16-bit window.
	 */
	if (off == SDHCI_TRANSFER_MODE && sc->cmd_shadowed)
		val = sc->shadow_cmd;
	else if ((off == SDHCI_BLOCK_SIZE || off == SDHCI_BLOCK_COUNT) &&
	    sc->blk_shadowed)
		val = sc->shadow_blk;
	else
		val = bus_read_4(sc->mem_res, off & ~3);
	return ((val >> REG_OFFSET_IN_BITS(off)) & 0xFFFF);
}

static uint8_t
sdhci_iproc_read_1(device_t dev, struct sdhci_slot *slot, bus_size_t off)
{
	struct sdhci_iproc_softc *sc = device_get_softc(dev);
	uint32_t val;

	(void)slot;
	val = bus_read_4(sc->mem_res, off & ~3);
	return ((val >> REG_OFFSET_IN_BITS(off)) & 0xFF);
}

/*
 * 32-bit write with optional slow-clock settle delay.  At SD ID
 * frequencies the controller needs four SD-clock cycles to commit a
 * write; this delays roughly the right amount based on the host's
 * current clock setting.
 */
static void
sdhci_iproc_writel_settle(struct sdhci_iproc_softc *sc, bus_size_t off,
    uint32_t val)
{
	bus_write_4(sc->mem_res, off, val);
	if (sc->slot.clock != 0 && sc->slot.clock <= 400000) {
		/*
		 * Round up to whole micros, four SD-clock cycles' worth.
		 * At 400 kHz: 4 * 2.5 us = 10 us.  At 100 kHz: 40 us.
		 */
		uint32_t us = (4 * 1000000U + sc->slot.clock - 1) /
		    sc->slot.clock;
		DELAY(us);
	} else if (sc->slot.clock == 0) {
		DELAY(10);
	}
}

static void
sdhci_iproc_write_4(device_t dev, struct sdhci_slot *slot, bus_size_t off,
    uint32_t val)
{
	struct sdhci_iproc_softc *sc = device_get_softc(dev);

	(void)slot;
	sdhci_iproc_writel_settle(sc, off, val);
}

/*
 * 16-bit write — the heart of the iProc quirk handling.  BLOCK_SIZE,
 * BLOCK_COUNT, and TRANSFER_MODE are shadowed; the actual hardware
 * write of all four happens at COMMAND time, paired into two 32-bit
 * writes (BLOCK_SIZE+COUNT, then TRANSFER+COMMAND).  Anything else
 * gets a normal read-modify-write of the enclosing 32-bit word.
 */
static void
sdhci_iproc_write_2(device_t dev, struct sdhci_slot *slot, bus_size_t off,
    uint16_t val)
{
	struct sdhci_iproc_softc *sc = device_get_softc(dev);
	uint32_t word_shift, mask, oldval, newval;

	(void)slot;
	word_shift = REG_OFFSET_IN_BITS(off);
	mask = 0xFFFFU << word_shift;

	if (off == SDHCI_COMMAND_FLAGS) {
		/* Flush pending BLOCK_SIZE/COUNT before issuing the
		 * command. */
		if (sc->blk_shadowed) {
			sdhci_iproc_writel_settle(sc, SDHCI_BLOCK_SIZE,
			    sc->shadow_blk);
			sc->blk_shadowed = false;
		}
		/* Combine TRANSFER_MODE shadow with the COMMAND we're
		 * writing, then emit as one 32-bit write. */
		oldval = sc->cmd_shadowed ? sc->shadow_cmd :
		    bus_read_4(sc->mem_res, SDHCI_TRANSFER_MODE);
		newval = (oldval & ~mask) | ((uint32_t)val << word_shift);
		sc->cmd_shadowed = false;
		sdhci_iproc_writel_settle(sc, SDHCI_TRANSFER_MODE, newval);
		return;
	}
	if (off == SDHCI_BLOCK_SIZE || off == SDHCI_BLOCK_COUNT) {
		oldval = sc->blk_shadowed ? sc->shadow_blk :
		    bus_read_4(sc->mem_res, SDHCI_BLOCK_SIZE);
		newval = (oldval & ~mask) | ((uint32_t)val << word_shift);
		sc->shadow_blk = newval;
		sc->blk_shadowed = true;
		return;
	}
	if (off == SDHCI_TRANSFER_MODE) {
		oldval = sc->cmd_shadowed ? sc->shadow_cmd :
		    bus_read_4(sc->mem_res, SDHCI_TRANSFER_MODE);
		newval = (oldval & ~mask) | ((uint32_t)val << word_shift);
		sc->shadow_cmd = newval;
		sc->cmd_shadowed = true;
		return;
	}

	/* Default: RMW the enclosing dword. */
	oldval = bus_read_4(sc->mem_res, off & ~3);
	newval = (oldval & ~mask) | ((uint32_t)val << word_shift);
	sdhci_iproc_writel_settle(sc, off & ~3, newval);
}

static void
sdhci_iproc_write_1(device_t dev, struct sdhci_slot *slot, bus_size_t off,
    uint8_t val)
{
	struct sdhci_iproc_softc *sc = device_get_softc(dev);
	uint32_t byte_shift, mask, oldval, newval;

	(void)slot;
	byte_shift = REG_OFFSET_IN_BITS(off);
	mask = 0xFFU << byte_shift;
	oldval = bus_read_4(sc->mem_res, off & ~3);
	newval = (oldval & ~mask) | ((uint32_t)val << byte_shift);
	sdhci_iproc_writel_settle(sc, off & ~3, newval);
}

/* ------------------------------------------------------------------
 * Block I/O helpers — multi-word reads/writes for the FIFO data
 * register at SDHCI_BUFFER (offset 0x20).  This is a 32-bit FIFO so
 * the SDHCI framework already requests 4-byte chunks; we just
 * forward to bus_read_multi_4 / bus_write_multi_4.
 * ------------------------------------------------------------------ */

static void
sdhci_iproc_read_multi_4(device_t dev, struct sdhci_slot *slot,
    bus_size_t off, uint32_t *data, bus_size_t count)
{
	struct sdhci_iproc_softc *sc = device_get_softc(dev);

	(void)slot;
	bus_read_multi_4(sc->mem_res, off, data, count);
}

static void
sdhci_iproc_write_multi_4(device_t dev, struct sdhci_slot *slot,
    bus_size_t off, uint32_t *data, bus_size_t count)
{
	struct sdhci_iproc_softc *sc = device_get_softc(dev);

	(void)slot;
	bus_write_multi_4(sc->mem_res, off, data, count);
}

/* ------------------------------------------------------------------
 * Newbus glue
 * ------------------------------------------------------------------ */

static int
sdhci_iproc_probe(device_t dev)
{
	const struct ofw_compat_data *d;

	if (!ofw_bus_status_okay(dev))
		return (ENXIO);
	d = ofw_bus_search_compatible(dev, compat_data);
	if (d->ocd_str == NULL)
		return (ENXIO);
	/*
	 * Master safety gate.  Even on a compat match, return ENXIO unless
	 * the operator has explicitly enabled us.  This keeps the module
	 * loadable-but-inert on first boot: a `kldload sdhci_iproc` adds
	 * the driver to the bus but it claims nothing.  Then a
	 * `sysctl hw.sdhci_iproc.enable=1; devctl rescan ofwbus0` brings
	 * it live with no kernel restart.
	 */
	if (sdhci_iproc_enable == 0) {
		DPRINTF(1, dev, "probe: matched %s but enable=0, declining\n",
		    d->ocd_str);
		return (ENXIO);
	}
	DPRINTF(1, dev, "probe: matched %s, enable=1, claiming\n", d->ocd_str);
	device_set_desc(dev,
	    ((const struct sdhci_iproc_data *)d->ocd_data)->name);
	/*
	 * BUS_PROBE_VENDOR beats bcm2835_sdhci.c (which returns
	 * BUS_PROBE_DEFAULT) for the brcm,bcm271{1,8}-emmc2 strings.
	 * We don't need to remove those compat entries from
	 * bcm2835_sdhci.c — the higher probe value wins automatically.
	 */
	return (BUS_PROBE_VENDOR);
}

static void
sdhci_iproc_intr(void *arg)
{
	struct sdhci_iproc_softc *sc = arg;

	sdhci_generic_intr(&sc->slot);
}

static int
sdhci_iproc_attach(device_t dev)
{
	struct sdhci_iproc_softc *sc = device_get_softc(dev);
	const struct ofw_compat_data *d;
	int rid, err;

	sc->dev = dev;
	d = ofw_bus_search_compatible(dev, compat_data);
	sc->variant = (const struct sdhci_iproc_data *)d->ocd_data;
	DPRINTF(1, dev, "attach: variant=%s\n", sc->variant->name);

	rid = 0;
	sc->mem_res = bus_alloc_resource_any(dev, SYS_RES_MEMORY, &rid,
	    RF_ACTIVE);
	if (sc->mem_res == NULL) {
		device_printf(dev, "cannot allocate MMIO\n");
		return (ENXIO);
	}
	DPRINTF(1, dev, "attach: MMIO @ %p size 0x%lx\n",
	    (void *)rman_get_start(sc->mem_res),
	    (unsigned long)rman_get_size(sc->mem_res));

	rid = 0;
	sc->irq_res = bus_alloc_resource_any(dev, SYS_RES_IRQ, &rid,
	    RF_ACTIVE);
	if (sc->irq_res == NULL) {
		device_printf(dev, "cannot allocate IRQ\n");
		bus_release_resource(dev, SYS_RES_MEMORY, 0, sc->mem_res);
		return (ENXIO);
	}
	DPRINTF(1, dev, "attach: IRQ allocated\n");

	if (sdhci_iproc_stop_after == 1) {
		device_printf(dev, "stop_after=1: halting after resource alloc, "
		    "releasing and returning ENXIO\n");
		bus_release_resource(dev, SYS_RES_IRQ, 0, sc->irq_res);
		bus_release_resource(dev, SYS_RES_MEMORY, 0, sc->mem_res);
		return (ENXIO);
	}

	/*
	 * Initialise the slot.  Quirks we know we need on iProc:
	 *  - 32-bit-only register access (handled by our read/write ops)
	 *  - NO_HISPD_BIT on Cygnus, NO_HIGHSPEED on bcm2835 — left to
	 *    the framework's defaults for now; specific variants can
	 *    set them via slot.quirks once observed.
	 *
	 * If the variant says missing_caps, override the framework's
	 * read of the controller's CAPABILITIES register with our
	 * compile-time values.
	 */
	sc->slot.quirks = SDHCI_QUIRK_DATA_TIMEOUT_USES_SDCLK;
	if (sc->variant->missing_caps) {
		sc->slot.caps = sc->variant->caps;
		sc->slot.quirks |= SDHCI_QUIRK_MISSING_CAPS;
	}
	DPRINTF(1, dev, "attach: quirks=0x%x caps=0x%x\n", sc->slot.quirks,
	    sc->slot.caps);

	if (sdhci_iproc_stop_after == 2) {
		device_printf(dev, "stop_after=2: halting before sdhci_init_slot, "
		    "releasing and returning ENXIO\n");
		bus_release_resource(dev, SYS_RES_IRQ, 0, sc->irq_res);
		bus_release_resource(dev, SYS_RES_MEMORY, 0, sc->mem_res);
		return (ENXIO);
	}

	err = sdhci_init_slot(dev, &sc->slot, 0);
	if (err != 0) {
		device_printf(dev, "sdhci_init_slot failed: %d\n", err);
		bus_release_resource(dev, SYS_RES_IRQ, 0, sc->irq_res);
		bus_release_resource(dev, SYS_RES_MEMORY, 0, sc->mem_res);
		return (err);
	}
	DPRINTF(1, dev, "attach: sdhci_init_slot OK\n");

	if (sdhci_iproc_stop_after == 3) {
		device_printf(dev, "stop_after=3: halting after sdhci_init_slot\n");
		sdhci_cleanup_slot(&sc->slot);
		bus_release_resource(dev, SYS_RES_IRQ, 0, sc->irq_res);
		bus_release_resource(dev, SYS_RES_MEMORY, 0, sc->mem_res);
		return (ENXIO);
	}

	err = bus_setup_intr(dev, sc->irq_res, INTR_TYPE_MISC | INTR_MPSAFE,
	    NULL, sdhci_iproc_intr, sc, &sc->irq_cookie);
	if (err != 0) {
		device_printf(dev, "bus_setup_intr failed: %d\n", err);
		sdhci_cleanup_slot(&sc->slot);
		bus_release_resource(dev, SYS_RES_IRQ, 0, sc->irq_res);
		bus_release_resource(dev, SYS_RES_MEMORY, 0, sc->mem_res);
		return (err);
	}
	DPRINTF(1, dev, "attach: bus_setup_intr OK\n");

	if (sdhci_iproc_stop_after == 4) {
		device_printf(dev, "stop_after=4: halting after bus_setup_intr\n");
		bus_teardown_intr(dev, sc->irq_res, sc->irq_cookie);
		sdhci_cleanup_slot(&sc->slot);
		bus_release_resource(dev, SYS_RES_IRQ, 0, sc->irq_res);
		bus_release_resource(dev, SYS_RES_MEMORY, 0, sc->mem_res);
		return (ENXIO);
	}

	bus_attach_children(dev);
	sdhci_start_slot(&sc->slot);
	DPRINTF(1, dev, "attach: complete\n");
	return (0);
}

static int
sdhci_iproc_detach(device_t dev)
{
	struct sdhci_iproc_softc *sc = device_get_softc(dev);

	bus_generic_detach(dev);
	if (sc->irq_cookie)
		bus_teardown_intr(dev, sc->irq_res, sc->irq_cookie);
	sdhci_cleanup_slot(&sc->slot);
	if (sc->irq_res)
		bus_release_resource(dev, SYS_RES_IRQ, 0, sc->irq_res);
	if (sc->mem_res)
		bus_release_resource(dev, SYS_RES_MEMORY, 0, sc->mem_res);
	return (0);
}

/*
 * BCM2711 polling-clock workaround.  When the SDHCI framework asks
 * for the minimum supported frequency, return 200 kHz on bcm2711
 * variants instead of letting it use the framework default (often
 * 100 kHz) — that hangs the controller when no card is inserted.
 */
static uint32_t
sdhci_iproc_min_freq(device_t dev, struct sdhci_slot *slot)
{
	struct sdhci_iproc_softc *sc = device_get_softc(dev);

	(void)slot;
	if (sc->variant->min_freq != 0)
		return (sc->variant->min_freq);
	return (sdhci_generic_min_freq(dev, slot));
}

static device_method_t sdhci_iproc_methods[] = {
	DEVMETHOD(device_probe,			sdhci_iproc_probe),
	DEVMETHOD(device_attach,		sdhci_iproc_attach),
	DEVMETHOD(device_detach,		sdhci_iproc_detach),

	DEVMETHOD(bus_read_ivar,		sdhci_generic_read_ivar),
	DEVMETHOD(bus_write_ivar,		sdhci_generic_write_ivar),

	DEVMETHOD(mmcbr_update_ios,		sdhci_generic_update_ios),
	DEVMETHOD(mmcbr_request,		sdhci_generic_request),
	DEVMETHOD(mmcbr_get_ro,			sdhci_generic_get_ro),
	DEVMETHOD(mmcbr_acquire_host,		sdhci_generic_acquire_host),
	DEVMETHOD(mmcbr_release_host,		sdhci_generic_release_host),

	DEVMETHOD(sdhci_read_1,			sdhci_iproc_read_1),
	DEVMETHOD(sdhci_read_2,			sdhci_iproc_read_2),
	DEVMETHOD(sdhci_read_4,			sdhci_iproc_read_4),
	DEVMETHOD(sdhci_read_multi_4,		sdhci_iproc_read_multi_4),
	DEVMETHOD(sdhci_write_1,		sdhci_iproc_write_1),
	DEVMETHOD(sdhci_write_2,		sdhci_iproc_write_2),
	DEVMETHOD(sdhci_write_4,		sdhci_iproc_write_4),
	DEVMETHOD(sdhci_write_multi_4,		sdhci_iproc_write_multi_4),
	DEVMETHOD(sdhci_get_card_present,	sdhci_generic_get_card_present),
	DEVMETHOD(sdhci_set_uhs_timing,		sdhci_generic_set_uhs_timing),
	DEVMETHOD(sdhci_min_freq,		sdhci_iproc_min_freq),

	DEVMETHOD_END
};

static driver_t sdhci_iproc_driver = {
	"sdhci_iproc",
	sdhci_iproc_methods,
	sizeof(struct sdhci_iproc_softc),
};

DRIVER_MODULE(sdhci_iproc, simplebus, sdhci_iproc_driver, NULL, NULL);
SDHCI_DEPEND(sdhci_iproc);
#ifndef MMCCAM
MMC_DECLARE_BRIDGE(sdhci_iproc);
#endif
