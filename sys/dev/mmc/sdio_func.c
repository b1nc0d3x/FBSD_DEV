/*-
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Copyright (c) 2026 Kyle Crenshaw <b1nc0d3x@gmail.com>
 *
 * SDIO function-bus driver.  Attaches as a child of mmc(4) when
 * mmc_go_discovery() lands on an SDIO card (mode_sdio), parses
 * CCCR + CIS, then instantiates one newbus child per advertised
 * I/O function (1..7).  Function drivers — bwfm_sdio, sdiobt,
 * etc. — probe against the (manfid, prodid) ivars on each child.
 *
 * What this file does NOT do: it never reads or writes user data
 * payloads.  All real I/O is initiated from the function driver
 * via the sdio_{read,write}_* helpers, which marshal CMD52/CMD53
 * and submit through the mmc bus.
 */

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/bus.h>
#include <sys/kernel.h>
#include <sys/lock.h>
#include <sys/malloc.h>
#include <sys/module.h>
#include <sys/mutex.h>

#include <dev/mmc/bridge.h>
#include <dev/mmc/mmcreg.h>
#include <dev/mmc/mmcvar.h>
#include <dev/mmc/mmc_subr.h>
#include <dev/mmc/sdio_func.h>
#include <dev/mmc/sdioreg.h>

#include "mmcbus_if.h"

#include <sys/sysctl.h>

MALLOC_DECLARE(M_SDIO);
MALLOC_DEFINE(M_SDIO, "sdio", "SDIO function-bus state");

/*
 * Bring-up debug knob.  When >0, every CMD52 against function 0 prints
 * the raw R5 response (cmd.resp[0]).  Lets us tell at a glance whether
 * the card is genuinely returning data-byte=0 (suspect quirk) vs our
 * R5 parser is reading the wrong field.  Default 1 until SDIO bring-up
 * is stable, then drop to 0.
 */
static int sdio_debug = 1;
SYSCTL_NODE(_hw, OID_AUTO, sdio, CTLFLAG_RD | CTLFLAG_MPSAFE, 0,
    "SDIO function bus");
SYSCTL_INT(_hw_sdio, OID_AUTO, debug, CTLFLAG_RWTUN, &sdio_debug, 0,
    "debug verbosity: 0=silent 1=show raw R5 responses 2=tuple bytes");

struct sdio_softc {
	device_t		 sc_dev;	/* the sdio bus device */
	device_t		 sc_mmcbus;	/* parent mmc bus */
	uint16_t		 sc_manfid;	/* CISTPL_MANFID code */
	uint16_t		 sc_prodid;	/* CISTPL_MANFID product */
	uint8_t			 sc_nfn;	/* # I/O functions present */
	device_t		 sc_func[8];	/* function children */
};

/* Per-function ivars carried on each newbus child. */
struct sdio_func_ivars {
	uint8_t		fi_num;
	uint16_t	fi_manfid;
	uint16_t	fi_prodid;
	uint8_t		fi_class;
	uint16_t	fi_blksize;
};

/* ------------------------------------------------------------------
 * CMD52 / CMD53 helpers — the public face of this module.  Function
 * drivers call into these to do byte / block I/O against their slot.
 * Both helpers route to the parent mmc bus, which knows how to drive
 * the controller (sdhci_bcm on a Pi 4).
 * ------------------------------------------------------------------ */

/*
 * CMD52 — single-byte direct I/O.  Builds a 32-bit argument from
 * the (rw, func, raw, addr, data) tuple per the SD spec, submits
 * the command, and on success returns the response byte (or stores
 * it for read).  R5 response: low byte = data, high byte = status
 * (we check the error bits and translate to errno).
 */
static int
sdio_cmd52(device_t func, uint8_t func_num, uint32_t addr, bool write,
    uint8_t in_val, uint8_t *out_val)
{
	struct sdio_softc *sc = device_get_softc(device_get_parent(func));
	struct mmc_command cmd;
	uint32_t arg;
	int err;

	if (addr & ~SD_ARG_CMD52_REG_MASK)
		return (EINVAL);

	memset(&cmd, 0, sizeof(cmd));
	cmd.opcode = SD_IO_RW_DIRECT;
	cmd.flags = MMC_RSP_R5 | MMC_CMD_AC;
	arg = ((uint32_t)func_num & SD_ARG_CMD52_FUNC_MASK) <<
	    SD_ARG_CMD52_FUNC_SHIFT;
	arg |= (addr & SD_ARG_CMD52_REG_MASK) << SD_ARG_CMD52_REG_SHIFT;
	if (write) {
		arg |= SD_ARG_CMD52_WRITE;
		arg |= (in_val & SD_ARG_CMD52_DATA_MASK);
		if (out_val != NULL)
			arg |= SD_ARG_CMD52_EXCHANGE;
	}
	cmd.arg = arg;

	err = mmc_wait_for_cmd(sc->sc_mmcbus, sc->sc_dev, &cmd, 0);
	if (err != MMC_ERR_NONE)
		return (EIO);

	/* R5 status: high 8 bits of resp[0] carry flags; bits 8-10 are
	 * COM_CRC_ERROR, ILLEGAL_COMMAND, IO_CURRENT_STATE.  Any bit in
	 * the upper status nibbles other than ready means a card error. */
	if ((cmd.resp[0] & 0xCB00) != 0)
		return (EIO);

	if (out_val != NULL)
		*out_val = (uint8_t)(cmd.resp[0] & 0xFF);
	return (0);
}

int
sdio_read_byte(device_t func, uint32_t addr, uint8_t *val)
{
	uint8_t fn = sdio_get_func_num(func);

	return (sdio_cmd52(func, fn, addr, false, 0, val));
}

int
sdio_write_byte(device_t func, uint32_t addr, uint8_t val)
{
	uint8_t fn = sdio_get_func_num(func);

	return (sdio_cmd52(func, fn, addr, true, val, NULL));
}

int
sdio_read_multi(device_t func, uint32_t addr, void *buf, size_t len, bool incr)
{
	/*
	 * CMD53 read — block mode when len is a multiple of the function
	 * block size, byte mode otherwise.  Built on a struct mmc_data
	 * scatter-gather; the host driver (sdhci_bcm in our case)
	 * handles DMA.
	 *
	 * TODO: when the host controller wiring is debugged, replace
	 * this stub with the real implementation.  For now bytes-only
	 * via repeated CMD52 keeps test code running.
	 */
	uint8_t *p = buf;
	int err;
	uint8_t fn = sdio_get_func_num(func);

	for (; len > 0; len--) {
		err = sdio_cmd52(func, fn, addr, false, 0, p);
		if (err != 0)
			return (err);
		p++;
		if (incr)
			addr++;
	}
	return (0);
}

int
sdio_write_multi(device_t func, uint32_t addr, const void *buf, size_t len,
    bool incr)
{
	const uint8_t *p = buf;
	int err;
	uint8_t fn = sdio_get_func_num(func);

	for (; len > 0; len--) {
		err = sdio_cmd52(func, fn, addr, true, *p, NULL);
		if (err != 0)
			return (err);
		p++;
		if (incr)
			addr++;
	}
	return (0);
}

int
sdio_enable_intr(device_t func, driver_intr_t *handler, void *arg)
{
	(void)func; (void)handler; (void)arg;
	return (ENOSYS);
}

void
sdio_disable_intr(device_t func)
{
	(void)func;
}

/* ------------------------------------------------------------------
 * Bus enumeration — CCCR header + CIS walk.  Called from attach().
 * ------------------------------------------------------------------ */

/*
 * Read one byte from CCCR/CIS address space via CMD52 to function 0.
 * CIS tuples are accessed against function 0 even when describing
 * a higher-numbered function — the FBR redirects.
 */
static int
sdio_f0_read_byte(struct sdio_softc *sc, uint32_t addr, uint8_t *val)
{
	struct mmc_command cmd;
	int err;

	memset(&cmd, 0, sizeof(cmd));
	cmd.opcode = SD_IO_RW_DIRECT;
	cmd.flags = MMC_RSP_R5 | MMC_CMD_AC;
	cmd.arg = (0 << SD_ARG_CMD52_FUNC_SHIFT) |
	    ((addr & SD_ARG_CMD52_REG_MASK) << SD_ARG_CMD52_REG_SHIFT);

	err = mmc_wait_for_cmd(sc->sc_mmcbus, sc->sc_dev, &cmd, 0);
	if (sdio_debug >= 1) {
		device_printf(sc->sc_dev,
		    "F0 read [0x%05x]: mmc_err=%d resp[0]=0x%08x "
		    "resp[1]=0x%08x\n",
		    addr, err, cmd.resp[0], cmd.resp[1]);
	}
	if (err != MMC_ERR_NONE)
		return (EIO);
	if ((cmd.resp[0] & 0xCB00) != 0)
		return (EIO);
	*val = (uint8_t)(cmd.resp[0] & 0xFF);
	return (0);
}

static int
sdio_parse_cis(struct sdio_softc *sc, uint32_t cisptr,
    struct sdio_func_ivars *out)
{
	uint8_t code, len, data[16];
	uint32_t addr = cisptr;
	int err, i;

	/*
	 * CIS tuples are a chain of (code, length, data...) records
	 * starting at the address held in SD_IO_CCCR_CISPTR (for
	 * function 0) or FBR[n].CIS_OFFSET (for function n).  Each
	 * tuple is read byte-by-byte via CMD52 against function 0;
	 * the FBR/CCCR pointer addressing handles the redirection.
	 *
	 * Tuples we care about:
	 *   CISTPL_MANFID (0x20)  4-byte: manfid LSB MSB prodid LSB MSB
	 *   CISTPL_FUNCID (0x21)  2-byte: class | sysinit-mask
	 *   CISTPL_FUNCE  (0x22)  function extension (block size,
	 *                         max transfer rate, etc.)
	 *   CISTPL_END    (0xff)  terminator
	 *
	 * 200-tuple safety bound covers any reasonable CIS table.
	 */
	for (i = 0; i < 200; i++) {
		err = sdio_f0_read_byte(sc, addr++, &code);
		if (err != 0)
			return (err);
		if (code == SD_IO_CISTPL_END)
			return (0);
		err = sdio_f0_read_byte(sc, addr++, &len);
		if (err != 0)
			return (err);
		if (len > sizeof(data))
			len = sizeof(data);
		for (uint8_t j = 0; j < len; j++) {
			err = sdio_f0_read_byte(sc, addr++, &data[j]);
			if (err != 0)
				return (err);
		}

		switch (code) {
		case SD_IO_CISTPL_MANFID:
			if (len >= 4) {
				out->fi_manfid = data[0] | (data[1] << 8);
				out->fi_prodid = data[2] | (data[3] << 8);
			}
			break;
		case SD_IO_CISTPL_FUNCID:
			if (len >= 1)
				out->fi_class = data[0];
			break;
		case SD_IO_CISTPL_FUNCE:
			/*
			 * FUNCE extension byte 0 is the sub-type:
			 *   0x00 = common (function 0) extension
			 *   0x01 = per-function extension
			 *
			 * Per-function FUNCE layout (TPLFE_*):
			 *   0   type (=0x01)
			 *   1   function info byte
			 *   2   std IO rev
			 *   3-6 card PSN (LE)
			 *   7-10 CSA size (LE)
			 *   11  CSA property
			 *   12-13 max block size (LE)  <- what we want
			 *   ...
			 *
			 * We need to grow our local data[] buffer to at least
			 * 14 bytes for this read; otherwise the truncation
			 * in the tuple-read loop above silently zeroes block
			 * size.  buf grew to 16 to handle this.
			 */
			if (data[0] == 0x01 && len >= 14)
				out->fi_blksize = data[12] |
				    ((uint16_t)data[13] << 8);
			break;
		}
	}
	return (0);
}

static int
sdio_read_cisptr(struct sdio_softc *sc, uint32_t fbr_base, uint32_t *cisptr)
{
	uint8_t lo, mid, hi;
	int err;

	err = sdio_f0_read_byte(sc, fbr_base + 0, &lo);
	if (err != 0)
		return (err);
	err = sdio_f0_read_byte(sc, fbr_base + 1, &mid);
	if (err != 0)
		return (err);
	err = sdio_f0_read_byte(sc, fbr_base + 2, &hi);
	if (err != 0)
		return (err);
	*cisptr = (uint32_t)lo | ((uint32_t)mid << 8) | ((uint32_t)hi << 16);
	return (0);
}

static int
sdio_attach_children(struct sdio_softc *sc)
{
	uint32_t cisptr;
	uint8_t i, cardcap;
	int err;

	/*
	 * Hex-dump the first 0x18 bytes of CCCR — covers the SDIO/CCCR
	 * revision, capability bits, and the function-0 CIS pointer.
	 * If these come back zero the card isn't talking to us at all
	 * (vs talking but reporting empty fields), which sharply narrows
	 * the diagnosis next.
	 */
	if (sdio_debug >= 1) {
		uint8_t cccr[0x18];
		int j;
		for (j = 0; j < (int)sizeof(cccr); j++) {
			if (sdio_f0_read_byte(sc, j, &cccr[j]) != 0)
				cccr[j] = 0xff;
		}
		device_printf(sc->sc_dev,
		    "CCCR 0x00-0x17 dump:\n");
		for (j = 0; j < (int)sizeof(cccr); j += 8) {
			device_printf(sc->sc_dev,
			    "  %02x: %02x %02x %02x %02x %02x %02x %02x %02x\n",
			    j, cccr[j+0], cccr[j+1], cccr[j+2], cccr[j+3],
			    cccr[j+4], cccr[j+5], cccr[j+6], cccr[j+7]);
		}
	}

	/*
	 * Read CCCR header bytes we care about.  CARDCAP byte tells us
	 * whether the card supports CMD53 block-mode (the SMB bit) —
	 * we keep that on the sc for later sysctl reporting / sanity
	 * checks but don't gate function discovery on it.
	 */
	(void)sdio_f0_read_byte(sc, SD_IO_CCCR_CARDCAP, &cardcap);
	device_printf(sc->sc_dev,
	    "SDIO bus: CARDCAP=0x%02x (block-mode %s, low-speed %s)\n",
	    cardcap,
	    (cardcap & CCCR_CC_SMB) ? "yes" : "no",
	    (cardcap & CCCR_CC_LSC) ? "yes" : "no");

	for (i = 1; i <= sc->sc_nfn; i++) {
		struct sdio_func_ivars *iv;
		device_t child;
		uint32_t fbr;

		iv = malloc(sizeof(*iv), M_SDIO, M_WAITOK | M_ZERO);
		iv->fi_num = i;

		/*
		 * Function 0 holds the function-0 CIS at SD_IO_CCCR_CISPTR.
		 * Functions 1..N have their own CIS pointers at the start
		 * of their FBR block.
		 */
		fbr = SD_IO_FBR_START_F(i) + SD_IO_FBR_CIS_OFFSET;
		err = sdio_read_cisptr(sc, fbr, &cisptr);
		if (err == 0 && cisptr != 0)
			(void)sdio_parse_cis(sc, cisptr, iv);

		child = device_add_child(sc->sc_dev, NULL, DEVICE_UNIT_ANY);
		if (child == NULL) {
			free(iv, M_SDIO);
			continue;
		}
		device_set_ivars(child, iv);
		sc->sc_func[i] = child;
		device_printf(sc->sc_dev,
		    "sdio func %u: manfid=0x%04x prodid=0x%04x class=0x%02x "
		    "blksize=%u\n",
		    iv->fi_num, iv->fi_manfid, iv->fi_prodid, iv->fi_class,
		    iv->fi_blksize);
	}

	bus_attach_children(sc->sc_dev);
	return (0);
}

/* ------------------------------------------------------------------
 * Newbus glue
 * ------------------------------------------------------------------ */

static int
sdio_probe(device_t dev)
{
	device_set_desc(dev, "SDIO bus");
	return (BUS_PROBE_DEFAULT);
}

static int
sdio_attach(device_t dev)
{
	struct sdio_softc *sc = device_get_softc(dev);
	struct sdio_bus_ivars *sbi;
	struct mmc_command cmd;
	int err;

	sc->sc_dev = dev;
	sc->sc_mmcbus = device_get_parent(dev);

	/*
	 * mmc_go_discovery() does the CMD5/CMD3/CMD7 sequence and hands
	 * us the negotiated (rca, nfn, ocr) via newbus ivars.  Pull them
	 * here so sdio_attach_children() knows how many function slots
	 * to walk.  If ivars are missing the SDIO probe never finished,
	 * which is a bug in mmc.c — bail with ENXIO rather than silently
	 * enumerating zero functions.
	 */
	sbi = device_get_ivars(dev);
	if (sbi == NULL) {
		device_printf(dev,
		    "no sdio_bus_ivars from parent mmc bus; bailing\n");
		return (ENXIO);
	}
	sc->sc_nfn = sbi->sbi_nfn;
	device_printf(dev,
	    "SDIO bus attach: nfn=%u rca=0x%04x ocr=0x%08x\n",
	    sbi->sbi_nfn, sbi->sbi_rca, sbi->sbi_ocr);

	/*
	 * Re-select the card.  mmc_go_discovery() did CMD7 to put the
	 * card in TRAN state, but then mmc_calculate_clock() at the end
	 * of go_discovery does mmc_select_card(sc, 0) which deselects
	 * ALL cards back to STANDBY — including ours.  Without this
	 * second CMD7, every subsequent CMD52 times out.  Found 2026-06-17
	 * during BCM43455 bring-up on Pi 4.
	 */
	memset(&cmd, 0, sizeof(cmd));
	cmd.opcode = MMC_SELECT_CARD;
	cmd.arg = (uint32_t)sbi->sbi_rca << 16;
	cmd.flags = MMC_RSP_R1B | MMC_CMD_AC;
	err = mmc_wait_for_cmd(sc->sc_mmcbus, sc->sc_dev, &cmd, 0);
	if (err != MMC_ERR_NONE) {
		device_printf(dev,
		    "CMD7 reselect (RCA=0x%04x) failed err=%d\n",
		    sbi->sbi_rca, err);
		return (ENXIO);
	}
	device_printf(dev, "CMD7 reselect OK, card in TRAN state\n");

	return (sdio_attach_children(sc));
}

static int
sdio_detach(device_t dev)
{
	struct sdio_softc *sc = device_get_softc(dev);
	int i;

	for (i = 1; i < (int)nitems(sc->sc_func); i++) {
		if (sc->sc_func[i] != NULL) {
			void *iv = device_get_ivars(sc->sc_func[i]);

			device_delete_child(dev, sc->sc_func[i]);
			if (iv != NULL)
				free(iv, M_SDIO);
			sc->sc_func[i] = NULL;
		}
	}
	return (0);
}

/* ------------------------------------------------------------------
 * Ivar plumbing — exposes the per-function metadata to child drivers
 * via newbus.  Pairs with the SDIO_ACCESSOR() macros in sdio_func.h.
 * ------------------------------------------------------------------ */

static int
sdio_read_ivar(device_t bus, device_t child, int which, uintptr_t *result)
{
	struct sdio_func_ivars *iv = device_get_ivars(child);

	if (iv == NULL)
		return (ENOENT);

	switch (which) {
	case SDIO_IVAR_FUNC_NUM:	*result = iv->fi_num;     return (0);
	case SDIO_IVAR_MANFID:		*result = iv->fi_manfid;  return (0);
	case SDIO_IVAR_PRODID:		*result = iv->fi_prodid;  return (0);
	case SDIO_IVAR_FUNC_CLASS:	*result = iv->fi_class;   return (0);
	case SDIO_IVAR_BLOCKSIZE:	*result = iv->fi_blksize; return (0);
	default:			return (ENOENT);
	}
}

static device_method_t sdio_methods[] = {
	DEVMETHOD(device_probe,		sdio_probe),
	DEVMETHOD(device_attach,	sdio_attach),
	DEVMETHOD(device_detach,	sdio_detach),

	DEVMETHOD(bus_read_ivar,	sdio_read_ivar),
	DEVMETHOD(bus_print_child,	bus_generic_print_child),

	DEVMETHOD_END
};

static driver_t sdio_driver = {
	"sdio",
	sdio_methods,
	sizeof(struct sdio_softc)
};

DRIVER_MODULE(sdio, mmc, sdio_driver, NULL, NULL);
MODULE_VERSION(sdio, 1);
