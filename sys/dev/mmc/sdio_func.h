/*-
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Copyright (c) 2026 Kyle Crenshaw <b1nc0d3x@gmail.com>
 *
 * SDIO function-bus API.  The mmc(4) probe layer creates one
 * sdio_func device per I/O function (1-7) advertised via the
 * card's CCCR.  Each function carries the parsed CIS metadata
 * (manfid / prodid / class code) on its ivars so newbus probe
 * methods can match.  Function drivers (bwfm_sdio, sdiobt, etc.)
 * call the CMD52 / CMD53 helpers here to talk to their slice of
 * the chip without learning the underlying mmc command machinery.
 */

#ifndef _DEV_MMC_SDIO_FUNC_H_
#define _DEV_MMC_SDIO_FUNC_H_

#include <sys/types.h>
#include <sys/bus.h>

/*
 * Handoff structure from mmc(4) to the sdio function bus.  Allocated
 * by mmc_go_discovery() when it lands on an SDIO card, attached to
 * the newly-created `sdio` child as its newbus ivars.  sdio_attach()
 * reads it via device_get_ivars() to learn the RCA + nfn the mmc
 * layer already negotiated, instead of redoing CMD3/CMD7/CMD5.
 */
struct sdio_bus_ivars {
	uint32_t	sbi_ocr;	/* OCR returned by second CMD5 */
	uint16_t	sbi_rca;	/* relative card address */
	uint8_t		sbi_nfn;	/* # I/O functions (1..7) */
};

/*
 * Ivar accessors for SDIO function children.  Each child of the
 * sdio bus exposes these via newbus.
 */
enum sdio_func_ivar {
	SDIO_IVAR_FUNC_NUM = 1,	/* function number 1..7 */
	SDIO_IVAR_MANFID,	/* CISTPL_MANFID manufacturer code */
	SDIO_IVAR_PRODID,	/* CISTPL_MANFID product code */
	SDIO_IVAR_FUNC_CLASS,	/* CISTPL_FUNCID class code */
	SDIO_IVAR_BLOCKSIZE,	/* function block size (CMD53) */
};

#define	SDIO_ACCESSOR(var, ivar, type)					\
	__BUS_ACCESSOR(sdio, var, SDIO, ivar, type)

SDIO_ACCESSOR(func_num,    FUNC_NUM,    uint8_t)
SDIO_ACCESSOR(manfid,      MANFID,      uint16_t)
SDIO_ACCESSOR(prodid,      PRODID,      uint16_t)
SDIO_ACCESSOR(func_class,  FUNC_CLASS,  uint8_t)
SDIO_ACCESSOR(blocksize,   BLOCKSIZE,   uint16_t)

#undef SDIO_ACCESSOR

/*
 * CMD52 — single-byte direct read/write to function `func` at
 * `addr` within that function's address space.  Returns 0 on
 * success, errno on failure (timeout, R5 response error bit).
 *
 * Function 0 is the CCCR — common control registers shared across
 * the whole card.  Functions 1..N are vendor-defined.
 */
int sdio_read_byte(device_t func, uint32_t addr, uint8_t *val);
int sdio_write_byte(device_t func, uint32_t addr, uint8_t val);

/*
 * CMD53 — multi-byte / multi-block transfer.  `incr` true means
 * the on-chip address advances with each byte; false leaves it
 * fixed (typical for FIFO-style registers).  When `len` is a
 * multiple of the function's block size, CMD53 runs in block mode
 * which is dramatically faster — the caller doesn't have to know
 * the difference; the helper picks the mode.
 */
int sdio_read_multi(device_t func, uint32_t addr, void *buf,
    size_t len, bool incr);
int sdio_write_multi(device_t func, uint32_t addr, const void *buf,
    size_t len, bool incr);

/*
 * Function-level interrupt enable.  The SDIO card multiplexes
 * function-specific interrupts onto the host's MMC IRQ; the host
 * arms via CCCR.INT_ENABLE bit N and the function driver claims
 * via this API.  Disabling routes the interrupt to /dev/null.
 */
int sdio_enable_intr(device_t func, driver_intr_t *handler, void *arg);
void sdio_disable_intr(device_t func);

#endif /* _DEV_MMC_SDIO_FUNC_H_ */
