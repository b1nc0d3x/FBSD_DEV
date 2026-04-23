/*-
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Copyright (c) 2026 Kyle Crenshaw
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE AUTHOR OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 */

/*
 * Rockchip RK3399 Cadence DisplayPort controller driver.
 *
 * Goal: USB-C DisplayPort Alt Mode on RockPro64 — single USB-C cable for
 * both power delivery and video output, replacing the HDMI cable.
 *
 * The RK3399 routes DisplayPort through the Cadence CDN-DP IP block, which
 * sits behind the TYPE-C PHY (rk_typec_phy).  The fusb302 USB-C PD controller
 * negotiates the cable orientation and Alt Mode entry; this driver owns the
 * CDN-DP controller itself: clocks, resets, PHY lane assignment, AUX channel,
 * and eventually DRM connector plumbing.
 *
 * Current state — scaffold phase:
 *   - attach the CDN-DP controller node and bring up its power/clock/reset tree
 *   - switch the TYPE-C PHY lanes into DP mode and enable them
 *   - implement the raw AUX channel so DPCD capability reads are possible
 *   - expose an attach-time debug probe (tunable-gated) to exercise AUX/HPD
 *
 * Not yet implemented:
 *   - link training (required before pixels flow)
 *   - HPD interrupt handling and hot-plug event delivery
 *   - Alt Mode negotiation integration with fusb302
 *   - DRM connector registration so Xorg sees a DP output
 */

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/bus.h>
#include <sys/kernel.h>
#include <sys/module.h>
#include <sys/sysctl.h>
#include <sys/rman.h>

#include <machine/bus.h>

#include <dev/clk/clk.h>
#include <dev/hwreset/hwreset.h>
#include <dev/iicbus/usb/fusb302_var.h>
#include <dev/ofw/ofw_bus.h>
#include <dev/ofw/ofw_bus_subr.h>
#include <dev/ofw/openfirm.h>
#include <dev/phy/phy.h>

#include "rk3399_power.h"

/* RK3399 supports up to two TYPE-C ports, each providing one DP PHY. */
#define	RK_CDN_DP_MAX_PHYS	2

/* CDN-DP requires four clocks and four resets; counts kept here so loops
 * and array sizes stay in sync with the name tables below. */
#define	RK_CDN_DP_NCLKS		4
#define	RK_CDN_DP_NRSTS		4

/* AUX channel payload limit per the DisplayPort 1.4 spec (16 bytes). */
#define	RK_CDN_DP_AUX_MAX_XFER	16

/* Number of DPCD registers read in one shot during the capability probe.
 * Bytes 0x000–0x00E cover revision, link rate, lane count, and key caps. */
#define	RK_CDN_DP_DPCD_CAP_SIZE	15

/*
 * CDN-DP APB register offsets.
 * All offsets are relative to the MMIO base at 0xfec00000 (RK3399 TRM Part2).
 */

/* DP_INT_STA — interrupt status; written to clear reply/error sticky bits
 * before starting an AUX transaction so stale flags don't confuse the wait. */
#define	RK_CDN_DP_DP_INT_STA		0x03DC
#define	 RK_CDN_DP_INT_RPLY_RECEIV	(1U << 1)
#define	 RK_CDN_DP_INT_AUX_ERR		(1U << 0)

/* SYS_CTL_3 — system control register 3; holds the live HPD status bit and
 * the software HPD force bits used for Type-C debug without a physical sink. */
#define	RK_CDN_DP_SYS_CTL_3		0x0608
#define	 RK_CDN_DP_SYS_CTL_3_HPD_STATUS	(1U << 6)
#define	 RK_CDN_DP_SYS_CTL_3_F_HPD	(1U << 5)
#define	 RK_CDN_DP_SYS_CTL_3_HPD_CTRL	(1U << 4)

/* AUX_CH_STA — AUX channel status; polled to detect transaction completion
 * and to read the result code after the CDN-DP engine finishes an exchange. */
#define	RK_CDN_DP_AUX_CH_STA		0x0780
#define	 RK_CDN_DP_AUX_BUSY		(1U << 4)
#define	 RK_CDN_DP_AUX_STATUS_MASK	0x0f

/* AUX_ERR_NUM — extended error count; printed alongside the status name to
 * give more context when AUX transactions fail during bring-up. */
#define	RK_CDN_DP_AUX_ERR_NUM		0x0784

/* BUFFER_DATA_CTL — AUX data buffer control; BUF_CLR flushes stale payload
 * bytes before a new transaction, BUF_HAVE_DATA signals that read bytes are
 * present in BUF_DATA_0..N after a completed native read. */
#define	RK_CDN_DP_BUFFER_DATA_CTL	0x0790
#define	 RK_CDN_DP_BUF_CLR		(1U << 7)
#define	 RK_CDN_DP_BUF_HAVE_DATA	(1U << 4)
#define	 RK_CDN_DP_BUF_DATA_COUNT_MASK	0x0f

/* AUX channel control and address registers; split into three byte-wide
 * address registers because the DPCD address space is 20 bits wide. */
#define	RK_CDN_DP_AUX_CH_CTL_1		0x0794
#define	RK_CDN_DP_AUX_ADDR_7_0		0x0798
#define	RK_CDN_DP_AUX_ADDR_15_8	0x079C
#define	RK_CDN_DP_AUX_ADDR_19_16	0x07A0
#define	RK_CDN_DP_AUX_CH_CTL_2		0x07A4
#define	 RK_CDN_DP_ADDR_ONLY		(1U << 1)	/* address-only (0-byte) transfer */
#define	 RK_CDN_DP_AUX_EN		(1U << 0)	/* self-clearing start bit */

/* BUF_DATA_0 — base of the 16-entry AUX payload FIFO; each slot is a full
 * 32-bit register even though only the low byte carries data. */
#define	RK_CDN_DP_BUF_DATA_0		0x07C0

/* DisplayPort native AUX command codes (4-bit field in AUX_CH_CTL_1). */
#define	RK_CDN_DP_AUX_CMD_NATIVE_WRITE	0x8
#define	RK_CDN_DP_AUX_CMD_NATIVE_READ	0x9

enum rk_cdn_dp_res_id {
	RK_CDN_DP_RES_MEM = 0,
	RK_CDN_DP_RES_IRQ,
	RK_CDN_DP_RES_COUNT
};

enum rk_cdn_dp_clk_id {
	RK_CDN_DP_CLK_CORE = 0,
	RK_CDN_DP_CLK_PCLK,
	RK_CDN_DP_CLK_SPDIF,
	RK_CDN_DP_CLK_GRF
};

enum rk_cdn_dp_rst_id {
	RK_CDN_DP_RST_SPDIF = 0,
	RK_CDN_DP_RST_DPTX,
	RK_CDN_DP_RST_APB,
	RK_CDN_DP_RST_CORE
};

/* Clock names match the DTS clock-names property in rk3399-rockpro64.dtb. */
static const char *rk_cdn_dp_clk_names[RK_CDN_DP_NCLKS] = {
	"core-clk",
	"pclk",
	"spdif",
	"grf",
};

/*
 * Reset names match the DTS reset-names property.
 * Non-const because FreeBSD's hwreset_get_by_ofw_name takes a mutable char *
 * even though the string is never modified by the callee.
 */
static char *rk_cdn_dp_rst_names[RK_CDN_DP_NRSTS] = {
	"spdif",
	"dptx",
	"apb",
	"core",
};

static struct ofw_compat_data rk_cdn_dp_compat_data[] = {
	{ "rockchip,rk3399-cdn-dp", 1 },
	{ NULL, 0 }
};

static struct resource_spec rk_cdn_dp_spec[] = {
	{ SYS_RES_MEMORY, RK_CDN_DP_RES_MEM, RF_ACTIVE },
	{ SYS_RES_IRQ, 0, RF_ACTIVE | RF_OPTIONAL },	/* IRQ absent on some boards */
	{ -1, 0 }
};

struct rk_cdn_dp_softc {
	device_t		dev;
	phandle_t		node;
	struct resource		*res[RK_CDN_DP_RES_COUNT];
	clk_t			clks[RK_CDN_DP_NCLKS];
	hwreset_t		rsts[RK_CDN_DP_NRSTS];
	phy_t			phys[RK_CDN_DP_MAX_PHYS];
	int			nphys;
	bool			clks_enabled;
	bool			rsts_deasserted;
	bool			phys_enabled;
	bool			has_extcon;	/* cable orientation via extcon rather than fusb302 */
	device_t		extcon_dev;
	device_t		power_dev;	/* handle to rk3399_power provider */
	uint32_t		power_domain_id;

	/* Bring-up is staged so the module can be loaded safely. */
	int			stage;
};

static void	rk_cdn_dp_release(device_t dev);
static int	rk_cdn_dp_probe_dpcd_caps(struct rk_cdn_dp_softc *sc);
static int	rk_cdn_dp_get_power_domain(struct rk_cdn_dp_softc *sc);
static int	rk_cdn_dp_get_extcon(struct rk_cdn_dp_softc *sc);

enum rk_cdn_dp_stage {
	RK_CDN_DP_STAGE_ATTACHED	= 0,
	RK_CDN_DP_STAGE_POWER		= 1,
	RK_CDN_DP_STAGE_HANDLES		= 2,
	RK_CDN_DP_STAGE_CLOCKS		= 3,
	RK_CDN_DP_STAGE_RESETS		= 4,
	RK_CDN_DP_STAGE_PHYS		= 5,
	RK_CDN_DP_STAGE_AUX_PROBE	= 6,
};

static const char *
rk_cdn_dp_stage_name(int stage)
{
	switch (stage) {
	case RK_CDN_DP_STAGE_ATTACHED:
		return ("attached");
	case RK_CDN_DP_STAGE_POWER:
		return ("power-domain");
	case RK_CDN_DP_STAGE_HANDLES:
		return ("handles");
	case RK_CDN_DP_STAGE_CLOCKS:
		return ("clocks");
	case RK_CDN_DP_STAGE_RESETS:
		return ("resets");
	case RK_CDN_DP_STAGE_PHYS:
		return ("phys");
	case RK_CDN_DP_STAGE_AUX_PROBE:
		return ("aux-probe");
	default:
		return ("unknown");
	}
}

static const char *
rk_cdn_dp_typec_role_name(enum fusb302_typec_role role)
{
	switch (role) {
	case FUSB302_TYPEC_ROLE_SOURCE:
		return ("source");
	case FUSB302_TYPEC_ROLE_SINK:
		return ("sink");
	case FUSB302_TYPEC_ROLE_ACCESSORY:
		return ("accessory");
	case FUSB302_TYPEC_ROLE_NONE:
		return ("none");
	default:
		return ("unknown");
	}
}

static const char *
rk_cdn_dp_typec_orientation_name(enum fusb302_typec_orientation orientation)
{
	switch (orientation) {
	case FUSB302_TYPEC_ORIENT_CC1:
		return ("cc1");
	case FUSB302_TYPEC_ORIENT_CC2:
		return ("cc2");
	case FUSB302_TYPEC_ORIENT_NONE:
		return ("none");
	default:
		return ("unknown");
	}
}

/*
 * rk_cdn_dp_read_4 / rk_cdn_dp_write_4
 *
 * Thin wrappers around bus_read_4/bus_write_4 so call sites name the
 * controller rather than the resource slot.  Keeping MMIO access centralised
 * here makes it easy to add barrier or debug instrumentation in one place.
 */
static inline uint32_t
rk_cdn_dp_read_4(struct rk_cdn_dp_softc *sc, bus_size_t off)
{
	return (bus_read_4(sc->res[RK_CDN_DP_RES_MEM], off));
}

static inline void
rk_cdn_dp_write_4(struct rk_cdn_dp_softc *sc, bus_size_t off, uint32_t val)
{
	bus_write_4(sc->res[RK_CDN_DP_RES_MEM], off, val);
}

/*
 * rk_cdn_dp_force_typec_dp
 *
 * Returns true when the loader tunable hw.rk3399_typec_dp_force is set.
 * Used during bring-up to assert HPD in software when no physical USB-C sink
 * is connected, bypassing the normal fusb302 Alt Mode negotiation path.
 * Without this escape hatch, AUX transactions cannot be tested until the full
 * Type-C policy stack is wired up.
 */
static bool
rk_cdn_dp_force_typec_dp(void)
{
	int force_typec_dp;

	force_typec_dp = 0;
	if (!TUNABLE_INT_FETCH("hw.rk3399_typec_dp_force", &force_typec_dp))
		return (false);
	return (force_typec_dp != 0);
}

/*
 * rk_cdn_dp_attach_debug_probe
 *
 * Returns true when hw.rk_cdn_dp_attach_debug_probe is set at boot.
 * Gates the HPD status read and DPCD capability probe that run at the end of
 * attach.  Kept behind a tunable so the module can be loaded safely on a
 * board without a connected DP sink — the AUX engine will not be exercised
 * unless the operator explicitly opts in.
 */
static bool
rk_cdn_dp_attach_debug_probe(void)
{
	int debug_probe;

	debug_probe = 0;
	if (!TUNABLE_INT_FETCH("hw.rk_cdn_dp_attach_debug_probe", &debug_probe))
		return (false);
	return (debug_probe != 0);
}

/*
 * rk_cdn_dp_hpd_status
 *
 * Reads the live HPD pin state from SYS_CTL_3.HPD_STATUS.  This is the
 * hardware-reported presence of a downstream DP sink (monitor or dock).
 * Checked before attempting AUX transactions because the AUX engine will
 * time out immediately if no sink is asserting HPD.
 */
static bool
rk_cdn_dp_hpd_status(struct rk_cdn_dp_softc *sc)
{
	return ((rk_cdn_dp_read_4(sc, RK_CDN_DP_SYS_CTL_3) &
	    RK_CDN_DP_SYS_CTL_3_HPD_STATUS) != 0);
}

/*
 * rk_cdn_dp_force_hpd
 *
 * Asserts HPD in software via SYS_CTL_3.F_HPD + HPD_CTRL.  Needed during
 * Type-C bring-up before fusb302 Alt Mode negotiation is implemented: the
 * CDN-DP AUX engine will not respond to transactions unless it believes a
 * sink is present, so this unlocks AUX access for DPCD probing without
 * requiring a physically negotiated Alt Mode cable.
 */
static void
rk_cdn_dp_force_hpd(struct rk_cdn_dp_softc *sc)
{
	uint32_t val;

	val = rk_cdn_dp_read_4(sc, RK_CDN_DP_SYS_CTL_3);
	val |= RK_CDN_DP_SYS_CTL_3_HPD_CTRL | RK_CDN_DP_SYS_CTL_3_F_HPD;
	rk_cdn_dp_write_4(sc, RK_CDN_DP_SYS_CTL_3, val);
	DELAY(1000);
	device_printf(sc->dev, "forcing HPD for Type-C DP debug\n");
}

/*
 * rk_cdn_dp_aux_status_name
 *
 * Maps the 4-bit AUX_CH_STA status field to a human-readable string.
 * The CDN-DP controller reports these codes after every AUX transaction;
 * having names in dmesg rather than raw numbers makes bring-up failures
 * much faster to diagnose on serial console without a debugger.
 */
static const char *
rk_cdn_dp_aux_status_name(uint32_t status)
{
	switch (status & RK_CDN_DP_AUX_STATUS_MASK) {
	case 0:
		return ("ok");
	case 1:
		return ("nack");
	case 2:
		return ("timeout");
	case 3:
		return ("unknown");
	case 4:
		return ("much-defer");
	case 5:
		return ("tx-short");
	case 6:
		return ("rx-short");
	case 7:
		return ("nack-without-m");
	case 8:
		return ("i2c-nack");
	default:
		return ("reserved");
	}
}

/*
 * rk_cdn_dp_aux_wait
 *
 * Polls until the CDN-DP AUX engine has finished a transaction.  The engine
 * signals completion by clearing AUX_EN in AUX_CH_CTL_2 and AUX_BUSY in
 * AUX_CH_STA.  Both must be clear to rule out a race where AUX_EN clears
 * before the internal state machine has fully settled.
 *
 * Timeout is 200ms (2000 iterations × 100µs) — sufficient for the worst-case
 * DEFER-retry loop defined by the DisplayPort spec while keeping attach fast
 * on a healthy link.
 */
static int
rk_cdn_dp_aux_wait(struct rk_cdn_dp_softc *sc)
{
	uint32_t ctl2, sta;
	int i;

	for (i = 0; i < 2000; i++) {
		ctl2 = rk_cdn_dp_read_4(sc, RK_CDN_DP_AUX_CH_CTL_2);
		sta = rk_cdn_dp_read_4(sc, RK_CDN_DP_AUX_CH_STA);
		if ((ctl2 & RK_CDN_DP_AUX_EN) == 0 &&
		    (sta & RK_CDN_DP_AUX_BUSY) == 0)
			return (0);
		DELAY(100);
	}

	return (ETIMEDOUT);
}

/*
 * rk_cdn_dp_aux_xfer
 *
 * Executes a single native AUX read or write against a DPCD register address.
 * This is the lowest-level AUX primitive; everything that needs to talk to the
 * downstream sink (DPCD capability reads, link training, EDID via I2C-over-AUX)
 * goes through here.
 *
 * Sequence per RK3399 TRM Part2, CDN-DP AUX section:
 *   1. Clear interrupt status and flush the payload buffer.
 *   2. For writes, pre-load the payload FIFO before arming the engine.
 *   3. Program the 20-bit DPCD address across three byte registers.
 *   4. Set length and command in CTL_1, then assert AUX_EN in CTL_2.
 *   5. Wait for completion, check status, harvest read data if applicable.
 */
static int
rk_cdn_dp_aux_xfer(struct rk_cdn_dp_softc *sc, uint8_t cmd, uint32_t addr,
    uint8_t *buf, int len)
{
	uint32_t buf_ctl, sta;
	int count, error, i;

	if (len < 0 || len > RK_CDN_DP_AUX_MAX_XFER)
		return (EINVAL);

	rk_cdn_dp_write_4(sc, RK_CDN_DP_DP_INT_STA,
	    RK_CDN_DP_INT_RPLY_RECEIV | RK_CDN_DP_INT_AUX_ERR);
	rk_cdn_dp_write_4(sc, RK_CDN_DP_BUFFER_DATA_CTL, RK_CDN_DP_BUF_CLR);

	if (cmd == RK_CDN_DP_AUX_CMD_NATIVE_WRITE) {
		for (i = 0; i < len; i++)
			rk_cdn_dp_write_4(sc, RK_CDN_DP_BUF_DATA_0 + (i * 4),
			    buf[i]);
	}

	rk_cdn_dp_write_4(sc, RK_CDN_DP_AUX_ADDR_7_0, addr & 0xff);
	rk_cdn_dp_write_4(sc, RK_CDN_DP_AUX_ADDR_15_8, (addr >> 8) & 0xff);
	rk_cdn_dp_write_4(sc, RK_CDN_DP_AUX_ADDR_19_16, (addr >> 16) & 0x0f);
	rk_cdn_dp_write_4(sc, RK_CDN_DP_AUX_CH_CTL_1,
	    (((len > 0 ? len - 1 : 0) & 0xf) << 4) | (cmd & 0xf));
	rk_cdn_dp_write_4(sc, RK_CDN_DP_AUX_CH_CTL_2,
	    (len == 0 ? RK_CDN_DP_ADDR_ONLY : 0) | RK_CDN_DP_AUX_EN);

	error = rk_cdn_dp_aux_wait(sc);
	if (error != 0) {
		device_printf(sc->dev, "AUX wait timeout cmd=0x%x addr=0x%x\n",
		    cmd, addr);
		return (error);
	}

	sta = rk_cdn_dp_read_4(sc, RK_CDN_DP_AUX_CH_STA);
	if ((sta & RK_CDN_DP_AUX_STATUS_MASK) != 0) {
		device_printf(sc->dev,
		    "AUX error cmd=0x%x addr=0x%x status=%s(%u) errnum=%u\n",
		    cmd, addr, rk_cdn_dp_aux_status_name(sta),
		    sta & RK_CDN_DP_AUX_STATUS_MASK,
		    rk_cdn_dp_read_4(sc, RK_CDN_DP_AUX_ERR_NUM) & 0xff);
		return (EIO);
	}

	if (cmd == RK_CDN_DP_AUX_CMD_NATIVE_READ) {
		buf_ctl = rk_cdn_dp_read_4(sc, RK_CDN_DP_BUFFER_DATA_CTL);
		if ((buf_ctl & RK_CDN_DP_BUF_HAVE_DATA) == 0)
			return (ENXIO);
		count = buf_ctl & RK_CDN_DP_BUF_DATA_COUNT_MASK;
		if (count > len)
			count = len;
		for (i = 0; i < count; i++)
			buf[i] = rk_cdn_dp_read_4(sc,
			    RK_CDN_DP_BUF_DATA_0 + (i * 4)) & 0xff;
		if (count < len)
			return (EMSGSIZE);
	}

	return (0);
}

/*
 * rk_cdn_dp_probe_dpcd_caps
 *
 * Reads the first 15 bytes of the sink's DPCD register space (0x000–0x00E).
 * This range covers the DPCD revision, maximum link rate, maximum lane count,
 * and key capability flags — the minimum information needed to plan link
 * training.  Called from the attach-time debug probe to verify that the AUX
 * channel is alive end-to-end before link training is implemented.
 */
static int
rk_cdn_dp_probe_dpcd_caps(struct rk_cdn_dp_softc *sc)
{
	uint8_t dpcd[RK_CDN_DP_DPCD_CAP_SIZE];
	int error;

	error = rk_cdn_dp_aux_xfer(sc, RK_CDN_DP_AUX_CMD_NATIVE_READ, 0x000,
	    dpcd, sizeof(dpcd));
	if (error != 0)
		return (error);

	device_printf(sc->dev,
	    "DPCD rev=%#x max_link_rate=%#x max_lane_count=%#x\n",
	    dpcd[0], dpcd[1], dpcd[2]);
	device_printf(sc->dev,
	    "DPCD caps raw: %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x\n",
	    dpcd[0], dpcd[1], dpcd[2], dpcd[3], dpcd[4], dpcd[5], dpcd[6],
	    dpcd[7], dpcd[8], dpcd[9], dpcd[10], dpcd[11], dpcd[12],
	    dpcd[13], dpcd[14]);

	return (0);
}

/*
 * rk_cdn_dp_probe
 *
 * Standard FreeBSD device probe.  Rejects devices marked disabled in the DTB
 * so the driver is only active when the DTB overlay sets status = "okay" —
 * keeping the module safe to load on boards where the CDN-DP node is present
 * but intentionally not in use.
 */
static int
rk_cdn_dp_probe(device_t dev)
{
	if (!ofw_bus_status_okay(dev))
		return (ENXIO);
	if (ofw_bus_search_compatible(dev, rk_cdn_dp_compat_data)->ocd_data == 0)
		return (ENXIO);

	device_set_desc(dev, "Rockchip RK3399 Cadence DisplayPort scaffold");
	return (BUS_PROBE_DEFAULT);
}

/*
 * rk_cdn_dp_get_clocks
 *
 * Acquires handles for the four CDN-DP clocks from the DTS clock-names list.
 * Handles are stored in sc->clks[] for later enable/disable in rk_cdn_dp_enable
 * and rk_cdn_dp_release.  Acquiring separately from enabling allows the release
 * path to safely release only the handles that were successfully obtained.
 */
static int
rk_cdn_dp_get_clocks(struct rk_cdn_dp_softc *sc)
{
	int i, error;

	for (i = 0; i < RK_CDN_DP_NCLKS; i++) {
		error = clk_get_by_ofw_name(sc->dev, 0, rk_cdn_dp_clk_names[i],
		    &sc->clks[i]);
		if (error != 0) {
			device_printf(sc->dev, "cannot get clock %s\n",
			    rk_cdn_dp_clk_names[i]);
			return (error);
		}
	}

	return (0);
}

/*
 * rk_cdn_dp_get_resets
 *
 * Acquires handles for the four CDN-DP reset lines from the DTS reset-names
 * list.  Kept separate from the enable step for the same reason as clocks:
 * the release path needs to know exactly which handles exist before freeing.
 */
static int
rk_cdn_dp_get_resets(struct rk_cdn_dp_softc *sc)
{
	int i, error;

	for (i = 0; i < RK_CDN_DP_NRSTS; i++) {
		error = hwreset_get_by_ofw_name(sc->dev, 0, rk_cdn_dp_rst_names[i],
		    &sc->rsts[i]);
		if (error != 0) {
			device_printf(sc->dev, "cannot get reset %s\n",
			    rk_cdn_dp_rst_names[i]);
			return (error);
		}
	}

	return (0);
}

/*
 * rk_cdn_dp_get_phys
 *
 * Discovers and acquires the TYPE-C PHY lanes assigned to the CDN-DP
 * controller.  The RK3399 has two TYPE-C ports; the DTS phys property lists
 * whichever ports are wired to CDN-DP on the board (RockPro64 exposes both).
 * ENOENT from phy_get_by_ofw_idx means the list is exhausted — not an error.
 * At least one PHY must be present for DP to be possible at all.
 */
static int
rk_cdn_dp_get_phys(struct rk_cdn_dp_softc *sc)
{
	phy_t phy;
	int error, i;

	sc->nphys = 0;
	for (i = 0; i < RK_CDN_DP_MAX_PHYS; i++) {
		error = phy_get_by_ofw_idx(sc->dev, sc->node, i, &phy);
		if (error != 0) {
			if (error == ENOENT)
				break;
			device_printf(sc->dev, "cannot get phy index %d\n", i);
			return (error);
		}
		sc->phys[sc->nphys++] = phy;
	}
	if (sc->nphys == 0) {
		device_printf(sc->dev, "no DP phys available\n");
		return (ENXIO);
	}

	return (0);
}

/*
 * rk_cdn_dp_get_power_domain
 *
 * Resolves the power-domain phandle from the DTS into a device handle and
 * domain ID that can be passed to rk3399_power_enable_domain.  The CDN-DP
 * controller sits in the HDCP power domain (domain 21 in PMU_PWRDN_CON),
 * which must be ungated before any CDN-DP MMIO access; failure to do so
 * causes an ARM SError (asynchronous external abort) because the APB bus
 * returns a fault for accesses to unpowered blocks.
 *
 * If the DTS has no power-domains property the controller is assumed always
 * powered and sc->power_dev is left NULL.
 */
static int
rk_cdn_dp_get_power_domain(struct rk_cdn_dp_softc *sc)
{
	pcell_t *cells;
	phandle_t xref;
	int error, ncells;

	cells = NULL;
	error = ofw_bus_parse_xref_list_alloc(sc->node, "power-domains",
	    "#power-domain-cells", 0, &xref, &ncells, &cells);
	if (error == ENOENT)
		return (0);
	if (error != 0)
		return (error);
	if (ncells != 1) {
		OF_prop_free(cells);
		device_printf(sc->dev, "invalid power-domains specifier\n");
		return (EINVAL);
	}

	sc->power_dev = OF_device_from_xref(xref);
	sc->power_domain_id = cells[0];
	OF_prop_free(cells);

	if (sc->power_dev == NULL) {
		device_printf(sc->dev,
		    "power-domain provider is not attached yet\n");
		return (ENXIO);
	}

	return (0);
}

/*
 * rk_cdn_dp_get_extcon
 *
 * Resolves the optional extcon provider from the DT extcon property into a
 * device handle.  On RockPro64 this is the FUSB302 Type-C controller.  The
 * current bridge is intentionally minimal: it lets the DP side query attach,
 * role, and orientation state without pretending that a full USB-PD policy
 * manager already exists in FreeBSD.
 */
static int
rk_cdn_dp_get_extcon(struct rk_cdn_dp_softc *sc)
{
	pcell_t xref;
	ssize_t len;

	if (!sc->has_extcon)
		return (0);

	len = OF_getencprop(sc->node, "extcon", &xref, sizeof(xref));
	if (len <= 0)
		return (ENOENT);
	if (len < (ssize_t)sizeof(xref))
		return (EINVAL);

	sc->extcon_dev = OF_device_from_xref(xref);
	if (sc->extcon_dev == NULL) {
		device_printf(sc->dev,
		    "extcon provider is not attached yet\n");
		return (ENXIO);
	}

	return (0);
}

/*
 * Split enable sequence into discrete phases so we can bisect crashes/hangs
 * during USB-C DP bring-up.
 */
static int
rk_cdn_dp_enable_clocks(struct rk_cdn_dp_softc *sc)
{
	int error, i;

	for (i = 0; i < RK_CDN_DP_NCLKS; i++) {
		error = clk_enable(sc->clks[i]);
		if (error != 0) {
			device_printf(sc->dev, "cannot enable clock %s\n",
			    rk_cdn_dp_clk_names[i]);
			return (error);
		}
	}
	sc->clks_enabled = true;

	return (0);
}

static int
rk_cdn_dp_deassert_resets(struct rk_cdn_dp_softc *sc)
{
	int error, i;

	for (i = 0; i < RK_CDN_DP_NRSTS; i++) {
		error = hwreset_deassert(sc->rsts[i]);
		if (error != 0) {
			device_printf(sc->dev, "cannot deassert reset %s\n",
			    rk_cdn_dp_rst_names[i]);
			return (error);
		}
	}
	sc->rsts_deasserted = true;

	DELAY(20000); /* 20ms for CDN DP APB to be accessible after reset release */

	return (0);
}

static int
rk_cdn_dp_enable_phys(struct rk_cdn_dp_softc *sc)
{
	int error, i;

	for (i = 0; i < sc->nphys; i++) {
		error = phy_set_mode(sc->phys[i], PHY_MODE_DP, PHY_SUBMODE_NA);
		if (error != 0) {
			device_printf(sc->dev,
			    "cannot set phy %d to DP mode\n", i);
			return (error);
		}
		error = phy_enable(sc->phys[i]);
		if (error != 0) {
			device_printf(sc->dev, "cannot enable phy %d\n", i);
			return (error);
		}
	}
	sc->phys_enabled = true;

	return (0);
}

static bool
rk_cdn_dp_defer_enable(void)
{
	int defer;

	defer = 1;
	(void)TUNABLE_INT_FETCH("hw.rk_cdn_dp_defer_enable", &defer);
	return (defer != 0);
}

static int
rk_cdn_dp_set_stage(struct rk_cdn_dp_softc *sc, int target)
{
	int error, next;

	if (target < sc->stage)
		return (EINVAL);
	if (target > RK_CDN_DP_STAGE_AUX_PROBE)
		return (EINVAL);

	for (next = sc->stage + 1; next <= target; next++) {
		device_printf(sc->dev, "stage %d (%s): begin\n",
		    next, rk_cdn_dp_stage_name(next));

		switch (next) {
		case RK_CDN_DP_STAGE_POWER:
			if (sc->power_dev != NULL) {
				error = rk3399_power_enable_domain(sc->power_dev,
				    sc->power_domain_id);
				if (error != 0) {
					device_printf(sc->dev,
					    "cannot enable power-domain %u\n",
					    sc->power_domain_id);
					return (error);
				}
				DELAY(1000);
			}
			break;
		case RK_CDN_DP_STAGE_HANDLES:
			error = rk_cdn_dp_get_clocks(sc);
			if (error != 0)
				return (error);
			error = rk_cdn_dp_get_resets(sc);
			if (error != 0)
				return (error);
			error = rk_cdn_dp_get_phys(sc);
			if (error != 0)
				return (error);
			break;
		case RK_CDN_DP_STAGE_CLOCKS:
			error = rk_cdn_dp_enable_clocks(sc);
			if (error != 0)
				return (error);
			break;
		case RK_CDN_DP_STAGE_RESETS:
			error = rk_cdn_dp_deassert_resets(sc);
			if (error != 0)
				return (error);
			break;
		case RK_CDN_DP_STAGE_PHYS:
			error = rk_cdn_dp_enable_phys(sc);
			if (error != 0)
				return (error);
			break;
		case RK_CDN_DP_STAGE_AUX_PROBE:
			if (!rk_cdn_dp_hpd_status(sc) && rk_cdn_dp_force_typec_dp())
				rk_cdn_dp_force_hpd(sc);
			error = rk_cdn_dp_probe_dpcd_caps(sc);
			if (error != 0)
				device_printf(sc->dev, "AUX probe not ready (%d)\n",
				    error);
			break;
		default:
			return (EINVAL);
		}

		sc->stage = next;
		device_printf(sc->dev, "stage %d (%s): ok\n",
		    sc->stage, rk_cdn_dp_stage_name(sc->stage));
	}

	return (0);
}

static int
rk_cdn_dp_sysctl_stage(SYSCTL_HANDLER_ARGS)
{
	struct rk_cdn_dp_softc *sc;
	int error, stage;

	sc = arg1;
	stage = sc->stage;
	error = sysctl_handle_int(oidp, &stage, 0, req);
	if (error != 0 || req->newptr == NULL)
		return (error);

	return (rk_cdn_dp_set_stage(sc, stage));
}

/*
 * rk_cdn_dp_attach
 *
 * Top-level attach.  Brings the CDN-DP controller from cold reset to a state
 * where the AUX channel is operational.  The sequence is ordered to avoid the
 * ARM SError that plagued earlier bring-up:
 *
 *   1. Allocate MMIO and IRQ resources (no register access yet).
 *   2. Enable the power domain — MMIO accesses fault if the domain is off.
 *   3. 1ms settle after power domain enable before touching clocks.
 *   4. Acquire and enable clocks, resets, and PHYs via rk_cdn_dp_enable.
 *   5. Optionally run the HPD + DPCD debug probe (tunable-gated).
 *
 * The driver is currently loaded as a module (not built into the kernel) so
 * that a panic during bring-up does not prevent the board from booting.
 */
static int
rk_cdn_dp_attach(device_t dev)
{
	struct rk_cdn_dp_softc *sc;
	struct sysctl_ctx_list *ctx;
	struct sysctl_oid *tree;
	int error;
	bool defer;

	sc = device_get_softc(dev);
	sc->dev = dev;
	sc->node = ofw_bus_get_node(dev);
	sc->has_extcon = OF_hasprop(sc->node, "extcon");
	sc->extcon_dev = NULL;
	sc->stage = RK_CDN_DP_STAGE_ATTACHED;
	sc->nphys = 0;
	sc->clks_enabled = false;
	sc->rsts_deasserted = false;
	sc->phys_enabled = false;
	sc->power_dev = NULL;
	{
		int i;
		for (i = 0; i < RK_CDN_DP_NCLKS; i++)
			sc->clks[i] = NULL;
		for (i = 0; i < RK_CDN_DP_NRSTS; i++)
			sc->rsts[i] = NULL;
		for (i = 0; i < RK_CDN_DP_MAX_PHYS; i++)
			sc->phys[i] = NULL;
	}

	error = bus_alloc_resources(dev, rk_cdn_dp_spec, sc->res);
	if (error != 0) {
		device_printf(dev, "cannot allocate resources\n");
		return (ENXIO);
	}

	error = rk_cdn_dp_get_power_domain(sc);
	if (error != 0 && error != ENXIO)
		goto fail;
	if (error == ENXIO) {
		/* Provider may attach later; staging keeps this safe. */
		sc->power_dev = NULL;
		error = 0;
	}
	error = rk_cdn_dp_get_extcon(sc);
	if (error != 0 && error != ENOENT) {
		device_printf(dev, "extcon provider state unavailable (%d)\n",
		    error);
		error = 0;
	}

	ctx = device_get_sysctl_ctx(dev);
	tree = device_get_sysctl_tree(dev);
	SYSCTL_ADD_PROC(ctx, SYSCTL_CHILDREN(tree), OID_AUTO,
	    "stage", CTLTYPE_INT | CTLFLAG_RW | CTLFLAG_MPSAFE,
	    sc, 0, rk_cdn_dp_sysctl_stage, "I",
	    "Bring-up stage (monotonic). 0=attached 1=power 2=handles 3=clocks 4=resets 5=phys 6=aux-probe");

	device_printf(dev, "Cadence DP scaffold attached: extcon=%s irq=%s\n",
	    sc->has_extcon ? "yes" : "no",
	    sc->res[RK_CDN_DP_RES_IRQ] != NULL ? "present" : "absent");
	if (sc->extcon_dev != NULL) {
		struct fusb302_typec_status tc;

		error = fusb302_get_typec_status(sc->extcon_dev, &tc);
		if (error == 0) {
			device_printf(dev,
			    "Type-C provider: attached=%s role=%s orient=%s vbusok=%s irq=%s togss=%u\n",
			    tc.attached ? "yes" : "no",
			    rk_cdn_dp_typec_role_name(tc.role),
			    rk_cdn_dp_typec_orientation_name(tc.orientation),
			    tc.vbusok ? "yes" : "no",
			    tc.has_irq ? "yes" : "no",
			    tc.togss_raw);
		} else {
			device_printf(dev,
			    "Type-C provider present but state is not ready yet (%d)\n",
			    error);
		}
	}

	defer = rk_cdn_dp_defer_enable();
	device_printf(dev, "bring-up deferred=%s (tunable hw.rk_cdn_dp_defer_enable)\n",
	    defer ? "yes" : "no");
	if (!defer) {
		error = rk_cdn_dp_set_stage(sc, RK_CDN_DP_STAGE_PHYS);
		if (error != 0)
			goto fail;
		if (rk_cdn_dp_attach_debug_probe())
			(void)rk_cdn_dp_set_stage(sc, RK_CDN_DP_STAGE_AUX_PROBE);
	}

	return (0);

fail:
	rk_cdn_dp_release(dev);
	return (ENXIO);
}

/*
 * rk_cdn_dp_release
 *
 * Tears down all resources acquired during attach in strict reverse order:
 * PHYs disabled and released before resets are re-asserted, resets before
 * clocks are disabled, so that no block loses its clock while still active.
 * Boolean flags (phys_enabled, rsts_deasserted, clks_enabled) ensure each
 * step is only attempted if the corresponding acquire succeeded, making this
 * safe to call from any point in a partial attach failure.
 */
static void
rk_cdn_dp_release(device_t dev)
{
	struct rk_cdn_dp_softc *sc;
	int i;

	sc = device_get_softc(dev);

	if (sc->phys_enabled) {
		for (i = 0; i < sc->nphys; i++)
			(void)phy_disable(sc->phys[i]);
		sc->phys_enabled = false;
	}

	for (i = 0; i < sc->nphys; i++) {
		if (sc->phys[i] != NULL) {
			phy_release(sc->phys[i]);
			sc->phys[i] = NULL;
		}
	}
	sc->nphys = 0;

	if (sc->rsts_deasserted) {
		for (i = RK_CDN_DP_NRSTS - 1; i >= 0; i--)
			(void)hwreset_assert(sc->rsts[i]);
		sc->rsts_deasserted = false;
	}
	for (i = 0; i < RK_CDN_DP_NRSTS; i++) {
		if (sc->rsts[i] != NULL) {
			hwreset_release(sc->rsts[i]);
			sc->rsts[i] = NULL;
		}
	}

	if (sc->clks_enabled) {
		for (i = RK_CDN_DP_NCLKS - 1; i >= 0; i--)
			(void)clk_disable(sc->clks[i]);
		sc->clks_enabled = false;
	}
	for (i = 0; i < RK_CDN_DP_NCLKS; i++) {
		if (sc->clks[i] != NULL) {
			clk_release(sc->clks[i]);
			sc->clks[i] = NULL;
		}
	}

	bus_release_resources(dev, rk_cdn_dp_spec, sc->res);
}

/*
 * rk_cdn_dp_detach
 *
 * Module detach entry point.  Delegates entirely to rk_cdn_dp_release so
 * that the teardown logic lives in one place and detach cannot diverge from
 * the partial-fail cleanup path in attach.
 */
static int
rk_cdn_dp_detach(device_t dev)
{
	rk_cdn_dp_release(dev);
	return (0);
}

static device_method_t rk_cdn_dp_methods[] = {
	DEVMETHOD(device_probe,		rk_cdn_dp_probe),
	DEVMETHOD(device_attach,	rk_cdn_dp_attach),
	DEVMETHOD(device_detach,	rk_cdn_dp_detach),

	DEVMETHOD_END
};

static driver_t rk_cdn_dp_driver = {
	"rk_cdn_dp",
	rk_cdn_dp_methods,
	sizeof(struct rk_cdn_dp_softc),
};

DRIVER_MODULE(rk_cdn_dp, simplebus, rk_cdn_dp_driver, 0, 0);
MODULE_VERSION(rk_cdn_dp, 1);
MODULE_DEPEND(rk_cdn_dp, ofwbus, 1, 1, 1);
MODULE_DEPEND(rk_cdn_dp, clk, 1, 1, 1);
MODULE_DEPEND(rk_cdn_dp, hwreset, 1, 1, 1);
MODULE_DEPEND(rk_cdn_dp, phy, 1, 1, 1);
MODULE_DEPEND(rk_cdn_dp, rk3399_power, 1, 1, 1);
