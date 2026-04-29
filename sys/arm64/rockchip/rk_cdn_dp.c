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
#include <sys/malloc.h>
#include <sys/module.h>
#include <sys/firmware.h>
#include <sys/linker.h>
#include <sys/sysctl.h>
#include <sys/taskqueue.h>
#include <sys/rman.h>

#include <machine/bus.h>

#include <dev/clk/clk.h>
#include <dev/hwreset/hwreset.h>
#include <dev/ofw/ofw_bus.h>
#include <dev/ofw/ofw_bus_subr.h>
#include <dev/ofw/openfirm.h>
#include <dev/phy/phy.h>
#include <dev/syscon/syscon.h>
#include <dev/iicbus/usb/fusb302_var.h>
#include "syscon_if.h"
#include "rk3399_power.h"
#include "rk3399_typec_altmode_var.h"

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
#define	RK_CDN_DP_DPCD_SINK_COUNT	0x200

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

/*
 * Cadence APB/mailbox register block.  The production path uses the
 * firmware-backed mailbox
 * path for DPCD/HPD/register access on RK3399 instead of programming the raw
 * AUX engine directly.
 */
#define	RK_CDN_DP_APB_CTRL		0x0000
#define	RK_CDN_DP_MAILBOX_FULL_ADDR	0x0008
#define	RK_CDN_DP_MAILBOX_EMPTY_ADDR	0x000c
#define	RK_CDN_DP_MAILBOX0_WR_DATA	0x0010
#define	RK_CDN_DP_MAILBOX0_RD_DATA	0x0014
#define	RK_CDN_DP_KEEP_ALIVE		0x0018
#define	RK_CDN_DP_VER_L		0x001c
#define	RK_CDN_DP_VER_H		0x0020
#define	RK_CDN_DP_VER_LIB_L_ADDR	0x0024
#define	RK_CDN_DP_VER_LIB_H_ADDR	0x0028
#define	RK_CDN_DP_SW_CLK_H		0x0040
#define	RK_CDN_DP_SW_EVENTS0		0x0044
#define	RK_CDN_DP_APB_INT_MASK		0x006c

#define	RK_CDN_DP_ADDR_IMEM		0x10000
#define	RK_CDN_DP_ADDR_DMEM		0x20000

#define	RK_CDN_DP_SOURCE_DPTX_CAR	0x0904
#define	RK_CDN_DP_SOURCE_PHY_CAR	0x0908
#define	RK_CDN_DP_SOURCE_PKT_CAR	0x0918
#define	RK_CDN_DP_SOURCE_AIF_CAR	0x091c
#define	RK_CDN_DP_SOURCE_CIPHER_CAR	0x0920
#define	RK_CDN_DP_SOURCE_CRYPTO_CAR	0x0924

#define	RK_CDN_DP_DP_AUX_SWAP_INVERSION_CONTROL	0x280c

#define	RK_CDN_DP_GRF_SOC_CON26		0x6268
#define	RK_CDN_DP_DPTX_HPD_SEL		(3U << 12)
#define	RK_CDN_DP_DPTX_HPD_DEL		(2U << 12)
#define	RK_CDN_DP_DPTX_HPD_SEL_MASK	(3U << 28)

#define	RK_CDN_DP_APB_IRAM_PATH		(1U << 2)
#define	RK_CDN_DP_APB_DRAM_PATH		(1U << 1)
#define	RK_CDN_DP_APB_XT_RESET		(1U << 0)

#define	RK_CDN_DP_DPTX_FRMR_DATA_CLK_RSTN_EN	(1U << 11)
#define	RK_CDN_DP_DPTX_FRMR_DATA_CLK_EN	(1U << 10)
#define	RK_CDN_DP_DPTX_PHY_DATA_RSTN_EN	(1U << 9)
#define	RK_CDN_DP_DPTX_PHY_DATA_CLK_EN	(1U << 8)
#define	RK_CDN_DP_DPTX_PHY_CHAR_RSTN_EN	(1U << 7)
#define	RK_CDN_DP_DPTX_PHY_CHAR_CLK_EN	(1U << 6)
#define	RK_CDN_DP_SOURCE_AUX_SYS_CLK_RSTN_EN	(1U << 5)
#define	RK_CDN_DP_SOURCE_AUX_SYS_CLK_EN	(1U << 4)
#define	RK_CDN_DP_DPTX_SYS_CLK_RSTN_EN	(1U << 3)
#define	RK_CDN_DP_DPTX_SYS_CLK_EN	(1U << 2)
#define	RK_CDN_DP_CFG_DPTX_VIF_CLK_RSTN_EN	(1U << 1)
#define	RK_CDN_DP_CFG_DPTX_VIF_CLK_EN	(1U << 0)
#define	RK_CDN_DP_SOURCE_PHY_RSTN_EN	(1U << 1)
#define	RK_CDN_DP_SOURCE_PHY_CLK_EN	(1U << 0)
#define	RK_CDN_DP_SOURCE_PKT_SYS_RSTN_EN	(1U << 3)
#define	RK_CDN_DP_SOURCE_PKT_SYS_CLK_EN	(1U << 2)
#define	RK_CDN_DP_SOURCE_PKT_DATA_RSTN_EN	(1U << 1)
#define	RK_CDN_DP_SOURCE_PKT_DATA_CLK_EN	(1U << 0)
#define	RK_CDN_DP_SPDIF_CDR_CLK_RSTN_EN	(1U << 5)
#define	RK_CDN_DP_SPDIF_CDR_CLK_EN	(1U << 4)
#define	RK_CDN_DP_SOURCE_AIF_SYS_RSTN_EN	(1U << 3)
#define	RK_CDN_DP_SOURCE_AIF_SYS_CLK_EN	(1U << 2)
#define	RK_CDN_DP_SOURCE_AIF_CLK_RSTN_EN	(1U << 1)
#define	RK_CDN_DP_SOURCE_AIF_CLK_EN	(1U << 0)
#define	RK_CDN_DP_SOURCE_CIPHER_SYSTEM_CLK_RSTN_EN	(1U << 3)
#define	RK_CDN_DP_SOURCE_CIPHER_SYS_CLK_EN	(1U << 2)
#define	RK_CDN_DP_SOURCE_CIPHER_CHAR_CLK_RSTN_EN	(1U << 1)
#define	RK_CDN_DP_SOURCE_CIPHER_CHAR_CLK_EN	(1U << 0)
#define	RK_CDN_DP_SOURCE_CRYPTO_SYS_CLK_RSTN_EN	(1U << 1)
#define	RK_CDN_DP_SOURCE_CRYPTO_SYS_CLK_EN	(1U << 0)

#define	RK_CDN_DP_MB_MODULE_ID_DP_TX	0x01
#define	RK_CDN_DP_MB_MODULE_ID_GENERAL	0x0a
#define	RK_CDN_DP_GENERAL_MAIN_CONTROL	0x01
#define	RK_CDN_DP_DPTX_SET_HOST_CAPABILITIES	0x01
#define	RK_CDN_DP_DPTX_READ_DPCD	0x03
#define	RK_CDN_DP_DPTX_ENABLE_EVENT	0x05
#define	RK_CDN_DP_DPTX_WRITE_REGISTER	0x06
#define	RK_CDN_DP_DPTX_READ_EVENT	0x0a
#define	RK_CDN_DP_DPTX_HPD_STATE	0x11

#define	RK_CDN_DP_FW_STANDBY		0
#define	RK_CDN_DP_FW_ACTIVE		1

#define	RK_CDN_DP_DPTX_EVENT_ENABLE_HPD	(1U << 0)
#define	RK_CDN_DP_DPTX_EVENT_ENABLE_TRAINING	(1U << 1)

#define	RK_CDN_DP_AUX_HOST_INVERT	3
#define	RK_CDN_DP_FAST_LT_NOT_SUPPORT	0
#define	RK_CDN_DP_LANE_MAPPING_NORMAL	0x1b
#define	RK_CDN_DP_LANE_MAPPING_FLIPPED	0xe4
#define	RK_CDN_DP_ENHANCED		1
#define	RK_CDN_DP_SCRAMBLER_EN		(1U << 4)
#define	RK_CDN_DP_VOLTAGE_LEVEL_2	2
#define	RK_CDN_DP_PRE_EMPHASIS_LEVEL_3	3
#define	RK_CDN_DP_PTS1			(1U << 0)
#define	RK_CDN_DP_PTS2			(1U << 1)
#define	RK_CDN_DP_PTS3			(1U << 2)
#define	RK_CDN_DP_PTS4			(1U << 3)
#define	RK_CDN_DP_MAX_LINK_RATE_CODE	0x14

#define	RK_CDN_DP_FW_ALIVE_TIMEOUT_US	1000000
#define	RK_CDN_DP_MAILBOX_RETRY_US	1000
#define	RK_CDN_DP_MAILBOX_TIMEOUT_US	5000000
#define	RK_CDN_DP_MAILBOX_READ_TIMEOUT_US	2000000
#define	RK_CDN_DP_DPCD_READ_RETRIES	3

#define	RK_CDN_DP_FIRMWARE_NAME		"rockchip/dptx.bin"
#define	RK_CDN_DP_FIRMWARE_PATH		"/boot/firmware/rockchip/dptx.bin"
#define	RK_CDN_DP_FIRMWARE_BASENAME	"dptx.bin"
#define	RK_CDN_DP_FIRMWARE_OLDNAME	"rk3399_dptx_fw"

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

#define	RK_CDN_DP_PMU_PWRDN_ST		0x0018
#define	RK_CDN_DP_PMU_BUS_IDLE_ST	0x0064
#define	RK_CDN_DP_PMU_BUS_IDLE_ACK	0x0068
#define	RK_CDN_DP_HDCP_BUS_BIT		11U

struct rk_cdn_dp_softc {
	device_t		dev;
	phandle_t		node;
	struct resource		*res[RK_CDN_DP_RES_COUNT];
	clk_t			clks[RK_CDN_DP_NCLKS];
	hwreset_t		rsts[RK_CDN_DP_NRSTS];
	phy_t			phys[RK_CDN_DP_MAX_PHYS];
	int			nphys;
	int			active_port;
	bool			rockpro64_typec0_only;
	bool			clks_enabled;
	bool			rsts_deasserted;
	bool			phys_enabled;
	bool			detached;
	bool			has_extcon;	/* cable orientation via extcon rather than fusb302 */
	device_t		extcon_dev;
	bool			has_power_domain;
	device_t		power_dev;	/* handle to rk3399_power provider */
	uint32_t		power_domain_id;
	struct syscon		*pmu_syscon;
	struct syscon		*grf;
	const struct firmware	*fw;
	bool			fw_active;
	uint32_t		fw_version;
	int			hpd_status;
	int			active_port_override;
	int			hostcap_lanes_override;
	int			hostcap_flip_override;
	int			hostcap_usb_ss_override;
	int			skip_aux_swap;
	uint32_t		aux_swap_value;
	int			dp_altmode_valid;
	int			dp_altmode_ready;
	int			dp_altmode_usb_ss;
	uint32_t		dp_altmode_pin_assignment;
	uint32_t		dp_altmode_status;

	/* Bring-up is staged so the module can be loaded safely. */
	int			stage;
	int			last_error;
	bool			allow_phys;
	bool			allow_aux;
	bool			aux_trace_reads;
	bus_size_t		aux_last_read_off;
	uint32_t		aux_last_read_val;
	bus_size_t		aux_last_write_off;
	uint32_t		aux_last_write_val;
	uint32_t		mbox_bad_header_count;
	uint32_t		mbox_last_header;
	uint32_t		mbox_last_expect;
	uint32_t		mbox_last_body0_3;
	uint32_t		mbox_last_body4;
	uint32_t		mbox_last_empty;
	uint32_t		mbox_last_full;
	uint32_t		mbox_last_empty_after_send;
	uint32_t		mbox_last_events0;
	uint32_t		mbox_last_keep_alive;
	uint32_t		mbox_last_apb_int_mask;
	uint32_t		mbox_last_send_header;
	uint32_t		mbox_last_send_size;
	uint32_t		mbox_last_send_written;
	uint32_t		mbox_last_write_full_first;
	uint32_t		mbox_last_write_full_last;
	uint32_t		mbox_last_write_full_polls;
	bool			aux_prepared;
	uint8_t			aux_pending_cmd;
	uint32_t		aux_pending_addr;
	int			aux_pending_len;
	uint8_t			aux_dpcd[RK_CDN_DP_DPCD_CAP_SIZE];
};

struct rk_cdn_dp_fw_header {
	uint32_t		size_bytes;
	uint32_t		header_size;
	uint32_t		iram_size;
	uint32_t		dram_size;
};

static void	rk_cdn_dp_release(device_t dev);
static int	rk_cdn_dp_probe_dpcd_caps(struct rk_cdn_dp_softc *sc);
static int	rk_cdn_dp_mailbox_get_firmware(struct rk_cdn_dp_softc *sc);
static int	rk_cdn_dp_mailbox_prepare_ucpu(struct rk_cdn_dp_softc *sc);
static int	rk_cdn_dp_mailbox_load_fw(struct rk_cdn_dp_softc *sc);
static int	rk_cdn_dp_mailbox_enable_events(struct rk_cdn_dp_softc *sc);
static int	rk_cdn_dp_select_hpd(struct rk_cdn_dp_softc *sc);
static int	rk_cdn_dp_mailbox_get_hpd_state(struct rk_cdn_dp_softc *sc);
static int	rk_cdn_dp_mailbox_set_host_cap(struct rk_cdn_dp_softc *sc);
static void	rk_cdn_dp_get_hostcap_config(struct rk_cdn_dp_softc *sc,
		    uint8_t *lanes, bool *flip);
static void	rk_cdn_dp_mailbox_drain(struct rk_cdn_dp_softc *sc, int limit);
static int	rk_cdn_dp_mailbox_drain_events(struct rk_cdn_dp_softc *sc);
static void	rk_cdn_dp_mailbox_capture_state(struct rk_cdn_dp_softc *sc);
static int	rk_cdn_dp_wait_sink_ready(struct rk_cdn_dp_softc *sc);
static int	rk_cdn_dp_mailbox_probe_dpcd_caps(struct rk_cdn_dp_softc *sc);
static bool	rk_cdn_dp_altmode_signature_ok(
		    const struct rk3399_typec_dp_altmode_status *status);
static int	rk_cdn_dp_get_power_domain(struct rk_cdn_dp_softc *sc);
static int	rk_cdn_dp_get_extcon(struct rk_cdn_dp_softc *sc);
static int	rk_cdn_dp_select_active_port(struct rk_cdn_dp_softc *sc);
static int	rk_cdn_dp_do_enable_phys(struct rk_cdn_dp_softc *sc);
static bool	rk_cdn_dp_is_rockpro64(device_t dev);
static bool	rk_cdn_dp_get_typec_status(struct rk_cdn_dp_softc *sc,
		    struct fusb302_typec_status *status);
static bool	rk_cdn_dp_get_altmode_status(struct rk_cdn_dp_softc *sc,
		    struct rk3399_typec_dp_altmode_status *status);
static int	rk_cdn_dp_lookup_typec_status_cb(linker_file_t lf, void *arg);
static int	(*rk_cdn_dp_lookup_typec_status(void))
		    (device_t, struct fusb302_typec_status *);
static int	rk_cdn_dp_lookup_altmode_status_cb(linker_file_t lf, void *arg);
static int	(*rk_cdn_dp_lookup_altmode_status(void))
		    (device_t, struct rk3399_typec_dp_altmode_status *);
static bool	rk_cdn_dp_power_domain_ready(struct rk_cdn_dp_softc *sc);
static bool	rk_cdn_dp_tunable_flag(const char *name, int defval);
static int	rk_cdn_dp_sysctl_flag(SYSCTL_HANDLER_ARGS);
static int	rk_cdn_dp_sysctl_hostcap_lanes(SYSCTL_HANDLER_ARGS);
static int	rk_cdn_dp_sysctl_hostcap_flip(SYSCTL_HANDLER_ARGS);
static int	rk_cdn_dp_sysctl_hostcap_usb_ss(SYSCTL_HANDLER_ARGS);
static int	rk_cdn_dp_sysctl_active_port(SYSCTL_HANDLER_ARGS);
static int	rk_cdn_dp_sysctl_reprobe(SYSCTL_HANDLER_ARGS);
static int	rk_cdn_dp_rebind_child(bool reprobe);
static void	rk_cdn_dp_reset_runtime_state(struct rk_cdn_dp_softc *sc);
static void	rk_cdn_dp_rebind_taskfn(void *context, int pending);
static int	rk_cdn_dp_module_event(module_t mod, int what, void *arg);

static int	rk_cdn_dp_rebind_attempts;
static int	rk_cdn_dp_rebind_matches;
static int	rk_cdn_dp_rebind_last_error;

SYSCTL_NODE(_hw, OID_AUTO, rk_cdn_dp, CTLFLAG_RD | CTLFLAG_MPSAFE, 0,
    "RK3399 CDN-DP module controls");
SYSCTL_INT(_hw_rk_cdn_dp, OID_AUTO, rebind_attempts, CTLFLAG_RD,
    &rk_cdn_dp_rebind_attempts, 0, "Number of explicit child reprobe attempts");
SYSCTL_INT(_hw_rk_cdn_dp, OID_AUTO, rebind_matches, CTLFLAG_RD,
    &rk_cdn_dp_rebind_matches, 0, "Number of matching CDN-DP OFW children found");
SYSCTL_INT(_hw_rk_cdn_dp, OID_AUTO, rebind_last_error, CTLFLAG_RD,
    &rk_cdn_dp_rebind_last_error, 0, "Last error returned by the explicit child reprobe");
SYSCTL_PROC(_hw_rk_cdn_dp, OID_AUTO, reprobe,
    CTLTYPE_INT | CTLFLAG_RW | CTLFLAG_MPSAFE, NULL, 0,
    rk_cdn_dp_sysctl_reprobe, "I",
    "Write 1 to reprobe the existing RK3399 CDN-DP child");

enum rk_cdn_dp_stage {
	RK_CDN_DP_STAGE_ATTACHED	= 0,
	RK_CDN_DP_STAGE_POWER		= 1,
	RK_CDN_DP_STAGE_HANDLES		= 2,
	RK_CDN_DP_STAGE_CLOCKS		= 3,
	RK_CDN_DP_STAGE_RESETS		= 4,
	RK_CDN_DP_STAGE_PHYS		= 5,
	RK_CDN_DP_STAGE_FW_GET		= 6,
	RK_CDN_DP_STAGE_FW_PREP		= 7,
	RK_CDN_DP_STAGE_FW_LOAD		= 8,
	RK_CDN_DP_STAGE_FW_ACTIVE	= 9,
	RK_CDN_DP_STAGE_HPD_SEL		= 10,
	RK_CDN_DP_STAGE_HPD_STATE	= 11,
	RK_CDN_DP_STAGE_HOSTCAP		= 12,
	RK_CDN_DP_STAGE_DPCD_READ	= 13,
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
	case RK_CDN_DP_STAGE_FW_GET:
		return ("fw-get");
	case RK_CDN_DP_STAGE_FW_PREP:
		return ("fw-prep");
	case RK_CDN_DP_STAGE_FW_LOAD:
		return ("fw-load");
	case RK_CDN_DP_STAGE_FW_ACTIVE:
		return ("fw-active");
	case RK_CDN_DP_STAGE_HPD_SEL:
		return ("hpd-sel");
	case RK_CDN_DP_STAGE_HPD_STATE:
		return ("hpd-state");
	case RK_CDN_DP_STAGE_HOSTCAP:
		return ("host-cap");
	case RK_CDN_DP_STAGE_DPCD_READ:
		return ("dpcd-read");
	default:
		return ("unknown");
	}
}

static const char *
rk_cdn_dp_reg_name(bus_size_t off)
{
	switch (off) {
	case RK_CDN_DP_SYS_CTL_3:
		return ("SYS_CTL_3");
	case RK_CDN_DP_DP_INT_STA:
		return ("DP_INT_STA");
	case RK_CDN_DP_AUX_CH_STA:
		return ("AUX_CH_STA");
	case RK_CDN_DP_AUX_ERR_NUM:
		return ("AUX_ERR_NUM");
	case RK_CDN_DP_BUFFER_DATA_CTL:
		return ("BUFFER_DATA_CTL");
	case RK_CDN_DP_AUX_CH_CTL_1:
		return ("AUX_CH_CTL_1");
	case RK_CDN_DP_AUX_ADDR_7_0:
		return ("AUX_ADDR_7_0");
	case RK_CDN_DP_AUX_ADDR_15_8:
		return ("AUX_ADDR_15_8");
	case RK_CDN_DP_AUX_ADDR_19_16:
		return ("AUX_ADDR_19_16");
	case RK_CDN_DP_AUX_CH_CTL_2:
		return ("AUX_CH_CTL_2");
	case RK_CDN_DP_BUF_DATA_0:
		return ("BUF_DATA_0");
	default:
		return ("unknown");
	}
}

/*
 * rk_cdn_dp_tunable_flag
 *
 * Reads a boolean-style loader tunable and returns it as a C bool.  Keeping
 * the parsing in one helper ensures every stage gate uses the same defaulting
 * rules and keeps attach compact.
 */
static bool
rk_cdn_dp_tunable_flag(const char *name, int defval)
{
	int value;

	value = defval;
	(void)TUNABLE_INT_FETCH(name, &value);
	return (value != 0);
}

/* FUSB302 integration is not wired up yet; keep this module self-contained. */

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
	if (sc->aux_trace_reads)
		sc->aux_last_read_off = off;
	sc->aux_last_read_val = bus_read_4(sc->res[RK_CDN_DP_RES_MEM], off);
	return (sc->aux_last_read_val);
}

static inline void
rk_cdn_dp_write_4(struct rk_cdn_dp_softc *sc, bus_size_t off, uint32_t val)
{
	if (sc->aux_trace_reads) {
		sc->aux_last_write_off = off;
		sc->aux_last_write_val = val;
	}
	bus_write_4(sc->res[RK_CDN_DP_RES_MEM], off, val);
}

static int
rk_cdn_dp_mailbox_mmio_ready(struct rk_cdn_dp_softc *sc)
{
	if (sc == NULL)
		return (ENXIO);
	if (sc->detached)
		return (ENXIO);
	if (sc->res[RK_CDN_DP_RES_MEM] == NULL)
		return (ENXIO);

	return (0);
}

static void
rk_cdn_dp_set_fw_clk(struct rk_cdn_dp_softc *sc, uint64_t hz)
{
	rk_cdn_dp_write_4(sc, RK_CDN_DP_SW_CLK_H, hz / 1000000);
}

static void
rk_cdn_dp_clock_reset(struct rk_cdn_dp_softc *sc)
{
	uint32_t val;

	val = RK_CDN_DP_DPTX_FRMR_DATA_CLK_RSTN_EN |
	    RK_CDN_DP_DPTX_FRMR_DATA_CLK_EN |
	    RK_CDN_DP_DPTX_PHY_DATA_RSTN_EN |
	    RK_CDN_DP_DPTX_PHY_DATA_CLK_EN |
	    RK_CDN_DP_DPTX_PHY_CHAR_RSTN_EN |
	    RK_CDN_DP_DPTX_PHY_CHAR_CLK_EN |
	    RK_CDN_DP_SOURCE_AUX_SYS_CLK_RSTN_EN |
	    RK_CDN_DP_SOURCE_AUX_SYS_CLK_EN |
	    RK_CDN_DP_DPTX_SYS_CLK_RSTN_EN |
	    RK_CDN_DP_DPTX_SYS_CLK_EN |
	    RK_CDN_DP_CFG_DPTX_VIF_CLK_RSTN_EN |
	    RK_CDN_DP_CFG_DPTX_VIF_CLK_EN;
	rk_cdn_dp_write_4(sc, RK_CDN_DP_SOURCE_DPTX_CAR, val);

	val = RK_CDN_DP_SOURCE_PHY_RSTN_EN | RK_CDN_DP_SOURCE_PHY_CLK_EN;
	rk_cdn_dp_write_4(sc, RK_CDN_DP_SOURCE_PHY_CAR, val);

	val = RK_CDN_DP_SOURCE_PKT_SYS_RSTN_EN |
	    RK_CDN_DP_SOURCE_PKT_SYS_CLK_EN |
	    RK_CDN_DP_SOURCE_PKT_DATA_RSTN_EN |
	    RK_CDN_DP_SOURCE_PKT_DATA_CLK_EN;
	rk_cdn_dp_write_4(sc, RK_CDN_DP_SOURCE_PKT_CAR, val);

	val = RK_CDN_DP_SPDIF_CDR_CLK_RSTN_EN |
	    RK_CDN_DP_SPDIF_CDR_CLK_EN |
	    RK_CDN_DP_SOURCE_AIF_SYS_RSTN_EN |
	    RK_CDN_DP_SOURCE_AIF_SYS_CLK_EN |
	    RK_CDN_DP_SOURCE_AIF_CLK_RSTN_EN |
	    RK_CDN_DP_SOURCE_AIF_CLK_EN;
	rk_cdn_dp_write_4(sc, RK_CDN_DP_SOURCE_AIF_CAR, val);

	val = RK_CDN_DP_SOURCE_CIPHER_SYSTEM_CLK_RSTN_EN |
	    RK_CDN_DP_SOURCE_CIPHER_SYS_CLK_EN |
	    RK_CDN_DP_SOURCE_CIPHER_CHAR_CLK_RSTN_EN |
	    RK_CDN_DP_SOURCE_CIPHER_CHAR_CLK_EN;
	rk_cdn_dp_write_4(sc, RK_CDN_DP_SOURCE_CIPHER_CAR, val);

	val = RK_CDN_DP_SOURCE_CRYPTO_SYS_CLK_RSTN_EN |
	    RK_CDN_DP_SOURCE_CRYPTO_SYS_CLK_EN;
	rk_cdn_dp_write_4(sc, RK_CDN_DP_SOURCE_CRYPTO_CAR, val);

	rk_cdn_dp_write_4(sc, RK_CDN_DP_APB_INT_MASK, 0);
}

static int
rk_cdn_dp_mailbox_read(struct rk_cdn_dp_softc *sc, uint8_t *val)
{
	int error;
	int i;

	error = rk_cdn_dp_mailbox_mmio_ready(sc);
	if (error != 0)
		return (error);

	for (i = 0; i < RK_CDN_DP_MAILBOX_READ_TIMEOUT_US / RK_CDN_DP_MAILBOX_RETRY_US;
	    i++) {
		if (rk_cdn_dp_read_4(sc, RK_CDN_DP_MAILBOX_EMPTY_ADDR) == 0) {
			*val = rk_cdn_dp_read_4(sc, RK_CDN_DP_MAILBOX0_RD_DATA) &
			    0xff;
			return (0);
		}
		DELAY(RK_CDN_DP_MAILBOX_RETRY_US);
	}

	return (ETIMEDOUT);
}

static void
rk_cdn_dp_mailbox_drain(struct rk_cdn_dp_softc *sc, int limit)
{
	uint8_t trash;
	int i;

	if (rk_cdn_dp_mailbox_mmio_ready(sc) != 0)
		return;

	for (i = 0; i < limit; i++) {
		if (rk_cdn_dp_read_4(sc, RK_CDN_DP_MAILBOX_EMPTY_ADDR) != 0)
			break;
		trash = rk_cdn_dp_read_4(sc, RK_CDN_DP_MAILBOX0_RD_DATA) & 0xff;
		(void)trash;
	}
}

static void
rk_cdn_dp_mailbox_capture_state(struct rk_cdn_dp_softc *sc)
{

	if (rk_cdn_dp_mailbox_mmio_ready(sc) != 0)
		return;

	sc->mbox_last_empty = rk_cdn_dp_read_4(sc, RK_CDN_DP_MAILBOX_EMPTY_ADDR);
	sc->mbox_last_full = rk_cdn_dp_read_4(sc, RK_CDN_DP_MAILBOX_FULL_ADDR);
	sc->mbox_last_events0 = rk_cdn_dp_read_4(sc, RK_CDN_DP_SW_EVENTS0);
	sc->mbox_last_keep_alive = rk_cdn_dp_read_4(sc, RK_CDN_DP_KEEP_ALIVE);
	sc->mbox_last_apb_int_mask = rk_cdn_dp_read_4(sc, RK_CDN_DP_APB_INT_MASK);
}

static int
rk_cdn_dp_mailbox_write(struct rk_cdn_dp_softc *sc, uint8_t val)
{
	uint32_t full, ka_start, ka_end;
	int error;
	int i;

	error = rk_cdn_dp_mailbox_mmio_ready(sc);
	if (error != 0)
		return (error);

	sc->mbox_last_write_full_first = 0xffffffff;
	sc->mbox_last_write_full_last = 0xffffffff;
	sc->mbox_last_write_full_polls = 0;
	ka_start = rk_cdn_dp_read_4(sc, RK_CDN_DP_KEEP_ALIVE);

	for (i = 0; i < RK_CDN_DP_MAILBOX_TIMEOUT_US / RK_CDN_DP_MAILBOX_RETRY_US;
	    i++) {
		full = rk_cdn_dp_read_4(sc, RK_CDN_DP_MAILBOX_FULL_ADDR);
		if (i == 0)
			sc->mbox_last_write_full_first = full;
		sc->mbox_last_write_full_last = full;
		sc->mbox_last_write_full_polls = i + 1;
		if (full == 0) {
			rk_cdn_dp_write_4(sc, RK_CDN_DP_MAILBOX0_WR_DATA, val);
			return (0);
		}
		/*
		 * Drain FW→HOST FIFO while waiting for FULL to clear.
		 * After AUX_SWAP_INVERSION_CONTROL the firmware sends
		 * unsolicited AUX-engine events; if we don't consume them the
		 * firmware's output FIFO fills, hardware flow-control sets
		 * FULL=1 on the HOST→FW side, and the firmware hangs.
		 */
		rk_cdn_dp_mailbox_drain(sc, 64);
		DELAY(RK_CDN_DP_MAILBOX_RETRY_US);
	}

	ka_end = rk_cdn_dp_read_4(sc, RK_CDN_DP_KEEP_ALIVE);
	device_printf(sc->dev,
	    "mailbox_write timeout: FULL never cleared, "
	    "keep_alive 0x%08x->0x%08x EMPTY=0x%x\n",
	    ka_start, ka_end,
	    rk_cdn_dp_read_4(sc, RK_CDN_DP_MAILBOX_EMPTY_ADDR));
	return (ETIMEDOUT);
}

static int
rk_cdn_dp_mailbox_read_receive(struct rk_cdn_dp_softc *sc, uint8_t *buf,
    uint16_t len)
{
	int error;
	uint16_t i;

	if (buf == NULL && len != 0)
		return (EINVAL);

	for (i = 0; i < len; i++) {
		error = rk_cdn_dp_mailbox_read(sc, &buf[i]);
		if (error != 0)
			return (error);
	}

	return (0);
}

static int
rk_cdn_dp_mailbox_validate_receive(struct rk_cdn_dp_softc *sc, uint8_t module_id,
    uint8_t opcode, uint16_t req_size)
{
	uint8_t header[4], trash;
	uint16_t mbox_size, i;
	int error;

	error = rk_cdn_dp_mailbox_read_receive(sc, header, sizeof(header));
	if (error != 0) {
		rk_cdn_dp_mailbox_drain(sc, 64);
		return (error);
	}

	mbox_size = ((uint16_t)header[2] << 8) | header[3];
	if (header[0] == opcode && header[1] == module_id &&
	    mbox_size == req_size)
		return (0);

	sc->mbox_bad_header_count++;
	sc->mbox_last_header = ((uint32_t)header[0] << 24) |
	    ((uint32_t)header[1] << 16) |
	    ((uint32_t)header[2] << 8) |
	    header[3];
	sc->mbox_last_expect = ((uint32_t)opcode << 24) |
	    ((uint32_t)module_id << 16) |
	    req_size;
	sc->mbox_last_body0_3 = 0;
	sc->mbox_last_body4 = 0;

	for (i = 0; i < mbox_size; i++) {
		error = rk_cdn_dp_mailbox_read(sc, &trash);
		if (error == 0) {
			if (i < 4) {
				sc->mbox_last_body0_3 |=
				    (uint32_t)trash << ((3 - i) * 8);
			} else if (i == 4) {
				sc->mbox_last_body4 = trash;
			}
		}
		if (error != 0)
			break;
	}

	return (EINVAL);
}

static int
rk_cdn_dp_mailbox_send(struct rk_cdn_dp_softc *sc, uint8_t module_id,
    uint8_t opcode, uint16_t size, const uint8_t *msg)
{
	uint8_t header[4];
	int error;
	uint16_t i;

	error = rk_cdn_dp_mailbox_mmio_ready(sc);
	if (error != 0) {
		if (sc != NULL && sc->dev != NULL)
			device_printf(sc->dev,
			    "mailbox send with missing MMIO resource\n");
		return (error);
	}
	if (size != 0 && msg == NULL)
		return (EINVAL);

	header[0] = opcode;
	header[1] = module_id;
	header[2] = (size >> 8) & 0xff;
	header[3] = size & 0xff;
	sc->mbox_last_send_header = ((uint32_t)header[0] << 24) |
	    ((uint32_t)header[1] << 16) |
	    ((uint32_t)header[2] << 8) | header[3];
	sc->mbox_last_send_size = sizeof(header) + size;
	sc->mbox_last_send_written = 0;

	for (i = 0; i < sizeof(header); i++) {
		error = rk_cdn_dp_mailbox_write(sc, header[i]);
		if (error != 0)
			return (error);
		sc->mbox_last_send_written++;
	}
	for (i = 0; i < size; i++) {
		error = rk_cdn_dp_mailbox_write(sc, msg[i]);
		if (error != 0)
			return (error);
		sc->mbox_last_send_written++;
	}
	if (rk_cdn_dp_mailbox_mmio_ready(sc) == 0)
		sc->mbox_last_empty_after_send =
		    rk_cdn_dp_read_4(sc, RK_CDN_DP_MAILBOX_EMPTY_ADDR);

	return (0);
}

static int
rk_cdn_dp_mailbox_reg_write(struct rk_cdn_dp_softc *sc, uint16_t addr, uint32_t val)
{
	uint8_t msg[6];

	msg[0] = (addr >> 8) & 0xff;
	msg[1] = addr & 0xff;
	msg[2] = (val >> 24) & 0xff;
	msg[3] = (val >> 16) & 0xff;
	msg[4] = (val >> 8) & 0xff;
	msg[5] = val & 0xff;

	return (rk_cdn_dp_mailbox_send(sc, RK_CDN_DP_MB_MODULE_ID_DP_TX,
	    RK_CDN_DP_DPTX_WRITE_REGISTER, sizeof(msg), msg));
}

static int
rk_cdn_dp_load_firmware(struct rk_cdn_dp_softc *sc)
{
	const struct rk_cdn_dp_fw_header *hdr;
	const uint8_t *data, *iram_data, *dram_data;
	uint32_t size_bytes, header_size, iram_size, dram_size, reg, val;
	uint32_t i;

	if (sc->fw == NULL)
		return (ENOENT);
	if (sc->fw->datasize < sizeof(*hdr))
		return (EINVAL);

	data = sc->fw->data;
	hdr = (const struct rk_cdn_dp_fw_header *)data;
	size_bytes = hdr->size_bytes;
	header_size = hdr->header_size;
	iram_size = hdr->iram_size;
	dram_size = hdr->dram_size;
	if (size_bytes != sc->fw->datasize || header_size > size_bytes ||
	    header_size + iram_size + dram_size > size_bytes)
		return (EINVAL);

	iram_data = data + header_size;
	dram_data = iram_data + iram_size;

	device_printf(sc->dev,
	    "fw-load: hdr=%u iram=%u dram=%u total=%u\n",
	    header_size, iram_size, dram_size, size_bytes);

	rk_cdn_dp_write_4(sc, RK_CDN_DP_APB_CTRL,
	    RK_CDN_DP_APB_IRAM_PATH |
	    RK_CDN_DP_APB_DRAM_PATH |
	    RK_CDN_DP_APB_XT_RESET);

	reg = rk_cdn_dp_read_4(sc, RK_CDN_DP_APB_CTRL);
	device_printf(sc->dev, "fw-load: APB_CTRL after hold=%08x\n", reg);

	for (i = 0; i < iram_size; i += 4) {
		val = ((const uint32_t *)iram_data)[i / 4];
		rk_cdn_dp_write_4(sc, RK_CDN_DP_ADDR_IMEM + i, val);
	}
	for (i = 0; i < dram_size; i += 4) {
		val = ((const uint32_t *)dram_data)[i / 4];
		rk_cdn_dp_write_4(sc, RK_CDN_DP_ADDR_DMEM + i, val);
	}

	/* verify first IMEM word was retained */
	val = rk_cdn_dp_read_4(sc, RK_CDN_DP_ADDR_IMEM);
	device_printf(sc->dev, "fw-load: IMEM[0] readback=0x%08x expected=0x%08x\n",
	    val, ((const uint32_t *)iram_data)[0]);

	reg = rk_cdn_dp_read_4(sc, RK_CDN_DP_KEEP_ALIVE);
	device_printf(sc->dev, "fw-load: KEEP_ALIVE before release=%u\n", reg);

	rk_cdn_dp_write_4(sc, RK_CDN_DP_APB_CTRL, 0);

	reg = rk_cdn_dp_read_4(sc, RK_CDN_DP_APB_CTRL);
	device_printf(sc->dev, "fw-load: APB_CTRL after release=%08x\n", reg);

	for (i = 0; i < RK_CDN_DP_FW_ALIVE_TIMEOUT_US / 2000; i++) {
		reg = rk_cdn_dp_read_4(sc, RK_CDN_DP_KEEP_ALIVE);
		if (reg != 0)
			break;
		if (i % 50 == 49)
			device_printf(sc->dev,
			    "fw-load: waiting for KEEP_ALIVE (%u ms elapsed)\n",
			    (i + 1) * 2);
		DELAY(2000);
	}
	device_printf(sc->dev, "fw-load: KEEP_ALIVE final=%u (i=%u)\n", reg, i);
	if (reg == 0)
		return (ETIMEDOUT);

	sc->fw_version = rk_cdn_dp_read_4(sc, RK_CDN_DP_VER_L) & 0xff;
	sc->fw_version |= (rk_cdn_dp_read_4(sc, RK_CDN_DP_VER_H) & 0xff) << 8;
	sc->fw_version |= (rk_cdn_dp_read_4(sc, RK_CDN_DP_VER_LIB_L_ADDR) & 0xff) << 16;
	sc->fw_version |= (rk_cdn_dp_read_4(sc, RK_CDN_DP_VER_LIB_H_ADDR) & 0xff) << 24;

	return (0);
}

static int
rk_cdn_dp_set_firmware_active(struct rk_cdn_dp_softc *sc, bool enable)
{
	uint8_t msg[5], val;
	int error;
	size_t i;

	msg[0] = RK_CDN_DP_GENERAL_MAIN_CONTROL;
	msg[1] = RK_CDN_DP_MB_MODULE_ID_GENERAL;
	msg[2] = 0;
	msg[3] = 1;
	msg[4] = enable ? RK_CDN_DP_FW_ACTIVE : RK_CDN_DP_FW_STANDBY;

	error = rk_cdn_dp_mailbox_mmio_ready(sc);
	if (error != 0)
		return (error);

	rk_cdn_dp_mailbox_drain(sc, 64);

	for (i = 0; i < nitems(msg); i++) {
		error = rk_cdn_dp_mailbox_write(sc, msg[i]);
		if (error != 0) {
			rk_cdn_dp_mailbox_drain(sc, 64);
			return (error);
		}
	}

	for (i = 0; i < nitems(msg); i++) {
		error = rk_cdn_dp_mailbox_read(sc, &val);
		if (error != 0) {
			rk_cdn_dp_mailbox_drain(sc, 64);
			return (error);
		}
		msg[i] = val;
	}

	sc->fw_active = enable;
	return (0);
}

static int
rk_cdn_dp_wait_keep_alive(struct rk_cdn_dp_softc *sc, uint32_t *lastp)
{
	uint32_t prev, cur;
	int i;

	prev = rk_cdn_dp_read_4(sc, RK_CDN_DP_KEEP_ALIVE);
	for (i = 0; i < RK_CDN_DP_FW_ALIVE_TIMEOUT_US / 2000; i++) {
		DELAY(2000);
		cur = rk_cdn_dp_read_4(sc, RK_CDN_DP_KEEP_ALIVE);
		if (cur != 0 && cur != prev) {
			if (lastp != NULL)
				*lastp = cur;
			return (0);
		}
		prev = cur;
	}
	if (lastp != NULL)
		*lastp = prev;
	return (ETIMEDOUT);
}

static int
rk_cdn_dp_event_config(struct rk_cdn_dp_softc *sc)
{
	uint8_t msg[5];

	memset(msg, 0, sizeof(msg));
	msg[0] = RK_CDN_DP_DPTX_EVENT_ENABLE_HPD |
	    RK_CDN_DP_DPTX_EVENT_ENABLE_TRAINING;

	return (rk_cdn_dp_mailbox_send(sc, RK_CDN_DP_MB_MODULE_ID_DP_TX,
	    RK_CDN_DP_DPTX_ENABLE_EVENT, sizeof(msg), msg));
}

static int
rk_cdn_dp_set_host_cap(struct rk_cdn_dp_softc *sc, uint8_t lanes, bool flip)
{
	uint8_t msg[8];
	int error;

	msg[0] = RK_CDN_DP_MAX_LINK_RATE_CODE;
	msg[1] = lanes | RK_CDN_DP_SCRAMBLER_EN;
	msg[2] = RK_CDN_DP_VOLTAGE_LEVEL_2;
	msg[3] = RK_CDN_DP_PRE_EMPHASIS_LEVEL_3;
	msg[4] = RK_CDN_DP_PTS1 | RK_CDN_DP_PTS2 |
	    RK_CDN_DP_PTS3 | RK_CDN_DP_PTS4;
	msg[5] = RK_CDN_DP_FAST_LT_NOT_SUPPORT;
	msg[6] = flip ? RK_CDN_DP_LANE_MAPPING_FLIPPED :
	    RK_CDN_DP_LANE_MAPPING_NORMAL;
	msg[7] = RK_CDN_DP_ENHANCED;

	error = rk_cdn_dp_mailbox_send(sc, RK_CDN_DP_MB_MODULE_ID_DP_TX,
	    RK_CDN_DP_DPTX_SET_HOST_CAPABILITIES, sizeof(msg), msg);
	if (error != 0)
		return (error);

	if (!sc->skip_aux_swap) {
		device_printf(sc->dev,
		    "host_cap: AUX_SWAP=0x%x\n", sc->aux_swap_value);
		error = rk_cdn_dp_mailbox_reg_write(sc,
		    RK_CDN_DP_DP_AUX_SWAP_INVERSION_CONTROL,
		    sc->aux_swap_value);
		if (error != 0)
			return (error);
	} else {
		device_printf(sc->dev, "host_cap: skipping AUX_SWAP write\n");
	}

	return (0);
}

static int
rk_cdn_dp_mailbox_dpcd_read(struct rk_cdn_dp_softc *sc, uint32_t addr,
    uint8_t *buf, uint16_t len)
{
	uint8_t msg[5], reg[5];
	uint32_t short_expect;
	int error, attempt;

	msg[0] = (len >> 8) & 0xff;
	msg[1] = len & 0xff;
	msg[2] = (addr >> 16) & 0xff;
	msg[3] = (addr >> 8) & 0xff;
	msg[4] = addr & 0xff;

	short_expect = ((uint32_t)RK_CDN_DP_DPTX_READ_DPCD << 24) |
	    ((uint32_t)RK_CDN_DP_MB_MODULE_ID_DP_TX << 16) |
	    sizeof(reg);

	for (attempt = 0; attempt < RK_CDN_DP_DPCD_READ_RETRIES; attempt++) {
		sc->mbox_last_header = 0;
		sc->mbox_last_expect = 0;
		sc->mbox_last_body0_3 = 0;
		sc->mbox_last_body4 = 0;
		rk_cdn_dp_mailbox_capture_state(sc);

		error = rk_cdn_dp_mailbox_send(sc, RK_CDN_DP_MB_MODULE_ID_DP_TX,
		    RK_CDN_DP_DPTX_READ_DPCD, sizeof(msg), msg);
		if (error != 0) {
			device_printf(sc->dev,
			    "mailbox DPCD send failed (%d)\n", error);
			return (error);
		}

		error = rk_cdn_dp_mailbox_validate_receive(sc,
		    RK_CDN_DP_MB_MODULE_ID_DP_TX, RK_CDN_DP_DPTX_READ_DPCD,
		    sizeof(reg) + len);
		if (error == 0)
			break;

		if (error == EINVAL && len > 0 &&
		    sc->mbox_last_header == short_expect) {
			error = rk_cdn_dp_mailbox_read_receive(sc, reg, sizeof(reg));
			if (error != 0) {
				device_printf(sc->dev,
				    "mailbox DPCD short reply reg failed (%d)\n",
				    error);
				return (error);
			}
			sc->mbox_last_body0_3 = ((uint32_t)reg[0] << 24) |
			    ((uint32_t)reg[1] << 16) |
			    ((uint32_t)reg[2] << 8) | reg[3];
			sc->mbox_last_body4 = reg[4];
			device_printf(sc->dev,
			    "mailbox DPCD short reply hdr=0x%08x body=%02x %02x %02x %02x %02x\n",
			    sc->mbox_last_header, reg[0], reg[1], reg[2], reg[3],
			    reg[4]);
			return (EPROTO);
		}

		if (error == ETIMEDOUT && attempt + 1 < RK_CDN_DP_DPCD_READ_RETRIES) {
			/*
			 * Drain any unsolicited/late firmware responses before
			 * retrying.  Without this, a late AUX-timeout response
			 * accumulates in the FW→HOST FIFO and we also send a
			 * second READ_DPCD while the firmware may still be
			 * processing the first, filling the HOST→FW FIFO and
			 * causing FULL=1.
			 */
			rk_cdn_dp_mailbox_drain(sc, 64);
			DELAY(5000);
			continue;
		}

		device_printf(sc->dev, "mailbox DPCD reply header failed (%d)\n",
		    error);
		return (error);
	}

	if (error != 0)
		return (error);

	error = rk_cdn_dp_mailbox_read_receive(sc, reg, sizeof(reg));
	if (error != 0) {
		device_printf(sc->dev, "mailbox DPCD reply reg failed (%d)\n",
		    error);
		return (error);
	}

	error = rk_cdn_dp_mailbox_read_receive(sc, buf, len);
	if (error != 0)
		device_printf(sc->dev, "mailbox DPCD payload failed (%d)\n",
		    error);
	return (error);
}

static int
rk_cdn_dp_mailbox_get_firmware(struct rk_cdn_dp_softc *sc)
{
	static const char * const fw_names[] = {
		RK_CDN_DP_FIRMWARE_NAME,
		RK_CDN_DP_FIRMWARE_PATH,
		RK_CDN_DP_FIRMWARE_BASENAME,
		RK_CDN_DP_FIRMWARE_OLDNAME,
	};
	const char *loaded;
	size_t i;

	if (sc->fw != NULL)
		return (0);

	loaded = NULL;
	for (i = 0; i < nitems(fw_names); i++) {
		sc->fw = firmware_get_flags(fw_names[i], FIRMWARE_GET_NOWARN);
		if (sc->fw != NULL) {
			loaded = fw_names[i];
			break;
		}
	}
	if (sc->fw == NULL) {
		device_printf(sc->dev,
		    "missing firmware (%s, %s, %s, %s)\n",
		    RK_CDN_DP_FIRMWARE_NAME, RK_CDN_DP_FIRMWARE_PATH,
		    RK_CDN_DP_FIRMWARE_BASENAME, RK_CDN_DP_FIRMWARE_OLDNAME);
		return (ENOENT);
	}
	device_printf(sc->dev, "using firmware %s (%zu bytes)\n",
	    loaded, sc->fw->datasize);

	return (0);
}

static int
rk_cdn_dp_mailbox_prepare_ucpu(struct rk_cdn_dp_softc *sc)
{
	uint64_t rate;
	int error;

	error = clk_get_freq(sc->clks[RK_CDN_DP_CLK_CORE], &rate);
	if (error != 0)
		return (error);

	rk_cdn_dp_set_fw_clk(sc, rate);
	rk_cdn_dp_clock_reset(sc);

	return (0);
}

static int
rk_cdn_dp_mailbox_load_fw(struct rk_cdn_dp_softc *sc)
{
	return (rk_cdn_dp_load_firmware(sc));
}

static int
rk_cdn_dp_mailbox_enable_events(struct rk_cdn_dp_softc *sc)
{
	return (rk_cdn_dp_event_config(sc));
}

static int
rk_cdn_dp_select_hpd(struct rk_cdn_dp_softc *sc)
{
	int error;

	if (sc->grf == NULL)
		return (ENXIO);

	error = clk_enable(sc->clks[RK_CDN_DP_CLK_GRF]);
	if (error != 0)
		return (error);
	error = SYSCON_WRITE_4(sc->grf, RK_CDN_DP_GRF_SOC_CON26,
	    RK_CDN_DP_DPTX_HPD_SEL_MASK | RK_CDN_DP_DPTX_HPD_SEL);
	(void)clk_disable(sc->clks[RK_CDN_DP_CLK_GRF]);
	return (error);
}

static int
rk_cdn_dp_mailbox_get_hpd_state(struct rk_cdn_dp_softc *sc)
{
	uint8_t status;
	int error;

	error = rk_cdn_dp_mailbox_send(sc, RK_CDN_DP_MB_MODULE_ID_DP_TX,
	    RK_CDN_DP_DPTX_HPD_STATE, 0, NULL);
	if (error != 0)
		return (error);
	error = rk_cdn_dp_mailbox_validate_receive(sc,
	    RK_CDN_DP_MB_MODULE_ID_DP_TX, RK_CDN_DP_DPTX_HPD_STATE,
	    sizeof(status));
	if (error != 0)
		return (error);
	error = rk_cdn_dp_mailbox_read_receive(sc, &status, sizeof(status));
	if (error != 0)
		return (error);

	sc->hpd_status = status;
	device_printf(sc->dev, "mailbox HPD status=%d\n", status);
	if (status <= 0)
		return (ENODEV);
	return (0);
}

static void
rk_cdn_dp_get_hostcap_config(struct rk_cdn_dp_softc *sc, uint8_t *lanes,
    bool *flip)
{
	struct fusb302_typec_status typec;
	struct rk3399_typec_dp_altmode_status altmode;

	/*
	 * Mirror the extcon semantics:
	 *   USB_SS = 1 => USB3 + DP => 2 lanes
	 *   USB_SS = 0 => DP-only    => 4 lanes
	 *
	 * Without a native FreeBSD extcon property provider yet, expose the
	 * same state explicitly through dev.rk_cdn_dp.0.hostcap_usb_ss.
	 */
	*lanes = 4;
	*flip = false;

	if (rk_cdn_dp_get_typec_status(sc, &typec)) {
		if (typec.orientation == FUSB302_TYPEC_ORIENT_CC2)
			*flip = true;
	}
	if (rk_cdn_dp_get_altmode_status(sc, &altmode)) {
		if (altmode.usb_ss >= 0)
			*lanes = altmode.usb_ss != 0 ? 2 : 4;
	}

	if (sc->hostcap_usb_ss_override >= 0)
		*lanes = sc->hostcap_usb_ss_override != 0 ? 2 : 4;
	if (sc->hostcap_lanes_override != 0)
		*lanes = sc->hostcap_lanes_override;
	if (sc->hostcap_flip_override >= 0)
		*flip = sc->hostcap_flip_override != 0;
}

static int
rk_cdn_dp_mailbox_set_host_cap(struct rk_cdn_dp_softc *sc)
{
	uint8_t lanes;
	bool flip;

	rk_cdn_dp_get_hostcap_config(sc, &lanes, &flip);

	device_printf(sc->dev,
	    "host-cap using lanes=%u flip=%u usb_ss=%d%s%s\n",
	    lanes, flip ? 1 : 0, sc->hostcap_usb_ss_override,
	    sc->hostcap_flip_override >= 0 ? " flip-override" : "",
	    sc->hostcap_lanes_override != 0 ? " lanes-override" : "");
	return (rk_cdn_dp_set_host_cap(sc, lanes, flip));
}

static int
rk_cdn_dp_mailbox_drain_events(struct rk_cdn_dp_softc *sc)
{
	uint8_t event[2];
	uint32_t pending;
	int error, i;

	for (i = 0; i < 16; i++) {
		pending = rk_cdn_dp_read_4(sc, RK_CDN_DP_SW_EVENTS0);
		if (pending == 0)
			return (0);

		error = rk_cdn_dp_mailbox_send(sc, RK_CDN_DP_MB_MODULE_ID_DP_TX,
		    RK_CDN_DP_DPTX_READ_EVENT, 0, NULL);
		if (error != 0)
			return (error);
		error = rk_cdn_dp_mailbox_validate_receive(sc,
		    RK_CDN_DP_MB_MODULE_ID_DP_TX, RK_CDN_DP_DPTX_READ_EVENT,
		    sizeof(event));
		if (error != 0)
			return (error);
		error = rk_cdn_dp_mailbox_read_receive(sc, event, sizeof(event));
		if (error != 0)
			return (error);
	}

	return (EAGAIN);
}

static int
rk_cdn_dp_wait_sink_ready(struct rk_cdn_dp_softc *sc)
{
	uint8_t sink_count;
	int error, i;

	for (i = 0; i < 20; i++) {
		error = rk_cdn_dp_mailbox_dpcd_read(sc,
		    RK_CDN_DP_DPCD_SINK_COUNT, &sink_count, 1);
		if (error == 0 && (sink_count & 0x3f) != 0)
			return (0);
		DELAY(10000);
	}

	return (ETIMEDOUT);
}

static int
rk_cdn_dp_mailbox_probe_dpcd_caps(struct rk_cdn_dp_softc *sc)
{
	struct rk3399_typec_dp_altmode_status altmode;
	int error;

	/*
	 * Do not wait for KEEP_ALIVE to change here.  The CDN-DP firmware
	 * only increments KEEP_ALIVE while actively processing mailbox
	 * commands; it stops when idle (waiting for the next command).
	 * Requiring KEEP_ALIVE to change before sending READ_DPCD creates a
	 * deadlock: we wait for the firmware to prove it's alive, but the
	 * firmware is waiting for us to give it work.  Linux does not do this
	 * check before dpcd_read — just send the command and wait for the
	 * response mailbox to fill.
	 */
	device_printf(sc->dev,
	    "probe_dpcd: enter FULL=0x%x EMPTY=0x%x keep_alive=0x%08x\n",
	    rk_cdn_dp_read_4(sc, RK_CDN_DP_MAILBOX_FULL_ADDR),
	    rk_cdn_dp_read_4(sc, RK_CDN_DP_MAILBOX_EMPTY_ADDR),
	    rk_cdn_dp_read_4(sc, RK_CDN_DP_KEEP_ALIVE));

	/*
	 * Drain unsolicited firmware→host events.  After AUX_SWAP processing
	 * the firmware may send an HPD/training event notification that arrives
	 * with a short delay (~10 ms).  Poll for up to 100 ms draining any data
	 * that arrives; once EMPTY=1 has been stable for 20 ms, the mailbox is
	 * quiet and we can safely send the first READ_DPCD command.
	 */
	{
		uint32_t stable = 0;
		int i;
		for (i = 0; i < 100; i++) {
			rk_cdn_dp_mailbox_drain(sc, 64);
			if (rk_cdn_dp_read_4(sc, RK_CDN_DP_MAILBOX_EMPTY_ADDR) != 0)
				stable++;
			else
				stable = 0;
			if (stable >= 20)
				break;
			DELAY(1000);
		}
	}

	device_printf(sc->dev,
	    "probe_dpcd: post-drain FULL=0x%x EMPTY=0x%x keep_alive=0x%08x\n",
	    rk_cdn_dp_read_4(sc, RK_CDN_DP_MAILBOX_FULL_ADDR),
	    rk_cdn_dp_read_4(sc, RK_CDN_DP_MAILBOX_EMPTY_ADDR),
	    rk_cdn_dp_read_4(sc, RK_CDN_DP_KEEP_ALIVE));

	if (rk_cdn_dp_get_altmode_status(sc, &altmode)) {
		device_printf(sc->dev,
		    "altmode state: ready=%u usb_ss=%d pin=0x%x dp_status=0x%x\n",
		    altmode.dp_ready ? 1 : 0, altmode.usb_ss,
		    altmode.pin_assignment, altmode.dp_status);
		if (!rk_cdn_dp_altmode_signature_ok(&altmode))
			return (EAGAIN);
	}

	error = rk_cdn_dp_wait_sink_ready(sc);
	if (error != 0) {
		device_printf(sc->dev,
		    "sink-count wait failed (%d), trying caps read anyway\n",
		    error);
	}

	error = rk_cdn_dp_mailbox_dpcd_read(sc, 0x000, sc->aux_dpcd,
	    sizeof(sc->aux_dpcd));
	if (error != 0)
		return (error);

	device_printf(sc->dev,
	    "mailbox DPCD rev=%#x max_link_rate=%#x max_lane_count=%#x fw=%#x\n",
	    sc->aux_dpcd[0], sc->aux_dpcd[1], sc->aux_dpcd[2], sc->fw_version);
	return (0);
}

static bool
rk_cdn_dp_altmode_signature_ok(
    const struct rk3399_typec_dp_altmode_status *status)
{

	if (status == NULL || !status->valid || !status->dp_ready)
		return (false);

	/*
	 * Working RockPro64 legacy state before CDN enable:
	 *   pin_assignment = 0x8
	 *   dp_status      = 0x9a, then 0x19a after enable/retrain
	 *
	 * Accept any status that carries the same 0x9a readiness bits so the
	 * later 0x19a transition remains valid too.
	 */
	if (status->pin_assignment != 0x8)
		return (false);
	if ((status->dp_status & 0x9a) != 0x9a)
		return (false);

	return (true);
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
	return (rk_cdn_dp_tunable_flag("hw.rk3399_typec_dp_force", 0));
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
	rk_cdn_dp_write_4(sc, RK_CDN_DP_SYS_CTL_3,
	    RK_CDN_DP_SYS_CTL_3_HPD_CTRL | RK_CDN_DP_SYS_CTL_3_F_HPD);
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
rk_cdn_dp_aux_prepare(struct rk_cdn_dp_softc *sc, uint8_t cmd, uint32_t addr,
    uint8_t *buf, int len)
{
	int i;

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
	if (((addr >> 16) & 0x0f) != 0)
		rk_cdn_dp_write_4(sc, RK_CDN_DP_AUX_ADDR_19_16,
		    (addr >> 16) & 0x0f);
	rk_cdn_dp_write_4(sc, RK_CDN_DP_AUX_CH_CTL_1,
	    (((len > 0 ? len - 1 : 0) & 0xf) << 4) | (cmd & 0xf));
	sc->aux_pending_cmd = cmd;
	sc->aux_pending_addr = addr;
	sc->aux_pending_len = len;
	sc->aux_prepared = true;

	return (0);
}

static int
rk_cdn_dp_aux_finish(struct rk_cdn_dp_softc *sc, uint8_t *buf, int len)
{
	uint32_t buf_ctl, sta;
	int count, error, i;

	if (!sc->aux_prepared)
		return (EINVAL);

	rk_cdn_dp_write_4(sc, RK_CDN_DP_AUX_CH_CTL_2,
	    (sc->aux_pending_len == 0 ? RK_CDN_DP_ADDR_ONLY : 0) |
	    RK_CDN_DP_AUX_EN);

	error = rk_cdn_dp_aux_wait(sc);
	if (error != 0) {
		device_printf(sc->dev, "AUX wait timeout cmd=0x%x addr=0x%x\n",
		    sc->aux_pending_cmd, sc->aux_pending_addr);
		return (error);
	}

	sta = rk_cdn_dp_read_4(sc, RK_CDN_DP_AUX_CH_STA);
	if ((sta & RK_CDN_DP_AUX_STATUS_MASK) != 0) {
		device_printf(sc->dev,
		    "AUX error cmd=0x%x addr=0x%x status=%s(%u) errnum=%u\n",
		    sc->aux_pending_cmd, sc->aux_pending_addr,
		    rk_cdn_dp_aux_status_name(sta),
		    sta & RK_CDN_DP_AUX_STATUS_MASK,
		    rk_cdn_dp_read_4(sc, RK_CDN_DP_AUX_ERR_NUM) & 0xff);
		return (EIO);
	}

	if (sc->aux_pending_cmd == RK_CDN_DP_AUX_CMD_NATIVE_READ) {
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

	sc->aux_prepared = false;

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
	int error;

	error = rk_cdn_dp_aux_prepare(sc, RK_CDN_DP_AUX_CMD_NATIVE_READ, 0x000,
	    sc->aux_dpcd, sizeof(sc->aux_dpcd));
	if (error != 0)
		return (error);

	return (0);
}

static int
rk_cdn_dp_complete_dpcd_caps(struct rk_cdn_dp_softc *sc)
{
	int error;

	error = rk_cdn_dp_aux_finish(sc, sc->aux_dpcd, sizeof(sc->aux_dpcd));
	if (error != 0)
		return (error);

	device_printf(sc->dev,
	    "DPCD rev=%#x max_link_rate=%#x max_lane_count=%#x\n",
	    sc->aux_dpcd[0], sc->aux_dpcd[1], sc->aux_dpcd[2]);
	device_printf(sc->dev,
	    "DPCD caps raw: %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x\n",
	    sc->aux_dpcd[0], sc->aux_dpcd[1], sc->aux_dpcd[2],
	    sc->aux_dpcd[3], sc->aux_dpcd[4], sc->aux_dpcd[5],
	    sc->aux_dpcd[6], sc->aux_dpcd[7], sc->aux_dpcd[8],
	    sc->aux_dpcd[9], sc->aux_dpcd[10], sc->aux_dpcd[11],
	    sc->aux_dpcd[12], sc->aux_dpcd[13], sc->aux_dpcd[14]);

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
	if (ofw_bus_search_compatible(dev, rk_cdn_dp_compat_data)->ocd_data == 0) {
		printf("rk_cdn_dp: probe reject %s%d compat mismatch\n",
		    device_get_name(dev), device_get_unit(dev));
		return (ENXIO);
	}

	printf("rk_cdn_dp: probe accept %s%d node=%s status=%s\n",
	    device_get_name(dev), device_get_unit(dev), ofw_bus_get_name(dev),
	    ofw_bus_status_okay(dev) ? "okay" : "disabled");
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
	if (sc->rockpro64_typec0_only && sc->nphys > 1) {
		device_printf(sc->dev,
		    "board=rockpro64: restricting CDN-DP to TYPEC0/PHY0 (ignoring %d extra phys)\n",
		    sc->nphys - 1);
		sc->nphys = 1;
	}

	return (0);
}

/*
 * rk_cdn_dp_get_power_domain
 *
 * Resolves the power-domain phandle from the DTS into a device handle and
 * domain ID that can be passed to rk3399_power_enable_domain_for_node.  The CDN-DP
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
	phandle_t pnode;
	phandle_t xref;
	int error, ncells;

	cells = NULL;
	error = ofw_bus_parse_xref_list_alloc(sc->node, "power-domains",
	    "#power-domain-cells", 0, &xref, &ncells, &cells);
	if (error == ENOENT)
		return (0);
	if (error != 0)
		return (error);
	sc->has_power_domain = true;
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

	pnode = ofw_bus_get_node(sc->power_dev);
	if (pnode > 0 &&
	    syscon_get_by_ofw_node(sc->dev, OF_parent(pnode),
	    &sc->pmu_syscon) != 0)
		sc->pmu_syscon = NULL;

	return (0);
}

/*
 * rk_cdn_dp_power_domain_ready
 *
 * Checks the RK3399 PMU state directly.  If the HDCP domain is already
 * powered and both bus-idle bits are clear, there is nothing left for the
 * rk3399_power provider to do and stage 1 can safely skip it.
 */
static bool
rk_cdn_dp_power_domain_ready(struct rk_cdn_dp_softc *sc)
{
	uint32_t bus_mask, domain_mask;
	uint32_t bus_ack, bus_st, pwrdn_st;

	if (!sc->has_power_domain || sc->pmu_syscon == NULL)
		return (false);

	domain_mask = (1U << sc->power_domain_id);
	bus_mask = (1U << RK_CDN_DP_HDCP_BUS_BIT);
	pwrdn_st = SYSCON_READ_4(sc->pmu_syscon, RK_CDN_DP_PMU_PWRDN_ST);
	bus_st = SYSCON_READ_4(sc->pmu_syscon, RK_CDN_DP_PMU_BUS_IDLE_ST);
	bus_ack = SYSCON_READ_4(sc->pmu_syscon, RK_CDN_DP_PMU_BUS_IDLE_ACK);

	return ((pwrdn_st & domain_mask) == 0 &&
	    (bus_st & bus_mask) == 0 &&
	    (bus_ack & bus_mask) == 0);
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
	devclass_t dc;
	pcell_t xref;
	ssize_t len;

	if (!sc->has_extcon)
		goto fallback;

	len = OF_getencprop(sc->node, "extcon", &xref, sizeof(xref));
	if (len <= 0)
		goto fallback;
	if (len < (ssize_t)sizeof(xref))
		return (EINVAL);

	sc->extcon_dev = OF_device_from_xref(xref);
	if (sc->extcon_dev == NULL) {
		device_printf(sc->dev,
		    "extcon provider is not attached yet\n");
		goto fallback;
	}

	return (0);

fallback:
	dc = devclass_find("fusb302");
	if (dc != NULL)
		sc->extcon_dev = devclass_get_device(dc, 0);
	if (sc->extcon_dev != NULL) {
		device_printf(sc->dev,
		    "using fusb302 fallback extcon provider %s%d\n",
		    device_get_name(sc->extcon_dev),
		    device_get_unit(sc->extcon_dev));
		return (0);
	}
	return (ENOENT);
}

static bool
rk_cdn_dp_is_rockpro64(device_t dev)
{
	phandle_t root;

	root = OF_finddevice("/");
	if (root <= 0)
		return (false);

	return (ofw_bus_node_is_compatible(root, "pine64,rockpro64") ||
	    ofw_bus_node_is_compatible(root, "pine64,rockpro64-v2.0") ||
	    ofw_bus_node_is_compatible(root, "pine64,rockpro64-v2.1"));
}

static bool
rk_cdn_dp_get_typec_status(struct rk_cdn_dp_softc *sc,
    struct fusb302_typec_status *status)
{
	int (*get_status)(device_t, struct fusb302_typec_status *);

	if (sc->extcon_dev == NULL || status == NULL)
		return (false);

	get_status = rk_cdn_dp_lookup_typec_status();
	if (get_status == NULL)
		return (false);

	return (get_status(sc->extcon_dev, status) == 0 &&
	    status->state_valid);
}

static bool
rk_cdn_dp_get_altmode_status(struct rk_cdn_dp_softc *sc,
    struct rk3399_typec_dp_altmode_status *status)
{
	int (*get_status)(device_t, struct rk3399_typec_dp_altmode_status *);

	if (status == NULL)
		return (false);

	get_status = rk_cdn_dp_lookup_altmode_status();
	if (get_status == NULL)
		return (false);
	if (get_status(sc->extcon_dev, status) != 0 || !status->valid)
		return (false);

	sc->dp_altmode_valid = status->valid ? 1 : 0;
	sc->dp_altmode_ready = status->dp_ready ? 1 : 0;
	sc->dp_altmode_usb_ss = status->usb_ss;
	sc->dp_altmode_pin_assignment = status->pin_assignment;
	sc->dp_altmode_status = status->dp_status;
	return (true);
}

static int
rk_cdn_dp_lookup_typec_status_cb(linker_file_t lf, void *arg)
{
	caddr_t sym;

	sym = linker_file_lookup_symbol(lf, "fusb302_get_typec_status", 0);
	if (sym == 0)
		return (0);

	*(caddr_t *)arg = sym;
	return (1);
}

static int
rk_cdn_dp_lookup_altmode_status_cb(linker_file_t lf, void *arg)
{
	caddr_t sym;

	sym = linker_file_lookup_symbol(lf, "fusb302_get_dp_altmode_state", 0);
	if (sym != 0) {
		*(caddr_t *)arg = sym;
		return (1);
	}

	sym = linker_file_lookup_symbol(lf, "rk3399_typec_dp_altmode_get_state", 0);
	if (sym == 0)
		return (0);

	*(caddr_t *)arg = sym;
	return (1);
}

static int
(*rk_cdn_dp_lookup_typec_status(void))(device_t, struct fusb302_typec_status *)
{
	caddr_t sym;

	sym = 0;
	(void)linker_file_foreach(rk_cdn_dp_lookup_typec_status_cb, &sym);
	return ((int (*)(device_t, struct fusb302_typec_status *))sym);
}

static int
(*rk_cdn_dp_lookup_altmode_status(void))(device_t,
    struct rk3399_typec_dp_altmode_status *)
{
	caddr_t sym;

	sym = 0;
	(void)linker_file_foreach(rk_cdn_dp_lookup_altmode_status_cb, &sym);
	return ((int (*)(device_t, struct rk3399_typec_dp_altmode_status *))sym);
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
rk_cdn_dp_select_active_port(struct rk_cdn_dp_softc *sc)
{
	struct fusb302_typec_status typec;

	if (sc->nphys <= 0)
		return (ENXIO);

	/*
	 * Mirror the connected-port choice as closely as current FreeBSD
	 * scaffolding allows.
	 *
	 * RockPro64 is special: board docs and schematic show that USB-C video
	 * is always on TYPEC0/PHY0, while the other RK3399 Type-C PHY is wired
	 * out as the fixed USB3 A-path. Cable orientation therefore changes AUX
	 * polarity/lane flip on the same TYPEC0 port; it does not select PHY1.
	 */
	sc->active_port = 0;
	if (sc->active_port_override >= 0 &&
	    sc->active_port_override < sc->nphys) {
		sc->active_port = sc->active_port_override;
		device_printf(sc->dev,
		    "active-port override=%d nphys=%d extcon=%s\n",
		    sc->active_port, sc->nphys,
		    sc->extcon_dev != NULL ? "yes" : "no");
		return (0);
	}
	if (sc->rockpro64_typec0_only) {
		device_printf(sc->dev,
		    "active-port=%d nphys=%d extcon=%s board=rockpro64-typec0\n",
		    sc->active_port, sc->nphys,
		    sc->extcon_dev != NULL ? "yes" : "no");
		return (0);
	}

	if (rk_cdn_dp_get_typec_status(sc, &typec)) {
		if (!typec.attached)
			return (ENODEV);
		if (typec.orientation == FUSB302_TYPEC_ORIENT_CC2 &&
		    sc->nphys > 1)
			sc->active_port = 1;
	}

	device_printf(sc->dev, "active-port=%d nphys=%d extcon=%s\n",
	    sc->active_port, sc->nphys, sc->extcon_dev != NULL ? "yes" : "no");
	return (0);
}

static int
rk_cdn_dp_do_enable_phys(struct rk_cdn_dp_softc *sc)
{
	int error, i;

	for (i = 0; i < sc->nphys; i++) {
		if (i != sc->active_port)
			continue;
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

/*
 * rk_cdn_dp_defer_enable
 *
 * Preserves the existing loader knob in case operators already set it, but
 * attach is now inert regardless.  The return value is only used for status
 * reporting so dmesg reflects the boot environment that the operator chose.
 */
static bool
rk_cdn_dp_defer_enable(void)
{
	return (rk_cdn_dp_tunable_flag("hw.rk_cdn_dp_defer_enable", 1));
}

/*
 * rk_cdn_dp_allow_phys
 *
 * Returns true only when the operator explicitly allows stage 5.  PHY lane
 * switching is one of the first places where board-specific instability can
 * wedge the machine, so keep it behind an opt-in gate even after the driver
 * has attached safely.
 */
static bool
rk_cdn_dp_allow_phys(struct rk_cdn_dp_softc *sc)
{
	return (sc->allow_phys);
}

/*
 * rk_cdn_dp_allow_aux
 *
 * Returns true only when the operator explicitly allows the firmware/mailbox
 * stages.  This remains a risky area, so keep it behind a second gate instead
 * of letting any later stage write reach the Cadence block automatically.
 */
static bool
rk_cdn_dp_allow_aux(struct rk_cdn_dp_softc *sc)
{
	return (sc->allow_aux);
}

/*
 * rk_cdn_dp_sysctl_flag
 *
 * Handles read/write boolean sysctls stored directly in the softc.  The same
 * helper backs allow_phys and allow_aux so operators can open those gates at
 * runtime without rebuilding the module.
 */
static int
rk_cdn_dp_sysctl_flag(SYSCTL_HANDLER_ARGS)
{
	bool *flag;
	int error, value;

	flag = arg1;
	value = *flag ? 1 : 0;
	error = sysctl_handle_int(oidp, &value, 0, req);
	if (error != 0 || req->newptr == NULL)
		return (error);
	*flag = (value != 0);
	return (0);
}

/*
 * rk_cdn_dp_sysctl_reprobe
 *
 * Exposes the single-child reprobe path through a module-global sysctl so it
 * can be driven explicitly from userspace on a stable kernel.  This avoids
 * depending on boot-time bus hooks while still letting us retry binding the
 * existing `dp@fec00000` OFW child after the module is loaded.
 */
static int
rk_cdn_dp_sysctl_reprobe(SYSCTL_HANDLER_ARGS)
{
	int error, value;

	value = 0;
	error = sysctl_handle_int(oidp, &value, 0, req);
	if (error != 0 || req->newptr == NULL)
		return (error);
	if (value != 1)
		return (EINVAL);

	error = rk_cdn_dp_rebind_child(true);
	rk_cdn_dp_rebind_last_error = error;
	return (error);
}

static int
rk_cdn_dp_sysctl_hostcap_lanes(SYSCTL_HANDLER_ARGS)
{
	int error, value;

	value = *(int *)arg1;
	error = sysctl_handle_int(oidp, &value, 0, req);
	if (error != 0 || req->newptr == NULL)
		return (error);
	if (value != 0 && value != 2 && value != 4)
		return (EINVAL);
	*(int *)arg1 = value;
	return (0);
}

static int
rk_cdn_dp_sysctl_hostcap_flip(SYSCTL_HANDLER_ARGS)
{
	int error, value;

	value = *(int *)arg1;
	error = sysctl_handle_int(oidp, &value, 0, req);
	if (error != 0 || req->newptr == NULL)
		return (error);
	if (value < -1 || value > 1)
		return (EINVAL);
	*(int *)arg1 = value;
	return (0);
}

static int
rk_cdn_dp_sysctl_hostcap_usb_ss(SYSCTL_HANDLER_ARGS)
{
	int error, value;

	value = *(int *)arg1;
	error = sysctl_handle_int(oidp, &value, 0, req);
	if (error != 0 || req->newptr == NULL)
		return (error);
	if (value < -1 || value > 1)
		return (EINVAL);
	*(int *)arg1 = value;
	return (0);
}

static int
rk_cdn_dp_sysctl_active_port(SYSCTL_HANDLER_ARGS)
{
	int error, value, max_port;
	struct rk_cdn_dp_softc *sc;

	sc = arg1;
	value = sc->active_port_override;
	error = sysctl_handle_int(oidp, &value, 0, req);
	if (error != 0 || req->newptr == NULL)
		return (error);
	max_port = sc->nphys > 0 ? sc->nphys - 1 : RK_CDN_DP_MAX_PHYS - 1;
	if (value < -1 || value > max_port)
		return (EINVAL);
	sc->active_port_override = value;
	return (0);
}

/*
 * rk_cdn_dp_rebind_child
 *
 * Finds the existing `dp@fec00000` OFW child beneath `ofwbus0` and optionally
 * reprobes just that child.  This is the safest currently available binding
 * path: it avoids broad bus rescans and only touches the single unattached
 * RK3399 CDN-DP node that already exists in the device tree.
 */
static int
rk_cdn_dp_rebind_child(bool reprobe)
{
	device_t *children, bus, child;
	devclass_t bus_dc, child_dc;
	phandle_t node;
	char path[128], status[32];
	ssize_t slen;
	int count, error, i;

	rk_cdn_dp_rebind_matches = 0;
	rk_cdn_dp_rebind_last_error = 0;
	children = NULL;
	error = ENXIO;

	bus_topo_lock();
	bus_dc = devclass_find("ofwbus");
	if (bus_dc == NULL) {
		error = ENXIO;
		goto out;
	}
	bus = devclass_get_device(bus_dc, 0);
	if (bus == NULL) {
		error = ENXIO;
		goto out;
	}
	if (device_get_children(bus, &children, &count) != 0) {
		error = ENXIO;
		goto out;
	}

	for (i = 0; i < count; i++) {
		child = children[i];
		node = ofw_bus_get_node(child);
		if (node <= 0)
			continue;
		if (!ofw_bus_node_is_compatible(node, "rockchip,rk3399-cdn-dp"))
			continue;
		path[0] = '\0';
		status[0] = '\0';
		OF_package_to_path(node, path, sizeof(path));
		slen = OF_getprop(node, "status", status, sizeof(status) - 1);
		if (slen > 0)
			status[slen] = '\0';
		else
			strcpy(status, "<absent>");
		printf("rk_cdn_dp: rebind match child=%s node=%s status=%s state=%d\n",
		    device_get_nameunit(child), path[0] != '\0' ? path : "<path?>",
		    status, device_get_state(child));
		rk_cdn_dp_rebind_matches++;
		if (!reprobe) {
			error = 0;
			continue;
		}
		if (device_get_state(child) != DS_NOTPRESENT) {
			error = EBUSY;
			continue;
		}
		if (!device_is_devclass_fixed(child)) {
			child_dc = device_get_devclass(child);
			if (child_dc != NULL &&
			    strcmp(devclass_get_name(child_dc), "unknown") == 0)
				(void)device_set_devclass(child, NULL);
		}
		error = device_probe_and_attach(child);
		if (error == 0) {
			rk_cdn_dp_rebind_last_error = 0;
			goto out;
		}
		rk_cdn_dp_rebind_last_error = error;
	}

	if (rk_cdn_dp_rebind_matches == 0)
		error = ENOENT;
out:
	if (children != NULL)
		free(children, M_TEMP);
	bus_topo_unlock();
	return (error);
}

/*
 * rk_cdn_dp_rebind_taskfn
 *
 * Runs one tick after module load so the standard DRIVER_MODULE() path has
 * already registered the driver with ofwbus.  Deferring the reprobe this way
 * avoids custom module registration glue while still keeping the rebinding
 * logic local to the DP module.
 */
static void
rk_cdn_dp_rebind_taskfn(void *context, int pending)
{
	rk_cdn_dp_rebind_attempts++;
	rk_cdn_dp_rebind_last_error = EOPNOTSUPP;
	printf("rk_cdn_dp: rebind task disabled during staged bring-up (pending=%d)\n",
	    pending);
}

/*
 * rk_cdn_dp_module_event
 *
 * Schedules or cancels the deferred single-child reprobe around normal module
 * lifecycle events.  The actual driver registration remains on the standard
 * DRIVER_MODULE() path so kldload registers a normal module entry and the
 * delayed callback only runs after registration has completed.
 */
static int
rk_cdn_dp_module_event(module_t mod, int what, void *arg)
{

	printf("rk_cdn_dp: module event what=%d\n", what);
	switch (what) {
	case MOD_LOAD:
		break;
	case MOD_QUIESCE:
	case MOD_UNLOAD:
		break;
	default:
		break;
	}

	return (0);
}

static int
rk_cdn_dp_set_stage(struct rk_cdn_dp_softc *sc, int target)
{
	int error, next;

	sc->last_error = 0;
	if (target < sc->stage)
		return (EINVAL);
	if (target > RK_CDN_DP_STAGE_DPCD_READ)
		return (EINVAL);

	for (next = sc->stage + 1; next <= target; next++) {
		if (next == RK_CDN_DP_STAGE_PHYS && !rk_cdn_dp_allow_phys(sc)) {
			device_printf(sc->dev,
			    "stage %d (%s) blocked; set dev.rk_cdn_dp.%d.allow_phys=1 first\n",
			    next, rk_cdn_dp_stage_name(next), device_get_unit(sc->dev));
			sc->last_error = EPERM;
			return (EPERM);
		}
		if (next >= RK_CDN_DP_STAGE_FW_GET && !rk_cdn_dp_allow_aux(sc)) {
			device_printf(sc->dev,
			    "stage %d (%s) blocked; set dev.rk_cdn_dp.%d.allow_aux=1 first\n",
			    next, rk_cdn_dp_stage_name(next), device_get_unit(sc->dev));
			sc->last_error = EPERM;
			return (EPERM);
		}
		device_printf(sc->dev, "stage %d (%s): begin\n",
		    next, rk_cdn_dp_stage_name(next));

		switch (next) {
		case RK_CDN_DP_STAGE_POWER:
			if (sc->has_power_domain) {
				if (sc->power_dev == NULL) {
					device_printf(sc->dev,
					    "power-domain provider not ready; load/enable rk3399_power first\n");
					sc->last_error = ENXIO;
					return (ENXIO);
				}
				if (rk_cdn_dp_power_domain_ready(sc)) {
					device_printf(sc->dev,
					    "power-domain %u already on and bus-open; skipping provider enable\n",
					    sc->power_domain_id);
				} else {
					error = rk3399_power_enable_domain(
					    sc->power_dev, sc->power_domain_id);
					if (error != 0) {
						device_printf(sc->dev,
						    "cannot enable power-domain %u\n",
						    sc->power_domain_id);
						sc->last_error = error;
						return (error);
					}
				}
				DELAY(1000);
			}
			break;
		case RK_CDN_DP_STAGE_HANDLES:
			error = rk_cdn_dp_get_clocks(sc);
			if (error != 0)
				sc->last_error = error;
			if (error != 0)
				return (error);
			error = rk_cdn_dp_get_resets(sc);
			if (error != 0)
				sc->last_error = error;
			if (error != 0)
				return (error);
			error = rk_cdn_dp_get_phys(sc);
			if (error != 0)
				sc->last_error = error;
			if (error != 0)
				return (error);
			break;
		case RK_CDN_DP_STAGE_CLOCKS:
			error = rk_cdn_dp_enable_clocks(sc);
			if (error != 0)
				sc->last_error = error;
			if (error != 0)
				return (error);
			break;
		case RK_CDN_DP_STAGE_RESETS:
			error = rk_cdn_dp_deassert_resets(sc);
			if (error != 0)
				sc->last_error = error;
			if (error != 0)
				return (error);
			break;
		case RK_CDN_DP_STAGE_PHYS:
			/*
			 * Stage 5 just enables the CDN-DP SOURCE_PHY_CAR (which
			 * provides the TC-PHY external PSM clock source). Actual
			 * phy_enable happens at end of stage 9, after CDN-DP
			 * firmware is active and driving PIPE PowerDown correctly.
			 * This matches the Linux cdn_dp_clk_enable → firmware_init
			 * → enable_phy ordering.
			 */
			rk_cdn_dp_clock_reset(sc);
			error = rk_cdn_dp_select_active_port(sc);
			if (error != 0) {
				sc->last_error = error;
				return (error);
			}
			break;
		case RK_CDN_DP_STAGE_FW_GET:
			error = rk_cdn_dp_mailbox_get_firmware(sc);
			if (error != 0) {
				sc->last_error = error;
				device_printf(sc->dev, "firmware get failed (%d)\n",
				    error);
				return (error);
			}
			break;
		case RK_CDN_DP_STAGE_FW_PREP:
			error = rk_cdn_dp_mailbox_prepare_ucpu(sc);
			if (error != 0) {
				sc->last_error = error;
				device_printf(sc->dev, "firmware prep failed (%d)\n",
				    error);
				return (error);
			}
			break;
		case RK_CDN_DP_STAGE_FW_LOAD:
			error = rk_cdn_dp_mailbox_load_fw(sc);
			if (error != 0) {
				sc->last_error = error;
				device_printf(sc->dev, "firmware load failed (%d)\n",
				    error);
				return (error);
			}
			break;
		case RK_CDN_DP_STAGE_FW_ACTIVE:
			/* Linux: firmware_init = set_firmware_active + event_config */
			error = rk_cdn_dp_set_firmware_active(sc, true);
			if (error != 0) {
				sc->last_error = error;
				device_printf(sc->dev, "firmware activate failed (%d)\n",
				    error);
				return (error);
			}
			error = rk_cdn_dp_wait_keep_alive(sc,
			    &sc->mbox_last_keep_alive);
			if (error != 0) {
				sc->last_error = error;
				device_printf(sc->dev,
				    "firmware keep_alive failed (0x%08x, %d)\n",
				    sc->mbox_last_keep_alive, error);
				return (error);
			}
			error = rk_cdn_dp_event_config(sc);
			if (error != 0) {
				sc->last_error = error;
				device_printf(sc->dev,
				    "event_config failed (%d)\n", error);
				return (error);
			}
			/* Linux: enable_phy = phy_power_on then HPD_SEL (stage 10) */
			(void)rk_cdn_dp_do_enable_phys(sc);
			break;
		case RK_CDN_DP_STAGE_HPD_SEL:
			error = rk_cdn_dp_select_hpd(sc);
			if (error != 0) {
				sc->last_error = error;
				device_printf(sc->dev, "HPD select failed (%d)\n",
				    error);
				return (error);
			}
			break;
		case RK_CDN_DP_STAGE_HPD_STATE:
			error = rk_cdn_dp_mailbox_get_hpd_state(sc);
			if (error != 0) {
				sc->last_error = error;
				device_printf(sc->dev, "HPD state failed (%d)\n",
				    error);
				return (error);
			}
			break;
		case RK_CDN_DP_STAGE_HOSTCAP:
			/* Linux: set_host_cap after get_hpd_status (stage 11) */
			error = rk_cdn_dp_mailbox_set_host_cap(sc);
			if (error != 0) {
				sc->last_error = error;
				device_printf(sc->dev, "set_host_cap failed (%d)\n",
				    error);
				return (error);
			}
			break;
		case RK_CDN_DP_STAGE_DPCD_READ:
			error = rk_cdn_dp_mailbox_probe_dpcd_caps(sc);
			if (error != 0) {
				sc->last_error = error;
				device_printf(sc->dev, "mailbox DPCD read failed (%d)\n",
				    error);
				return (error);
			}
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

/*
 * rk_cdn_dp_sysctl_stage
 *
 * Exposes the monotonic stage controller through sysctl.  Reads report the
 * current stage, and writes advance the driver one step at a time subject to
 * the allow_phys/allow_aux gates.
 */
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

static void
rk_cdn_dp_reset_runtime_state(struct rk_cdn_dp_softc *sc)
{
	sc->stage = RK_CDN_DP_STAGE_ATTACHED;
	sc->last_error = 0;
	sc->active_port = -1;
	sc->allow_phys = rk_cdn_dp_tunable_flag("hw.rk_cdn_dp_allow_phys", 0);
	sc->allow_aux = rk_cdn_dp_tunable_flag("hw.rk_cdn_dp_allow_aux", 0);
	sc->hpd_status = -1;
	sc->active_port_override = -1;
	sc->hostcap_lanes_override = 0;
	sc->hostcap_flip_override = -1;
	sc->hostcap_usb_ss_override = -1;
	sc->skip_aux_swap = 0;
	sc->aux_swap_value = RK_CDN_DP_AUX_HOST_INVERT;
	sc->dp_altmode_valid = 0;
	sc->dp_altmode_ready = 0;
	sc->dp_altmode_usb_ss = -1;
	sc->dp_altmode_pin_assignment = 0;
	sc->dp_altmode_status = 0;
	sc->aux_trace_reads = false;
	sc->aux_last_read_off = 0;
	sc->aux_last_read_val = 0;
	sc->aux_last_write_off = 0;
	sc->aux_last_write_val = 0;
	sc->mbox_bad_header_count = 0;
	sc->mbox_last_header = 0;
	sc->mbox_last_expect = 0;
	sc->mbox_last_body0_3 = 0;
	sc->mbox_last_body4 = 0;
	sc->mbox_last_empty = 0;
	sc->mbox_last_full = 0;
	sc->mbox_last_empty_after_send = 0;
	sc->mbox_last_events0 = 0;
	sc->mbox_last_keep_alive = 0;
	sc->mbox_last_apb_int_mask = 0;
	sc->mbox_last_send_header = 0;
	sc->mbox_last_send_size = 0;
	sc->mbox_last_send_written = 0;
	sc->mbox_last_write_full_first = 0;
	sc->mbox_last_write_full_last = 0;
	sc->mbox_last_write_full_polls = 0;
	sc->aux_prepared = false;
	sc->aux_pending_cmd = 0;
	sc->aux_pending_addr = 0;
	sc->aux_pending_len = 0;
	sc->fw_active = false;
	sc->fw_version = 0;
}

/*
 * rk_cdn_dp_attach
 *
 * Top-level attach.  The safe-mode attach path is intentionally inert:
 *
 *   1. Allocate MMIO and IRQ resources (no dangerous register programming).
 *   2. Resolve optional provider handles (power-domain and extcon).
 *   3. Register sysctls that let an operator advance one stage at a time.
 *   4. Stop at stage 0 until the operator explicitly opts into later stages.
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
	sc->rockpro64_typec0_only = rk_cdn_dp_is_rockpro64(dev);
	sc->has_extcon = OF_hasprop(sc->node, "extcon");
	sc->extcon_dev = NULL;
	sc->detached = false;
	rk_cdn_dp_reset_runtime_state(sc);
	sc->nphys = 0;
	sc->clks_enabled = false;
	sc->rsts_deasserted = false;
	sc->phys_enabled = false;
	sc->has_power_domain = false;
	sc->power_dev = NULL;
	sc->pmu_syscon = NULL;
	sc->grf = NULL;
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
	device_printf(dev, "attach: resources allocated\n");

	error = rk_cdn_dp_get_power_domain(sc);
	if (error != 0 && error != ENXIO) {
		device_printf(dev, "attach: power-domain lookup failed (%d)\n",
		    error);
		goto fail;
	}
	if (error == ENXIO) {
		/* Provider may attach later; staging keeps this safe. */
		device_printf(dev,
		    "attach: power-domain provider not ready yet; deferring provider use\n");
		sc->power_dev = NULL;
		error = 0;
	}
	device_printf(dev,
	    "attach: power-domain state has_prop=%s provider=%s id=%u\n",
	    sc->has_power_domain ? "yes" : "no",
	    sc->power_dev != NULL ? "yes" : "no", sc->power_domain_id);
	error = rk_cdn_dp_get_extcon(sc);
	if (error != 0 && error != ENOENT) {
		device_printf(dev, "extcon provider state unavailable (%d)\n",
		    error);
		error = 0;
	}
	device_printf(dev, "attach: extcon dt=%s provider=%s\n",
	    sc->has_extcon ? "yes" : "no",
	    sc->extcon_dev != NULL ? device_get_nameunit(sc->extcon_dev) : "none");
	if (OF_hasprop(sc->node, "rockchip,grf") &&
	    syscon_get_by_ofw_property(dev, sc->node, "rockchip,grf",
	    &sc->grf) != 0) {
		device_printf(dev, "cannot get rockchip,grf handle\n");
		error = ENXIO;
		goto fail;
	}
	device_printf(dev, "attach: grf=%s\n", sc->grf != NULL ? "yes" : "no");

	ctx = device_get_sysctl_ctx(dev);
	tree = device_get_sysctl_tree(dev);
	SYSCTL_ADD_PROC(ctx, SYSCTL_CHILDREN(tree), OID_AUTO,
	    "stage", CTLTYPE_INT | CTLFLAG_RW | CTLFLAG_MPSAFE,
	    sc, 0, rk_cdn_dp_sysctl_stage, "I",
	    "Bring-up stage (monotonic). 0=attached 1=power 2=handles 3=clocks 4=resets 5=phys 6=fw-get 7=fw-prep 8=fw-load 9=fw-active 10=hpd-sel 11=hpd-state 12=host-cap 13=dpcd-read");
	SYSCTL_ADD_PROC(ctx, SYSCTL_CHILDREN(tree), OID_AUTO,
	    "allow_phys", CTLTYPE_INT | CTLFLAG_RW | CTLFLAG_MPSAFE,
	    &sc->allow_phys, 0, rk_cdn_dp_sysctl_flag, "I",
	    "Allow manual entry into stage 5 (PHY mode switch and enable)");
	SYSCTL_ADD_PROC(ctx, SYSCTL_CHILDREN(tree), OID_AUTO,
	    "allow_aux", CTLTYPE_INT | CTLFLAG_RW | CTLFLAG_MPSAFE,
	    &sc->allow_aux, 0, rk_cdn_dp_sysctl_flag, "I",
	    "Allow manual entry into stages 6-13 (firmware/mailbox bring-up)");
	SYSCTL_ADD_INT(ctx, SYSCTL_CHILDREN(tree), OID_AUTO,
	    "last_error", CTLFLAG_RD, &sc->last_error, 0,
	    "Last non-zero error returned by a staged bring-up step");
	SYSCTL_ADD_INT(ctx, SYSCTL_CHILDREN(tree), OID_AUTO,
	    "hpd_status", CTLFLAG_RD, &sc->hpd_status, 0,
	    "Last mailbox HPD status");
	SYSCTL_ADD_PROC(ctx, SYSCTL_CHILDREN(tree), OID_AUTO,
	    "active_port", CTLTYPE_INT | CTLFLAG_RW | CTLFLAG_MPSAFE,
	    sc, 0, rk_cdn_dp_sysctl_active_port, "I",
	    "Active TYPE-C DP port override: -1=auto, otherwise 0..nphys-1");
	SYSCTL_ADD_PROC(ctx, SYSCTL_CHILDREN(tree), OID_AUTO,
	    "hostcap_lanes", CTLTYPE_INT | CTLFLAG_RW | CTLFLAG_MPSAFE,
	    &sc->hostcap_lanes_override, 0, rk_cdn_dp_sysctl_hostcap_lanes,
	    "I", "Host-cap lanes override: 0=auto, 2 or 4 explicit");
	SYSCTL_ADD_PROC(ctx, SYSCTL_CHILDREN(tree), OID_AUTO,
	    "hostcap_flip", CTLTYPE_INT | CTLFLAG_RW | CTLFLAG_MPSAFE,
	    &sc->hostcap_flip_override, 0, rk_cdn_dp_sysctl_hostcap_flip,
	    "I", "Host-cap flip override: -1=auto, 0=normal, 1=flipped");
	SYSCTL_ADD_PROC(ctx, SYSCTL_CHILDREN(tree), OID_AUTO,
	    "hostcap_usb_ss", CTLTYPE_INT | CTLFLAG_RW | CTLFLAG_MPSAFE,
	    &sc->hostcap_usb_ss_override, 0, rk_cdn_dp_sysctl_hostcap_usb_ss,
	    "I", "USB_SS state: -1=auto, 0=DP-only(4 lanes), 1=USB3+DP(2 lanes)");
	SYSCTL_ADD_INT(ctx, SYSCTL_CHILDREN(tree), OID_AUTO,
	    "skip_aux_swap", CTLFLAG_RW, &sc->skip_aux_swap, 0,
	    "Skip WRITE_REGISTER(AUX_SWAP_INVERSION_CONTROL) in host-cap (debug)");
	SYSCTL_ADD_U32(ctx, SYSCTL_CHILDREN(tree), OID_AUTO,
	    "aux_swap_value", CTLFLAG_RW, &sc->aux_swap_value, 0,
	    "Value for AUX_SWAP_INVERSION_CONTROL (0=none,1=invert,3=invert+init)");
	SYSCTL_ADD_INT(ctx, SYSCTL_CHILDREN(tree), OID_AUTO,
	    "dp_altmode_valid", CTLFLAG_RD, &sc->dp_altmode_valid, 0,
	    "Last observed DP Alt Mode helper presence");
	SYSCTL_ADD_INT(ctx, SYSCTL_CHILDREN(tree), OID_AUTO,
	    "dp_altmode_ready", CTLFLAG_RD, &sc->dp_altmode_ready, 0,
	    "Last observed DP Alt Mode ready state");
	SYSCTL_ADD_INT(ctx, SYSCTL_CHILDREN(tree), OID_AUTO,
	    "dp_altmode_usb_ss", CTLFLAG_RD, &sc->dp_altmode_usb_ss, 0,
	    "Last observed DP Alt Mode USB_SS state");
	SYSCTL_ADD_U32(ctx, SYSCTL_CHILDREN(tree), OID_AUTO,
	    "dp_altmode_pin_assignment", CTLFLAG_RD,
	    &sc->dp_altmode_pin_assignment, 0,
	    "Last observed DP Alt Mode pin assignment");
	SYSCTL_ADD_U32(ctx, SYSCTL_CHILDREN(tree), OID_AUTO,
	    "dp_altmode_status", CTLFLAG_RD, &sc->dp_altmode_status, 0,
	    "Last observed DP Alt Mode status value");
	SYSCTL_ADD_ULONG(ctx, SYSCTL_CHILDREN(tree), OID_AUTO,
	    "aux_last_read_off", CTLFLAG_RD, &sc->aux_last_read_off,
	    "Last stage-6 MMIO read offset");
	SYSCTL_ADD_U32(ctx, SYSCTL_CHILDREN(tree), OID_AUTO,
	    "aux_last_read_val", CTLFLAG_RD, &sc->aux_last_read_val, 0,
	    "Last stage-6 MMIO read value");
	SYSCTL_ADD_ULONG(ctx, SYSCTL_CHILDREN(tree), OID_AUTO,
	    "aux_last_write_off", CTLFLAG_RD, &sc->aux_last_write_off,
	    "Last stage-6 MMIO write offset");
	SYSCTL_ADD_U32(ctx, SYSCTL_CHILDREN(tree), OID_AUTO,
	    "aux_last_write_val", CTLFLAG_RD, &sc->aux_last_write_val, 0,
	    "Last stage-6 MMIO write value");
	SYSCTL_ADD_U32(ctx, SYSCTL_CHILDREN(tree), OID_AUTO,
	    "mbox_bad_header_count", CTLFLAG_RD, &sc->mbox_bad_header_count, 0,
	    "Count of unexpected Cadence mailbox headers");
	SYSCTL_ADD_U32(ctx, SYSCTL_CHILDREN(tree), OID_AUTO,
	    "mbox_last_header", CTLFLAG_RD, &sc->mbox_last_header, 0,
	    "Last unexpected mailbox header bytes as 0xAABBCCDD");
	SYSCTL_ADD_U32(ctx, SYSCTL_CHILDREN(tree), OID_AUTO,
	    "mbox_last_expect", CTLFLAG_RD, &sc->mbox_last_expect, 0,
	    "Expected mailbox header as 0xAABBCCCC (opcode,module,size)");
	SYSCTL_ADD_U32(ctx, SYSCTL_CHILDREN(tree), OID_AUTO,
	    "mbox_last_body0_3", CTLFLAG_RD, &sc->mbox_last_body0_3, 0,
	    "First four bytes drained from the last unexpected mailbox payload");
	SYSCTL_ADD_U32(ctx, SYSCTL_CHILDREN(tree), OID_AUTO,
	    "mbox_last_body4", CTLFLAG_RD, &sc->mbox_last_body4, 0,
	    "Fifth byte drained from the last unexpected mailbox payload");
	SYSCTL_ADD_U32(ctx, SYSCTL_CHILDREN(tree), OID_AUTO,
	    "mbox_last_empty", CTLFLAG_RD, &sc->mbox_last_empty, 0,
	    "MAILBOX_EMPTY_ADDR sampled before READ_DPCD");
	SYSCTL_ADD_U32(ctx, SYSCTL_CHILDREN(tree), OID_AUTO,
	    "mbox_last_full", CTLFLAG_RD, &sc->mbox_last_full, 0,
	    "MAILBOX_FULL_ADDR sampled before READ_DPCD");
	SYSCTL_ADD_U32(ctx, SYSCTL_CHILDREN(tree), OID_AUTO,
	    "mbox_last_empty_after_send", CTLFLAG_RD,
	    &sc->mbox_last_empty_after_send, 0,
	    "MAILBOX_EMPTY_ADDR sampled immediately after mailbox_send");
	SYSCTL_ADD_U32(ctx, SYSCTL_CHILDREN(tree), OID_AUTO,
	    "mbox_last_events0", CTLFLAG_RD, &sc->mbox_last_events0, 0,
	    "SW_EVENTS0 sampled before READ_DPCD");
	SYSCTL_ADD_U32(ctx, SYSCTL_CHILDREN(tree), OID_AUTO,
	    "mbox_last_keep_alive", CTLFLAG_RD, &sc->mbox_last_keep_alive, 0,
	    "KEEP_ALIVE sampled before READ_DPCD");
	SYSCTL_ADD_U32(ctx, SYSCTL_CHILDREN(tree), OID_AUTO,
	    "mbox_last_apb_int_mask", CTLFLAG_RD, &sc->mbox_last_apb_int_mask, 0,
	    "APB_INT_MASK sampled before READ_DPCD");
	SYSCTL_ADD_U32(ctx, SYSCTL_CHILDREN(tree), OID_AUTO,
	    "mbox_last_send_header", CTLFLAG_RD, &sc->mbox_last_send_header, 0,
	    "Last mailbox send header as 0xAABBCCDD");
	SYSCTL_ADD_U32(ctx, SYSCTL_CHILDREN(tree), OID_AUTO,
	    "mbox_last_send_size", CTLFLAG_RD, &sc->mbox_last_send_size, 0,
	    "Last mailbox send total byte count");
	SYSCTL_ADD_U32(ctx, SYSCTL_CHILDREN(tree), OID_AUTO,
	    "mbox_last_send_written", CTLFLAG_RD, &sc->mbox_last_send_written, 0,
	    "Last mailbox send successfully written byte count");
	SYSCTL_ADD_U32(ctx, SYSCTL_CHILDREN(tree), OID_AUTO,
	    "mbox_last_write_full_first", CTLFLAG_RD,
	    &sc->mbox_last_write_full_first, 0,
	    "First MAILBOX_FULL_ADDR sample seen by mailbox_write");
	SYSCTL_ADD_U32(ctx, SYSCTL_CHILDREN(tree), OID_AUTO,
	    "mbox_last_write_full_last", CTLFLAG_RD,
	    &sc->mbox_last_write_full_last, 0,
	    "Last MAILBOX_FULL_ADDR sample seen by mailbox_write");
	SYSCTL_ADD_U32(ctx, SYSCTL_CHILDREN(tree), OID_AUTO,
	    "mbox_last_write_full_polls", CTLFLAG_RD,
	    &sc->mbox_last_write_full_polls, 0,
	    "MAILBOX_FULL_ADDR poll count in last mailbox_write");

	device_printf(dev, "Cadence DP scaffold attached: extcon=%s irq=%s\n",
	    sc->has_extcon ? "yes" : "no",
	    sc->res[RK_CDN_DP_RES_IRQ] != NULL ? "present" : "absent");
	if (sc->rockpro64_typec0_only)
		device_printf(dev,
		    "board recipe: RockPro64 USB-C DP is fixed to TYPEC0/PHY0\n");
	if (sc->extcon_dev != NULL)
		device_printf(dev, "Type-C extcon provider: %s\n",
		    device_get_nameunit(sc->extcon_dev));

	defer = rk_cdn_dp_defer_enable();
	device_printf(dev,
	    "bring-up deferred=%s allow_phys=%s allow_aux=%s\n",
	    defer ? "yes" : "no",
	    sc->allow_phys ? "yes" : "no",
	    sc->allow_aux ? "yes" : "no");
	if (!defer)
		device_printf(dev,
		    "automatic bring-up is intentionally disabled; use dev.rk_cdn_dp.%d.stage manually\n",
		    device_get_unit(dev));

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
	sc->detached = true;

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

	if (sc->fw != NULL) {
		firmware_put(sc->fw, FIRMWARE_UNLOAD);
		sc->fw = NULL;
	}
	sc->fw_active = false;
	sc->fw_version = 0;

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

DRIVER_MODULE(rk_cdn_dp, ofwbus, rk_cdn_dp_driver, rk_cdn_dp_module_event,
    NULL);
MODULE_VERSION(rk_cdn_dp, 1);
/*
 * Avoid MODULE_DEPEND() on in-kernel subsystems (clk/hwreset/phy/ofwbus) here.
 * On our target images these are built into the kernel (no clk.ko/hwreset.ko/
 * phy.ko), and declaring a dependency prevents kldload from working.
 */
