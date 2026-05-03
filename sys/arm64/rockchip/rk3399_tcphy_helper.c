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
 * Post-boot RK3399 Type-C PHY helper.
 *
 * This module intentionally avoids touching the early rk_typec_phy driver.
 * It resolves the live TYPEC0 PHY node and applies a small set of GRF fields
 * after boot, on explicit request through sysctl.
 */

#include <sys/param.h>
#include <sys/kernel.h>
#include <sys/module.h>
#include <sys/systm.h>
#include <sys/cons.h>
#include <sys/sysctl.h>
#include <sys/bus.h>
#include <sys/kdb.h>
#include <sys/stdarg.h>

#include <dev/ofw/ofw_bus.h>
#include <dev/ofw/openfirm.h>
#include <dev/syscon/syscon.h>
#include <vm/pmap.h>

#include "syscon_if.h"

#define	RK3399_TCPHY_PATH			"/phy@ff7c0000"
#define	RK3399_TCPHY0_BASE			0xff7c0000
#define	RK3399_TCPHY0_SIZE			0x40000
#define	RK3399_CRU_BASE				0xff760000
#define	RK3399_CRU_SIZE				0x10000
#define	RK3399_CRU_SOFTRST_CON12		0x0430
#define	RK3399_CRU_CLKGATE_CON13		(0x300 + (13 * 4))
#define	RK3399_CRU_CLKGATE_CON21		(0x300 + (21 * 4))
#define	RK3399_CRU_SRST_UPHY0			(1U << 0)
#define	RK3399_CRU_SRST_UPHY0_PIPE_L00		(1U << 1)
#define	RK3399_CRU_SRST_P_UPHY0_TCPHY		(1U << 2)
#define	RK3399_TCPHY_PMA_LANE_CFG		(0xc000 << 2)
#define	RK3399_TCPHY_PIPE_CMN_CTRL1		(0xc001 << 2)
#define	RK3399_TCPHY_PIPE_CMN_CTRL2		(0xc002 << 2)
#define	RK3399_TCPHY_DP_MODE_CTL		(0xc008 << 2)
#define	RK3399_TCPHY_DP_CLK_CTL			(0xc009 << 2)
#define	RK3399_TCPHY_PHY_DP_TX_CTL		(0xc408 << 2)
#define	RK3399_TCPHY_PMA_CMN_CTRL1		(0xc800 << 2)
#define	RK3399_TCPHY_PIN_ASSIGN_C_E		0x51d9
#define	RK3399_TCPHY_PIN_ASSIGN_D_F		0x5100
#define	RK3399_TCPHY_DP_MODE_A0			(1U << 4)
#define	RK3399_TCPHY_DP_MODE_A2			(1U << 6)
#define	RK3399_TCPHY_DP_MODE_ENTER_A0		0xc101
#define	RK3399_TCPHY_DP_MODE_ENTER_A2		0xc104
#define	RK3399_TCPHY_CMN_PLLSM1_USER_DEF_CTRL	(0x37 << 2)
#define	RK3399_TCPHY_CMN_PLL1_VCOCAL_START	(0xa1 << 2)
#define	RK3399_TCPHY_CMN_PLL1_VCOCAL_INIT	(0xa4 << 2)
#define	RK3399_TCPHY_CMN_PLL1_VCOCAL_ITER	(0xa5 << 2)
#define	RK3399_TCPHY_CMN_PLL1_INTDIV		(0xb4 << 2)
#define	RK3399_TCPHY_CMN_PLL1_FRACDIV		(0xb5 << 2)
#define	RK3399_TCPHY_CMN_PLL1_HIGH_THR		(0xb6 << 2)
#define	RK3399_TCPHY_CMN_PLL1_DSM_DIAG		(0xb7 << 2)
#define	RK3399_TCPHY_CMN_PLL1_SS_CTRL1		(0xb8 << 2)
#define	RK3399_TCPHY_CMN_PLL1_SS_CTRL2		(0xb9 << 2)
#define	RK3399_TCPHY_CMN_TXPUCAL_CTRL		(0x00e0 << 2)
#define	RK3399_TCPHY_CMN_TXPDCAL_CTRL		(0x00f0 << 2)
#define	RK3399_TCPHY_CMN_TXPU_ADJ_CTRL		(0x0108 << 2)
#define	RK3399_TCPHY_CMN_TXPD_ADJ_CTRL		(0x010c << 2)
#define	RK3399_TCPHY_CMN_DIAG_PLL1_FBH_OVRD	(0x1d0 << 2)
#define	RK3399_TCPHY_CMN_DIAG_PLL1_FBL_OVRD	(0x1d1 << 2)
#define	RK3399_TCPHY_CMN_DIAG_PLL1_OVRD		(0x1d2 << 2)
#define	RK3399_TCPHY_CMN_DIAG_PLL1_V2I_TUNE	(0x1d5 << 2)
#define	RK3399_TCPHY_CMN_DIAG_PLL1_CP_TUNE	(0x1d6 << 2)
#define	RK3399_TCPHY_CMN_DIAG_PLL1_LF_PROG	(0x1d7 << 2)
#define	RK3399_TCPHY_CMN_DIAG_PLL1_PTATIS_TUNE1	(0x1d8 << 2)
#define	RK3399_TCPHY_CMN_DIAG_PLL1_PTATIS_TUNE2	(0x1d9 << 2)
#define	RK3399_TCPHY_CMN_DIAG_PLL1_INCLK_CTRL	(0x1da << 2)
#define	RK3399_TCPHY_CMN_DIAG_HSCLK_SEL		(0x1e0 << 2)
#define	RK3399_TCPHY_XCVR_PSM_RCTRL(n)		((0x4001 | ((n) << 9)) << 2)
#define	RK3399_TCPHY_TX_TXCC_CPOST_MULT_00(n)	((0x404c | ((n) << 9)) << 2)
#define	RK3399_TCPHY_TX_TXCC_CPOST_MULT_01(n)	((0x404d | ((n) << 9)) << 2)
#define	RK3399_TCPHY_TX_TXCC_CPOST_MULT_10(n)	((0x404e | ((n) << 9)) << 2)
#define	RK3399_TCPHY_TX_TXCC_CPOST_MULT_11(n)	((0x404f | ((n) << 9)) << 2)
#define	RK3399_TCPHY_TX_TXCC_MGNFS_MULT_000(n)	((0x4050 | ((n) << 9)) << 2)
#define	RK3399_TCPHY_TX_TXCC_MGNFS_MULT_001(n)	((0x4051 | ((n) << 9)) << 2)
#define	RK3399_TCPHY_TX_TXCC_MGNFS_MULT_010(n)	((0x4052 | ((n) << 9)) << 2)
#define	RK3399_TCPHY_TX_TXCC_MGNFS_MULT_011(n)	((0x4053 | ((n) << 9)) << 2)
#define	RK3399_TCPHY_TX_TXCC_MGNFS_MULT_100(n)	((0x4054 | ((n) << 9)) << 2)
#define	RK3399_TCPHY_TX_TXCC_MGNFS_MULT_101(n)	((0x4055 | ((n) << 9)) << 2)
#define	RK3399_TCPHY_TX_TXCC_MGNFS_MULT_110(n)	((0x4056 | ((n) << 9)) << 2)
#define	RK3399_TCPHY_TX_TXCC_MGNFS_MULT_111(n)	((0x4057 | ((n) << 9)) << 2)
#define	RK3399_TCPHY_XCVR_DIAG_PLLDRC_CTRL(n)	((0x40e0 | ((n) << 9)) << 2)
#define	RK3399_TCPHY_XCVR_DIAG_LANE_FCM_EN_MGN(n) \
	((0x40f2 | ((n) << 9)) << 2)
#define	RK3399_TCPHY_TX_PSC_A0(n)		((0x4100 | ((n) << 9)) << 2)
#define	RK3399_TCPHY_TX_PSC_A1(n)		((0x4101 | ((n) << 9)) << 2)
#define	RK3399_TCPHY_TX_PSC_A2(n)		((0x4102 | ((n) << 9)) << 2)
#define	RK3399_TCPHY_TX_PSC_A3(n)		((0x4103 | ((n) << 9)) << 2)
#define	RK3399_TCPHY_TX_RCVDET_EN_TMR(n)	((0x4122 | ((n) << 9)) << 2)
#define	RK3399_TCPHY_TX_RCVDET_ST_TMR(n)	((0x4123 | ((n) << 9)) << 2)
#define	RK3399_TCPHY_TX_DIAG_TX_DRV(n)		((0x41e1 | ((n) << 9)) << 2)
#define	RK3399_TCPHY_TX_ANA_CTRL_REG_1		(0x5020 << 2)
#define	RK3399_TCPHY_TX_ANA_CTRL_REG_2		(0x5021 << 2)
#define	RK3399_TCPHY_TXDA_COEFF_CALC_CTRL	(0x5022 << 2)
#define	RK3399_TCPHY_TX_DIG_CTRL_REG_2		(0x5024 << 2)
#define	RK3399_TCPHY_TXDA_CYA_AUXDA_CYA		(0x5025 << 2)
#define	RK3399_TCPHY_TX_ANA_CTRL_REG_3		(0x5026 << 2)
#define	RK3399_TCPHY_TX_ANA_CTRL_REG_4		(0x5027 << 2)
#define	RK3399_TCPHY_TX_ANA_CTRL_REG_5		(0x5029 << 2)
#define	RK3399_TCPHY_AUX_CH_LANE		8
#define	RK3399_TCPHY_TX_TXCC_CAL_SCLR_MULT(n)	(((0x4047 | ((n) << 9))) << 2)
#define	RK3399_TCPHY_TXDA_DP_AUX_EN		(1U << 15)
#define	RK3399_TCPHY_TXDA_CAL_LATCH_EN		(1U << 13)
#define	RK3399_TCPHY_AUXDA_POLARITY		(1U << 12)
#define	RK3399_TCPHY_TXDA_BGREF_EN		(1U << 8)
#define	RK3399_TCPHY_TXDA_DRV_LDO_EN		(1U << 7)
#define	RK3399_TCPHY_TXDA_DECAP_EN_DEL		(1U << 6)
#define	RK3399_TCPHY_TXDA_DECAP_EN		(1U << 5)
#define	RK3399_TCPHY_TXDA_UPHY_SUPPLY_EN_DEL	(1U << 4)
#define	RK3399_TCPHY_TXDA_UPHY_SUPPLY_EN	(1U << 3)
#define	RK3399_TCPHY_XCVR_DECAP_EN_DEL		(1U << 9)
#define	RK3399_TCPHY_XCVR_DECAP_EN		(1U << 8)
#define	RK3399_TCPHY_TXDA_DRV_PREDRV_EN_DEL	(1U << 1)
#define	RK3399_TCPHY_TXDA_DRV_PREDRV_EN		(1U << 0)
#define	RK3399_TCPHY_TX_HIGH_Z_TM_EN		(1U << 15)
#define	RK3399_TCPHY_TX_RESCAL_CODE_OFFSET	0
#define	RK3399_TCPHY_TX_RESCAL_CODE_MASK	0x3f
#define	RK3399_TCPHY_CLK_PLL_CONFIG		0x30
#define	RK3399_TCPHY_CLK_PLL_MASK		0x33
#define	RK3399_TCPHY_DP_PLL_CLOCK_ENABLE	(1U << 2)
#define	RK3399_TCPHY_DP_PLL_ENABLE		(1U << 0)
#define	RK3399_TCPHY_DP_PLL_DATA_RATE_RBR	((2U << 12) | (4U << 8))

#define	RK3399_TCPHY_HELPER_F_CONN_DIR		(1U << 0)
#define	RK3399_TCPHY_HELPER_F_EXT_PSM		(1U << 1)
#define	RK3399_TCPHY_HELPER_F_UPHY_DP_SEL	(1U << 2)
#define	RK3399_TCPHY_HELPER_F_PMA_LANE_CFG	(1U << 3)
#define	RK3399_TCPHY_HELPER_F_DP_MODE		(1U << 4)
#define	RK3399_TCPHY_HELPER_F_AUX_CAL		(1U << 5)
#define	RK3399_TCPHY_HELPER_F_24M		(1U << 6)
#define	RK3399_TCPHY_HELPER_F_DP_PLL		(1U << 7)
#define	RK3399_TCPHY_HELPER_F_AUX_FLIP		(1U << 8)
#define	RK3399_TCPHY_HELPER_F_DP_CFG_LANES	(1U << 9)
#define	RK3399_TCPHY_HELPER_F_PMA_READY		(1U << 10)
#define	RK3399_TCPHY_HELPER_F_PIPE_RST		(1U << 11)
#define	RK3399_TCPHY_HELPER_F_A0_READY		(1U << 12)
#define	RK3399_TCPHY_HELPER_F_ASSERT_RSTS	(1U << 13)
#define	RK3399_TCPHY_HELPER_F_TCPHY_RST		(1U << 14)
#define	RK3399_TCPHY_HELPER_F_UPHY_RST		(1U << 15)

struct rk3399_tcphy_phy_reg {
	uint16_t	value;
	uint32_t	addr;
};

struct rk3399_tcphy_helper_prop {
	uint32_t	reg;
	uint32_t	lsb;
	uint32_t	msb;
};

static const struct rk3399_tcphy_helper_prop rk3399_tcphy0_conn_dir = {
	.reg = 0x0e580,
	.lsb = 0,
	.msb = 0,
};
static const struct rk3399_tcphy_helper_prop rk3399_tcphy0_external_psm = {
	.reg = 0x0e588,
	.lsb = 14,
	.msb = 14,
};
static const struct rk3399_tcphy_helper_prop rk3399_tcphy0_uphy_dp_sel = {
	.reg = 0x6268,
	.lsb = 19,
	.msb = 19,
};

static struct sysctl_ctx_list rk3399_tcphy_helper_ctx;
static struct sysctl_oid *rk3399_tcphy_helper_tree;

static int rk3399_tcphy_helper_flip = 1;
static int rk3399_tcphy_helper_external_psm = 1;
static int rk3399_tcphy_helper_uphy_dp_sel = 1;
static int rk3399_tcphy_helper_lane_cfg = 1;
static int rk3399_tcphy_helper_dp_mode = 0;
static int rk3399_tcphy_helper_aux_cal = 0;
static int rk3399_tcphy_helper_cfg_24m = 0;
static int rk3399_tcphy_helper_cfg_dp_pll = 0;
static int rk3399_tcphy_helper_aux_flip = 0;
static int rk3399_tcphy_helper_dp_cfg_lanes = 0;
static int rk3399_tcphy_helper_assert_all_rsts = 0;
static int rk3399_tcphy_helper_deassert_tcphy_rst = 0;
static int rk3399_tcphy_helper_poll_pma_ready = 0;
static int rk3399_tcphy_helper_deassert_uphy_rst = 0;
static int rk3399_tcphy_helper_deassert_pipe_rst = 0;
static int rk3399_tcphy_helper_full_reset_seq = 0;
static int rk3399_tcphy_helper_poll_a0 = 0;
static int rk3399_tcphy_helper_enter_kdb_on_error = 0;
static int rk3399_tcphy_helper_break_before_pipe_rst = 0;
static int rk3399_tcphy_helper_break_before_a0 = 0;
static int rk3399_tcphy_helper_stop_after_step = -1;
static int rk3399_tcphy_helper_last_error;
static int rk3399_tcphy_helper_last_applied_mask;
static uint32_t rk3399_tcphy_helper_last_pma_cmn_ctrl1;
static uint32_t rk3399_tcphy_helper_last_dp_mode_ctl;
static uint32_t rk3399_tcphy_helper_breadcrumb[16];
static uint32_t rk3399_tcphy_helper_last_pma_poll_count;
static uint32_t rk3399_tcphy_helper_last_a0_100ms;
static uint32_t rk3399_tcphy_helper_uphy_rst_mask = RK3399_CRU_SRST_UPHY0;
static uint32_t rk3399_tcphy_helper_tcphy_rst_mask =
    RK3399_CRU_SRST_P_UPHY0_TCPHY;
static uint32_t rk3399_tcphy_helper_pipe_rst_mask =
    RK3399_CRU_SRST_UPHY0_PIPE_L00;

static void
rk3399_tcphy_helper_consolef(const char *fmt, ...)
{
	char buf[192];
	va_list ap;

	va_start(ap, fmt);
	vsnprintf(buf, sizeof(buf), fmt, ap);
	va_end(ap);
	cnputs(buf);
}

static void
rk3399_tcphy_helper_kdb_on_error(const char *why, int error)
{

	if (!rk3399_tcphy_helper_enter_kdb_on_error)
		return;

	rk3399_tcphy_helper_consolef(
	    "tcphy frs: entering ddb on error %d (%s)\n", error, why);
	kdb_enter(KDB_WHY_UNSET, why);
}

static bool
rk3399_tcphy_helper_stop_checkpoint(int step, const char *why, int *errorp)
{

	if (rk3399_tcphy_helper_stop_after_step != step)
		return (false);

	rk3399_tcphy_helper_consolef(
	    "tcphy frs: stop_after_step=%d entering ddb (%s)\n", step, why);
	kdb_enter(KDB_WHY_UNSET, why);
	if (errorp != NULL)
		*errorp = ECANCELED;
	return (true);
}

static const struct rk3399_tcphy_phy_reg rk3399_tcphy_dp_pll_cfg[] = {
	{ 0xf0,	RK3399_TCPHY_CMN_PLL1_VCOCAL_INIT },
	{ 0x18,	RK3399_TCPHY_CMN_PLL1_VCOCAL_ITER },
	{ 0x30b9,	RK3399_TCPHY_CMN_PLL1_VCOCAL_START },
	{ 0x21c,	RK3399_TCPHY_CMN_PLL1_INTDIV },
	{ 0x0,		RK3399_TCPHY_CMN_PLL1_FRACDIV },
	{ 0x5,		RK3399_TCPHY_CMN_PLL1_HIGH_THR },
	{ 0x35,	RK3399_TCPHY_CMN_PLL1_SS_CTRL1 },
	{ 0x7f1e,	RK3399_TCPHY_CMN_PLL1_SS_CTRL2 },
	{ 0x20,	RK3399_TCPHY_CMN_PLL1_DSM_DIAG },
	{ 0x0,		RK3399_TCPHY_CMN_PLLSM1_USER_DEF_CTRL },
	{ 0x0,		RK3399_TCPHY_CMN_DIAG_PLL1_OVRD },
	{ 0x0,		RK3399_TCPHY_CMN_DIAG_PLL1_FBH_OVRD },
	{ 0x0,		RK3399_TCPHY_CMN_DIAG_PLL1_FBL_OVRD },
	{ 0x6,		RK3399_TCPHY_CMN_DIAG_PLL1_V2I_TUNE },
	{ 0x45,	RK3399_TCPHY_CMN_DIAG_PLL1_CP_TUNE },
	{ 0x8,		RK3399_TCPHY_CMN_DIAG_PLL1_LF_PROG },
	{ 0x100,	RK3399_TCPHY_CMN_DIAG_PLL1_PTATIS_TUNE1 },
	{ 0x7,		RK3399_TCPHY_CMN_DIAG_PLL1_PTATIS_TUNE2 },
	{ 0x4,		RK3399_TCPHY_CMN_DIAG_PLL1_INCLK_CTRL },
};

static int
rk3399_tcphy_helper_get_prop(phandle_t node, const char *name,
    struct rk3399_tcphy_helper_prop *prop)
{
	pcell_t cells[3];

	if (OF_getencprop(node, name, cells, sizeof(cells)) != sizeof(cells))
		return (ENOENT);

	prop->reg = cells[0];
	prop->lsb = cells[1];
	prop->msb = cells[2];
	if (prop->msb < prop->lsb || prop->msb >= 16)
		return (EINVAL);

	return (0);
}

static int
rk3399_tcphy_helper_get_fallback_prop(phandle_t node, const char *name,
    struct rk3399_tcphy_helper_prop *prop)
{
	pcell_t reg[4];

	if (OF_getencprop(node, "reg", reg, sizeof(reg)) != sizeof(reg))
		return (ENOENT);

	/*
	 * The modern rk3399 typec-phy binding deprecates the explicit field
	 * properties and derives them from the compatible + controller base.
	 * RockPro64 USB-C video uses tcphy0 at ff7c0000.
	 */
	if (reg[1] != 0xff7c0000)
		return (ENOENT);

	if (strcmp(name, "rockchip,typec-conn-dir") == 0)
		*prop = rk3399_tcphy0_conn_dir;
	else if (strcmp(name, "rockchip,external-psm") == 0)
		*prop = rk3399_tcphy0_external_psm;
	else if (strcmp(name, "rockchip,uphy-dp-sel") == 0)
		*prop = rk3399_tcphy0_uphy_dp_sel;
	else
		return (ENOENT);

	return (0);
}

static int
rk3399_tcphy_helper_get_grf(struct syscon **grf, phandle_t *node_out)
{
	device_t dev;
	phandle_t node;
	pcell_t xref;

	node = OF_finddevice(RK3399_TCPHY_PATH);
	if (node <= 0)
		return (ENOENT);
	if (OF_getencprop(node, "rockchip,grf", &xref, sizeof(xref)) <= 0)
		return (ENOENT);

	dev = OF_device_from_xref(xref);
	if (dev == NULL)
		return (ENXIO);
	if (syscon_get_by_ofw_node(dev, OF_node_from_xref(xref), grf) != 0)
		return (ENXIO);

	*node_out = node;
	return (0);
}

static int
rk3399_tcphy_helper_set_field(struct syscon *grf,
    const struct rk3399_tcphy_helper_prop *prop, uint32_t value)
{
	uint32_t field_mask, regval, width_mask;

	field_mask = ((1u << (prop->msb - prop->lsb + 1)) - 1u) << prop->lsb;
	width_mask = (value << prop->lsb) & field_mask;
	regval = width_mask | (field_mask << 16);
	return (SYSCON_WRITE_4(grf, prop->reg, regval));
}

static int
rk3399_tcphy_helper_apply_grf_fields(struct syscon *grf, phandle_t node,
    int *maskp)
{
	struct rk3399_tcphy_helper_prop prop;
	int error, mask;

	mask = 0;

	error = rk3399_tcphy_helper_get_prop(node, "rockchip,typec-conn-dir",
	    &prop);
	if (error == ENOENT)
		error = rk3399_tcphy_helper_get_fallback_prop(node,
		    "rockchip,typec-conn-dir", &prop);
	if (error == 0) {
		error = rk3399_tcphy_helper_set_field(grf, &prop,
		    rk3399_tcphy_helper_flip ? 1 : 0);
		if (error != 0)
			return (error);
		mask |= RK3399_TCPHY_HELPER_F_CONN_DIR;
	}

	if (rk3399_tcphy_helper_external_psm) {
		error = rk3399_tcphy_helper_get_prop(node, "rockchip,external-psm",
		    &prop);
		if (error == ENOENT)
			error = rk3399_tcphy_helper_get_fallback_prop(node,
			    "rockchip,external-psm", &prop);
		if (error == 0) {
			error = rk3399_tcphy_helper_set_field(grf, &prop, 1);
			if (error != 0)
				return (error);
			mask |= RK3399_TCPHY_HELPER_F_EXT_PSM;
		}
	}

	if (rk3399_tcphy_helper_uphy_dp_sel) {
		error = rk3399_tcphy_helper_get_prop(node, "rockchip,uphy-dp-sel",
		    &prop);
		if (error == ENOENT)
			error = rk3399_tcphy_helper_get_fallback_prop(node,
			    "rockchip,uphy-dp-sel", &prop);
		if (error == 0) {
			error = rk3399_tcphy_helper_set_field(grf, &prop, 1);
			if (error != 0)
				return (error);
			mask |= RK3399_TCPHY_HELPER_F_UPHY_DP_SEL;
		}
	}

	if (maskp != NULL)
		*maskp |= mask;

	return (0);
}

static int
rk3399_tcphy_helper_set_lane_cfg(int mode)
{
	volatile uint32_t *reg;
	void *va;
	uint32_t val;

	if (mode == 0)
		return (0);
	if (mode != 1 && mode != 2)
		return (EINVAL);

	va = pmap_mapdev(RK3399_TCPHY0_BASE, RK3399_TCPHY0_SIZE);
	if (va == 0)
		return (ENOMEM);

	reg = (volatile uint32_t *)((uintptr_t)va + RK3399_TCPHY_PMA_LANE_CFG);
	val = mode == 1 ? RK3399_TCPHY_PIN_ASSIGN_C_E :
	    RK3399_TCPHY_PIN_ASSIGN_D_F;
	*reg = val;
	pmap_unmapdev(va, RK3399_TCPHY0_SIZE);

	return (0);
}

static int
rk3399_tcphy_helper_cfg_24m_apply(void)
{
	volatile uint32_t *base;
	void *va;
	uint32_t i, rdata;

	va = pmap_mapdev(RK3399_TCPHY0_BASE, RK3399_TCPHY0_SIZE);
	if (va == NULL)
		return (ENOMEM);
	base = (volatile uint32_t *)va;

	base[RK3399_TCPHY_PMA_CMN_CTRL1 / 4] = 0x830;
	for (i = 0; i < 4; i++) {
		base[RK3399_TCPHY_XCVR_DIAG_LANE_FCM_EN_MGN(i) / 4] = 0x90;
		base[RK3399_TCPHY_TX_RCVDET_EN_TMR(i) / 4] = 0x960;
		base[RK3399_TCPHY_TX_RCVDET_ST_TMR(i) / 4] = 0x30;
	}

	rdata = base[RK3399_TCPHY_CMN_DIAG_HSCLK_SEL / 4];
	rdata &= ~RK3399_TCPHY_CLK_PLL_MASK;
	rdata |= RK3399_TCPHY_CLK_PLL_CONFIG;
	base[RK3399_TCPHY_CMN_DIAG_HSCLK_SEL / 4] = rdata;

	pmap_unmapdev(va, RK3399_TCPHY0_SIZE);
	return (0);
}

static int
rk3399_tcphy_helper_cfg_24m_base(volatile uint32_t *base)
{
	uint32_t val;
	u_int i;

	base[RK3399_TCPHY_PMA_CMN_CTRL1 / 4] = 0x830;
	for (i = 0; i < 4; i++) {
		base[RK3399_TCPHY_XCVR_DIAG_LANE_FCM_EN_MGN(i) / 4] = 0x90;
		base[RK3399_TCPHY_TX_RCVDET_EN_TMR(i) / 4] = 0x960;
		base[RK3399_TCPHY_TX_RCVDET_ST_TMR(i) / 4] = 0x30;
	}
	val = base[RK3399_TCPHY_CMN_DIAG_HSCLK_SEL / 4];
	val &= ~RK3399_TCPHY_CLK_PLL_MASK;
	val |= RK3399_TCPHY_CLK_PLL_CONFIG;
	base[RK3399_TCPHY_CMN_DIAG_HSCLK_SEL / 4] = val;
	return (0);
}

static int
rk3399_tcphy_helper_cfg_dp_pll_apply(void)
{
	volatile uint32_t *base;
	void *va;
	u_int i;

	va = pmap_mapdev(RK3399_TCPHY0_BASE, RK3399_TCPHY0_SIZE);
	if (va == NULL)
		return (ENOMEM);
	base = (volatile uint32_t *)va;

	base[RK3399_TCPHY_DP_CLK_CTL / 4] =
	    RK3399_TCPHY_DP_PLL_CLOCK_ENABLE |
	    RK3399_TCPHY_DP_PLL_ENABLE |
	    RK3399_TCPHY_DP_PLL_DATA_RATE_RBR;

	for (i = 0; i < nitems(rk3399_tcphy_dp_pll_cfg); i++)
		base[rk3399_tcphy_dp_pll_cfg[i].addr / 4] =
		    rk3399_tcphy_dp_pll_cfg[i].value;

	pmap_unmapdev(va, RK3399_TCPHY0_SIZE);
	return (0);
}

static int
rk3399_tcphy_helper_cfg_dp_pll_base(volatile uint32_t *base)
{
	size_t i;

	base[RK3399_TCPHY_DP_CLK_CTL / 4] =
	    RK3399_TCPHY_DP_PLL_CLOCK_ENABLE |
	    RK3399_TCPHY_DP_PLL_ENABLE |
	    RK3399_TCPHY_DP_PLL_DATA_RATE_RBR;

	for (i = 0; i < nitems(rk3399_tcphy_dp_pll_cfg); i++)
		base[rk3399_tcphy_dp_pll_cfg[i].addr / 4] =
		    rk3399_tcphy_dp_pll_cfg[i].value;

	return (0);
}

static int
rk3399_tcphy_helper_aux_flip_apply(int flip)
{
	volatile uint32_t *base;
	void *va;
	uint32_t val;

	va = pmap_mapdev(RK3399_TCPHY0_BASE, RK3399_TCPHY0_SIZE);
	if (va == NULL)
		return (ENOMEM);
	base = (volatile uint32_t *)va;

	val = base[RK3399_TCPHY_TX_ANA_CTRL_REG_1 / 4];
	if (flip == 0)
		val |= RK3399_TCPHY_AUXDA_POLARITY;
	else
		val &= ~RK3399_TCPHY_AUXDA_POLARITY;
	base[RK3399_TCPHY_TX_ANA_CTRL_REG_1 / 4] = val;

	pmap_unmapdev(va, RK3399_TCPHY0_SIZE);
	return (0);
}

static void
rk3399_tcphy_helper_dp_cfg_lane(volatile uint32_t *base, u_int lane)
{
	uint32_t rdata;

	base[RK3399_TCPHY_XCVR_PSM_RCTRL(lane) / 4] = 0xbefc;
	base[RK3399_TCPHY_TX_PSC_A0(lane) / 4] = 0x6799;
	base[RK3399_TCPHY_TX_PSC_A1(lane) / 4] = 0x6798;
	base[RK3399_TCPHY_TX_PSC_A2(lane) / 4] = 0x98;
	base[RK3399_TCPHY_TX_PSC_A3(lane) / 4] = 0x98;

	base[RK3399_TCPHY_TX_TXCC_MGNFS_MULT_000(lane) / 4] = 0;
	base[RK3399_TCPHY_TX_TXCC_MGNFS_MULT_001(lane) / 4] = 0;
	base[RK3399_TCPHY_TX_TXCC_MGNFS_MULT_010(lane) / 4] = 0;
	base[RK3399_TCPHY_TX_TXCC_MGNFS_MULT_011(lane) / 4] = 0;
	base[RK3399_TCPHY_TX_TXCC_MGNFS_MULT_100(lane) / 4] = 0;
	base[RK3399_TCPHY_TX_TXCC_MGNFS_MULT_101(lane) / 4] = 0;
	base[RK3399_TCPHY_TX_TXCC_MGNFS_MULT_110(lane) / 4] = 0;
	base[RK3399_TCPHY_TX_TXCC_MGNFS_MULT_111(lane) / 4] = 0;
	base[RK3399_TCPHY_TX_TXCC_CPOST_MULT_10(lane) / 4] = 0;
	base[RK3399_TCPHY_TX_TXCC_CPOST_MULT_01(lane) / 4] = 0;
	base[RK3399_TCPHY_TX_TXCC_CPOST_MULT_00(lane) / 4] = 0;
	base[RK3399_TCPHY_TX_TXCC_CPOST_MULT_11(lane) / 4] = 0;

	base[RK3399_TCPHY_TX_TXCC_CAL_SCLR_MULT(lane) / 4] = 0x128;
	base[RK3399_TCPHY_TX_DIAG_TX_DRV(lane) / 4] = 0x400;

	rdata = base[RK3399_TCPHY_XCVR_DIAG_PLLDRC_CTRL(lane) / 4];
	rdata = (rdata & 0x8fff) | 0x6000;
	base[RK3399_TCPHY_XCVR_DIAG_PLLDRC_CTRL(lane) / 4] = rdata;
}

static int
rk3399_tcphy_helper_dp_cfg_lanes_apply(void)
{
	volatile uint32_t *base;
	void *va;
	u_int lane;

	va = pmap_mapdev(RK3399_TCPHY0_BASE, RK3399_TCPHY0_SIZE);
	if (va == NULL)
		return (ENOMEM);
	base = (volatile uint32_t *)va;

	for (lane = 0; lane < 4; lane++)
		rk3399_tcphy_helper_dp_cfg_lane(base, lane);

	pmap_unmapdev(va, RK3399_TCPHY0_SIZE);
	return (0);
}

static int
rk3399_tcphy_helper_set_lane_cfg_base(volatile uint32_t *base, int mode)
{
	if (mode == 0)
		return (0);
	if (mode == 1)
		base[RK3399_TCPHY_PMA_LANE_CFG / 4] = RK3399_TCPHY_PIN_ASSIGN_C_E;
	else if (mode == 2)
		base[RK3399_TCPHY_PMA_LANE_CFG / 4] = RK3399_TCPHY_PIN_ASSIGN_D_F;
	else
		return (EINVAL);
	return (0);
}

static int
rk3399_tcphy_helper_poll_pma_ready_apply(void)
{
	volatile uint32_t *base;
	void *va;
	uint32_t val;
	int i;

	va = pmap_mapdev(RK3399_TCPHY0_BASE, RK3399_TCPHY0_SIZE);
	if (va == NULL)
		return (ENOMEM);
	base = (volatile uint32_t *)va;

	for (i = 0; i < 10000; i++) {
		val = base[RK3399_TCPHY_PMA_CMN_CTRL1 / 4];
		rk3399_tcphy_helper_last_pma_cmn_ctrl1 = val;
		if ((val & 0x1) != 0) {
			pmap_unmapdev(va, RK3399_TCPHY0_SIZE);
			return (0);
		}
		DELAY(10);
	}

	pmap_unmapdev(va, RK3399_TCPHY0_SIZE);
	return (ETIMEDOUT);
}

static int
rk3399_tcphy_helper_poll_a0_apply(void)
{
	volatile uint32_t *base;
	void *va;
	uint32_t val;
	int i;

	va = pmap_mapdev(RK3399_TCPHY0_BASE, RK3399_TCPHY0_SIZE);
	if (va == NULL)
		return (ENOMEM);
	base = (volatile uint32_t *)va;

	for (i = 0; i < 10000; i++) {
		val = base[RK3399_TCPHY_DP_MODE_CTL / 4];
		rk3399_tcphy_helper_last_dp_mode_ctl = val;
		if ((val & RK3399_TCPHY_DP_MODE_A0) != 0) {
			pmap_unmapdev(va, RK3399_TCPHY0_SIZE);
			return (0);
		}
		DELAY(10);
	}

	pmap_unmapdev(va, RK3399_TCPHY0_SIZE);
	return (ETIMEDOUT);
}

static int
rk3399_tcphy_helper_cru_reset_write(uint32_t set_bits, uint32_t clear_bits)
{
	volatile uint32_t *reg;
	void *va;
	uint32_t writev;

	va = pmap_mapdev(RK3399_CRU_BASE, RK3399_CRU_SIZE);
	if (va == NULL)
		return (ENOMEM);

	reg = (volatile uint32_t *)((uintptr_t)va + RK3399_CRU_SOFTRST_CON12);
	writev = ((set_bits | clear_bits) << 16) | set_bits;
	*reg = writev;
	DELAY(10);
	pmap_unmapdev(va, RK3399_CRU_SIZE);
	return (0);
}

static int
rk3399_tcphy_helper_assert_all_rsts_direct(void)
{
	uint32_t mask;

	mask = rk3399_tcphy_helper_uphy_rst_mask |
	    rk3399_tcphy_helper_tcphy_rst_mask |
	    rk3399_tcphy_helper_pipe_rst_mask;
	return (rk3399_tcphy_helper_cru_reset_write(mask, 0));
}

static int
rk3399_tcphy_helper_deassert_tcphy_rst_direct(void)
{
	return (rk3399_tcphy_helper_cru_reset_write(0,
	    rk3399_tcphy_helper_tcphy_rst_mask));
}

static int
rk3399_tcphy_helper_deassert_uphy_rst_direct(void)
{
	return (rk3399_tcphy_helper_cru_reset_write(0,
	    rk3399_tcphy_helper_uphy_rst_mask));
}

static int
rk3399_tcphy_helper_deassert_pipe_rst_direct(void)
{
	return (rk3399_tcphy_helper_cru_reset_write(0,
	    rk3399_tcphy_helper_pipe_rst_mask));
}

static int
rk3399_tcphy_helper_calib_code_pos(uint32_t v)
{
	return ((int)(v & 0x7f));
}

static int
rk3399_tcphy_helper_calib_code_signed(uint32_t v)
{
	int x;

	x = (int)(v & 0xff);
	if ((x & 0x80) != 0)
		x -= 0x100;
	return (x);
}

static int
rk3399_tcphy_helper_run_aux_cal_base(volatile uint32_t *base)
{
	uint32_t tx1, tx2, val;
	int pu_calib_code, pd_calib_code, pu_adj, pd_adj;
	int calib;

	pu_calib_code = rk3399_tcphy_helper_calib_code_pos(
	    base[RK3399_TCPHY_CMN_TXPUCAL_CTRL / 4]);
	pd_calib_code = rk3399_tcphy_helper_calib_code_pos(
	    base[RK3399_TCPHY_CMN_TXPDCAL_CTRL / 4]);
	pu_adj = rk3399_tcphy_helper_calib_code_signed(
	    base[RK3399_TCPHY_CMN_TXPU_ADJ_CTRL / 4]);
	pd_adj = rk3399_tcphy_helper_calib_code_signed(
	    base[RK3399_TCPHY_CMN_TXPD_ADJ_CTRL / 4]);
	calib = ((pu_calib_code + pd_calib_code) / 2) + pu_adj + pd_adj;
	if (calib < 0)
		calib = 0;
	if (calib > RK3399_TCPHY_TX_RESCAL_CODE_MASK)
		calib = RK3399_TCPHY_TX_RESCAL_CODE_MASK;

	tx1 = base[RK3399_TCPHY_TX_ANA_CTRL_REG_1 / 4];
	tx1 &= ~RK3399_TCPHY_TXDA_CAL_LATCH_EN;
	base[RK3399_TCPHY_TX_ANA_CTRL_REG_1 / 4] = tx1;

	val = base[RK3399_TCPHY_TX_DIG_CTRL_REG_2 / 4];
	val &= ~(RK3399_TCPHY_TX_RESCAL_CODE_MASK <<
	    RK3399_TCPHY_TX_RESCAL_CODE_OFFSET);
	val |= ((uint32_t)calib << RK3399_TCPHY_TX_RESCAL_CODE_OFFSET);
	base[RK3399_TCPHY_TX_DIG_CTRL_REG_2 / 4] = val;
	DELAY(10000);

	tx1 |= RK3399_TCPHY_TXDA_CAL_LATCH_EN;
	base[RK3399_TCPHY_TX_ANA_CTRL_REG_1 / 4] = tx1;
	DELAY(200);

	base[RK3399_TCPHY_PHY_DP_TX_CTL / 4] = 0;
	tx2 = RK3399_TCPHY_XCVR_DECAP_EN;
	base[RK3399_TCPHY_TX_ANA_CTRL_REG_2 / 4] = tx2;
	DELAY(1);
	tx2 |= RK3399_TCPHY_XCVR_DECAP_EN_DEL;
	base[RK3399_TCPHY_TX_ANA_CTRL_REG_2 / 4] = tx2;
	base[RK3399_TCPHY_TX_ANA_CTRL_REG_3 / 4] = 0;

	tx1 |= RK3399_TCPHY_TXDA_UPHY_SUPPLY_EN;
	base[RK3399_TCPHY_TX_ANA_CTRL_REG_1 / 4] = tx1;
	DELAY(1);
	tx1 |= RK3399_TCPHY_TXDA_UPHY_SUPPLY_EN_DEL;
	base[RK3399_TCPHY_TX_ANA_CTRL_REG_1 / 4] = tx1;

	base[RK3399_TCPHY_TX_ANA_CTRL_REG_5 / 4] = 0;
	base[RK3399_TCPHY_TX_ANA_CTRL_REG_4 / 4] = 0x1001;

	tx1 |= RK3399_TCPHY_TXDA_DRV_LDO_EN;
	base[RK3399_TCPHY_TX_ANA_CTRL_REG_1 / 4] = tx1;
	DELAY(5);
	tx1 |= RK3399_TCPHY_TXDA_BGREF_EN;
	base[RK3399_TCPHY_TX_ANA_CTRL_REG_1 / 4] = tx1;

	tx2 |= RK3399_TCPHY_TXDA_DRV_PREDRV_EN;
	base[RK3399_TCPHY_TX_ANA_CTRL_REG_2 / 4] = tx2;
	DELAY(1);
	tx2 |= RK3399_TCPHY_TXDA_DRV_PREDRV_EN_DEL;
	base[RK3399_TCPHY_TX_ANA_CTRL_REG_2 / 4] = tx2;

	tx1 |= RK3399_TCPHY_TXDA_DP_AUX_EN | RK3399_TCPHY_TXDA_DECAP_EN;
	tx1 &= ~RK3399_TCPHY_TXDA_DRV_LDO_EN;
	tx1 &= ~RK3399_TCPHY_TXDA_BGREF_EN;
	base[RK3399_TCPHY_TX_ANA_CTRL_REG_1 / 4] = tx1;
	DELAY(1);
	tx1 |= RK3399_TCPHY_TXDA_DECAP_EN_DEL;
	base[RK3399_TCPHY_TX_ANA_CTRL_REG_1 / 4] = tx1;

	base[RK3399_TCPHY_TX_ANA_CTRL_REG_4 / 4] = 0;
	base[RK3399_TCPHY_TXDA_COEFF_CALC_CTRL / 4] = 0;
	base[RK3399_TCPHY_TXDA_CYA_AUXDA_CYA / 4] = 0;
	base[RK3399_TCPHY_TX_TXCC_CAL_SCLR_MULT(RK3399_TCPHY_AUX_CH_LANE) / 4] =
	    0x128;
	val = base[RK3399_TCPHY_TX_DIG_CTRL_REG_2 / 4];
	val |= RK3399_TCPHY_TX_HIGH_Z_TM_EN;
	base[RK3399_TCPHY_TX_DIG_CTRL_REG_2 / 4] = val;
	return (0);
}

static int
rk3399_tcphy_helper_run_aux_cal(void)
{
	volatile uint32_t *base;
	void *va;
	uint32_t tx1, tx2, val;
	int pu_calib_code, pd_calib_code, pu_adj, pd_adj;
	int calib;

	va = pmap_mapdev(RK3399_TCPHY0_BASE, RK3399_TCPHY0_SIZE);
	if (va == NULL)
		return (ENOMEM);
	base = (volatile uint32_t *)va;

	pu_calib_code = rk3399_tcphy_helper_calib_code_pos(
	    base[RK3399_TCPHY_CMN_TXPUCAL_CTRL / 4]);
	pd_calib_code = rk3399_tcphy_helper_calib_code_pos(
	    base[RK3399_TCPHY_CMN_TXPDCAL_CTRL / 4]);
	pu_adj = rk3399_tcphy_helper_calib_code_signed(
	    base[RK3399_TCPHY_CMN_TXPU_ADJ_CTRL / 4]);
	pd_adj = rk3399_tcphy_helper_calib_code_signed(
	    base[RK3399_TCPHY_CMN_TXPD_ADJ_CTRL / 4]);
	calib = ((pu_calib_code + pd_calib_code) / 2) + pu_adj + pd_adj;
	if (calib < 0)
		calib = 0;
	if (calib > RK3399_TCPHY_TX_RESCAL_CODE_MASK)
		calib = RK3399_TCPHY_TX_RESCAL_CODE_MASK;

	tx1 = base[RK3399_TCPHY_TX_ANA_CTRL_REG_1 / 4];
	tx1 &= ~RK3399_TCPHY_TXDA_CAL_LATCH_EN;
	base[RK3399_TCPHY_TX_ANA_CTRL_REG_1 / 4] = tx1;

	val = base[RK3399_TCPHY_TX_DIG_CTRL_REG_2 / 4];
	val &= ~(RK3399_TCPHY_TX_RESCAL_CODE_MASK <<
	    RK3399_TCPHY_TX_RESCAL_CODE_OFFSET);
	val |= ((uint32_t)calib << RK3399_TCPHY_TX_RESCAL_CODE_OFFSET);
	base[RK3399_TCPHY_TX_DIG_CTRL_REG_2 / 4] = val;
	DELAY(10000);

	tx1 |= RK3399_TCPHY_TXDA_CAL_LATCH_EN;
	base[RK3399_TCPHY_TX_ANA_CTRL_REG_1 / 4] = tx1;
	DELAY(200);

	base[RK3399_TCPHY_PHY_DP_TX_CTL / 4] = 0;

	tx2 = RK3399_TCPHY_XCVR_DECAP_EN;
	base[RK3399_TCPHY_TX_ANA_CTRL_REG_2 / 4] = tx2;
	DELAY(1);
	tx2 |= RK3399_TCPHY_XCVR_DECAP_EN_DEL;
	base[RK3399_TCPHY_TX_ANA_CTRL_REG_2 / 4] = tx2;

	base[RK3399_TCPHY_TX_ANA_CTRL_REG_3 / 4] = 0;

	tx1 |= RK3399_TCPHY_TXDA_UPHY_SUPPLY_EN;
	base[RK3399_TCPHY_TX_ANA_CTRL_REG_1 / 4] = tx1;
	DELAY(1);
	tx1 |= RK3399_TCPHY_TXDA_UPHY_SUPPLY_EN_DEL;
	base[RK3399_TCPHY_TX_ANA_CTRL_REG_1 / 4] = tx1;

	base[RK3399_TCPHY_TX_ANA_CTRL_REG_5 / 4] = 0;
	base[RK3399_TCPHY_TX_ANA_CTRL_REG_4 / 4] = 0x1001;

	tx1 |= RK3399_TCPHY_TXDA_DRV_LDO_EN;
	base[RK3399_TCPHY_TX_ANA_CTRL_REG_1 / 4] = tx1;
	DELAY(5);
	tx1 |= RK3399_TCPHY_TXDA_BGREF_EN;
	base[RK3399_TCPHY_TX_ANA_CTRL_REG_1 / 4] = tx1;

	tx2 |= RK3399_TCPHY_TXDA_DRV_PREDRV_EN;
	base[RK3399_TCPHY_TX_ANA_CTRL_REG_2 / 4] = tx2;
	DELAY(1);
	tx2 |= RK3399_TCPHY_TXDA_DRV_PREDRV_EN_DEL;
	base[RK3399_TCPHY_TX_ANA_CTRL_REG_2 / 4] = tx2;

	tx1 |= RK3399_TCPHY_TXDA_DP_AUX_EN | RK3399_TCPHY_TXDA_DECAP_EN;
	tx1 &= ~RK3399_TCPHY_TXDA_DRV_LDO_EN;
	tx1 &= ~RK3399_TCPHY_TXDA_BGREF_EN;
	base[RK3399_TCPHY_TX_ANA_CTRL_REG_1 / 4] = tx1;
	DELAY(1);
	tx1 |= RK3399_TCPHY_TXDA_DECAP_EN_DEL;
	base[RK3399_TCPHY_TX_ANA_CTRL_REG_1 / 4] = tx1;

	base[RK3399_TCPHY_TX_ANA_CTRL_REG_4 / 4] = 0;
	base[RK3399_TCPHY_TXDA_COEFF_CALC_CTRL / 4] = 0;
	base[RK3399_TCPHY_TXDA_CYA_AUXDA_CYA / 4] = 0;
	base[RK3399_TCPHY_TX_TXCC_CAL_SCLR_MULT(RK3399_TCPHY_AUX_CH_LANE) / 4] =
	    0x128;

	val = base[RK3399_TCPHY_TX_DIG_CTRL_REG_2 / 4];
	val |= RK3399_TCPHY_TX_HIGH_Z_TM_EN;
	base[RK3399_TCPHY_TX_DIG_CTRL_REG_2 / 4] = val;

	pmap_unmapdev(va, RK3399_TCPHY0_SIZE);
	return (0);
}

static int
rk3399_tcphy_helper_set_dp_mode(int mode)
{
	volatile uint32_t *reg;
	void *va;
	uint32_t val;

	if (mode == 0)
		return (0);
	if (mode < 0 || mode > 3)
		return (EINVAL);

	va = pmap_mapdev(RK3399_TCPHY0_BASE, RK3399_TCPHY0_SIZE);
	if (va == NULL)
		return (ENOMEM);

	reg = (volatile uint32_t *)((uintptr_t)va + RK3399_TCPHY_DP_MODE_CTL);
	if (mode == 3)
		*reg = RK3399_TCPHY_DP_MODE_ENTER_A0;
	else
		*reg = RK3399_TCPHY_DP_MODE_ENTER_A2;
	DELAY(1000);
	val = *reg;
	rk3399_tcphy_helper_last_dp_mode_ctl = val;
	rk3399_tcphy_helper_last_pma_cmn_ctrl1 =
	    *(volatile uint32_t *)((uintptr_t)va + RK3399_TCPHY_PMA_CMN_CTRL1);
	rk3399_tcphy_helper_last_dp_mode_ctl = val;
	pmap_unmapdev(va, RK3399_TCPHY0_SIZE);
	return (0);
}

static int
rk3399_tcphy_helper_finish_dp_mode_a0(void)
{
	volatile uint32_t *reg;
	void *va;
	uint32_t val;

	va = pmap_mapdev(RK3399_TCPHY0_BASE, RK3399_TCPHY0_SIZE);
	if (va == NULL)
		return (ENOMEM);

	reg = (volatile uint32_t *)((uintptr_t)va + RK3399_TCPHY_DP_MODE_CTL);
	*reg = RK3399_TCPHY_DP_MODE_ENTER_A0;
	DELAY(1000);
	val = *reg;
	rk3399_tcphy_helper_last_dp_mode_ctl = val;
	rk3399_tcphy_helper_last_pma_cmn_ctrl1 =
	    *(volatile uint32_t *)((uintptr_t)va + RK3399_TCPHY_PMA_CMN_CTRL1);
	pmap_unmapdev(va, RK3399_TCPHY0_SIZE);
	return (0);
}

static int
rk3399_tcphy_helper_full_reset_seq_apply(void)
{
	struct syscon *grf;
	volatile uint32_t *cru_reg, *gate13_reg, *gate21_reg, *base;
	void *cruva, *phyva;
	uint32_t all_mask, val, gate13, gate21;
	phandle_t node;
	int error, i;

	memset(rk3399_tcphy_helper_breadcrumb, 0,
	    sizeof(rk3399_tcphy_helper_breadcrumb));
	rk3399_tcphy_helper_last_pma_poll_count = 0;
	rk3399_tcphy_helper_last_a0_100ms = 0;
	error = 0;

	error = rk3399_tcphy_helper_get_grf(&grf, &node);
	if (error != 0)
		return (error);
	error = rk3399_tcphy_helper_apply_grf_fields(grf, node, NULL);
	if (error != 0)
		return (error);

	cruva = pmap_mapdev(RK3399_CRU_BASE, RK3399_CRU_SIZE);
	if (cruva == NULL)
		return (ENOMEM);
	phyva = pmap_mapdev(RK3399_TCPHY0_BASE, RK3399_TCPHY0_SIZE);
	if (phyva == NULL) {
		pmap_unmapdev(cruva, RK3399_CRU_SIZE);
		return (ENOMEM);
	}

	cru_reg = (volatile uint32_t *)((uintptr_t)cruva + RK3399_CRU_SOFTRST_CON12);
	gate13_reg = (volatile uint32_t *)((uintptr_t)cruva +
	    RK3399_CRU_CLKGATE_CON13);
	gate21_reg = (volatile uint32_t *)((uintptr_t)cruva +
	    RK3399_CRU_CLKGATE_CON21);
	base = (volatile uint32_t *)phyva;
	all_mask = rk3399_tcphy_helper_uphy_rst_mask |
	    rk3399_tcphy_helper_tcphy_rst_mask |
	    rk3399_tcphy_helper_pipe_rst_mask;

	rk3399_tcphy_helper_consolef("tcphy frs: 0 starting\n");
	if (rk3399_tcphy_helper_stop_checkpoint(0, "tcphy frs step 0", &error))
		goto out;

	/* 1. assert all resets */
	*cru_reg = (all_mask << 16) | all_mask;
	DELAY(10);
	rk3399_tcphy_helper_breadcrumb[0] = *cru_reg;
	gate13 = *gate13_reg;
	gate21 = *gate21_reg;
	rk3399_tcphy_helper_consolef(
	    "tcphy frs: 1 softrst_con12_after_assert=0x%08x clkgate13=0x%08x clkgate21=0x%08x\n",
	    rk3399_tcphy_helper_breadcrumb[0], gate13, gate21);
	if (rk3399_tcphy_helper_stop_checkpoint(1, "tcphy frs step 1", &error))
		goto out;
	/* 2. deassert tcphy reset only */
	*cru_reg = (rk3399_tcphy_helper_tcphy_rst_mask << 16);
	DELAY(10);
	rk3399_tcphy_helper_breadcrumb[1] =
	    base[RK3399_TCPHY_PMA_CMN_CTRL1 / 4];
	rk3399_tcphy_helper_consolef(
	    "tcphy frs: 2 pma_cmn_after_tcphy_deassert=0x%08x\n",
	    rk3399_tcphy_helper_breadcrumb[1]);
	if (rk3399_tcphy_helper_stop_checkpoint(2, "tcphy frs step 2", &error))
		goto out;

	/* 3. config writes on live config space */
	if (rk3399_tcphy_helper_flip == 0)
		base[RK3399_TCPHY_TX_ANA_CTRL_REG_1 / 4] |=
		    RK3399_TCPHY_AUXDA_POLARITY;
	else
		base[RK3399_TCPHY_TX_ANA_CTRL_REG_1 / 4] &=
		    ~RK3399_TCPHY_AUXDA_POLARITY;
	rk3399_tcphy_helper_cfg_24m_base(base);
	rk3399_tcphy_helper_cfg_dp_pll_base(base);
	for (u_int lane = 0; lane < 4; lane++)
		rk3399_tcphy_helper_dp_cfg_lane(base, lane);
	error = rk3399_tcphy_helper_set_lane_cfg_base(base,
	    rk3399_tcphy_helper_lane_cfg);
	if (error != 0)
		goto out;
	base[RK3399_TCPHY_DP_MODE_CTL / 4] = RK3399_TCPHY_DP_MODE_ENTER_A2;
	DELAY(1000);
	rk3399_tcphy_helper_breadcrumb[2] = base[RK3399_TCPHY_TX_PSC_A0(0) / 4];
	rk3399_tcphy_helper_breadcrumb[3] =
	    base[RK3399_TCPHY_XCVR_PSM_RCTRL(0) / 4];
	rk3399_tcphy_helper_breadcrumb[4] = base[RK3399_TCPHY_DP_CLK_CTL / 4];
	rk3399_tcphy_helper_last_dp_mode_ctl =
	    base[RK3399_TCPHY_DP_MODE_CTL / 4];
	rk3399_tcphy_helper_breadcrumb[5] =
	    rk3399_tcphy_helper_last_dp_mode_ctl;
	rk3399_tcphy_helper_consolef(
	    "tcphy frs: 3 tx_psc_a0=0x%08x xcvr_psm_rctrl=0x%08x dp_clk_ctl=0x%08x dp_mode_ctl=0x%08x\n",
	    rk3399_tcphy_helper_breadcrumb[2],
	    rk3399_tcphy_helper_breadcrumb[3],
	    rk3399_tcphy_helper_breadcrumb[4],
	    rk3399_tcphy_helper_breadcrumb[5]);
	if (rk3399_tcphy_helper_stop_checkpoint(3, "tcphy frs step 3", &error))
		goto out;

	/* 4. deassert uphy reset */
	gate13 = *gate13_reg;
	gate21 = *gate21_reg;
	rk3399_tcphy_helper_consolef(
	    "tcphy frs: 3.5 before_uphy_deassert clkgate13=0x%08x clkgate21=0x%08x\n",
	    gate13, gate21);
	*cru_reg = (rk3399_tcphy_helper_uphy_rst_mask << 16);
	DELAY(10);

	/* 5. poll PMA ready */
	for (i = 0; i < 10000; i++) {
		val = base[RK3399_TCPHY_PMA_CMN_CTRL1 / 4];
		if (i == 0)
			rk3399_tcphy_helper_breadcrumb[6] = val;
		rk3399_tcphy_helper_last_pma_cmn_ctrl1 = val;
		rk3399_tcphy_helper_last_pma_poll_count = i + 1;
		if (i < 16 || (i % 1000) == 0 || (val & 0x1) != 0)
			rk3399_tcphy_helper_consolef(
			    "tcphy frs: 4 pma_poll iter=%d val=0x%08x\n",
			    i, val);
		if ((val & 0x1) != 0)
			break;
		DELAY(10);
	}
	rk3399_tcphy_helper_breadcrumb[7] =
	    rk3399_tcphy_helper_last_pma_cmn_ctrl1;
	rk3399_tcphy_helper_breadcrumb[8] =
	    rk3399_tcphy_helper_last_pma_poll_count;
	rk3399_tcphy_helper_consolef(
	    "tcphy frs: 4 final first=0x%08x final=0x%08x count=%u\n",
	    rk3399_tcphy_helper_breadcrumb[6],
	    rk3399_tcphy_helper_breadcrumb[7],
	    rk3399_tcphy_helper_breadcrumb[8]);
	if (rk3399_tcphy_helper_stop_checkpoint(4, "tcphy frs step 4", &error))
		goto out;
	if (i == 10000) {
		error = ETIMEDOUT;
		rk3399_tcphy_helper_kdb_on_error("full_reset_seq PMA ready timeout",
		    error);
		goto out;
	}

	if (rk3399_tcphy_helper_break_before_pipe_rst)
		kdb_enter(KDB_WHY_UNSET, "tcphy frs before pipe_rst");

	/* 6. deassert pipe reset */
	*cru_reg = (rk3399_tcphy_helper_pipe_rst_mask << 16);
	DELAY(10);
	rk3399_tcphy_helper_breadcrumb[9] = *cru_reg;
	rk3399_tcphy_helper_breadcrumb[10] = base[RK3399_TCPHY_DP_MODE_CTL / 4];
	rk3399_tcphy_helper_breadcrumb[11] =
	    base[RK3399_TCPHY_XCVR_PSM_RCTRL(0) / 4];
	rk3399_tcphy_helper_consolef(
	    "tcphy frs: 5 softrst_after_pipe=0x%08x dp_mode=0x%08x xcvr=0x%08x\n",
	    rk3399_tcphy_helper_breadcrumb[9],
	    rk3399_tcphy_helper_breadcrumb[10],
	    rk3399_tcphy_helper_breadcrumb[11]);
	if (rk3399_tcphy_helper_stop_checkpoint(5, "tcphy frs step 5", &error))
		goto out;

	/* 7. AUX cal */
	rk3399_tcphy_helper_run_aux_cal_base(base);

	if (rk3399_tcphy_helper_break_before_a0)
		kdb_enter(KDB_WHY_UNSET, "tcphy frs before A0");

	/* 8. write A0 */
	base[RK3399_TCPHY_DP_MODE_CTL / 4] = RK3399_TCPHY_DP_MODE_ENTER_A0;
	rk3399_tcphy_helper_breadcrumb[12] = base[RK3399_TCPHY_DP_MODE_CTL / 4];
	DELAY(1000);
	rk3399_tcphy_helper_breadcrumb[13] = base[RK3399_TCPHY_DP_MODE_CTL / 4];
	rk3399_tcphy_helper_consolef("tcphy frs: 6 dp_mode_1ms=0x%08x\n",
	    rk3399_tcphy_helper_breadcrumb[13]);
	DELAY(9000);
	rk3399_tcphy_helper_breadcrumb[14] = base[RK3399_TCPHY_DP_MODE_CTL / 4];
	rk3399_tcphy_helper_consolef("tcphy frs: 6 dp_mode_10ms=0x%08x\n",
	    rk3399_tcphy_helper_breadcrumb[14]);
	DELAY(40000);
	rk3399_tcphy_helper_breadcrumb[15] = base[RK3399_TCPHY_DP_MODE_CTL / 4];
	rk3399_tcphy_helper_consolef("tcphy frs: 6 dp_mode_50ms=0x%08x\n",
	    rk3399_tcphy_helper_breadcrumb[15]);
	DELAY(50000);
	rk3399_tcphy_helper_last_a0_100ms =
	    base[RK3399_TCPHY_DP_MODE_CTL / 4];
	rk3399_tcphy_helper_consolef("tcphy frs: 6 dp_mode_100ms=0x%08x\n",
	    rk3399_tcphy_helper_last_a0_100ms);
	if (rk3399_tcphy_helper_stop_checkpoint(6, "tcphy frs step 6", &error))
		goto out;

	/* 9. poll A0 */
	for (i = 0; i < 10000; i++) {
		val = base[RK3399_TCPHY_DP_MODE_CTL / 4];
		rk3399_tcphy_helper_last_dp_mode_ctl = val;
		if ((val & RK3399_TCPHY_DP_MODE_A0) != 0)
			break;
		DELAY(10);
	}
	error = (i == 10000) ? ETIMEDOUT : 0;
	if (error != 0)
		rk3399_tcphy_helper_kdb_on_error("full_reset_seq A0 timeout", error);
out:
	pmap_unmapdev(phyva, RK3399_TCPHY0_SIZE);
	pmap_unmapdev(cruva, RK3399_CRU_SIZE);
	return (error);
}

static int
rk3399_tcphy_helper_apply(void)
{
	struct syscon *grf;
	phandle_t node;
	int error, mask;

	mask = 0;
	error = rk3399_tcphy_helper_get_grf(&grf, &node);
	if (error != 0)
		goto done;

	if (rk3399_tcphy_helper_full_reset_seq) {
		error = rk3399_tcphy_helper_full_reset_seq_apply();
		if (error == 0)
			mask = RK3399_TCPHY_HELPER_F_CONN_DIR |
			    RK3399_TCPHY_HELPER_F_EXT_PSM |
			    RK3399_TCPHY_HELPER_F_UPHY_DP_SEL |
			    RK3399_TCPHY_HELPER_F_PMA_LANE_CFG |
			    RK3399_TCPHY_HELPER_F_DP_MODE |
			    RK3399_TCPHY_HELPER_F_AUX_CAL |
			    RK3399_TCPHY_HELPER_F_24M |
			    RK3399_TCPHY_HELPER_F_DP_PLL |
			    RK3399_TCPHY_HELPER_F_AUX_FLIP |
			    RK3399_TCPHY_HELPER_F_DP_CFG_LANES |
			    RK3399_TCPHY_HELPER_F_PMA_READY |
			    RK3399_TCPHY_HELPER_F_PIPE_RST |
			    RK3399_TCPHY_HELPER_F_ASSERT_RSTS |
			    RK3399_TCPHY_HELPER_F_TCPHY_RST |
			    RK3399_TCPHY_HELPER_F_UPHY_RST |
			    RK3399_TCPHY_HELPER_F_A0_READY;
		goto done;
	}

	if (rk3399_tcphy_helper_assert_all_rsts) {
		error = rk3399_tcphy_helper_assert_all_rsts_direct();
		if (error != 0)
			goto done;
		mask |= RK3399_TCPHY_HELPER_F_ASSERT_RSTS;
	}

	if (rk3399_tcphy_helper_deassert_tcphy_rst) {
		error = rk3399_tcphy_helper_deassert_tcphy_rst_direct();
		if (error != 0)
			goto done;
		mask |= RK3399_TCPHY_HELPER_F_TCPHY_RST;
	}
	error = rk3399_tcphy_helper_apply_grf_fields(grf, node, &mask);
	if (error != 0)
		goto done;

	if (rk3399_tcphy_helper_aux_flip) {
		error = rk3399_tcphy_helper_aux_flip_apply(
		    rk3399_tcphy_helper_flip);
		if (error != 0)
			goto done;
		mask |= RK3399_TCPHY_HELPER_F_AUX_FLIP;
	}

	if (rk3399_tcphy_helper_cfg_24m) {
		error = rk3399_tcphy_helper_cfg_24m_apply();
		if (error != 0)
			goto done;
		mask |= RK3399_TCPHY_HELPER_F_24M;
	}

	if (rk3399_tcphy_helper_cfg_dp_pll) {
		error = rk3399_tcphy_helper_cfg_dp_pll_apply();
		if (error != 0)
			goto done;
		mask |= RK3399_TCPHY_HELPER_F_DP_PLL;
	}

	if (rk3399_tcphy_helper_dp_cfg_lanes) {
		error = rk3399_tcphy_helper_dp_cfg_lanes_apply();
		if (error != 0)
			goto done;
		mask |= RK3399_TCPHY_HELPER_F_DP_CFG_LANES;
	}

	error = rk3399_tcphy_helper_set_lane_cfg(rk3399_tcphy_helper_lane_cfg);
	if (error != 0)
		goto done;
	if (rk3399_tcphy_helper_lane_cfg != 0)
		mask |= RK3399_TCPHY_HELPER_F_PMA_LANE_CFG;

	error = rk3399_tcphy_helper_set_dp_mode(rk3399_tcphy_helper_dp_mode);
	if (error != 0)
		goto done;
	if (rk3399_tcphy_helper_dp_mode != 0)
		mask |= RK3399_TCPHY_HELPER_F_DP_MODE;

	if (rk3399_tcphy_helper_deassert_uphy_rst) {
		error = rk3399_tcphy_helper_deassert_uphy_rst_direct();
		if (error != 0)
			goto done;
		mask |= RK3399_TCPHY_HELPER_F_UPHY_RST;
	}

	if (rk3399_tcphy_helper_poll_pma_ready) {
		error = rk3399_tcphy_helper_poll_pma_ready_apply();
		if (error != 0)
			goto done;
		mask |= RK3399_TCPHY_HELPER_F_PMA_READY;
	}

	if (rk3399_tcphy_helper_deassert_pipe_rst) {
		error = rk3399_tcphy_helper_deassert_pipe_rst_direct();
		if (error != 0)
			goto done;
		mask |= RK3399_TCPHY_HELPER_F_PIPE_RST;
	}

	if (rk3399_tcphy_helper_aux_cal) {
		error = rk3399_tcphy_helper_run_aux_cal();
		if (error != 0)
			goto done;
		mask |= RK3399_TCPHY_HELPER_F_AUX_CAL;
	}

	if (rk3399_tcphy_helper_dp_mode == 2) {
		error = rk3399_tcphy_helper_finish_dp_mode_a0();
		if (error != 0)
			goto done;
	}

	if (rk3399_tcphy_helper_poll_a0) {
		error = rk3399_tcphy_helper_poll_a0_apply();
		if (error != 0)
			goto done;
		mask |= RK3399_TCPHY_HELPER_F_A0_READY;
	}

	error = 0;
done:
	rk3399_tcphy_helper_last_error = error;
	rk3399_tcphy_helper_last_applied_mask = mask;
	return (error);
}

static int
rk3399_tcphy_helper_apply_sysctl(SYSCTL_HANDLER_ARGS)
{
	int error, val;

	val = 0;
	error = sysctl_handle_int(oidp, &val, 0, req);
	if (error != 0 || req->newptr == NULL)
		return (error);
	if (val == 0)
		return (0);

	return (rk3399_tcphy_helper_apply());
}

static int
rk3399_tcphy_helper_modevent(module_t mod, int what, void *arg)
{
	switch (what) {
	case MOD_LOAD:
		sysctl_ctx_init(&rk3399_tcphy_helper_ctx);
		rk3399_tcphy_helper_tree = SYSCTL_ADD_NODE(
		    &rk3399_tcphy_helper_ctx,
		    SYSCTL_STATIC_CHILDREN(_hw),
		    OID_AUTO, "rk3399_tcphy_helper",
		    CTLFLAG_RW | CTLFLAG_MPSAFE, 0,
		    "RK3399 TC-PHY post-boot helper");
		if (rk3399_tcphy_helper_tree == NULL)
			return (ENOMEM);

		SYSCTL_ADD_INT(&rk3399_tcphy_helper_ctx,
		    SYSCTL_CHILDREN(rk3399_tcphy_helper_tree), OID_AUTO,
		    "flip", CTLFLAG_RWTUN, &rk3399_tcphy_helper_flip, 0,
		    "Connector orientation value for typec-conn-dir");
		SYSCTL_ADD_INT(&rk3399_tcphy_helper_ctx,
		    SYSCTL_CHILDREN(rk3399_tcphy_helper_tree), OID_AUTO,
		    "external_psm", CTLFLAG_RWTUN,
		    &rk3399_tcphy_helper_external_psm, 0,
		    "Apply external-psm field");
		SYSCTL_ADD_INT(&rk3399_tcphy_helper_ctx,
		    SYSCTL_CHILDREN(rk3399_tcphy_helper_tree), OID_AUTO,
		    "uphy_dp_sel", CTLFLAG_RWTUN,
		    &rk3399_tcphy_helper_uphy_dp_sel, 0,
		    "Apply uphy-dp-sel field");
		SYSCTL_ADD_INT(&rk3399_tcphy_helper_ctx,
		    SYSCTL_CHILDREN(rk3399_tcphy_helper_tree), OID_AUTO,
		    "aux_flip", CTLFLAG_RWTUN,
		    &rk3399_tcphy_helper_aux_flip, 0,
		    "Apply Linux-style AUX polarity write to TX_ANA_CTRL_REG_1");
		SYSCTL_ADD_INT(&rk3399_tcphy_helper_ctx,
		    SYSCTL_CHILDREN(rk3399_tcphy_helper_tree), OID_AUTO,
		    "cfg_24m", CTLFLAG_RWTUN,
		    &rk3399_tcphy_helper_cfg_24m, 0,
		    "Apply Linux tcphy_cfg_24m common clock setup");
		SYSCTL_ADD_INT(&rk3399_tcphy_helper_ctx,
		    SYSCTL_CHILDREN(rk3399_tcphy_helper_tree), OID_AUTO,
		    "cfg_dp_pll", CTLFLAG_RWTUN,
		    &rk3399_tcphy_helper_cfg_dp_pll, 0,
		    "Apply Linux tcphy_cfg_dp_pll sequence");
		SYSCTL_ADD_INT(&rk3399_tcphy_helper_ctx,
		    SYSCTL_CHILDREN(rk3399_tcphy_helper_tree), OID_AUTO,
		    "dp_cfg_lanes", CTLFLAG_RWTUN,
		    &rk3399_tcphy_helper_dp_cfg_lanes, 0,
		    "Apply Linux tcphy_dp_cfg_lane() to all 4 lanes");
		SYSCTL_ADD_INT(&rk3399_tcphy_helper_ctx,
		    SYSCTL_CHILDREN(rk3399_tcphy_helper_tree), OID_AUTO,
		    "full_reset_seq", CTLFLAG_RWTUN,
		    &rk3399_tcphy_helper_full_reset_seq, 0,
		    "Run full direct-CRU reset sequence with one CRU map and one TCPHY map");
		SYSCTL_ADD_INT(&rk3399_tcphy_helper_ctx,
		    SYSCTL_CHILDREN(rk3399_tcphy_helper_tree), OID_AUTO,
		    "enter_kdb_on_error", CTLFLAG_RWTUN,
		    &rk3399_tcphy_helper_enter_kdb_on_error, 0,
		    "Enter DDB on helper timeout/error paths");
		SYSCTL_ADD_INT(&rk3399_tcphy_helper_ctx,
		    SYSCTL_CHILDREN(rk3399_tcphy_helper_tree), OID_AUTO,
		    "break_before_pipe_rst", CTLFLAG_RWTUN,
		    &rk3399_tcphy_helper_break_before_pipe_rst, 0,
		    "Enter DDB after PMA-ready and before pipe_rst deassert");
		SYSCTL_ADD_INT(&rk3399_tcphy_helper_ctx,
		    SYSCTL_CHILDREN(rk3399_tcphy_helper_tree), OID_AUTO,
		    "break_before_a0", CTLFLAG_RWTUN,
		    &rk3399_tcphy_helper_break_before_a0, 0,
		    "Enter DDB after AUX cal and before DP_MODE_ENTER_A0");
		SYSCTL_ADD_INT(&rk3399_tcphy_helper_ctx,
		    SYSCTL_CHILDREN(rk3399_tcphy_helper_tree), OID_AUTO,
		    "stop_after_step", CTLFLAG_RWTUN,
		    &rk3399_tcphy_helper_stop_after_step, 0,
		    "Enter DDB and abort full_reset_seq after checkpoint step N (-1 disables)");
		SYSCTL_ADD_INT(&rk3399_tcphy_helper_ctx,
		    SYSCTL_CHILDREN(rk3399_tcphy_helper_tree), OID_AUTO,
		    "assert_all_rsts", CTLFLAG_RWTUN,
		    &rk3399_tcphy_helper_assert_all_rsts, 0,
		    "Assert tcphy/uphy/pipe resets before programming");
		SYSCTL_ADD_INT(&rk3399_tcphy_helper_ctx,
		    SYSCTL_CHILDREN(rk3399_tcphy_helper_tree), OID_AUTO,
		    "deassert_tcphy_rst", CTLFLAG_RWTUN,
		    &rk3399_tcphy_helper_deassert_tcphy_rst, 0,
		    "Deassert tcphy reset before config writes");
		SYSCTL_ADD_INT(&rk3399_tcphy_helper_ctx,
		    SYSCTL_CHILDREN(rk3399_tcphy_helper_tree), OID_AUTO,
		    "lane_cfg", CTLFLAG_RWTUN,
		    &rk3399_tcphy_helper_lane_cfg, 0,
		    "0=skip, 1=DP-only C/E, 2=USB3+DP D/F");
		SYSCTL_ADD_INT(&rk3399_tcphy_helper_ctx,
		    SYSCTL_CHILDREN(rk3399_tcphy_helper_tree), OID_AUTO,
		    "dp_mode", CTLFLAG_RWTUN,
		    &rk3399_tcphy_helper_dp_mode, 0,
		    "0=skip, 1=enter A2, 2=enter A2 then A0, 3=enter A0 only");
		SYSCTL_ADD_INT(&rk3399_tcphy_helper_ctx,
		    SYSCTL_CHILDREN(rk3399_tcphy_helper_tree), OID_AUTO,
		    "poll_pma_ready", CTLFLAG_RWTUN,
		    &rk3399_tcphy_helper_poll_pma_ready, 0,
		    "Poll PMA_CMN_CTRL1 BIT(0) after A2");
		SYSCTL_ADD_INT(&rk3399_tcphy_helper_ctx,
		    SYSCTL_CHILDREN(rk3399_tcphy_helper_tree), OID_AUTO,
		    "deassert_uphy_rst", CTLFLAG_RWTUN,
		    &rk3399_tcphy_helper_deassert_uphy_rst, 0,
		    "Deassert uphy reset before PMA ready poll");
		SYSCTL_ADD_INT(&rk3399_tcphy_helper_ctx,
		    SYSCTL_CHILDREN(rk3399_tcphy_helper_tree), OID_AUTO,
		    "deassert_pipe_rst", CTLFLAG_RWTUN,
		    &rk3399_tcphy_helper_deassert_pipe_rst, 0,
		    "Pulse SRST_UPHY0_PIPE_L00 via CRU after PMA ready");
		SYSCTL_ADD_U32(&rk3399_tcphy_helper_ctx,
		    SYSCTL_CHILDREN(rk3399_tcphy_helper_tree), OID_AUTO,
		    "uphy_rst_mask", CTLFLAG_RWTUN,
		    &rk3399_tcphy_helper_uphy_rst_mask, 0,
		    "CRU soft-reset bit mask for SRST_UPHY0");
		SYSCTL_ADD_U32(&rk3399_tcphy_helper_ctx,
		    SYSCTL_CHILDREN(rk3399_tcphy_helper_tree), OID_AUTO,
		    "tcphy_rst_mask", CTLFLAG_RWTUN,
		    &rk3399_tcphy_helper_tcphy_rst_mask, 0,
		    "CRU soft-reset bit mask for SRST_P_UPHY0_TCPHY");
		SYSCTL_ADD_U32(&rk3399_tcphy_helper_ctx,
		    SYSCTL_CHILDREN(rk3399_tcphy_helper_tree), OID_AUTO,
		    "pipe_rst_mask", CTLFLAG_RWTUN,
		    &rk3399_tcphy_helper_pipe_rst_mask, 0,
		    "CRU soft-reset bit mask for SRST_UPHY0_PIPE_L00");
		SYSCTL_ADD_INT(&rk3399_tcphy_helper_ctx,
		    SYSCTL_CHILDREN(rk3399_tcphy_helper_tree), OID_AUTO,
		    "aux_cal", CTLFLAG_RWTUN,
		    &rk3399_tcphy_helper_aux_cal, 0,
		    "Run RK3399 DP AUX calibration sequence");
		SYSCTL_ADD_INT(&rk3399_tcphy_helper_ctx,
		    SYSCTL_CHILDREN(rk3399_tcphy_helper_tree), OID_AUTO,
		    "poll_a0", CTLFLAG_RWTUN,
		    &rk3399_tcphy_helper_poll_a0, 0,
		    "Poll DP_MODE_CTL for DP_MODE_A0 after A0 request");
		SYSCTL_ADD_INT(&rk3399_tcphy_helper_ctx,
		    SYSCTL_CHILDREN(rk3399_tcphy_helper_tree), OID_AUTO,
		    "last_error", CTLFLAG_RD,
		    &rk3399_tcphy_helper_last_error, 0,
		    "Last apply error");
		SYSCTL_ADD_U32(&rk3399_tcphy_helper_ctx,
		    SYSCTL_CHILDREN(rk3399_tcphy_helper_tree), OID_AUTO,
		    "last_pma_cmn_ctrl1", CTLFLAG_RD,
		    &rk3399_tcphy_helper_last_pma_cmn_ctrl1, 0,
		    "Last PMA_CMN_CTRL1 readback during apply");
		SYSCTL_ADD_U32(&rk3399_tcphy_helper_ctx,
		    SYSCTL_CHILDREN(rk3399_tcphy_helper_tree), OID_AUTO,
		    "last_pma_poll_count", CTLFLAG_RD,
		    &rk3399_tcphy_helper_last_pma_poll_count, 0,
		    "Iterations taken for PMA ready in the latest full sequence");
		SYSCTL_ADD_U32(&rk3399_tcphy_helper_ctx,
		    SYSCTL_CHILDREN(rk3399_tcphy_helper_tree), OID_AUTO,
		    "last_a0_100ms", CTLFLAG_RD,
		    &rk3399_tcphy_helper_last_a0_100ms, 0,
		    "DP_MODE_CTL readback 100ms after the A0 write in full_reset_seq");
		SYSCTL_ADD_U32(&rk3399_tcphy_helper_ctx,
		    SYSCTL_CHILDREN(rk3399_tcphy_helper_tree), OID_AUTO,
		    "last_dp_mode_ctl", CTLFLAG_RD,
		    &rk3399_tcphy_helper_last_dp_mode_ctl, 0,
		    "Last DP_MODE_CTL readback after apply");
		SYSCTL_ADD_OPAQUE(&rk3399_tcphy_helper_ctx,
		    SYSCTL_CHILDREN(rk3399_tcphy_helper_tree), OID_AUTO,
		    "breadcrumbs", CTLFLAG_RD, rk3399_tcphy_helper_breadcrumb,
		    sizeof(rk3399_tcphy_helper_breadcrumb), "IU",
		    "full_reset_seq step breadcrumbs");
		SYSCTL_ADD_INT(&rk3399_tcphy_helper_ctx,
		    SYSCTL_CHILDREN(rk3399_tcphy_helper_tree), OID_AUTO,
		    "last_applied_mask", CTLFLAG_RD,
		    &rk3399_tcphy_helper_last_applied_mask, 0,
		    "Bitmask of fields successfully applied");
		SYSCTL_ADD_PROC(&rk3399_tcphy_helper_ctx,
		    SYSCTL_CHILDREN(rk3399_tcphy_helper_tree), OID_AUTO,
		    "apply", CTLTYPE_INT | CTLFLAG_MPSAFE | CTLFLAG_WR,
		    NULL, 0, rk3399_tcphy_helper_apply_sysctl, "I",
		    "Write 1 to apply selected TC-PHY fields");
		break;
	case MOD_UNLOAD:
		sysctl_ctx_free(&rk3399_tcphy_helper_ctx);
		break;
	default:
		break;
	}

	return (0);
}

static moduledata_t rk3399_tcphy_helper_mod = {
	"rk3399_tcphy_helper",
	rk3399_tcphy_helper_modevent,
	NULL
};

DECLARE_MODULE(rk3399_tcphy_helper, rk3399_tcphy_helper_mod,
    SI_SUB_DRIVERS, SI_ORDER_MIDDLE);
MODULE_VERSION(rk3399_tcphy_helper, 1);
