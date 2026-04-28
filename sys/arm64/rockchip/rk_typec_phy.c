/*-
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Copyright (c) 2019 Emmanuel Vadot <manu@FreeBSD.Org>
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
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE AUTHOR OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 */

/*
 * Rockchip PHY TYPEC
 */

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/bus.h>
#include <sys/rman.h>
#include <sys/kernel.h>
#include <sys/module.h>
#include <sys/gpio.h>
#include <machine/bus.h>

#include <dev/fdt/fdt_common.h>
#include <dev/ofw/ofw_bus.h>
#include <dev/ofw/ofw_bus_subr.h>
#include <dev/ofw/ofw_subr.h>

#include <dev/clk/clk.h>
#include <dev/phy/phy_usb.h>
#include <dev/syscon/syscon.h>
#include <dev/hwreset/hwreset.h>

#include "syscon_if.h"

#define	GRF_WRITE_MASK(mask)		((mask) << 16)
#define	GRF_USB3OTG_BASE(x)	(0x2430 + (0x10 * x))
#define	GRF_USB3OTG_CON0(x)	(GRF_USB3OTG_BASE(x) + 0x0)
#define	GRF_USB3OTG_CON1(x)	(GRF_USB3OTG_BASE(x) + 0x4)
#define	 USB3OTG_CON1_U3_DIS	(1 << 0)

#define	GRF_USB3PHY_BASE(x)	(0x0e580 + (0xc * (x)))
#define	GRF_USB3PHY_CON0(x)	(GRF_USB3PHY_BASE(x) + 0x0)
#define	 USB3PHY_CON0_USB2_ONLY	(1 << 3)
#define	GRF_USB3PHY_CON1(x)	(GRF_USB3PHY_BASE(x) + 0x4)
#define	GRF_USB3PHY_CON2(x)	(GRF_USB3PHY_BASE(x) + 0x8)
#define	GRF_USB3PHY_STATUS0	0x0e5c0
#define	GRF_USB3PHY_STATUS1	0x0e5c4

#define	CMN_PLL0_VCOCAL_INIT		(0x84 << 2)
#define	CMN_PLL0_VCOCAL_ITER		(0x85 << 2)
#define	CMN_PLL0_INTDIV			(0x94 << 2)
#define	CMN_PLL0_FRACDIV		(0x95 << 2)
#define	CMN_PLL0_HIGH_THR		(0x96 << 2)
#define	CMN_PLL0_DSM_DIAG		(0x97 << 2)
#define	CMN_PLL0_SS_CTRL1		(0x98 << 2)
#define	CMN_PLL0_SS_CTRL2		(0x99 << 2)
#define	CMN_DIAG_PLL0_FBH_OVRD		(0x1c0 << 2)
#define	CMN_DIAG_PLL0_FBL_OVRD		(0x1c1 << 2)
#define	CMN_DIAG_PLL0_OVRD		(0x1c2 << 2)
#define	CMN_DIAG_PLL0_V2I_TUNE		(0x1c5 << 2)
#define	CMN_DIAG_PLL0_CP_TUNE		(0x1c6 << 2)
#define	CMN_DIAG_PLL0_LF_PROG		(0x1c7 << 2)
#define	CMN_PLL1_VCOCAL_INIT		(0x104 << 2)
#define	CMN_PLL1_VCOCAL_ITER		(0x105 << 2)
#define	CMN_PLL1_VCOCAL_START		(0x108 << 2)
#define	CMN_PLL1_INTDIV			(0x114 << 2)
#define	CMN_PLL1_FRACDIV		(0x115 << 2)
#define	CMN_PLL1_HIGH_THR		(0x116 << 2)
#define	CMN_PLL1_DSM_DIAG		(0x117 << 2)
#define	CMN_PLL1_SS_CTRL1		(0x118 << 2)
#define	CMN_PLL1_SS_CTRL2		(0x119 << 2)
#define	CMN_PLLSM1_USER_DEF_CTRL	(0x11f << 2)
#define	CMN_DIAG_PLL1_FBH_OVRD		(0x1d0 << 2)
#define	CMN_DIAG_PLL1_FBL_OVRD		(0x1d1 << 2)
#define	CMN_DIAG_PLL1_OVRD		(0x1d2 << 2)
#define	CMN_DIAG_PLL1_V2I_TUNE		(0x1d5 << 2)
#define	CMN_DIAG_PLL1_CP_TUNE		(0x1d6 << 2)
#define	CMN_DIAG_PLL1_LF_PROG		(0x1d7 << 2)
#define	CMN_DIAG_PLL1_PTATIS_TUNE1	(0x1d8 << 2)
#define	CMN_DIAG_PLL1_PTATIS_TUNE2	(0x1d9 << 2)
#define	CMN_DIAG_PLL1_INCLK_CTRL	(0x1dc << 2)
#define	CMN_DIAG_HSCLK_SEL		(0x1e0 << 2)
#define	 CMN_DIAG_HSCLK_SEL_PLL_CONFIG	0x30
#define	 CMN_DIAG_HSCLK_SEL_PLL_MASK	0x33

#define	XCVR_PSM_RCTRL(lane)		((0x4001 | ((lane) << 9)) << 2)
#define	TX_TXCC_MGNFS_MULT_000(lane)	((0x4050 | ((lane) << 9)) << 2)
#define	TX_TXCC_MGNFS_MULT_001(lane)	((0x4051 | ((lane) << 9)) << 2)
#define	TX_TXCC_MGNFS_MULT_010(lane)	((0x4052 | ((lane) << 9)) << 2)
#define	TX_TXCC_MGNFS_MULT_011(lane)	((0x4053 | ((lane) << 9)) << 2)
#define	TX_TXCC_MGNFS_MULT_100(lane)	((0x4054 | ((lane) << 9)) << 2)
#define	TX_TXCC_MGNFS_MULT_101(lane)	((0x4055 | ((lane) << 9)) << 2)
#define	TX_TXCC_MGNFS_MULT_110(lane)	((0x4056 | ((lane) << 9)) << 2)
#define	TX_TXCC_MGNFS_MULT_111(lane)	((0x4057 | ((lane) << 9)) << 2)
#define	TX_TXCC_CPOST_MULT_10(lane)	((0x405c | ((lane) << 9)) << 2)
#define	TX_TXCC_CPOST_MULT_01(lane)	((0x405d | ((lane) << 9)) << 2)
#define	TX_TXCC_CPOST_MULT_00(lane)	((0x405e | ((lane) << 9)) << 2)
#define	TX_TXCC_CPOST_MULT_11(lane)	((0x405f | ((lane) << 9)) << 2)
#define	TX_TXCC_CAL_SCLR_MULT(lane)	((0x4047 | ((lane) << 9)) << 2)
#define	XCVR_DIAG_PLLDRC_CTRL(lane)	((0x40e0 | ((lane) << 9)) << 2)
#define	XCVR_DIAG_BIDI_CTRL(lane)	((0x40e8 | ((lane) << 9)) << 2)
#define	XCVR_DIAG_LANE_FCM_EN_MGN(lane)	((0x40f2 | ((lane) << 9)) << 2)
#define	TX_PSC_A0(lane)			((0x4100 | ((lane) << 9)) << 2)
#define	TX_PSC_A1(lane)			((0x4101 | ((lane) << 9)) << 2)
#define	TX_PSC_A2(lane)			((0x4102 | ((lane) << 9)) << 2)
#define	TX_PSC_A3(lane)			((0x4103 | ((lane) << 9)) << 2)
#define	TX_RCVDET_EN_TMR(lane)		((0x4122 | ((lane) << 9)) << 2)
#define	TX_RCVDET_ST_TMR(lane)		((0x4123 | ((lane) << 9)) << 2)
#define	TX_DIAG_TX_DRV(lane)		((0x41e1 | ((lane) << 9)) << 2)

#define	RX_PSC_A0(lane)			((0x8000 | ((lane) << 9)) << 2)
#define	RX_PSC_A1(lane)			((0x8001 | ((lane) << 9)) << 2)
#define	RX_PSC_A2(lane)			((0x8002 | ((lane) << 9)) << 2)
#define	RX_PSC_A3(lane)			((0x8003 | ((lane) << 9)) << 2)
#define	RX_PSC_CAL(lane)		((0x8006 | ((lane) << 9)) << 2)
#define	RX_PSC_RDY(lane)		((0x8007 | ((lane) << 9)) << 2)
#define	RX_SIGDET_HL_FILT_TMR(lane)	((0x8090 | ((lane) << 9)) << 2)
#define	RX_REE_CTRL_DATA_MASK(lane)	((0x81bb | ((lane) << 9)) << 2)
#define	RX_DIAG_SIGDET_TUNE(lane)	((0x81dc | ((lane) << 9)) << 2)

#define	PMA_LANE_CFG			(0xc000 << 2)
#define	PIN_ASSIGN_C_E			0x51d9
#define	PIN_ASSIGN_D_F			0x5100
#define	PIPE_CMN_CTRL1			(0xc001 << 2)
#define	DP_MODE_CTL			(0xc008 << 2)
#define	DP_CLK_CTL			(0xc009 << 2)
#define	 DP_PLL_CLOCK_ENABLE		(1U << 2)
#define	 DP_PLL_ENABLE			(1U << 0)
#define	 DP_PLL_DATA_RATE_RBR		((2U << 12) | (4U << 8))
#define	DP_MODE_ENTER_A2		0xc104
#define	DP_MODE_ENTER_A0		0xc101
#define	 DP_MODE_A0_READY		(1U << 4)
#define	 DP_MODE_A2_READY		(1U << 6)
#define	PMA_CMN_CTRL1			(0xc800 << 2)
#define	 PMA_CMN_CTRL1_READY		(1 << 0)

/* AUX channel calibration registers (within the same TC-PHY MMIO region). */
#define	CMN_TXPUCAL_CTRL		(0x00e0 << 2)
#define	CMN_TXPDCAL_CTRL		(0x00f0 << 2)
#define	CMN_TXPU_ADJ_CTRL		(0x0108 << 2)
#define	CMN_TXPD_ADJ_CTRL		(0x010c << 2)
#define	PHY_DP_TX_CTL			(0xc408 << 2)
#define	TX_ANA_CTRL_REG_1		(0x5020 << 2)
#define	TX_ANA_CTRL_REG_2		(0x5021 << 2)
#define	TXDA_COEFF_CALC_CTRL		(0x5022 << 2)
#define	TX_DIG_CTRL_REG_2		(0x5024 << 2)
#define	TXDA_CYA_AUXDA_CYA		(0x5025 << 2)
#define	TX_ANA_CTRL_REG_3		(0x5026 << 2)
#define	TX_ANA_CTRL_REG_4		(0x5027 << 2)
#define	TX_ANA_CTRL_REG_5		(0x5029 << 2)
#define	AUX_CH_LANE			8
#define	TXDA_DP_AUX_EN			(1U << 15)
#define	TXDA_CAL_LATCH_EN		(1U << 13)
#define	TXDA_BGREF_EN			(1U << 8)
#define	TXDA_DRV_LDO_EN			(1U << 7)
#define	TXDA_DECAP_EN_DEL		(1U << 6)
#define	TXDA_DECAP_EN			(1U << 5)
#define	TXDA_UPHY_SUPPLY_EN_DEL		(1U << 4)
#define	TXDA_UPHY_SUPPLY_EN		(1U << 3)
#define	XCVR_DECAP_EN_DEL		(1U << 9)
#define	XCVR_DECAP_EN			(1U << 8)
#define	TXDA_DRV_PREDRV_EN_DEL		(1U << 1)
#define	TXDA_DRV_PREDRV_EN		(1U << 0)
#define	TX_HIGH_Z_TM_EN			(1U << 15)
#define	TX_RESCAL_CODE_MASK		0x3f

struct rk_typec_phy_reg {
	uint16_t	value;
	uint32_t	addr;
};

struct rk_typec_phy_grf_prop {
	uint32_t	reg;
	uint32_t	lsb;
	uint32_t	msb;
};

static const struct rk_typec_phy_grf_prop rk3399_tcphy0_conn_dir = {
	.reg = 0x0e580,
	.lsb = 0,
	.msb = 0,
};
/* Selects external PSM clock source (from CDN-DP); required for PSM to run A2/A0. */
static const struct rk_typec_phy_grf_prop rk3399_tcphy0_external_psm = {
	.reg = 0x0e588,
	.lsb = 14,
	.msb = 14,
};
static const struct rk_typec_phy_grf_prop rk3399_tcphy0_uphy_dp_sel = {
	.reg = 0x6268,
	.lsb = 19,
	.msb = 19,
};

static const struct rk_typec_phy_reg rk3399_tcphy_dp_pll_cfg[] = {
	{ 0xf0,	CMN_PLL1_VCOCAL_INIT },
	{ 0x18,	CMN_PLL1_VCOCAL_ITER },
	{ 0x30b9,	CMN_PLL1_VCOCAL_START },
	{ 0x21c,	CMN_PLL1_INTDIV },
	{ 0x0,		CMN_PLL1_FRACDIV },
	{ 0x5,		CMN_PLL1_HIGH_THR },
	{ 0x35,	CMN_PLL1_SS_CTRL1 },
	{ 0x7f1e,	CMN_PLL1_SS_CTRL2 },
	{ 0x20,	CMN_PLL1_DSM_DIAG },
	{ 0x0,		CMN_PLLSM1_USER_DEF_CTRL },
	{ 0x0,		CMN_DIAG_PLL1_OVRD },
	{ 0x0,		CMN_DIAG_PLL1_FBH_OVRD },
	{ 0x0,		CMN_DIAG_PLL1_FBL_OVRD },
	{ 0x6,		CMN_DIAG_PLL1_V2I_TUNE },
	{ 0x45,	CMN_DIAG_PLL1_CP_TUNE },
	{ 0x8,		CMN_DIAG_PLL1_LF_PROG },
	{ 0x100,	CMN_DIAG_PLL1_PTATIS_TUNE1 },
	{ 0x7,		CMN_DIAG_PLL1_PTATIS_TUNE2 },
	{ 0x4,		CMN_DIAG_PLL1_INCLK_CTRL },
};

static struct ofw_compat_data compat_data[] = {
	{ "rockchip,rk3399-typec-phy",	1 },
	{ NULL,				0 }
};

static struct resource_spec rk_typec_phy_spec[] = {
	{ SYS_RES_MEMORY,	0,	RF_ACTIVE },
	{ -1, 0 }
};

struct rk_typec_phy_softc {
	device_t		dev;
	struct resource		*res;
	struct syscon		*grf;
	clk_t			tcpdcore;
	clk_t			tcpdphy_ref;
	hwreset_t		rst_uphy;
	hwreset_t		rst_pipe;
	hwreset_t		rst_tcphy;
	int			mode;
	phy_mode_t		dp_mode;
	phy_submode_t		dp_submode;
	int			phy_ctrl_id;
};

#define	RK_TYPEC_PHY_READ(sc, reg)		bus_read_4(sc->res, (reg))
#define	RK_TYPEC_PHY_WRITE(sc, reg, val)	bus_write_4(sc->res, (reg), (val))

/* Phy class and methods. */
static int rk_typec_phy_enable(struct phynode *phynode, bool enable);
static int rk_typec_phy_get_mode(struct phynode *phy, int *mode);
static int rk_typec_phy_set_mode(struct phynode *phy, int mode);
static int rk_typec_phy_set_phy_mode(struct phynode *phy, phy_mode_t mode,
    phy_submode_t submode);
static phynode_method_t rk_typec_phy_phynode_methods[] = {
	PHYNODEMETHOD(phynode_enable,		rk_typec_phy_enable),
	PHYNODEMETHOD(phynode_set_mode,		rk_typec_phy_set_phy_mode),
	PHYNODEMETHOD(phynode_usb_get_mode,	rk_typec_phy_get_mode),
	PHYNODEMETHOD(phynode_usb_set_mode,	rk_typec_phy_set_mode),

	PHYNODEMETHOD_END
};

DEFINE_CLASS_1(rk_typec_phy_phynode, rk_typec_phy_phynode_class,
    rk_typec_phy_phynode_methods,
    sizeof(struct phynode_usb_sc), phynode_usb_class);

enum RK3399_USBPHY {
	RK3399_TYPEC_PHY_DP = 0,
	RK3399_TYPEC_PHY_USB3,
};

static int
rk_typec_phy_set_field(struct rk_typec_phy_softc *sc,
    const struct rk_typec_phy_grf_prop *prop, uint32_t value)
{
	uint32_t field_mask, regval;

	field_mask = ((1u << (prop->msb - prop->lsb + 1)) - 1u) << prop->lsb;
	regval = ((value << prop->lsb) & field_mask) | GRF_WRITE_MASK(field_mask);
	return (SYSCON_WRITE_4(sc->grf, prop->reg, regval));
}

static int
rk_typec_phy_apply_dp_grf(struct rk_typec_phy_softc *sc)
{
	if (sc->phy_ctrl_id != 0)
		return (0);

	return (rk_typec_phy_set_field(sc, &rk3399_tcphy0_conn_dir, 1));
}

static int
rk_typec_phy_enable_dp_sel(struct rk_typec_phy_softc *sc)
{
	if (sc->phy_ctrl_id != 0)
		return (0);

	return (rk_typec_phy_set_field(sc, &rk3399_tcphy0_uphy_dp_sel, 1));
}

static void
rk_typec_phy_cfg_dp_pll(struct rk_typec_phy_softc *sc)
{
	size_t i;

	RK_TYPEC_PHY_WRITE(sc, DP_CLK_CTL, DP_PLL_CLOCK_ENABLE |
	    DP_PLL_ENABLE | DP_PLL_DATA_RATE_RBR);
	for (i = 0; i < nitems(rk3399_tcphy_dp_pll_cfg); i++)
		RK_TYPEC_PHY_WRITE(sc, rk3399_tcphy_dp_pll_cfg[i].addr,
		    rk3399_tcphy_dp_pll_cfg[i].value);
}

static void
rk_typec_phy_cfg_dp_lane(struct rk_typec_phy_softc *sc, u_int lane)
{
	uint32_t reg;

	RK_TYPEC_PHY_WRITE(sc, XCVR_PSM_RCTRL(lane), 0xbefc);
	RK_TYPEC_PHY_WRITE(sc, TX_PSC_A0(lane), 0x6799);
	RK_TYPEC_PHY_WRITE(sc, TX_PSC_A1(lane), 0x6798);
	RK_TYPEC_PHY_WRITE(sc, TX_PSC_A2(lane), 0x98);
	RK_TYPEC_PHY_WRITE(sc, TX_PSC_A3(lane), 0x98);

	RK_TYPEC_PHY_WRITE(sc, TX_TXCC_MGNFS_MULT_000(lane), 0x0);
	RK_TYPEC_PHY_WRITE(sc, TX_TXCC_MGNFS_MULT_001(lane), 0x0);
	RK_TYPEC_PHY_WRITE(sc, TX_TXCC_MGNFS_MULT_010(lane), 0x0);
	RK_TYPEC_PHY_WRITE(sc, TX_TXCC_MGNFS_MULT_011(lane), 0x0);
	RK_TYPEC_PHY_WRITE(sc, TX_TXCC_MGNFS_MULT_100(lane), 0x0);
	RK_TYPEC_PHY_WRITE(sc, TX_TXCC_MGNFS_MULT_101(lane), 0x0);
	RK_TYPEC_PHY_WRITE(sc, TX_TXCC_MGNFS_MULT_110(lane), 0x0);
	RK_TYPEC_PHY_WRITE(sc, TX_TXCC_MGNFS_MULT_111(lane), 0x0);
	RK_TYPEC_PHY_WRITE(sc, TX_TXCC_CPOST_MULT_10(lane), 0x0);
	RK_TYPEC_PHY_WRITE(sc, TX_TXCC_CPOST_MULT_01(lane), 0x0);
	RK_TYPEC_PHY_WRITE(sc, TX_TXCC_CPOST_MULT_00(lane), 0x0);
	RK_TYPEC_PHY_WRITE(sc, TX_TXCC_CPOST_MULT_11(lane), 0x0);

	RK_TYPEC_PHY_WRITE(sc, TX_TXCC_CAL_SCLR_MULT(lane), 0x128);
	RK_TYPEC_PHY_WRITE(sc, TX_DIAG_TX_DRV(lane), 0x400);

	reg = RK_TYPEC_PHY_READ(sc, XCVR_DIAG_PLLDRC_CTRL(lane));
	reg = (reg & 0x8fff) | 0x6000;
	RK_TYPEC_PHY_WRITE(sc, XCVR_DIAG_PLLDRC_CTRL(lane), reg);
}

static void
rk_typec_phy_set_usb2_only(struct rk_typec_phy_softc *sc, bool usb2only)
{
	uint32_t reg;

	/* Disable usb3tousb2 only */
	reg = SYSCON_READ_4(sc->grf, GRF_USB3PHY_CON0(sc->phy_ctrl_id));
	if (usb2only)
		reg |= USB3PHY_CON0_USB2_ONLY;
	else
		reg &= ~USB3PHY_CON0_USB2_ONLY;
	/* Write Mask */
	reg |= (USB3PHY_CON0_USB2_ONLY) << 16;
	SYSCON_WRITE_4(sc->grf, GRF_USB3PHY_CON0(sc->phy_ctrl_id), reg);

	/* Enable the USB3 Super Speed port */
	reg = SYSCON_READ_4(sc->grf, GRF_USB3OTG_CON1(sc->phy_ctrl_id));
	if (usb2only)
		reg |= USB3OTG_CON1_U3_DIS;
	else
		reg &= ~USB3OTG_CON1_U3_DIS;
	/* Write Mask */
	reg |= (USB3OTG_CON1_U3_DIS) << 16;
	SYSCON_WRITE_4(sc->grf, GRF_USB3OTG_CON1(sc->phy_ctrl_id), reg);
}

/*
 * rk_typec_phy_dp_aux_calibration
 *
 * Programs the AUX channel analog section using values derived from the PHY's
 * built-in PU/PD calibration registers.  Must be called after the PMA PLL has
 * locked (PMA_CMN_CTRL1_READY) and before requesting the A0 state transition.
 * Mirrors the Linux tcphy_dp_aux_calibration() sequence from
 * phy-rockchip-typec.c.
 */
static void
rk_typec_phy_dp_aux_calibration(struct rk_typec_phy_softc *sc)
{
	uint32_t tx1, tx2, val;
	int pu_calib, pd_calib, pu_adj, pd_adj, calib;

	pu_calib = (int)(RK_TYPEC_PHY_READ(sc, CMN_TXPUCAL_CTRL) & TX_RESCAL_CODE_MASK);
	pd_calib = (int)(RK_TYPEC_PHY_READ(sc, CMN_TXPDCAL_CTRL) & TX_RESCAL_CODE_MASK);
	pu_adj   = (int)(int8_t)(RK_TYPEC_PHY_READ(sc, CMN_TXPU_ADJ_CTRL) & 0xff);
	pd_adj   = (int)(int8_t)(RK_TYPEC_PHY_READ(sc, CMN_TXPD_ADJ_CTRL) & 0xff);
	calib = (pu_calib + pd_calib) / 2 + pu_adj + pd_adj;
	if (calib < 0)
		calib = 0;
	if (calib > TX_RESCAL_CODE_MASK)
		calib = TX_RESCAL_CODE_MASK;

	tx1 = RK_TYPEC_PHY_READ(sc, TX_ANA_CTRL_REG_1);
	tx1 &= ~TXDA_CAL_LATCH_EN;
	RK_TYPEC_PHY_WRITE(sc, TX_ANA_CTRL_REG_1, tx1);

	val = RK_TYPEC_PHY_READ(sc, TX_DIG_CTRL_REG_2);
	val &= ~TX_RESCAL_CODE_MASK;
	val |= (uint32_t)calib;
	RK_TYPEC_PHY_WRITE(sc, TX_DIG_CTRL_REG_2, val);
	DELAY(10000);

	tx1 |= TXDA_CAL_LATCH_EN;
	RK_TYPEC_PHY_WRITE(sc, TX_ANA_CTRL_REG_1, tx1);
	DELAY(200);

	RK_TYPEC_PHY_WRITE(sc, PHY_DP_TX_CTL, 0);

	tx2 = XCVR_DECAP_EN;
	RK_TYPEC_PHY_WRITE(sc, TX_ANA_CTRL_REG_2, tx2);
	DELAY(1);
	tx2 |= XCVR_DECAP_EN_DEL;
	RK_TYPEC_PHY_WRITE(sc, TX_ANA_CTRL_REG_2, tx2);

	RK_TYPEC_PHY_WRITE(sc, TX_ANA_CTRL_REG_3, 0);

	tx1 |= TXDA_UPHY_SUPPLY_EN;
	RK_TYPEC_PHY_WRITE(sc, TX_ANA_CTRL_REG_1, tx1);
	DELAY(1);
	tx1 |= TXDA_UPHY_SUPPLY_EN_DEL;
	RK_TYPEC_PHY_WRITE(sc, TX_ANA_CTRL_REG_1, tx1);

	RK_TYPEC_PHY_WRITE(sc, TX_ANA_CTRL_REG_5, 0);
	RK_TYPEC_PHY_WRITE(sc, TX_ANA_CTRL_REG_4, 0x1001);

	tx1 |= TXDA_DRV_LDO_EN;
	RK_TYPEC_PHY_WRITE(sc, TX_ANA_CTRL_REG_1, tx1);
	DELAY(5);
	tx1 |= TXDA_BGREF_EN;
	RK_TYPEC_PHY_WRITE(sc, TX_ANA_CTRL_REG_1, tx1);

	tx2 |= TXDA_DRV_PREDRV_EN;
	RK_TYPEC_PHY_WRITE(sc, TX_ANA_CTRL_REG_2, tx2);
	DELAY(1);
	tx2 |= TXDA_DRV_PREDRV_EN_DEL;
	RK_TYPEC_PHY_WRITE(sc, TX_ANA_CTRL_REG_2, tx2);

	tx1 |= TXDA_DP_AUX_EN | TXDA_DECAP_EN;
	tx1 &= ~TXDA_DRV_LDO_EN;
	tx1 &= ~TXDA_BGREF_EN;
	RK_TYPEC_PHY_WRITE(sc, TX_ANA_CTRL_REG_1, tx1);
	DELAY(1);
	tx1 |= TXDA_DECAP_EN_DEL;
	RK_TYPEC_PHY_WRITE(sc, TX_ANA_CTRL_REG_1, tx1);

	RK_TYPEC_PHY_WRITE(sc, TX_ANA_CTRL_REG_4, 0);
	RK_TYPEC_PHY_WRITE(sc, TXDA_COEFF_CALC_CTRL, 0);
	RK_TYPEC_PHY_WRITE(sc, TXDA_CYA_AUXDA_CYA, 0);
	RK_TYPEC_PHY_WRITE(sc, TX_TXCC_CAL_SCLR_MULT(AUX_CH_LANE), 0x128);

	val = RK_TYPEC_PHY_READ(sc, TX_DIG_CTRL_REG_2);
	val |= TX_HIGH_Z_TM_EN;
	RK_TYPEC_PHY_WRITE(sc, TX_DIG_CTRL_REG_2, val);
}

static int
rk_typec_phy_enable(struct phynode *phynode, bool enable)
{
	struct rk_typec_phy_softc *sc;
	device_t dev;
	intptr_t phy;
	uint32_t reg;
	int err, retry;

	dev = phynode_get_device(phynode);
	phy = phynode_get_id(phynode);
	sc = device_get_softc(dev);

	if (phy != RK3399_TYPEC_PHY_DP && phy != RK3399_TYPEC_PHY_USB3)
		return (ERANGE);

	if (phy == RK3399_TYPEC_PHY_USB3)
		rk_typec_phy_set_usb2_only(sc, false);

	err = clk_enable(sc->tcpdcore);
	if (err != 0) {
		device_printf(dev, "Could not enable clock %s\n",
		    clk_get_name(sc->tcpdcore));
		return (ENXIO);
	}
	err = clk_enable(sc->tcpdphy_ref);
	if (err != 0) {
		device_printf(dev, "Could not enable clock %s\n",
		    clk_get_name(sc->tcpdphy_ref));
		clk_disable(sc->tcpdcore);
		return (ENXIO);
	}

	if (phy == RK3399_TYPEC_PHY_DP && sc->phy_ctrl_id == 0) {
		/*
		 * Select external PSM clock (CDN-DP SOURCE_PHY_CAR provides it
		 * via rk_cdn_dp_clock_reset at stage 5).  The CDN-DP firmware
		 * needs this clock to drive TC-PHY A-state transitions via PIPE
		 * PowerDown signals after SET_HOST_CAPABILITIES processing.
		 * Matches Linux tcphy_dp_phy_init: property_enable(external_psm,1).
		 */
		err = rk_typec_phy_set_field(sc, &rk3399_tcphy0_external_psm, 1);
		if (err != 0) {
			device_printf(dev, "cannot set external PSM clock\n");
			return (err);
		}
	}

	hwreset_deassert(sc->rst_tcphy);

	if (phy == RK3399_TYPEC_PHY_DP) {
		err = rk_typec_phy_apply_dp_grf(sc);
		if (err != 0) {
			device_printf(dev, "cannot apply DP GRF routing\n");
			return (err);
		}

		/* Common 24 MHz setup matches the helper and Linux tcphy path. */
		RK_TYPEC_PHY_WRITE(sc, PMA_CMN_CTRL1, 0x830);
		for (int i = 0; i < 4; i++) {
			RK_TYPEC_PHY_WRITE(sc, XCVR_DIAG_LANE_FCM_EN_MGN(i), 0x90);
			RK_TYPEC_PHY_WRITE(sc, TX_RCVDET_EN_TMR(i), 0x960);
			RK_TYPEC_PHY_WRITE(sc, TX_RCVDET_ST_TMR(i), 0x30);
		}
		reg = RK_TYPEC_PHY_READ(sc, CMN_DIAG_HSCLK_SEL);
		reg &= ~CMN_DIAG_HSCLK_SEL_PLL_MASK;
		reg |= CMN_DIAG_HSCLK_SEL_PLL_CONFIG;
		RK_TYPEC_PHY_WRITE(sc, CMN_DIAG_HSCLK_SEL, reg);

		rk_typec_phy_cfg_dp_pll(sc);
		for (int i = 0; i < 4; i++)
			rk_typec_phy_cfg_dp_lane(sc, i);
		RK_TYPEC_PHY_WRITE(sc, PMA_LANE_CFG, PIN_ASSIGN_C_E);

		hwreset_deassert(sc->rst_uphy);
		for (retry = 10000; retry > 0; retry--) {
			reg = RK_TYPEC_PHY_READ(sc, PMA_CMN_CTRL1);
			if (reg & PMA_CMN_CTRL1_READY)
				break;
			DELAY(10);
		}
		if (retry == 0) {
			device_printf(sc->dev, "Timeout waiting for PMA\n");
			return (ENXIO);
		}

		/* Deassert pipe before A2 poll and enable uphy_dp_sel to
		 * connect CDN-DP to the PHY (matches Linux tcphy_phy_init). */
		hwreset_deassert(sc->rst_pipe);
		err = rk_typec_phy_enable_dp_sel(sc);
		if (err != 0) {
			device_printf(dev, "cannot enable DP sel GRF\n");
			return (err);
		}

		/*
		 * If CDN-DP firmware is already driving PIPE P0 (A0 active),
		 * the TC-PHY will show A0_READY without needing the A2→A0 dance.
		 * Check the current DP_MODE_CTL: if A0 is already achieved, skip
		 * the A2 request (which would break A0) and go straight to aux
		 * calibration. This is the normal path when called after fw-active.
		 */
		reg = RK_TYPEC_PHY_READ(sc, DP_MODE_CTL);
		device_printf(dev, "DP_MODE_CTL after pipe/sel: 0x%x pipe_cmn=0x%x\n",
		    reg, RK_TYPEC_PHY_READ(sc, PIPE_CMN_CTRL1));

		if (reg & DP_MODE_A0_READY) {
			device_printf(dev, "TC-PHY already in A0 (CDN-DP PIPE P0); skipping A2 sequence\n");
			rk_typec_phy_dp_aux_calibration(sc);
			return (0);
		}

		/* TC-PHY not in A0 yet; do the standard A2→A0 sequence. */
		RK_TYPEC_PHY_WRITE(sc, DP_MODE_CTL, DP_MODE_ENTER_A2);
		for (retry = 10000; retry > 0; retry--) {
			reg = RK_TYPEC_PHY_READ(sc, DP_MODE_CTL);
			if (reg & DP_MODE_A2_READY)
				break;
			DELAY(10);
		}
		if (retry == 0)
			device_printf(sc->dev,
			    "Timeout waiting for DP A2: dp_mode=0x%x pma_cmn=0x%x "
			    "xcvr_psm=0x%x tx_psc_a2=0x%x pma_lane=0x%x pipe_cmn=0x%x\n",
			    RK_TYPEC_PHY_READ(sc, DP_MODE_CTL),
			    RK_TYPEC_PHY_READ(sc, PMA_CMN_CTRL1),
			    RK_TYPEC_PHY_READ(sc, XCVR_PSM_RCTRL(0)),
			    RK_TYPEC_PHY_READ(sc, TX_PSC_A2(0)),
			    RK_TYPEC_PHY_READ(sc, PMA_LANE_CFG),
			    RK_TYPEC_PHY_READ(sc, PIPE_CMN_CTRL1));

		rk_typec_phy_dp_aux_calibration(sc);

		RK_TYPEC_PHY_WRITE(sc, DP_MODE_CTL, DP_MODE_ENTER_A0);
		for (retry = 10000; retry > 0; retry--) {
			reg = RK_TYPEC_PHY_READ(sc, DP_MODE_CTL);
			if (reg & DP_MODE_A0_READY)
				break;
			DELAY(10);
		}
		if (retry == 0)
			device_printf(sc->dev, "Timeout waiting for DP A0 (dp_mode_ctl=0x%x)\n",
			    RK_TYPEC_PHY_READ(sc, DP_MODE_CTL));
		return (0);
	}

	/* 24M configuration, magic values from rockchip */
	RK_TYPEC_PHY_WRITE(sc, PMA_CMN_CTRL1, 0x830);
	for (int i = 0; i < 4; i++) {
		RK_TYPEC_PHY_WRITE(sc, XCVR_DIAG_LANE_FCM_EN_MGN(i), 0x90);
		RK_TYPEC_PHY_WRITE(sc, TX_RCVDET_EN_TMR(i), 0x960);
		RK_TYPEC_PHY_WRITE(sc, TX_RCVDET_ST_TMR(i), 0x30);
	}
	reg = RK_TYPEC_PHY_READ(sc, CMN_DIAG_HSCLK_SEL);
	reg &= ~CMN_DIAG_HSCLK_SEL_PLL_MASK;
	reg |= CMN_DIAG_HSCLK_SEL_PLL_CONFIG;
	RK_TYPEC_PHY_WRITE(sc, CMN_DIAG_HSCLK_SEL, reg);

	/* PLL configuration, magic values from rockchip */
	RK_TYPEC_PHY_WRITE(sc, CMN_PLL0_VCOCAL_INIT, 0xf0);
	RK_TYPEC_PHY_WRITE(sc, CMN_PLL0_VCOCAL_ITER, 0x18);
	RK_TYPEC_PHY_WRITE(sc, CMN_PLL0_INTDIV, 0xd0);
	RK_TYPEC_PHY_WRITE(sc, CMN_PLL0_FRACDIV, 0x4a4a);
	RK_TYPEC_PHY_WRITE(sc, CMN_PLL0_HIGH_THR, 0x34);
	RK_TYPEC_PHY_WRITE(sc, CMN_PLL0_SS_CTRL1, 0x1ee);
	RK_TYPEC_PHY_WRITE(sc, CMN_PLL0_SS_CTRL2, 0x7f03);
	RK_TYPEC_PHY_WRITE(sc, CMN_PLL0_DSM_DIAG, 0x20);
	RK_TYPEC_PHY_WRITE(sc, CMN_DIAG_PLL0_OVRD, 0);
	RK_TYPEC_PHY_WRITE(sc, CMN_DIAG_PLL0_FBH_OVRD, 0);
	RK_TYPEC_PHY_WRITE(sc, CMN_DIAG_PLL0_FBL_OVRD, 0);
	RK_TYPEC_PHY_WRITE(sc, CMN_DIAG_PLL0_V2I_TUNE, 0x7);
	RK_TYPEC_PHY_WRITE(sc, CMN_DIAG_PLL0_CP_TUNE, 0x45);
	RK_TYPEC_PHY_WRITE(sc, CMN_DIAG_PLL0_LF_PROG, 0x8);

	/* Configure the TX and RX line, magic values from rockchip */
	RK_TYPEC_PHY_WRITE(sc, TX_PSC_A0(0), 0x7799);
	RK_TYPEC_PHY_WRITE(sc, TX_PSC_A1(0), 0x7798);
	RK_TYPEC_PHY_WRITE(sc, TX_PSC_A2(0), 0x5098);
	RK_TYPEC_PHY_WRITE(sc, TX_PSC_A3(0), 0x5098);
	RK_TYPEC_PHY_WRITE(sc, TX_TXCC_MGNFS_MULT_000(0), 0x0);
	RK_TYPEC_PHY_WRITE(sc, XCVR_DIAG_BIDI_CTRL(0), 0xbf);

	RK_TYPEC_PHY_WRITE(sc, RX_PSC_A0(1), 0xa6fd);
	RK_TYPEC_PHY_WRITE(sc, RX_PSC_A1(1), 0xa6fd);
	RK_TYPEC_PHY_WRITE(sc, RX_PSC_A2(1), 0xa410);
	RK_TYPEC_PHY_WRITE(sc, RX_PSC_A3(1), 0x2410);
	RK_TYPEC_PHY_WRITE(sc, RX_PSC_CAL(1), 0x23ff);
	RK_TYPEC_PHY_WRITE(sc, RX_SIGDET_HL_FILT_TMR(1), 0x13);
	RK_TYPEC_PHY_WRITE(sc, RX_REE_CTRL_DATA_MASK(1), 0x03e7);
	RK_TYPEC_PHY_WRITE(sc, RX_DIAG_SIGDET_TUNE(1), 0x1004);
	RK_TYPEC_PHY_WRITE(sc, RX_PSC_RDY(1), 0x2010);
	RK_TYPEC_PHY_WRITE(sc, XCVR_DIAG_BIDI_CTRL(1), 0xfb);

	RK_TYPEC_PHY_WRITE(sc, PMA_LANE_CFG, PIN_ASSIGN_D_F);

	RK_TYPEC_PHY_WRITE(sc, DP_MODE_CTL, DP_MODE_ENTER_A2);

	hwreset_deassert(sc->rst_uphy);

	for (retry = 10000; retry > 0; retry--) {
		reg = RK_TYPEC_PHY_READ(sc, PMA_CMN_CTRL1);
		if (reg & PMA_CMN_CTRL1_READY)
			break;
		DELAY(10);
	}
	if (retry == 0) {
		device_printf(sc->dev, "Timeout waiting for PMA\n");
		return (ENXIO);
	}

	hwreset_deassert(sc->rst_pipe);

	return (0);
}

static int
rk_typec_phy_get_mode(struct phynode *phynode, int *mode)
{
	struct rk_typec_phy_softc *sc;
	intptr_t phy;
	device_t dev;

	dev = phynode_get_device(phynode);
	phy = phynode_get_id(phynode);
	sc = device_get_softc(dev);

	if (phy != RK3399_TYPEC_PHY_USB3)
		return (ERANGE);

	*mode = sc->mode;

	return (0);
}

static int
rk_typec_phy_set_mode(struct phynode *phynode, int mode)
{
	struct rk_typec_phy_softc *sc;
	intptr_t phy;
	device_t dev;

	dev = phynode_get_device(phynode);
	phy = phynode_get_id(phynode);
	sc = device_get_softc(dev);

	if (phy != RK3399_TYPEC_PHY_USB3)
		return (ERANGE);

	sc->mode = mode;

	return (0);
}

static int
rk_typec_phy_set_phy_mode(struct phynode *phynode, phy_mode_t mode,
    phy_submode_t submode)
{
	struct rk_typec_phy_softc *sc;
	intptr_t phy;
	device_t dev;

	dev = phynode_get_device(phynode);
	phy = phynode_get_id(phynode);
	sc = device_get_softc(dev);

	if (phy == RK3399_TYPEC_PHY_DP) {
		if (mode != PHY_MODE_DP)
			return (EINVAL);
		sc->dp_mode = mode;
		sc->dp_submode = submode;
		return (0);
	}

	if (phy != RK3399_TYPEC_PHY_USB3)
		return (ERANGE);

	switch (mode) {
	case PHY_MODE_USB_HOST:
	case PHY_MODE_USB_DEVICE:
	case PHY_MODE_USB_OTG:
		return (0);
	default:
		return (EINVAL);
	}
}

static int
rk_typec_phy_probe(device_t dev)
{

	if (!ofw_bus_status_okay(dev))
		return (ENXIO);

	if (ofw_bus_search_compatible(dev, compat_data)->ocd_data == 0)
		return (ENXIO);

	device_set_desc(dev, "Rockchip RK3399 PHY TYPEC");
	return (BUS_PROBE_DEFAULT);
}

static int
rk_typec_phy_attach(device_t dev)
{
	struct rk_typec_phy_softc *sc;
	struct phynode_init_def phy_init;
	struct phynode *phynode;
	phandle_t node, dp, usb3;
	phandle_t reg_prop[4];

	sc = device_get_softc(dev);
	sc->dev = dev;
	node = ofw_bus_get_node(dev);

	/* 
	 * Find out which phy we are.
	 * There is not property for this so we need to know the
	 * address to use the correct GRF registers.
	 */
	if (OF_getencprop(node, "reg", reg_prop, sizeof(reg_prop)) <= 0) {
		device_printf(dev, "Cannot guess phy controller id\n");
		return (ENXIO);
	}
	switch (reg_prop[1]) {
	case 0xff7c0000:
		sc->phy_ctrl_id = 0;
		break;
	case 0xff800000:
		sc->phy_ctrl_id = 1;
		break;
	default:
		device_printf(dev, "Unknown address %x for typec-phy\n", reg_prop[1]);
		return (ENXIO);
	}

	if (bus_alloc_resources(dev, rk_typec_phy_spec, &sc->res) != 0) {
		device_printf(dev, "cannot allocate resources for device\n");
		goto fail;
	}

	if (syscon_get_by_ofw_property(dev, node,
	    "rockchip,grf", &sc->grf) != 0) {
		device_printf(dev, "Cannot get syscon handle\n");
		goto fail;
	}

	if (clk_get_by_ofw_name(dev, 0, "tcpdcore", &sc->tcpdcore) != 0) {
		device_printf(dev, "Cannot get tcpdcore clock\n");
		goto fail;
	}
	if (clk_get_by_ofw_name(dev, 0, "tcpdphy-ref", &sc->tcpdphy_ref) != 0) {
		device_printf(dev, "Cannot get tcpdphy-ref clock\n");
		goto fail;
	}

	if (hwreset_get_by_ofw_name(dev, 0, "uphy", &sc->rst_uphy) != 0) {
		device_printf(dev, "Cannot get uphy reset\n");
		goto fail;
	}
	if (hwreset_get_by_ofw_name(dev, 0, "uphy-pipe", &sc->rst_pipe) != 0) {
		device_printf(dev, "Cannot get uphy-pipe reset\n");
		goto fail;
	}
	if (hwreset_get_by_ofw_name(dev, 0, "uphy-tcphy", &sc->rst_tcphy) != 0) {
		device_printf(dev, "Cannot get uphy-tcphy reset\n");
		goto fail;
	}

	/* 
	 * Make sure that the module is asserted 
	 * We need to deassert in a certain order when we enable the phy
	 */
	hwreset_assert(sc->rst_uphy);
	hwreset_assert(sc->rst_pipe);
	hwreset_assert(sc->rst_tcphy);

	/* Set the assigned clocks parent and freq */
	if (clk_set_assigned(dev, node) != 0) {
		device_printf(dev, "clk_set_assigned failed\n");
		goto fail;
	}

	dp = ofw_bus_find_child(node, "dp-port");
	usb3 = ofw_bus_find_child(node, "usb3-port");
	if (usb3 == 0) {
		device_printf(dev, "Cannot find usb3-port child node\n");
		goto fail;
	}
	if (dp != 0 && ofw_bus_node_status_okay(dp)) {
		phy_init.id = RK3399_TYPEC_PHY_DP;
		phy_init.ofw_node = dp;
		phynode = phynode_create(dev, &rk_typec_phy_phynode_class,
		    &phy_init);
		if (phynode == NULL) {
			device_printf(dev, "failed to create phy dp-port\n");
			goto fail;
		}
		if (phynode_register(phynode) == NULL) {
			device_printf(dev, "failed to register phy dp-port\n");
			goto fail;
		}
		OF_device_register_xref(OF_xref_from_node(dp), dev);
	}
	/* If the child isn't enable attach the driver
	 *  but do not register the PHY. 
	 */
	if (!ofw_bus_node_status_okay(usb3))
		return (0);

	phy_init.id = RK3399_TYPEC_PHY_USB3;
	phy_init.ofw_node = usb3;
	phynode = phynode_create(dev, &rk_typec_phy_phynode_class, &phy_init);
	if (phynode == NULL) {
		device_printf(dev, "failed to create phy usb3-port\n");
		goto fail;
	}
	if (phynode_register(phynode) == NULL) {
		device_printf(dev, "failed to register phy usb3-port\n");
		goto fail;
	}

	OF_device_register_xref(OF_xref_from_node(usb3), dev);

	return (0);

fail:
	bus_release_resources(dev, rk_typec_phy_spec, &sc->res);

	return (ENXIO);
}

static device_method_t rk_typec_phy_methods[] = {
	/* Device interface */
	DEVMETHOD(device_probe,		rk_typec_phy_probe),
	DEVMETHOD(device_attach,	rk_typec_phy_attach),

	DEVMETHOD_END
};

static driver_t rk_typec_phy_driver = {
	"rk_typec_phy",
	rk_typec_phy_methods,
	sizeof(struct rk_typec_phy_softc)
};

EARLY_DRIVER_MODULE(rk_typec_phy, simplebus, rk_typec_phy_driver, 0, 0,
    BUS_PASS_SUPPORTDEV + BUS_PASS_ORDER_MIDDLE);
MODULE_VERSION(rk_typec_phy, 1);
