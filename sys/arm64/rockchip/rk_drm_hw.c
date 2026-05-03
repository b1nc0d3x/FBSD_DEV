/*-
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Copyright (c) 2026 Kyle Crenshaw
 * All rights reserved.
 */

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/bus.h>
#include <sys/errno.h>

#include <arm/include/fdt.h>

#include <machine/bus.h>
#include <machine/cpufunc.h>

#include <vm/vm.h>
#include <vm/pmap.h>

#include <dev/drm2/drmP.h>
#include <dev/drm2/drm_crtc.h>

#include "rk_drm.h"

#define HDMI_PHY_I2C_ADDR  0x69

#define RK_DRM_SYS_GRF_GPIO4C_IOMUX 0x0e028
#define RK_DRM_SYS_GRF_SOC_CON20    0x6250
#define RK_DRM_GRF_HDMI_LCDC_SEL    (1u << 6)
#define RK_DRM_GRF_GPIO4C_I2C3HDMI  0x003f0005u

#define RK_DRM_VOP_DSP_HTOTAL_HS_END 0x0188
#define RK_DRM_VOP_DSP_HACT_ST_END   0x018c
#define RK_DRM_VOP_DSP_VTOTAL_VS_END 0x0190
#define RK_DRM_VOP_DSP_VACT_ST_END   0x0194
#define RK_DRM_VOP_POST_DSP_HACT_INFO 0x0170
#define RK_DRM_VOP_POST_DSP_VACT_INFO 0x0174
#define RK_DRM_VOP_SYS_CTRL_STANDBY    (1u << 22)
#define RK_DRM_VOP_SYS_CTRL_MMU_EN     (1u << 20)
#define RK_DRM_VOP_SYS_CTRL_ENABLE     (1u << 11)
#define RK_DRM_VOP_SYS_CTRL_RGB_EN     (1u << 12)
#define RK_DRM_VOP_SYS_CTRL_HDMI_EN    (1u << 13)
#define RK_DRM_VOP_SYS_CTRL_EDP_EN     (1u << 14)
#define RK_DRM_VOP_SYS_CTRL_MIPI_EN    (1u << 15)
#define RK_DRM_VOP_SYS_CTRL_MIPI_DUAL  (1u << 3)
#define RK_DRM_VOP_DSP_OUT_MODE_MASK 0x0000000fu
#define RK_DRM_VOP_DSP_OUT_MODE_AAAA 0x0000000fu
#define RK_DRM_VOP_DSP_CTRL1_HDMI_PIN_POL_MASK  (0x7u << 20)
#define RK_DRM_VOP_DSP_CTRL1_HDMI_PIN_POL_POS   (0x3u << 20)
#define RK_DRM_VOP_DSP_CTRL1_HDMI_DCLK_POL      (1u << 23)
#define RK_DRM_VOP_WIN0_LB_MODE_RGB_1920X5 (4u << 5)
#define RK_DRM_VOP_WIN0_DATA_FMT_XRGB8888 0x00000000u
#define RK_DRM_VOP_WIN0_CTRL0_ENABLE  (RK_DRM_VOP_WIN0_LB_MODE_RGB_1920X5 | \
    RK_DRM_VOP_WIN0_DATA_FMT_XRGB8888 | 0x00000001u)
#define RK_DRM_VOP_WIN0_CTRL2_PRIMARY 0x00000021u

#define RK_DRM_FB_DMA_LOWADDR_TEST    0x0fffffffu

#define RK_DRM_CRU_VPLL_CON0         0x00c0
#define RK_DRM_CRU_VPLL_CON1         0x00c4
#define RK_DRM_CRU_VPLL_CON2         0x00c8
#define RK_DRM_CRU_VPLL_CON3         0x00cc
#define RK_DRM_CRU_CLKGATE_CON10     0x0328
#define RK_DRM_CRU_CLKGATE_CON28     0x0370
#define RK_DRM_CRU_SOFTRST_CON17     0x0444
#define RK_DRM_CRU_DRESETN_VOP0_REQ  (1u << 8)
#define RK_DRM_CRU_VPLL_CON2_LOCK    (1u << 31)
#define RK_DRM_CRU_PLL_MODE_SLOW     (0u << 8)
#define RK_DRM_CRU_PLL_MODE_NORMAL   (1u << 8)
#define RK_DRM_CRU_PLL_DSMPD         (1u << 3)
#define RK_DRM_CRU_PLL_BYPASS        (1u << 1)
#define RK_DRM_CRU_PLL_POWER_DOWN    (1u << 0)
#define RK_DRM_CRU_CLKGATE_VOP0_MASK \
	((1u << 12) | (1u << 9) | (1u << 8))
#define RK_DRM_CRU_CLKGATE_VOPB_MASK \
	((1u << 7) | (1u << 6) | (1u << 5) | (1u << 4) | \
	 (1u << 3) | (1u << 2) | (1u << 1) | (1u << 0))

#define RK_DRM_PMU_PWRDN_CON         0x0014
#define RK_DRM_PMU_PWRDN_ST          0x0018
#define RK_DRM_PMU_BUS_IDLE_REQ      0x0060
#define RK_DRM_PMU_PD_VO             (1u << 20)
#define RK_DRM_PMU_IDLE_VOPL         (1u << 8)
#define RK_DRM_PMU_IDLE_VOPB         (1u << 7)

#define RK_DRM_PMUCRU_GATEDIS_CON0   0x0130
#define RK_DRM_PMUCRU_GATEDIS_VOPB   (1u << 19)

#define RK_DRM_HDMI_TX_INVID0        0x0200
#define RK_DRM_HDMI_VP_PR_CD         0x0801
#define RK_DRM_HDMI_VP_STUFF         0x0802
#define RK_DRM_HDMI_VP_REMAP         0x0803
#define RK_DRM_HDMI_VP_CONF          0x0804
#define RK_DRM_HDMI_FC_INVIDCONF     0x1000
#define RK_DRM_HDMI_FC_INHACTV0      0x1001
#define RK_DRM_HDMI_FC_INHACTV1      0x1002
#define RK_DRM_HDMI_FC_INHBLANK0     0x1003
#define RK_DRM_HDMI_FC_INHBLANK1     0x1004
#define RK_DRM_HDMI_FC_INVACTV0      0x1005
#define RK_DRM_HDMI_FC_INVACTV1      0x1006
#define RK_DRM_HDMI_FC_INVBLANK      0x1007
#define RK_DRM_HDMI_FC_HSYNCINDELAY0 0x1008
#define RK_DRM_HDMI_FC_HSYNCINDELAY1 0x1009
#define RK_DRM_HDMI_FC_HSYNCINWIDTH0 0x100a
#define RK_DRM_HDMI_FC_HSYNCINWIDTH1 0x100b
#define RK_DRM_HDMI_FC_VSYNCINDELAY  0x100c
#define RK_DRM_HDMI_FC_VSYNCINWIDTH  0x100d
#define RK_DRM_HDMI_FC_CTRLDUR       0x1011
#define RK_DRM_HDMI_FC_EXCTRLDUR     0x1012
#define RK_DRM_HDMI_FC_EXCTRLSPAC    0x1013
#define RK_DRM_HDMI_FC_CH0PREAM      0x1014
#define RK_DRM_HDMI_FC_CH1PREAM      0x1015
#define RK_DRM_HDMI_FC_CH2PREAM      0x1016
#define RK_DRM_HDMI_FC_AVICONF3      0x1017
#define RK_DRM_HDMI_FC_GCP           0x1018
#define RK_DRM_HDMI_FC_AVICONF0      0x1019
#define RK_DRM_HDMI_FC_AVICONF1      0x101a
#define RK_DRM_HDMI_FC_AVICONF2      0x101b
#define RK_DRM_HDMI_FC_AVIVID        0x101c
#define RK_DRM_HDMI_FC_PACKET_TX_EN  0x10e3
#define RK_DRM_HDMI_IH_I2CMPHY_STAT0 0x0108
#define RK_DRM_HDMI_PHY_CONF0        0x3000
#define RK_DRM_HDMI_PHY_STAT0        0x3004
#define RK_DRM_HDMI_PHY_I2CM_SLAVE   0x3020
#define RK_DRM_HDMI_PHY_I2CM_ADDRESS 0x3021
#define RK_DRM_HDMI_PHY_I2CM_DATAO_1 0x3022
#define RK_DRM_HDMI_PHY_I2CM_DATAO_0 0x3023
#define RK_DRM_HDMI_PHY_I2CM_OPERATION 0x3026
#define RK_DRM_HDMI_PHY_I2CM_INT     0x3027
#define RK_DRM_HDMI_PHY_I2CM_CTLINT  0x3028
#define RK_DRM_HDMI_PHY_I2CM_DIV     0x3029
#define RK_DRM_HDMI_PHY_I2CM_SOFTRSTZ 0x302a
#define RK_DRM_HDMI_PHY_I2CM_SS_HCNT1 0x302b
#define RK_DRM_HDMI_PHY_I2CM_SS_HCNT0 0x302c
#define RK_DRM_HDMI_PHY_I2CM_SS_LCNT1 0x302d
#define RK_DRM_HDMI_PHY_I2CM_SS_LCNT0 0x302e
#define RK_DRM_HDMI_PHY_I2CM_FS_HCNT1 0x302f
#define RK_DRM_HDMI_PHY_I2CM_FS_HCNT0 0x3030
#define RK_DRM_HDMI_PHY_I2CM_FS_LCNT1 0x3031
#define RK_DRM_HDMI_PHY_I2CM_FS_LCNT0 0x3032
#define RK_DRM_HDMI_PHY_I2CM_SDA_HOLD 0x3033
#define RK_DRM_HDMI_PHY_JTAG_CFG     0x3034
#define RK_DRM_HDMI_MC_CLKDIS        0x4001
#define RK_DRM_HDMI_MC_SWRSTZREQ     0x4002
#define RK_DRM_HDMI_MC_FLOWCTRL      0x4004
#define RK_DRM_HDMI_MC_PHYRSTZ       0x4005
#define RK_DRM_HDMI_MC_LOCKONCLOCK   0x4006
#define RK_DRM_HDMI_MC_HEACPHY_RST   0x4007
#define RK_DRM_HDMI_BASE_SFRDIVLOW   0x4018
#define RK_DRM_HDMI_BASE_SFRDIVHIGH  0x4019
#define RK_DRM_HDMI_A_HDCPCFG0       0x5000
#define RK_DRM_HDMI_A_HDCPCFG1       0x5001
#define RK_DRM_HDMI_A_VIDPOLCFG      0x5009
#define RK_DRM_HDMI_PKT_SEND_CTL     0x0640

#define RK_DRM_HDMI_PHY_CONF0_PDZ          (1u << 7)
#define RK_DRM_HDMI_PHY_CONF0_ENTMDS       (1u << 6)
#define RK_DRM_HDMI_PHY_CONF0_SVSRET       (1u << 5)
#define RK_DRM_HDMI_PHY_CONF0_PDDQ         (1u << 4)
#define RK_DRM_HDMI_PHY_CONF0_TXPWRON      (1u << 3)
#define RK_DRM_HDMI_PHY_CONF0_ENHPDRXSENSE (1u << 2)
#define RK_DRM_HDMI_PHY_CONF0_SELDATAENPOL (1u << 1)
#define RK_DRM_HDMI_PHY_CONF0_SELDIPIF     (1u << 0)
#define RK_DRM_HDMI_PHY_I2CM_DIV_DEFAULT      0x0b
#define RK_DRM_HDMI_PHY_I2CM_SS_HCNT0_DEFAULT 0x6c
#define RK_DRM_HDMI_PHY_I2CM_SS_LCNT0_DEFAULT 0x7f
#define RK_DRM_HDMI_PHY_I2CM_FS_HCNT0_DEFAULT 0x11
#define RK_DRM_HDMI_PHY_I2CM_FS_LCNT0_DEFAULT 0x24
#define RK_DRM_HDMI_PHY_I2CM_SDA_HOLD_DEFAULT 0x09
#define RK_DRM_HDMI_PHY_JTAG_CFG_I2C 0x11
#define RK_DRM_HDMI_BASE_SFRDIVLOW_DEFAULT  0x93
#define RK_DRM_HDMI_BASE_SFRDIVHIGH_DEFAULT 0x69
#define RK_DRM_HDMI_MC_SWRST_TMDS    (1u << 1)
#define RK_DRM_HDMI_MC_SWRST_PIXEL   (1u << 0)
#define RK_DRM_HDMI_MC_CLKDIS_CECCLK_DISABLE  (1u << 5)
#define RK_DRM_HDMI_FC_INVIDCONF_VSYNC_HIGH   0x40
#define RK_DRM_HDMI_FC_INVIDCONF_HSYNC_HIGH   0x20
#define RK_DRM_HDMI_FC_INVIDCONF_DE_HIGH      0x10
#define RK_DRM_HDMI_FC_INVIDCONF_HDMI_MODE    0x08
#define RK_DRM_HDMI_FC_INVIDCONF_R_V_BLANK_HIGH 0x02
#define RK_DRM_HDMI_FC_INVIDCONF_INTERLACED   0x01
#define RK_DRM_HDMI_FC_AVICONF1_PICTURE_ASPECT_4_3  (1u << 4)
#define RK_DRM_HDMI_FC_AVICONF1_PICTURE_ASPECT_16_9 (2u << 4)
#define RK_DRM_HDMI_FC_PACKET_TX_EN_AVI (1u << 2)
#define RK_DRM_HDMI_FC_PACKET_TX_EN_GCP (1u << 1)
#define RK_DRM_HDMI_A_HDCPCFG0_HDMIDVI (1u << 0)
#define RK_DRM_HDMI_A_HDCPCFG1_SWRESETN          (1u << 0)
#define RK_DRM_HDMI_A_HDCPCFG1_ENCRYPTIONDISABLE (1u << 1)
#define RK_DRM_HDMI_A_HDCPCFG1_PH2UPSHFTENC      (1u << 2)
#define RK_DRM_HDMI_A_HDCPCFG1_DEFAULT \
	(RK_DRM_HDMI_A_HDCPCFG1_SWRESETN | \
	 RK_DRM_HDMI_A_HDCPCFG1_ENCRYPTIONDISABLE | \
	 RK_DRM_HDMI_A_HDCPCFG1_PH2UPSHFTENC)
#define RK_DRM_HDMI_A_VIDPOLCFG_DATAENPOL (1u << 4)
#define RK_DRM_HDMI_PKT_SEND_CTL_AVI_INFO_UP (1u << 6)
#define RK_DRM_HDMI_PKT_SEND_CTL_AVI_INFO_EN (1u << 2)

#define RK_DRM_HDMI_PHY_I2C_CKCALCTRL        0x05
#define RK_DRM_HDMI_PHY_I2C_CPCE_CTRL        0x06
#define RK_DRM_HDMI_PHY_I2C_CKSYMTXCTRL      0x09
#define RK_DRM_HDMI_PHY_I2C_VLEVCTRL         0x0e
#define RK_DRM_HDMI_PHY_I2C_CURRCTRL         0x10
#define RK_DRM_HDMI_PHY_I2C_PLLPHBYCTRL      0x13
#define RK_DRM_HDMI_PHY_I2C_GMPCTRL          0x15
#define RK_DRM_HDMI_PHY_I2C_MSM_CTRL         0x17
#define RK_DRM_HDMI_PHY_I2C_TXTERM           0x19
#define RK_DRM_HDMI_PHY_I2C_CKCALCTRL_OVERRIDE 0x8000
#define RK_DRM_HDMI_PHY_MSM_CTRL_FB_CLK      0x0006

struct rk_drm_pll_rate {
	uint32_t	clock_khz;
	uint16_t	refdiv;
	uint16_t	fbdiv;
	uint16_t	postdiv1;
	uint16_t	postdiv2;
};

struct rk_drm_mpll_config {
	uint32_t	pixel_clock;
	uint16_t	cpce;
	uint16_t	gmp;
	uint16_t	curr;
};

struct rk_drm_phy_config {
	uint32_t	pixel_clock;
	uint16_t	sym;
	uint16_t	term;
	uint16_t	vlev;
};

/*
 * RK3399 can synthesize more clocks than the initial bring-up set. Keep the
 * table explicit and use a small tolerance so standard DMT clocks like
 * 25.175 MHz and 81.62 MHz can map to nearby integer-mode VPLL settings
 * without opening the door to arbitrary clocks.
 */
#define RK_DRM_PLL_TOLERANCE_KHZ 250

static const struct rk_drm_pll_rate rk_drm_pll_rates[] = {
	{  25200, 5, 21, 4, 1 },
	{  27000, 1, 27, 6, 4 },
	{  40000, 3, 20, 4, 1 },
	{  54000, 1, 54, 6, 4 },
	{  65000, 1, 65, 6, 4 },
	{  74250, 2, 99, 4, 4 },
	{  81600, 5, 68, 4, 1 },
	{  96000, 1, 64, 4, 4 },
	{ 106500, 1, 71, 4, 4 },
	{ 108000, 3, 54, 4, 1 },
	{ 119000, 6, 119, 4, 1 },
	{ 148500, 4, 99, 4, 1 },
};

static const struct rk_drm_mpll_config rk_drm_mpll_configs[] = {
	{  40000, 0x00b3, 0x0000, 0x0018 },
	{  65000, 0x0072, 0x0001, 0x0028 },
	{  66000, 0x013e, 0x0003, 0x0038 },
	{  83500, 0x0072, 0x0001, 0x0028 },
	{ 146250, 0x0051, 0x0002, 0x0038 },
	{ 148500, 0x0051, 0x0003, 0x0000 },
	{ 0,      0x0051, 0x0003, 0x0000 },
};

static const struct rk_drm_phy_config rk_drm_phy_configs[] = {
	{  74250, 0x8009, 0x0004, 0x0272 },
	{ 148500, 0x802b, 0x0004, 0x028d },
	{ 0,      0x0000, 0x0000, 0x0000 },
};

static inline uint32_t
rk_drm_vop_read4(struct rk_drm_softc *sc, size_t off)
{
	return (bus_space_read_4(fdtbus_bs_tag, sc->vop_bsh, off));
}

static inline void
rk_drm_vop_write4(struct rk_drm_softc *sc, size_t off, uint32_t val)
{
	bus_space_write_4(fdtbus_bs_tag, sc->vop_bsh, off, val);
	bus_space_barrier(fdtbus_bs_tag, sc->vop_bsh, off, 4,
	    BUS_SPACE_BARRIER_WRITE);
}

static inline void
rk_drm_grf_write4(struct rk_drm_softc *sc, size_t off, uint32_t val)
{
	bus_space_write_4(fdtbus_bs_tag, sc->grf_bsh, off, val);
	bus_space_barrier(fdtbus_bs_tag, sc->grf_bsh, off, 4,
	    BUS_SPACE_BARRIER_WRITE);
}

static inline uint32_t
rk_drm_pmu_read4(struct rk_drm_softc *sc, size_t off)
{
	return (bus_space_read_4(fdtbus_bs_tag, sc->pmu_bsh, off));
}

static inline void
rk_drm_pmu_write4(struct rk_drm_softc *sc, size_t off, uint32_t val)
{
	bus_space_write_4(fdtbus_bs_tag, sc->pmu_bsh, off, val);
	bus_space_barrier(fdtbus_bs_tag, sc->pmu_bsh, off, 4,
	    BUS_SPACE_BARRIER_WRITE);
}

static inline uint32_t
rk_drm_pmucru_read4(struct rk_drm_softc *sc, size_t off)
{
	return (bus_space_read_4(fdtbus_bs_tag, sc->pmucru_bsh, off));
}

static inline void
rk_drm_pmucru_write4(struct rk_drm_softc *sc, size_t off, uint32_t val)
{
	bus_space_write_4(fdtbus_bs_tag, sc->pmucru_bsh, off, val);
	bus_space_barrier(fdtbus_bs_tag, sc->pmucru_bsh, off, 4,
	    BUS_SPACE_BARRIER_WRITE);
}

static inline uint32_t
rk_drm_cru_read4(struct rk_drm_softc *sc, size_t off)
{
	return (bus_space_read_4(fdtbus_bs_tag, sc->cru_bsh, off));
}

static inline void
rk_drm_cru_write4(struct rk_drm_softc *sc, size_t off, uint32_t val)
{
	bus_space_write_4(fdtbus_bs_tag, sc->cru_bsh, off, val);
	bus_space_barrier(fdtbus_bs_tag, sc->cru_bsh, off, 4,
	    BUS_SPACE_BARRIER_WRITE);
}

static inline uint8_t
rk_drm_hdmi_read1(struct rk_drm_softc *sc, size_t off)
{
	volatile uint32_t *reg;

	reg = (volatile uint32_t *)(sc->hdmi_va + (off << 2));
	return ((uint8_t)(*reg & 0xff));
}

static inline void
rk_drm_hdmi_write1(struct rk_drm_softc *sc, size_t off, uint8_t val)
{
	volatile uint32_t *reg;

	reg = (volatile uint32_t *)(sc->hdmi_va + (off << 2));
	*reg = val;
	__asm volatile("dsb sy" ::: "memory");
	__asm volatile("isb" ::: "memory");
}

static inline void
rk_drm_hdmi_write1_safe(struct rk_drm_softc *sc, size_t off, uint8_t val)
{
	uint64_t daif;

	__asm volatile("mrs %0, daif" : "=r"(daif));
	__asm volatile("msr daifset, #4");
	rk_drm_hdmi_write1(sc, off, val);
	__asm volatile("msr daif, %0" :: "r"(daif));
}

static void
rk_drm_default_mode_fill(struct drm_display_mode *mode)
{
	memset(mode, 0, sizeof(*mode));
	mode->clock = RK_DRM_DEFAULT_CLOCK_KHZ;
	mode->hdisplay = RK_DRM_DEFAULT_WIDTH;
	mode->hsync_start = RK_DRM_DEFAULT_HSYNC_START;
	mode->hsync_end = RK_DRM_DEFAULT_HSYNC_END;
	mode->htotal = RK_DRM_DEFAULT_HTOTAL;
	mode->vdisplay = RK_DRM_DEFAULT_HEIGHT;
	mode->vsync_start = RK_DRM_DEFAULT_VSYNC_START;
	mode->vsync_end = RK_DRM_DEFAULT_VSYNC_END;
	mode->vtotal = RK_DRM_DEFAULT_VTOTAL;
	mode->flags = DRM_MODE_FLAG_PHSYNC | DRM_MODE_FLAG_PVSYNC;
}

static const struct rk_drm_pll_rate *
rk_drm_find_pll_rate(uint32_t clock_khz)
{
	size_t i;
	const struct rk_drm_pll_rate *best;
	uint32_t best_delta;

	best = NULL;
	best_delta = UINT32_MAX;
	for (i = 0; i < nitems(rk_drm_pll_rates); i++) {
		uint32_t table_clock;
		uint32_t delta;

		table_clock = rk_drm_pll_rates[i].clock_khz;
		delta = (table_clock > clock_khz) ?
		    (table_clock - clock_khz) : (clock_khz - table_clock);
		if (delta < best_delta) {
			best = &rk_drm_pll_rates[i];
			best_delta = delta;
			if (delta == 0)
				break;
		}
	}
	if (best != NULL && best_delta <= RK_DRM_PLL_TOLERANCE_KHZ)
		return (best);
	return (NULL);
}

static const struct rk_drm_mpll_config *
rk_drm_find_mpll_config(uint32_t clock_khz)
{
	size_t i;

	for (i = 0; rk_drm_mpll_configs[i].pixel_clock != 0; i++) {
		if (clock_khz <= rk_drm_mpll_configs[i].pixel_clock)
			return (&rk_drm_mpll_configs[i]);
	}
	return (NULL);
}

static const struct rk_drm_phy_config *
rk_drm_find_phy_config(uint32_t clock_khz)
{
	size_t i;

	for (i = 0; rk_drm_phy_configs[i].pixel_clock != 0; i++) {
		if (clock_khz <= rk_drm_phy_configs[i].pixel_clock)
			return (&rk_drm_phy_configs[i]);
	}
	return (NULL);
}

static inline uint16_t
rk_drm_mode_hsync_len(const struct drm_display_mode *mode)
{
	return ((uint16_t)(mode->hsync_end - mode->hsync_start));
}

static inline uint16_t
rk_drm_mode_vsync_len(const struct drm_display_mode *mode)
{
	return ((uint16_t)(mode->vsync_end - mode->vsync_start));
}

static inline uint16_t
rk_drm_mode_hfront_porch(const struct drm_display_mode *mode)
{
	return ((uint16_t)(mode->hsync_start - mode->hdisplay));
}

static inline uint16_t
rk_drm_mode_vfront_porch(const struct drm_display_mode *mode)
{
	return ((uint16_t)(mode->vsync_start - mode->vdisplay));
}

static inline uint16_t
rk_drm_mode_hback_porch(const struct drm_display_mode *mode)
{
	return ((uint16_t)(mode->htotal - mode->hsync_end));
}

static inline uint16_t
rk_drm_mode_vback_porch(const struct drm_display_mode *mode)
{
	return ((uint16_t)(mode->vtotal - mode->vsync_end));
}

static inline uint16_t
rk_drm_mode_hblank(const struct drm_display_mode *mode)
{
	return ((uint16_t)(mode->htotal - mode->hdisplay));
}

static inline uint16_t
rk_drm_mode_vblank(const struct drm_display_mode *mode)
{
	return ((uint16_t)(mode->vtotal - mode->vdisplay));
}

static inline uint16_t
rk_drm_mode_hact_start(const struct drm_display_mode *mode)
{
	return ((uint16_t)(rk_drm_mode_hsync_len(mode) +
	    rk_drm_mode_hback_porch(mode)));
}

static inline uint16_t
rk_drm_mode_vact_start(const struct drm_display_mode *mode)
{
	return ((uint16_t)(rk_drm_mode_vsync_len(mode) +
	    rk_drm_mode_vback_porch(mode)));
}

bool
rk_drm_hw_mode_valid(const struct drm_display_mode *mode)
{
	if (mode == NULL)
		return (false);
	if (mode->clock == 0)
		return (false);
	if ((mode->flags & (DRM_MODE_FLAG_INTERLACE |
	    DRM_MODE_FLAG_DBLSCAN)) != 0)
		return (false);
	if (mode->hdisplay <= 0 || mode->vdisplay <= 0)
		return (false);
	if (mode->hdisplay > RK_DRM_MAX_WIDTH ||
	    mode->vdisplay > RK_DRM_MAX_HEIGHT)
		return (false);
	if (rk_drm_find_pll_rate(mode->clock) == NULL)
		return (false);
	if (rk_drm_find_mpll_config(mode->clock) == NULL)
		return (false);
	if (rk_drm_find_phy_config(mode->clock) == NULL)
		return (false);
	return (true);
}

static void
rk_drm_fb_dma_cb(void *arg, bus_dma_segment_t *segs, int nseg, int error)
{
	bus_addr_t *fb_busaddr;

	fb_busaddr = arg;
	if (error != 0 || nseg != 1)
		return;
	*fb_busaddr = segs[0].ds_addr;
}

static int
rk_drm_fb_alloc(struct rk_drm_softc *sc)
{
	const size_t alloc_size = round_page(sc->fb_size);
	bus_addr_t fb_busaddr;
	void *fb_kva;
	int error;

	error = bus_dma_tag_create(NULL, PAGE_SIZE, 0,
	    RK_DRM_FB_DMA_LOWADDR_TEST, BUS_SPACE_MAXADDR,
	    NULL, NULL, alloc_size, 1, alloc_size, 0,
	    NULL, NULL, &sc->fb_dma_tag);
	if (error != 0)
		return (error);

	error = bus_dmamem_alloc(sc->fb_dma_tag, &fb_kva,
	    BUS_DMA_WAITOK | BUS_DMA_COHERENT | BUS_DMA_ZERO,
	    &sc->fb_dma_map);
	if (error != 0) {
		bus_dma_tag_destroy(sc->fb_dma_tag);
		sc->fb_dma_tag = NULL;
		return (error);
	}

	fb_busaddr = 0;
	error = bus_dmamap_load(sc->fb_dma_tag, sc->fb_dma_map, fb_kva,
	    alloc_size, rk_drm_fb_dma_cb, &fb_busaddr, BUS_DMA_WAITOK);
	if (error != 0 || fb_busaddr == 0) {
		bus_dmamem_free(sc->fb_dma_tag, fb_kva, sc->fb_dma_map);
		bus_dma_tag_destroy(sc->fb_dma_tag);
		sc->fb_dma_map = NULL;
		sc->fb_dma_tag = NULL;
		return (error != 0 ? error : ENXIO);
	}

	sc->fb_va = (vm_offset_t)fb_kva;
	sc->fb_pa = (vm_paddr_t)fb_busaddr;
	return (0);
}

static void
rk_drm_fb_free(struct rk_drm_softc *sc)
{
	if (sc->fb_dma_tag != NULL) {
		if (sc->fb_dma_map != NULL && sc->fb_va != 0)
			bus_dmamap_unload(sc->fb_dma_tag, sc->fb_dma_map);
		if (sc->fb_va != 0)
			bus_dmamem_free(sc->fb_dma_tag, (void *)sc->fb_va,
			    sc->fb_dma_map);
		bus_dma_tag_destroy(sc->fb_dma_tag);
	}
	sc->fb_dma_map = NULL;
	sc->fb_dma_tag = NULL;
	sc->fb_va = 0;
	sc->fb_pa = 0;
}

static void
rk_drm_fb_fill(struct rk_drm_softc *sc, uint32_t color)
{
	uint32_t *fb32;
	size_t i, words;

	fb32 = (uint32_t *)sc->fb_va;
	words = sc->fb_size / sizeof(uint32_t);
	for (i = 0; i < words; i++)
		fb32[i] = color;

	if (sc->fb_dma_tag != NULL && sc->fb_dma_map != NULL)
		bus_dmamap_sync(sc->fb_dma_tag, sc->fb_dma_map,
		    BUS_DMASYNC_PREWRITE);
}

static void
rk_drm_display_domain_sanity(struct rk_drm_softc *sc)
{
	uint32_t pwrdn_con, pwrdn_st, idle_req, gatedis0;
	int i;

	pwrdn_con = rk_drm_pmu_read4(sc, RK_DRM_PMU_PWRDN_CON);
	pwrdn_st = rk_drm_pmu_read4(sc, RK_DRM_PMU_PWRDN_ST);
	idle_req = rk_drm_pmu_read4(sc, RK_DRM_PMU_BUS_IDLE_REQ);
	gatedis0 = rk_drm_pmucru_read4(sc, RK_DRM_PMUCRU_GATEDIS_CON0);

	if ((pwrdn_st & RK_DRM_PMU_PD_VO) != 0) {
		rk_drm_pmu_write4(sc, RK_DRM_PMU_PWRDN_CON,
		    pwrdn_con & ~RK_DRM_PMU_PD_VO);
		for (i = 0; i < 1000; i++) {
			pwrdn_st = rk_drm_pmu_read4(sc, RK_DRM_PMU_PWRDN_ST);
			if ((pwrdn_st & RK_DRM_PMU_PD_VO) == 0)
				break;
			DELAY(10);
		}
	}

	if ((idle_req & (RK_DRM_PMU_IDLE_VOPB | RK_DRM_PMU_IDLE_VOPL)) != 0)
		rk_drm_pmu_write4(sc, RK_DRM_PMU_BUS_IDLE_REQ,
		    idle_req & ~(RK_DRM_PMU_IDLE_VOPB | RK_DRM_PMU_IDLE_VOPL));

	if ((gatedis0 & RK_DRM_PMUCRU_GATEDIS_VOPB) == 0)
		rk_drm_pmucru_write4(sc, RK_DRM_PMUCRU_GATEDIS_CON0,
		    gatedis0 | RK_DRM_PMUCRU_GATEDIS_VOPB);

	rk_drm_cru_write4(sc, RK_DRM_CRU_CLKGATE_CON10,
	    (RK_DRM_CRU_CLKGATE_VOP0_MASK << 16));
	rk_drm_cru_write4(sc, RK_DRM_CRU_CLKGATE_CON28,
	    (RK_DRM_CRU_CLKGATE_VOPB_MASK << 16));
}

static void
rk_drm_route_vop_to_hdmi(struct rk_drm_softc *sc)
{
	rk_drm_grf_write4(sc, RK_DRM_SYS_GRF_SOC_CON20,
	    (RK_DRM_GRF_HDMI_LCDC_SEL << 16));
	rk_drm_grf_write4(sc, RK_DRM_SYS_GRF_GPIO4C_IOMUX,
	    RK_DRM_GRF_GPIO4C_I2C3HDMI);
}

static int
rk_drm_program_vpll(struct rk_drm_softc *sc, uint32_t clock_khz)
{
	const struct rk_drm_pll_rate *rate;
	const uint32_t con3_mask = (0x3u << 8) | RK_DRM_CRU_PLL_DSMPD |
	    RK_DRM_CRU_PLL_BYPASS | RK_DRM_CRU_PLL_POWER_DOWN;
	const uint32_t con1_mask = (0x7u << 12) | (0x7u << 8) | 0x3fu;
	uint32_t con3;
	int i;

	rate = rk_drm_find_pll_rate(clock_khz);
	if (rate == NULL)
		return (EINVAL);

	con3 = RK_DRM_CRU_PLL_MODE_SLOW | RK_DRM_CRU_PLL_DSMPD |
	    RK_DRM_CRU_PLL_POWER_DOWN;
	rk_drm_cru_write4(sc, RK_DRM_CRU_VPLL_CON3, (con3_mask << 16) | con3);
	DELAY(2);

	rk_drm_cru_write4(sc, RK_DRM_CRU_VPLL_CON0,
	    (0x0fffu << 16) | rate->fbdiv);
	rk_drm_cru_write4(sc, RK_DRM_CRU_VPLL_CON1,
	    (con1_mask << 16) |
	    (rate->postdiv2 << 12) |
	    (rate->postdiv1 << 8) |
	    rate->refdiv);
	rk_drm_cru_write4(sc, RK_DRM_CRU_VPLL_CON2, 0x00000000);

	con3 = RK_DRM_CRU_PLL_MODE_SLOW | RK_DRM_CRU_PLL_DSMPD;
	rk_drm_cru_write4(sc, RK_DRM_CRU_VPLL_CON3, (con3_mask << 16) | con3);

	for (i = 0; i < 5000; i++) {
		if ((rk_drm_cru_read4(sc, RK_DRM_CRU_VPLL_CON2) &
		    RK_DRM_CRU_VPLL_CON2_LOCK) != 0)
			break;
		DELAY(10);
	}
	if (i == 5000)
		return (ETIMEDOUT);

	con3 = RK_DRM_CRU_PLL_MODE_NORMAL | RK_DRM_CRU_PLL_DSMPD;
	rk_drm_cru_write4(sc, RK_DRM_CRU_VPLL_CON3, (con3_mask << 16) | con3);
	return (0);
}

static void
rk_drm_vop_pulse_dclk_reset(struct rk_drm_softc *sc)
{
	rk_drm_cru_write4(sc, RK_DRM_CRU_SOFTRST_CON17,
	    (RK_DRM_CRU_DRESETN_VOP0_REQ << 16) | RK_DRM_CRU_DRESETN_VOP0_REQ);
	DELAY(1000);
	rk_drm_cru_write4(sc, RK_DRM_CRU_SOFTRST_CON17,
	    (RK_DRM_CRU_DRESETN_VOP0_REQ << 16));
	DELAY(1000);
}

static void
rk_drm_vop_init_mode(struct rk_drm_softc *sc,
    const struct drm_display_mode *mode)
{
	uint32_t hact_start, vact_start;
	uint32_t sys_ctrl, dsp_ctrl0, dsp_ctrl1;

	hact_start = rk_drm_mode_hact_start(mode);
	vact_start = rk_drm_mode_vact_start(mode);

	if (rk_drm_program_vpll(sc, mode->clock) != 0)
		device_printf(sc->dev, "VPLL setup failed, continuing\n");

	rk_drm_cru_write4(sc, 0x01bc,
	    ((((0x1fu << 8) | (0x3u << 6) | 0x1fu) << 16) |
	    ((3u << 8) | (1u << 6) | 1u)));
	rk_drm_cru_write4(sc, 0x01c4,
	    ((((1u << 11) | (0x3u << 8) | 0xffu) << 16) | 0x0000u));

	sys_ctrl = rk_drm_vop_read4(sc, 0x0008);
	dsp_ctrl0 = rk_drm_vop_read4(sc, 0x0010);
	dsp_ctrl1 = rk_drm_vop_read4(sc, 0x0014);

	sys_ctrl &= ~(RK_DRM_VOP_SYS_CTRL_STANDBY |
	    RK_DRM_VOP_SYS_CTRL_MMU_EN |
	    RK_DRM_VOP_SYS_CTRL_EDP_EN |
	    RK_DRM_VOP_SYS_CTRL_MIPI_EN |
	    RK_DRM_VOP_SYS_CTRL_MIPI_DUAL);
	sys_ctrl |= RK_DRM_VOP_SYS_CTRL_ENABLE |
	    RK_DRM_VOP_SYS_CTRL_RGB_EN |
	    RK_DRM_VOP_SYS_CTRL_HDMI_EN;
	rk_drm_vop_write4(sc, 0x0008, sys_ctrl);

	dsp_ctrl0 &= ~RK_DRM_VOP_DSP_OUT_MODE_MASK;
	dsp_ctrl0 |= RK_DRM_VOP_DSP_OUT_MODE_AAAA;
	rk_drm_vop_write4(sc, 0x0010, dsp_ctrl0);

	dsp_ctrl1 &= ~(RK_DRM_VOP_DSP_CTRL1_HDMI_PIN_POL_MASK |
	    RK_DRM_VOP_DSP_CTRL1_HDMI_DCLK_POL);
	dsp_ctrl1 |= RK_DRM_VOP_DSP_CTRL1_HDMI_PIN_POL_POS |
	    RK_DRM_VOP_DSP_CTRL1_HDMI_DCLK_POL;
	rk_drm_vop_write4(sc, 0x0014, dsp_ctrl1);

	rk_drm_vop_write4(sc, 0x0038, 0x00000000);
	rk_drm_vop_write4(sc, 0x003c, sc->stride / 4);
	rk_drm_vop_write4(sc, 0x0040, (uint32_t)sc->fb_pa);
	rk_drm_vop_write4(sc, 0x0048,
	    (((uint32_t)mode->vdisplay - 1) << 16) |
	    ((uint32_t)mode->hdisplay - 1));
	rk_drm_vop_write4(sc, 0x004c,
	    (((uint32_t)mode->vdisplay - 1) << 16) |
	    ((uint32_t)mode->hdisplay - 1));
	rk_drm_vop_write4(sc, 0x0050,
	    (vact_start << 16) | hact_start);
	rk_drm_vop_write4(sc, 0x006c, RK_DRM_VOP_WIN0_CTRL2_PRIMARY);
	rk_drm_vop_write4(sc, RK_DRM_VOP_POST_DSP_HACT_INFO,
	    (hact_start << 16) | (hact_start + mode->hdisplay));
	rk_drm_vop_write4(sc, RK_DRM_VOP_POST_DSP_VACT_INFO,
	    (vact_start << 16) | (vact_start + mode->vdisplay));
	rk_drm_vop_write4(sc, 0x0030, RK_DRM_VOP_WIN0_CTRL0_ENABLE);
	rk_drm_vop_write4(sc, RK_DRM_VOP_DSP_HTOTAL_HS_END,
	    ((uint32_t)mode->htotal << 16) | rk_drm_mode_hsync_len(mode));
	rk_drm_vop_write4(sc, RK_DRM_VOP_DSP_HACT_ST_END,
	    (hact_start << 16) | (hact_start + mode->hdisplay));
	rk_drm_vop_write4(sc, RK_DRM_VOP_DSP_VTOTAL_VS_END,
	    ((uint32_t)mode->vtotal << 16) | rk_drm_mode_vsync_len(mode));
	rk_drm_vop_write4(sc, RK_DRM_VOP_DSP_VACT_ST_END,
	    (vact_start << 16) | (vact_start + mode->vdisplay));
	rk_drm_vop_write4(sc, 0x0000, 0x00000001);
	rk_drm_vop_pulse_dclk_reset(sc);
	DELAY(40000);
}

int
rk_drm_hw_set_scanout(struct rk_drm_softc *sc, vm_paddr_t paddr, uint32_t stride)
{
	if (!sc->hw_attached)
		return (ENXIO);
	if (paddr == 0 || stride == 0)
		return (EINVAL);

	rk_drm_vop_write4(sc, 0x003c, stride / 4);
	rk_drm_vop_write4(sc, 0x0040, (uint32_t)paddr);
	rk_drm_vop_write4(sc, 0x0000, 0x00000001);
	return (0);
}

void
rk_drm_hw_disable(struct rk_drm_softc *sc)
{
	uint32_t sys_ctrl;

	if (!sc->hw_attached)
		return;

	rk_drm_vop_write4(sc, 0x0030, 0x00000000);
	sys_ctrl = rk_drm_vop_read4(sc, 0x0008);
	sys_ctrl &= ~(RK_DRM_VOP_SYS_CTRL_ENABLE |
	    RK_DRM_VOP_SYS_CTRL_RGB_EN |
	    RK_DRM_VOP_SYS_CTRL_HDMI_EN);
	sys_ctrl |= RK_DRM_VOP_SYS_CTRL_STANDBY;
	rk_drm_vop_write4(sc, 0x0008, sys_ctrl);
	rk_drm_vop_write4(sc, 0x0000, 0x00000001);

	/*
	 * Blank TMDS output but keep HPD sense alive so native hotplug polling
	 * still works while the pipe is idle.
	 */
	rk_drm_hdmi_write1(sc, RK_DRM_HDMI_FC_PACKET_TX_EN, 0x00);
	rk_drm_hdmi_write1(sc, RK_DRM_HDMI_PKT_SEND_CTL, 0x00);
	rk_drm_hdmi_write1(sc, RK_DRM_HDMI_PHY_CONF0,
	    RK_DRM_HDMI_PHY_CONF0_PDDQ |
	    RK_DRM_HDMI_PHY_CONF0_ENHPDRXSENSE);
	sc->output_enabled = false;
}

static void
rk_drm_hdmi_toggle_main_reset(struct rk_drm_softc *sc, uint8_t mask)
{
	uint8_t reg;

	reg = rk_drm_hdmi_read1(sc, RK_DRM_HDMI_MC_SWRSTZREQ);
	rk_drm_hdmi_write1(sc, RK_DRM_HDMI_MC_SWRSTZREQ, reg & ~mask);
	DELAY(10);
	rk_drm_hdmi_write1(sc, RK_DRM_HDMI_MC_SWRSTZREQ, reg | mask);
	DELAY(10);
}

static void
rk_drm_hdmi_clear_overflow(struct rk_drm_softc *sc)
{
	uint8_t swrstz;
	uint8_t invidconf;
	int i;

	swrstz = rk_drm_hdmi_read1(sc, RK_DRM_HDMI_MC_SWRSTZREQ);
	rk_drm_hdmi_write1(sc, RK_DRM_HDMI_MC_SWRSTZREQ,
	    swrstz & ~RK_DRM_HDMI_MC_SWRST_TMDS);
	invidconf = rk_drm_hdmi_read1(sc, RK_DRM_HDMI_FC_INVIDCONF);
	for (i = 0; i < 4; i++)
		rk_drm_hdmi_write1(sc, RK_DRM_HDMI_FC_INVIDCONF, invidconf);
	rk_drm_hdmi_write1(sc, RK_DRM_HDMI_MC_SWRSTZREQ,
	    swrstz | RK_DRM_HDMI_MC_SWRST_TMDS);
}

static void
rk_drm_hdmi_program_av_composer(struct rk_drm_softc *sc,
    const struct drm_display_mode *mode, bool hdmi_mode)
{
	uint8_t inv_val;
	uint16_t vic;

	inv_val = RK_DRM_HDMI_FC_INVIDCONF_DE_HIGH;
	if ((mode->flags & DRM_MODE_FLAG_PVSYNC) != 0)
		inv_val |= RK_DRM_HDMI_FC_INVIDCONF_VSYNC_HIGH;
	if ((mode->flags & DRM_MODE_FLAG_PHSYNC) != 0)
		inv_val |= RK_DRM_HDMI_FC_INVIDCONF_HSYNC_HIGH;
	if ((mode->flags & DRM_MODE_FLAG_INTERLACE) != 0)
		inv_val |= RK_DRM_HDMI_FC_INVIDCONF_R_V_BLANK_HIGH |
		    RK_DRM_HDMI_FC_INVIDCONF_INTERLACED;
	if (hdmi_mode)
		inv_val |= RK_DRM_HDMI_FC_INVIDCONF_HDMI_MODE;

	rk_drm_hdmi_write1(sc, RK_DRM_HDMI_FC_INVIDCONF, inv_val);
	rk_drm_hdmi_write1(sc, RK_DRM_HDMI_FC_INHACTV1, mode->hdisplay >> 8);
	rk_drm_hdmi_write1(sc, RK_DRM_HDMI_FC_INHACTV0, mode->hdisplay & 0xff);
	rk_drm_hdmi_write1(sc, RK_DRM_HDMI_FC_INVACTV1, mode->vdisplay >> 8);
	rk_drm_hdmi_write1(sc, RK_DRM_HDMI_FC_INVACTV0, mode->vdisplay & 0xff);
	rk_drm_hdmi_write1(sc, RK_DRM_HDMI_FC_INHBLANK1,
	    rk_drm_mode_hblank(mode) >> 8);
	rk_drm_hdmi_write1(sc, RK_DRM_HDMI_FC_INHBLANK0,
	    rk_drm_mode_hblank(mode) & 0xff);
	rk_drm_hdmi_write1(sc, RK_DRM_HDMI_FC_INVBLANK,
	    rk_drm_mode_vblank(mode) & 0xff);
	rk_drm_hdmi_write1(sc, RK_DRM_HDMI_FC_HSYNCINDELAY1,
	    rk_drm_mode_hfront_porch(mode) >> 8);
	rk_drm_hdmi_write1(sc, RK_DRM_HDMI_FC_HSYNCINDELAY0,
	    rk_drm_mode_hfront_porch(mode) & 0xff);
	rk_drm_hdmi_write1(sc, RK_DRM_HDMI_FC_VSYNCINDELAY,
	    rk_drm_mode_vfront_porch(mode) & 0xff);
	rk_drm_hdmi_write1(sc, RK_DRM_HDMI_FC_HSYNCINWIDTH1,
	    rk_drm_mode_hsync_len(mode) >> 8);
	rk_drm_hdmi_write1(sc, RK_DRM_HDMI_FC_HSYNCINWIDTH0,
	    rk_drm_mode_hsync_len(mode) & 0xff);
	rk_drm_hdmi_write1(sc, RK_DRM_HDMI_FC_VSYNCINWIDTH,
	    rk_drm_mode_vsync_len(mode) & 0xff);

	vic = drm_mode_cea_vic(mode);
	rk_drm_hdmi_write1(sc, RK_DRM_HDMI_FC_AVIVID, vic & 0xff);
}

static uint8_t
rk_drm_hdmi_picture_aspect(const struct drm_display_mode *mode)
{
	if ((mode->hdisplay * 9) == (mode->vdisplay * 16))
		return (RK_DRM_HDMI_FC_AVICONF1_PICTURE_ASPECT_16_9);
	if ((mode->hdisplay * 3) == (mode->vdisplay * 4))
		return (RK_DRM_HDMI_FC_AVICONF1_PICTURE_ASPECT_4_3);
	return (0);
}

static void
rk_drm_hdmi_enable_dvi_mode(struct rk_drm_softc *sc,
    const struct drm_display_mode *mode)
{
	uint8_t hdcpcfg0;

	rk_drm_hdmi_program_av_composer(sc, mode, false);
	hdcpcfg0 = rk_drm_hdmi_read1(sc, RK_DRM_HDMI_A_HDCPCFG0);
	hdcpcfg0 &= ~RK_DRM_HDMI_A_HDCPCFG0_HDMIDVI;
	rk_drm_hdmi_write1(sc, RK_DRM_HDMI_A_HDCPCFG0, hdcpcfg0);
	rk_drm_hdmi_write1(sc, RK_DRM_HDMI_A_HDCPCFG1,
	    RK_DRM_HDMI_A_HDCPCFG1_DEFAULT);
	rk_drm_hdmi_write1(sc, RK_DRM_HDMI_A_VIDPOLCFG,
	    RK_DRM_HDMI_A_VIDPOLCFG_DATAENPOL);
}

static void
rk_drm_hdmi_enable_hdmi_mode(struct rk_drm_softc *sc,
    const struct drm_display_mode *mode)
{
	uint8_t hdcpcfg0;
	uint8_t pkt_en;
	uint8_t aspect;
	uint8_t vic;

	rk_drm_hdmi_program_av_composer(sc, mode, true);
	aspect = rk_drm_hdmi_picture_aspect(mode);
	vic = drm_mode_cea_vic(mode);
	rk_drm_hdmi_write1(sc, RK_DRM_HDMI_FC_AVICONF3, 0x00);
	rk_drm_hdmi_write1(sc, RK_DRM_HDMI_FC_AVICONF0, 0x00);
	rk_drm_hdmi_write1(sc, RK_DRM_HDMI_FC_AVICONF1, aspect);
	rk_drm_hdmi_write1(sc, RK_DRM_HDMI_FC_AVICONF2, 0x00);
	rk_drm_hdmi_write1(sc, RK_DRM_HDMI_FC_AVIVID, vic);

	pkt_en = rk_drm_hdmi_read1(sc, RK_DRM_HDMI_FC_PACKET_TX_EN);
	pkt_en |= RK_DRM_HDMI_FC_PACKET_TX_EN_AVI |
	    RK_DRM_HDMI_FC_PACKET_TX_EN_GCP;
	rk_drm_hdmi_write1(sc, RK_DRM_HDMI_FC_PACKET_TX_EN, pkt_en);
	rk_drm_hdmi_write1(sc, RK_DRM_HDMI_PKT_SEND_CTL,
	    RK_DRM_HDMI_PKT_SEND_CTL_AVI_INFO_UP);
	DELAY(10);
	rk_drm_hdmi_write1(sc, RK_DRM_HDMI_PKT_SEND_CTL,
	    RK_DRM_HDMI_PKT_SEND_CTL_AVI_INFO_EN);

	hdcpcfg0 = rk_drm_hdmi_read1(sc, RK_DRM_HDMI_A_HDCPCFG0);
	hdcpcfg0 |= RK_DRM_HDMI_A_HDCPCFG0_HDMIDVI;
	rk_drm_hdmi_write1(sc, RK_DRM_HDMI_A_HDCPCFG0, hdcpcfg0);
	rk_drm_hdmi_write1(sc, RK_DRM_HDMI_A_HDCPCFG1,
	    RK_DRM_HDMI_A_HDCPCFG1_DEFAULT);
	rk_drm_hdmi_write1(sc, RK_DRM_HDMI_A_VIDPOLCFG,
	    RK_DRM_HDMI_A_VIDPOLCFG_DATAENPOL);
	rk_drm_hdmi_clear_overflow(sc);
}

static void
rk_drm_dw_hdmi_init_mode(struct rk_drm_softc *sc,
    const struct drm_display_mode *mode)
{
	rk_drm_hdmi_enable_dvi_mode(sc, mode);
	rk_drm_hdmi_write1_safe(sc, RK_DRM_HDMI_FC_CTRLDUR, 12);
	rk_drm_hdmi_write1_safe(sc, RK_DRM_HDMI_FC_EXCTRLDUR, 32);
	rk_drm_hdmi_write1_safe(sc, RK_DRM_HDMI_FC_EXCTRLSPAC, 1);
	rk_drm_hdmi_write1_safe(sc, RK_DRM_HDMI_FC_CH0PREAM, 0x0b);
	rk_drm_hdmi_write1_safe(sc, RK_DRM_HDMI_FC_CH1PREAM, 0x16);
	rk_drm_hdmi_write1_safe(sc, RK_DRM_HDMI_FC_CH2PREAM, 0x21);
	rk_drm_hdmi_write1_safe(sc, RK_DRM_HDMI_FC_AVICONF3, 0x00);
	rk_drm_hdmi_write1_safe(sc, RK_DRM_HDMI_FC_GCP, 0x00);
	rk_drm_hdmi_write1_safe(sc, RK_DRM_HDMI_FC_AVICONF0, 0x00);
	rk_drm_hdmi_write1_safe(sc, RK_DRM_HDMI_FC_AVICONF1, 0x00);
	rk_drm_hdmi_write1_safe(sc, RK_DRM_HDMI_FC_AVICONF2, 0x00);
	rk_drm_hdmi_write1_safe(sc, RK_DRM_HDMI_FC_AVIVID,
	    drm_mode_cea_vic(mode));
	rk_drm_hdmi_write1_safe(sc, 0x01ff, 0x00);
	rk_drm_hdmi_write1_safe(sc, 0x0184, 0xfe);
}

static void
rk_drm_dw_hdmi_finish_mode(struct rk_drm_softc *sc,
    const struct drm_display_mode *mode)
{
	uint8_t clkdis;
	uint8_t val;

	rk_drm_hdmi_write1_safe(sc, RK_DRM_HDMI_MC_FLOWCTRL, 0x00);
	rk_drm_hdmi_write1_safe(sc, RK_DRM_HDMI_FC_CTRLDUR, 12);
	rk_drm_hdmi_write1_safe(sc, RK_DRM_HDMI_FC_EXCTRLDUR, 32);
	rk_drm_hdmi_write1_safe(sc, RK_DRM_HDMI_FC_EXCTRLSPAC, 1);
	rk_drm_hdmi_write1_safe(sc, RK_DRM_HDMI_FC_CH0PREAM, 0x0b);
	rk_drm_hdmi_write1_safe(sc, RK_DRM_HDMI_FC_CH1PREAM, 0x16);
	rk_drm_hdmi_write1_safe(sc, RK_DRM_HDMI_FC_CH2PREAM, 0x21);

	clkdis = rk_drm_hdmi_read1(sc, RK_DRM_HDMI_MC_CLKDIS) &
	    RK_DRM_HDMI_MC_CLKDIS_CECCLK_DISABLE;
	clkdis |= (uint8_t)~RK_DRM_HDMI_MC_CLKDIS_CECCLK_DISABLE;
	clkdis &= ~0x01;
	rk_drm_hdmi_write1_safe(sc, RK_DRM_HDMI_MC_CLKDIS, clkdis);
	clkdis &= ~0x02;
	rk_drm_hdmi_write1_safe(sc, RK_DRM_HDMI_MC_CLKDIS, clkdis);
	rk_drm_hdmi_write1_safe(sc, RK_DRM_HDMI_FC_VSYNCINWIDTH,
	    rk_drm_mode_vsync_len(mode));

	rk_drm_hdmi_write1_safe(sc, RK_DRM_HDMI_VP_PR_CD, 0x40);

	val = rk_drm_hdmi_read1(sc, RK_DRM_HDMI_VP_STUFF);
	val &= ~0x01;
	val |= 0x01;
	rk_drm_hdmi_write1_safe(sc, RK_DRM_HDMI_VP_STUFF, val);

	val = rk_drm_hdmi_read1(sc, RK_DRM_HDMI_VP_CONF);
	val &= ~(0x10 | 0x04);
	val |= 0x04;
	rk_drm_hdmi_write1_safe(sc, RK_DRM_HDMI_VP_CONF, val);

	rk_drm_hdmi_write1_safe(sc, RK_DRM_HDMI_VP_REMAP, 0x00);

	val = rk_drm_hdmi_read1(sc, RK_DRM_HDMI_VP_CONF);
	val &= ~(0x40 | 0x20 | 0x08);
	val |= 0x40;
	rk_drm_hdmi_write1_safe(sc, RK_DRM_HDMI_VP_CONF, val);

	val = rk_drm_hdmi_read1(sc, RK_DRM_HDMI_VP_STUFF);
	val &= ~(0x02 | 0x04);
	val |= 0x02 | 0x04;
	rk_drm_hdmi_write1_safe(sc, RK_DRM_HDMI_VP_STUFF, val);

	val = rk_drm_hdmi_read1(sc, RK_DRM_HDMI_VP_CONF);
	val &= ~0x03;
	val |= 0x03;
	rk_drm_hdmi_write1_safe(sc, RK_DRM_HDMI_VP_CONF, val);

	rk_drm_hdmi_write1_safe(sc, RK_DRM_HDMI_TX_INVID0, 0x01);
	rk_drm_hdmi_write1_safe(sc, 0x0201, 0x07);
	rk_drm_hdmi_write1_safe(sc, 0x0202, 0x00);
	rk_drm_hdmi_write1_safe(sc, 0x0203, 0x00);
	rk_drm_hdmi_write1_safe(sc, 0x0204, 0x00);
	rk_drm_hdmi_write1_safe(sc, 0x0205, 0x00);
	rk_drm_hdmi_write1_safe(sc, 0x0206, 0x00);
	rk_drm_hdmi_write1_safe(sc, 0x0207, 0x00);

	rk_drm_hdmi_clear_overflow(sc);
}

static int
rk_drm_hdmi_phy_i2c_reset(struct rk_drm_softc *sc)
{
	int timeout;

	rk_drm_hdmi_write1(sc, RK_DRM_HDMI_PHY_I2CM_SOFTRSTZ, 0x00);
	for (timeout = 100; timeout > 0; timeout--) {
		if ((rk_drm_hdmi_read1(sc, RK_DRM_HDMI_PHY_I2CM_SOFTRSTZ) &
		    0x01) != 0)
			return (0);
		DELAY(10);
	}
	return (ETIMEDOUT);
}

static int
rk_drm_hdmi_phy_i2c_write(struct rk_drm_softc *sc, uint8_t reg, uint16_t val)
{
	uint8_t stat;
	uint8_t err;
	uint8_t sticky;
	int timeout;

	if (rk_drm_hdmi_phy_i2c_reset(sc) != 0)
		return (ETIMEDOUT);

	rk_drm_hdmi_write1(sc, RK_DRM_HDMI_IH_I2CMPHY_STAT0, 0x03);
	rk_drm_hdmi_write1(sc, RK_DRM_HDMI_PHY_I2CM_SLAVE, HDMI_PHY_I2C_ADDR);
	rk_drm_hdmi_write1(sc, RK_DRM_HDMI_PHY_I2CM_ADDRESS, reg);
	rk_drm_hdmi_write1(sc, RK_DRM_HDMI_PHY_I2CM_DATAO_1, (val >> 8) & 0xff);
	rk_drm_hdmi_write1(sc, RK_DRM_HDMI_PHY_I2CM_DATAO_0, val & 0xff);
	rk_drm_hdmi_write1(sc, RK_DRM_HDMI_PHY_I2CM_OPERATION, 0x10);

	for (timeout = 200; timeout > 0; timeout--) {
		DELAY(1000);
		err = rk_drm_hdmi_read1(sc, RK_DRM_HDMI_PHY_I2CM_CTLINT);
		sticky = rk_drm_hdmi_read1(sc, RK_DRM_HDMI_IH_I2CMPHY_STAT0);
		if ((sticky & 0x01) != 0 || (err & 0x10) != 0 ||
		    (err & 0x01) != 0) {
			device_printf(sc->dev,
			    "phy i2c write reg=0x%02x val=0x%04x ctlint=0x%02x "
			    "int=0x%02x ih=0x%02x stat0=0x%02x conf0=0x%02x\n",
			    reg, val, err,
			    rk_drm_hdmi_read1(sc, RK_DRM_HDMI_PHY_I2CM_INT),
			    sticky,
			    rk_drm_hdmi_read1(sc, RK_DRM_HDMI_PHY_STAT0),
			    rk_drm_hdmi_read1(sc, RK_DRM_HDMI_PHY_CONF0));
			rk_drm_hdmi_write1(sc, RK_DRM_HDMI_IH_I2CMPHY_STAT0,
			    sticky);
			(void)rk_drm_hdmi_phy_i2c_reset(sc);
			return (EIO);
		}
		stat = rk_drm_hdmi_read1(sc, RK_DRM_HDMI_PHY_I2CM_INT);
		if ((sticky & 0x02) != 0 || (stat & 0x01) != 0) {
			rk_drm_hdmi_write1(sc, RK_DRM_HDMI_IH_I2CMPHY_STAT0, 0x02);
			return (0);
		}
	}
	device_printf(sc->dev,
	    "phy i2c write timeout reg=0x%02x val=0x%04x ctlint=0x%02x "
	    "int=0x%02x ih=0x%02x stat0=0x%02x conf0=0x%02x\n",
	    reg, val, rk_drm_hdmi_read1(sc, RK_DRM_HDMI_PHY_I2CM_CTLINT),
	    rk_drm_hdmi_read1(sc, RK_DRM_HDMI_PHY_I2CM_INT),
	    rk_drm_hdmi_read1(sc, RK_DRM_HDMI_IH_I2CMPHY_STAT0),
	    rk_drm_hdmi_read1(sc, RK_DRM_HDMI_PHY_STAT0),
	    rk_drm_hdmi_read1(sc, RK_DRM_HDMI_PHY_CONF0));
	(void)rk_drm_hdmi_phy_i2c_reset(sc);
	return (ETIMEDOUT);
}

static int
rk_drm_hdmi_phy_init(struct rk_drm_softc *sc,
    const struct drm_display_mode *mode)
{
	const struct rk_drm_mpll_config *mpll_conf;
	const struct rk_drm_phy_config *phy_conf;
	uint8_t phy_conf0;
	int iter;
	int timeout;

	mpll_conf = rk_drm_find_mpll_config(mode->clock);
	phy_conf = rk_drm_find_phy_config(mode->clock);
	if (mpll_conf == NULL || phy_conf == NULL)
		return (EINVAL);

	rk_drm_cru_write4(sc, 0x0240,
	    (1u << 25) | (1u << 26) | (0 << 9) | (0 << 10));
	rk_drm_cru_write4(sc, 0x0244, (1u << 18) | (0 << 2));
	rk_drm_cru_write4(sc, 0x0250, (1u << 28) | (0 << 12));
	rk_drm_cru_write4(sc, 0x0254, (1u << 24) | (0 << 8));
	DELAY(10000);

	for (iter = 0; iter < 2; iter++) {
		rk_drm_hdmi_write1(sc, RK_DRM_HDMI_MC_FLOWCTRL, 0x00);
		rk_drm_hdmi_write1(sc, RK_DRM_HDMI_MC_PHYRSTZ, 0x01);
		rk_drm_hdmi_write1(sc, RK_DRM_HDMI_VP_PR_CD, 0x40);
		DELAY(5000);
		rk_drm_hdmi_write1(sc, RK_DRM_HDMI_MC_PHYRSTZ, 0x00);
		DELAY(5000);
		rk_drm_hdmi_write1(sc, RK_DRM_HDMI_MC_HEACPHY_RST, 0x01);

		rk_drm_hdmi_write1(sc, RK_DRM_HDMI_BASE_SFRDIVLOW,
		    RK_DRM_HDMI_BASE_SFRDIVLOW_DEFAULT);
		rk_drm_hdmi_write1(sc, RK_DRM_HDMI_BASE_SFRDIVHIGH,
		    RK_DRM_HDMI_BASE_SFRDIVHIGH_DEFAULT);

		rk_drm_hdmi_write1(sc, RK_DRM_HDMI_PHY_JTAG_CFG,
		    RK_DRM_HDMI_PHY_JTAG_CFG_I2C);
		if (rk_drm_hdmi_phy_i2c_reset(sc) != 0) {
			device_printf(sc->dev,
			    "phy i2c reset timed out stat0=0x%02x conf0=0x%02x\n",
			    rk_drm_hdmi_read1(sc, RK_DRM_HDMI_PHY_STAT0),
			    rk_drm_hdmi_read1(sc, RK_DRM_HDMI_PHY_CONF0));
			return (ETIMEDOUT);
		}
		rk_drm_hdmi_write1(sc, RK_DRM_HDMI_PHY_I2CM_SLAVE,
		    HDMI_PHY_I2C_ADDR);
		rk_drm_hdmi_write1(sc, RK_DRM_HDMI_PHY_I2CM_DIV,
		    RK_DRM_HDMI_PHY_I2CM_DIV_DEFAULT);
		rk_drm_hdmi_write1(sc, RK_DRM_HDMI_PHY_I2CM_SS_HCNT1, 0x00);
		rk_drm_hdmi_write1(sc, RK_DRM_HDMI_PHY_I2CM_SS_HCNT0,
		    RK_DRM_HDMI_PHY_I2CM_SS_HCNT0_DEFAULT);
		rk_drm_hdmi_write1(sc, RK_DRM_HDMI_PHY_I2CM_SS_LCNT1, 0x00);
		rk_drm_hdmi_write1(sc, RK_DRM_HDMI_PHY_I2CM_SS_LCNT0,
		    RK_DRM_HDMI_PHY_I2CM_SS_LCNT0_DEFAULT);
		rk_drm_hdmi_write1(sc, RK_DRM_HDMI_PHY_I2CM_FS_HCNT1, 0x00);
		rk_drm_hdmi_write1(sc, RK_DRM_HDMI_PHY_I2CM_FS_HCNT0,
		    RK_DRM_HDMI_PHY_I2CM_FS_HCNT0_DEFAULT);
		rk_drm_hdmi_write1(sc, RK_DRM_HDMI_PHY_I2CM_FS_LCNT1, 0x00);
		rk_drm_hdmi_write1(sc, RK_DRM_HDMI_PHY_I2CM_FS_LCNT0,
		    RK_DRM_HDMI_PHY_I2CM_FS_LCNT0_DEFAULT);
		rk_drm_hdmi_write1(sc, RK_DRM_HDMI_PHY_I2CM_SDA_HOLD,
		    RK_DRM_HDMI_PHY_I2CM_SDA_HOLD_DEFAULT);

		phy_conf0 = rk_drm_hdmi_read1(sc, RK_DRM_HDMI_PHY_CONF0);
		phy_conf0 |= RK_DRM_HDMI_PHY_CONF0_SELDATAENPOL |
		    RK_DRM_HDMI_PHY_CONF0_PDDQ;
		phy_conf0 &= ~(RK_DRM_HDMI_PHY_CONF0_SELDIPIF |
		    RK_DRM_HDMI_PHY_CONF0_ENTMDS |
		    RK_DRM_HDMI_PHY_CONF0_PDZ |
		    RK_DRM_HDMI_PHY_CONF0_TXPWRON |
		    RK_DRM_HDMI_PHY_CONF0_SVSRET);
		rk_drm_hdmi_write1(sc, RK_DRM_HDMI_PHY_CONF0, phy_conf0);
		DELAY(1000);

		if (rk_drm_hdmi_phy_i2c_write(sc, RK_DRM_HDMI_PHY_I2C_CPCE_CTRL,
		    mpll_conf->cpce) != 0)
			return (EIO);
		if (rk_drm_hdmi_phy_i2c_write(sc, RK_DRM_HDMI_PHY_I2C_GMPCTRL,
		    mpll_conf->gmp) != 0)
			return (EIO);
		if (rk_drm_hdmi_phy_i2c_write(sc, RK_DRM_HDMI_PHY_I2C_CURRCTRL,
		    mpll_conf->curr) != 0)
			return (EIO);
		if (rk_drm_hdmi_phy_i2c_write(sc, RK_DRM_HDMI_PHY_I2C_PLLPHBYCTRL,
		    0x0000) != 0)
			return (EIO);
		if (rk_drm_hdmi_phy_i2c_write(sc, RK_DRM_HDMI_PHY_I2C_MSM_CTRL,
		    RK_DRM_HDMI_PHY_MSM_CTRL_FB_CLK) != 0)
			return (EIO);
		if (rk_drm_hdmi_phy_i2c_write(sc, RK_DRM_HDMI_PHY_I2C_TXTERM,
		    phy_conf->term) != 0)
			return (EIO);
		if (rk_drm_hdmi_phy_i2c_write(sc, RK_DRM_HDMI_PHY_I2C_CKSYMTXCTRL,
		    phy_conf->sym) != 0)
			return (EIO);
		if (rk_drm_hdmi_phy_i2c_write(sc, RK_DRM_HDMI_PHY_I2C_VLEVCTRL,
		    phy_conf->vlev) != 0)
			return (EIO);
		if (rk_drm_hdmi_phy_i2c_write(sc, RK_DRM_HDMI_PHY_I2C_CKCALCTRL,
		    RK_DRM_HDMI_PHY_I2C_CKCALCTRL_OVERRIDE) != 0)
			return (EIO);

		phy_conf0 = rk_drm_hdmi_read1(sc, RK_DRM_HDMI_PHY_CONF0);
		phy_conf0 |= RK_DRM_HDMI_PHY_CONF0_PDZ;
		rk_drm_hdmi_write1(sc, RK_DRM_HDMI_PHY_CONF0, phy_conf0);
		DELAY(1000);

		phy_conf0 &= ~RK_DRM_HDMI_PHY_CONF0_ENTMDS;
		rk_drm_hdmi_write1(sc, RK_DRM_HDMI_PHY_CONF0, phy_conf0);
		DELAY(1000);
		phy_conf0 |= RK_DRM_HDMI_PHY_CONF0_ENTMDS;
		rk_drm_hdmi_write1(sc, RK_DRM_HDMI_PHY_CONF0, phy_conf0);
		DELAY(1000);

		phy_conf0 |= RK_DRM_HDMI_PHY_CONF0_TXPWRON |
		    RK_DRM_HDMI_PHY_CONF0_SVSRET;
		phy_conf0 &= ~RK_DRM_HDMI_PHY_CONF0_PDDQ;
		rk_drm_hdmi_write1(sc, RK_DRM_HDMI_PHY_CONF0, phy_conf0);
		DELAY(5000);
	}

	rk_drm_hdmi_write1(sc, RK_DRM_HDMI_MC_CLKDIS, 0x00);
	rk_drm_hdmi_write1(sc, RK_DRM_HDMI_FC_VSYNCINWIDTH,
	    rk_drm_mode_vsync_len(mode));
	rk_drm_hdmi_toggle_main_reset(sc,
	    RK_DRM_HDMI_MC_SWRST_TMDS | RK_DRM_HDMI_MC_SWRST_PIXEL);

	for (timeout = 20; timeout > 0; timeout--) {
		DELAY(5000);
		if ((rk_drm_hdmi_read1(sc, RK_DRM_HDMI_PHY_STAT0) & 0x01) != 0)
			return (0);
	}
	device_printf(sc->dev,
	    "phy pll lock timeout stat0=0x%02x lock=0x%02x conf0=0x%02x\n",
	    rk_drm_hdmi_read1(sc, RK_DRM_HDMI_PHY_STAT0),
	    rk_drm_hdmi_read1(sc, RK_DRM_HDMI_MC_LOCKONCLOCK),
	    rk_drm_hdmi_read1(sc, RK_DRM_HDMI_PHY_CONF0));
	return (ETIMEDOUT);
}

int
rk_drm_hw_modeset(struct rk_drm_softc *sc, const struct drm_display_mode *mode)
{
	struct drm_display_mode default_mode;
	int error;

	if (!sc->hw_attached)
		return (ENXIO);
	if (mode == NULL) {
		rk_drm_default_mode_fill(&default_mode);
		mode = &default_mode;
	}
	if (!rk_drm_hw_mode_valid(mode))
		return (EINVAL);

	rk_drm_fb_fill(sc, RK_DRM_FB_BOOT_COLOR);
	rk_drm_display_domain_sanity(sc);
	rk_drm_route_vop_to_hdmi(sc);
	rk_drm_vop_init_mode(sc, mode);
	rk_drm_dw_hdmi_init_mode(sc, mode);
	error = rk_drm_hdmi_phy_init(sc, mode);
	if (error != 0) {
		device_printf(sc->dev, "HDMI PHY init failed: %d\n", error);
		return (error);
	}
	rk_drm_dw_hdmi_finish_mode(sc, mode);
	rk_drm_hdmi_enable_hdmi_mode(sc, mode);
	sc->output_enabled = true;
	return (0);
}

bool
rk_drm_hw_hpd(struct rk_drm_softc *sc)
{
	if (sc->hdmi_va == 0)
		return (false);
	return ((rk_drm_hdmi_read1(sc, RK_DRM_HDMI_PHY_STAT0) & 0x02) != 0);
}

static void
rk_drm_hw_unmap(struct rk_drm_softc *sc)
{
	if (sc->hdmi_va != 0)
		pmap_unmapdev((void *)sc->hdmi_va, sc->hdmi_size);
	if (sc->cru_va != 0)
		bus_space_unmap(fdtbus_bs_tag, sc->cru_bsh, sc->cru_size);
	if (sc->pmucru_va != 0)
		bus_space_unmap(fdtbus_bs_tag, sc->pmucru_bsh, sc->pmucru_size);
	if (sc->pmu_va != 0)
		bus_space_unmap(fdtbus_bs_tag, sc->pmu_bsh, sc->pmu_size);
	if (sc->grf_va != 0)
		bus_space_unmap(fdtbus_bs_tag, sc->grf_bsh, sc->grf_size);
	if (sc->vop_va != 0)
		bus_space_unmap(fdtbus_bs_tag, sc->vop_bsh, sc->vop_size);

	sc->hdmi_va = 0;
	sc->cru_va = 0;
	sc->pmucru_va = 0;
	sc->pmu_va = 0;
	sc->grf_va = 0;
	sc->vop_va = 0;
}

int
rk_drm_hw_attach(struct rk_drm_softc *sc)
{
	struct drm_display_mode default_mode;
	int error;

	sc->stride = RK_DRM_MAX_WIDTH * (RK_DRM_BPP / 8);
	sc->fb_size = sc->stride * RK_DRM_MAX_HEIGHT;

	error = rk_drm_fb_alloc(sc);
	if (error != 0) {
		device_printf(sc->dev, "framebuffer alloc failed: %d\n", error);
		return (error);
	}

	sc->vop_pa = 0xff900000;
	sc->vop_size = 0x10000;
	if (bus_space_map(fdtbus_bs_tag, sc->vop_pa, sc->vop_size, 0,
	    &sc->vop_bsh) != 0) {
		error = ENXIO;
		goto fail;
	}
	sc->vop_va = (vm_offset_t)sc->vop_bsh;

	sc->grf_pa = 0xff770000;
	sc->grf_size = 0x10000;
	if (bus_space_map(fdtbus_bs_tag, sc->grf_pa, sc->grf_size, 0,
	    &sc->grf_bsh) != 0) {
		error = ENXIO;
		goto fail;
	}
	sc->grf_va = (vm_offset_t)sc->grf_bsh;

	sc->pmu_pa = 0xff310000;
	sc->pmu_size = 0x1000;
	if (bus_space_map(fdtbus_bs_tag, sc->pmu_pa, sc->pmu_size, 0,
	    &sc->pmu_bsh) != 0) {
		error = ENXIO;
		goto fail;
	}
	sc->pmu_va = (vm_offset_t)sc->pmu_bsh;

	sc->pmucru_pa = 0xff750000;
	sc->pmucru_size = 0x1000;
	if (bus_space_map(fdtbus_bs_tag, sc->pmucru_pa, sc->pmucru_size, 0,
	    &sc->pmucru_bsh) != 0) {
		error = ENXIO;
		goto fail;
	}
	sc->pmucru_va = (vm_offset_t)sc->pmucru_bsh;

	sc->cru_pa = 0xff760000;
	sc->cru_size = 0x1000;
	if (bus_space_map(fdtbus_bs_tag, sc->cru_pa, sc->cru_size, 0,
	    &sc->cru_bsh) != 0) {
		error = ENXIO;
		goto fail;
	}
	sc->cru_va = (vm_offset_t)sc->cru_bsh;

	sc->hdmi_pa = 0xff940000;
	sc->hdmi_size = 0x20000;
	sc->hdmi_va = (vm_offset_t)pmap_mapdev(sc->hdmi_pa, sc->hdmi_size);
	if (sc->hdmi_va == 0) {
		error = ENXIO;
		goto fail;
	}

	sc->hw_attached = true;
	rk_drm_default_mode_fill(&default_mode);
	error = rk_drm_hw_modeset(sc, &default_mode);
	if (error != 0)
		goto fail;

	device_printf(sc->dev,
	    "initial scanout ready %dx%d stride=%u fb_pa=0x%jx hpd=%d\n",
	    default_mode.hdisplay, default_mode.vdisplay, sc->stride,
	    (uintmax_t)sc->fb_pa, rk_drm_hw_hpd(sc));
	return (0);

fail:
	rk_drm_hw_detach(sc);
	return (error);
}

void
rk_drm_hw_detach(struct rk_drm_softc *sc)
{
	rk_drm_hw_unmap(sc);
	rk_drm_fb_free(sc);
	sc->hw_attached = false;
}
