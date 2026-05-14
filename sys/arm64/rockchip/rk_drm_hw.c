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

#ifndef DRM_MODE_FLAG_PPIXDATA
#define	DRM_MODE_FLAG_PPIXDATA	0
#endif

#define HDMI_PHY_I2C_ADDR  0x69

#define RK_DRM_SYS_GRF_GPIO4C_IOMUX 0x0e028
#define RK_DRM_SYS_GRF_SOC_CON9     0x6224
#define RK_DRM_SYS_GRF_SOC_CON20    0x6250
#define RK_DRM_GRF_HDMI_LCDC_SEL    (1u << 6)
#define RK_DRM_GRF_EDP_LCDC_SEL     (1u << 5)
/*
 * SOC_CON9[12] = DP_SEL_VOP_LIT: 0 = route VOP_BIG to Cadence MHDP
 * (the USB-C DP encoder), 1 = route VOP_LITTLE.  Distinct from
 * SOC_CON20[5] which muxes the Analogix eDP encoder.
 */
#define RK_DRM_GRF_DP_SEL_VOP_LIT   (1u << 12)
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
#define RK_DRM_VOP_DSP_OUT_MODE_P888 0x00000000u	/* RGB888 24bpp */
#define RK_DRM_VOP_DSP_OUT_MODE_AAAA 0x0000000fu	/* RGB+alpha 30bpp */
#define RK_DRM_VOP_DSP_CTRL0_PIN_POL_MASK   (0x7u << 4)
#define RK_DRM_VOP_DSP_CTRL0_DCLK_POL       (1u << 7)
#define RK_DRM_VOP_DSP_CTRL0_P2I_EN         (1u << 5)
#define RK_DRM_VOP_DSP_CTRL0_INTERLACE      (1u << 10)
#define RK_DRM_VOP_DSP_CTRL1_DP_PIN_POL_MASK (0x7u << 16)
#define RK_DRM_VOP_DSP_CTRL1_DP_DCLK_POL    (1u << 19)
#define RK_DRM_VOP_DSP_CTRL1_HDMI_PIN_POL_MASK  (0x7u << 20)
#define RK_DRM_VOP_DSP_CTRL1_HDMI_PIN_POL_POS   (0x3u << 20)
#define RK_DRM_VOP_DSP_CTRL1_HDMI_DCLK_POL      (1u << 23)
#define RK_DRM_VOP_WIN0_LB_MODE_RGB_1920X5 (4u << 5)
#define RK_DRM_VOP_WIN0_DATA_FMT_XRGB8888 0x00000000u
/*
 * WIN0_CTRL0 upper bits encode csc_en + color-space + yuv-clip state.
 * Live read of working the reference build with display on USB-C DP
 * shows these set to 0x3a000000 — needed so the Cadence framer sees
 * pixels with the right CSC.  HDMI's RGB-out pipeline reads "No
 * Support" if these are set (the pre-USB-C HDMI path had them at 0).
 * Caller passes the right upper value per output route.
 */
#define RK_DRM_VOP_WIN0_CTRL0_UPPER_DP   0x3a000000u
#define RK_DRM_VOP_WIN0_CTRL0_UPPER_HDMI 0x00000000u
#define RK_DRM_VOP_WIN0_CTRL0_LOWER \
    (RK_DRM_VOP_WIN0_LB_MODE_RGB_1920X5 | \
    RK_DRM_VOP_WIN0_DATA_FMT_XRGB8888 | 0x00000001u)
#define RK_DRM_VOP_WIN0_CTRL2_PRIMARY 0x00000021u
#define RK_DRM_VOP_WIN0_SRC_ALPHA_CTRL_OPAQUE 0x00ff0000u
#define RK_DRM_VOP_WIN0_DST_ALPHA_CTRL_OPAQUE 0x00000000u

#define RK_DRM_FB_DMA_LOWADDR_TEST    0x0fffffffu

#define RK_DRM_CRU_VPLL_CON0         0x00c0
#define RK_DRM_CRU_VPLL_CON1         0x00c4
#define RK_DRM_CRU_VPLL_CON2         0x00c8
#define RK_DRM_CRU_VPLL_CON3         0x00cc
#define RK_DRM_CRU_CLKGATE_CON10     0x0328
#define RK_DRM_CRU_CLKGATE_CON28     0x0370
#define RK_DRM_CRU_SOFTRST_CON17     0x0444
#define RK_DRM_CRU_DRESETN_VOP0_REQ  (1u << 8)
#define RK_DRM_CRU_DRESETN_VOP1_REQ  (1u << 9)
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
/*
 * Audio block registers (Designware HDMI TX, mirrors sys/dev/hdmi/dwc_hdmireg.h
 * but uses our 4-byte-stride MMIO accessors).  Used by Stage 1 audio bring-up:
 * configure CTS/N for 48 kHz, select I2S input, send Audio InfoFrame.  Does not
 * yet drive PCM data; that requires the I2S source path to be wired up.
 */
#define RK_DRM_HDMI_FC_AUDICONF0     0x1025
#define RK_DRM_HDMI_FC_AUDICONF1     0x1026
#define RK_DRM_HDMI_FC_AUDICONF2     0x1027
#define RK_DRM_HDMI_FC_AUDICONF3     0x1028
#define RK_DRM_HDMI_FC_AUDSCONF      0x1063
#define RK_DRM_HDMI_FC_AUDSV         0x1065
#define RK_DRM_HDMI_AUD_CONF0        0x3100
#define RK_DRM_HDMI_AUD_CONF1        0x3101
#define RK_DRM_HDMI_AUD_N1           0x3200
#define RK_DRM_HDMI_AUD_N2           0x3201
#define RK_DRM_HDMI_AUD_N3           0x3202
#define RK_DRM_HDMI_AUD_CTS3         0x3205
#define RK_DRM_HDMI_AUD_INPUTCLKFS   0x3206

#define RK_DRM_HDMI_AUD_CONF0_INTERFACE_IIS  0x20
#define RK_DRM_HDMI_AUD_CONF0_I2SINEN_CH2    0x01
#define RK_DRM_HDMI_AUD_CONF0_I2SINEN_MASK   0x0f
#define RK_DRM_HDMI_AUD_CONF0_INTERFACE_MASK 0x20
#define RK_DRM_HDMI_AUD_CONF1_DATAMODE_IIS   0x00
#define RK_DRM_HDMI_AUD_CONF1_DATAMODE_MASK  0xe0
#define RK_DRM_HDMI_AUD_CONF1_DATWIDTH_16BIT 0x10
#define RK_DRM_HDMI_AUD_CONF1_DATWIDTH_MASK  0x1f
#define RK_DRM_HDMI_AUD_CTS3_N_SHIFT_MASK    0xe0
#define RK_DRM_HDMI_AUD_CTS3_CTS_MANUAL      0x10
#define RK_DRM_HDMI_AUD_INPUTCLKFS_64        0x04
#define RK_DRM_HDMI_MC_CLKDIS_AUDCLK_DISABLE (1u << 3)
#define RK_DRM_HDMI_FC_AUDICONF0_CC_2CH      (1u << 4) /* CC=1 (2 channels) */
#define RK_DRM_HDMI_FC_AUDSV_LR_VALID        0xee     /* L+R valid */

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
	{ 121750, 6, 487, 4, 4 },
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

/* VOP_LIT MMIO read/write — same register layout as VOP_BIG, different base. */
static inline uint32_t
rk_drm_vop_lit_read4(struct rk_drm_softc *sc, size_t off)
{
	return (bus_space_read_4(fdtbus_bs_tag, sc->vop_lit_bsh, off));
}

static inline void
rk_drm_vop_lit_write4(struct rk_drm_softc *sc, size_t off, uint32_t val)
{
	bus_space_write_4(fdtbus_bs_tag, sc->vop_lit_bsh, off, val);
	bus_space_barrier(fdtbus_bs_tag, sc->vop_lit_bsh, off, 4,
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
rk_drm_grf_read4(struct rk_drm_softc *sc, size_t off)
{
	return (bus_space_read_4(fdtbus_bs_tag, sc->grf_bsh, off));
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

void
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

/*
 * Route VOP B (the big VOP) to drive both DP encoders.
 *  - SOC_CON20[5] = EDP_LCDC_SEL: 0 = VOP_BIG -> Analogix eDP
 *  - SOC_CON20[6] = HDMI_LCDC_SEL: 1 = VOP_LIT -> HDMI (live the reference build has this set
 *                   even though display is on DP — routes HDMI off VOP_BIG)
 *  - SOC_CON9[12] = DP_SEL_VOP_LIT: 0 = VOP_BIG -> Cadence MHDP
 * Hiword-update: low half = value, high half = bit-mask.
 */
static void
rk_drm_route_vop_to_dp(struct rk_drm_softc *sc)
{
	rk_drm_grf_write4(sc, RK_DRM_SYS_GRF_SOC_CON20,
	    (RK_DRM_GRF_EDP_LCDC_SEL << 16));
	rk_drm_grf_write4(sc, RK_DRM_SYS_GRF_SOC_CON20,
	    (RK_DRM_GRF_HDMI_LCDC_SEL << 16) | RK_DRM_GRF_HDMI_LCDC_SEL);
	rk_drm_grf_write4(sc, RK_DRM_SYS_GRF_SOC_CON9,
	    (RK_DRM_GRF_DP_SEL_VOP_LIT << 16));
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

enum rk_drm_win0_route {
	RK_DRM_WIN0_ROUTE_HDMI,
	RK_DRM_WIN0_ROUTE_DP,
};

static void
rk_drm_vop_program_win0_opaque(struct rk_drm_softc *sc,
    const struct drm_display_mode *mode, uint32_t hact_start,
    uint32_t vact_start, enum rk_drm_win0_route route)
{
	uint32_t stride_bytes, stride_words;
	uint32_t ctrl0_upper;
	bool write_alpha;

	switch (route) {
	case RK_DRM_WIN0_ROUTE_HDMI:
		/*
		 * Pre-USB-C HDMI path wrote only the lower bits of
		 * WIN0_CTRL0 and never touched the alpha-control regs.
		 * Match that exactly: HDMI sinks (e.g. XYM W156F1) report
		 * "No Support" when CSC bits (0x3a000000) or forced-opaque
		 * alpha (0x00ff0000 in SRC_ALPHA) are set.
		 */
		ctrl0_upper = RK_DRM_VOP_WIN0_CTRL0_UPPER_HDMI;
		write_alpha = false;
		break;
	case RK_DRM_WIN0_ROUTE_DP:
	default:
		/*
		 * Cadence DP framer needs CSC bits + force-opaque alpha
		 * (mirrors reference vendor BSP state observed when display
		 * is on USB-C DP).  Without these, the framer drops pixels.
		 */
		ctrl0_upper = RK_DRM_VOP_WIN0_CTRL0_UPPER_DP;
		write_alpha = true;
		break;
	}

	stride_bytes = roundup2(mode->hdisplay, 16) * (RK_DRM_BPP / 8);
	stride_words = stride_bytes / 4;

	rk_drm_vop_write4(sc, 0x0038, 0x00000000);
	rk_drm_vop_write4(sc, 0x003c, stride_words);
	rk_drm_vop_write4(sc, 0x0040, (uint32_t)sc->fb_pa);
	rk_drm_vop_write4(sc, 0x0048,
	    (((uint32_t)mode->vdisplay - 1) << 16) |
	    ((uint32_t)mode->hdisplay - 1));
	rk_drm_vop_write4(sc, 0x004c,
	    (((uint32_t)mode->vdisplay - 1) << 16) |
	    ((uint32_t)mode->hdisplay - 1));
	rk_drm_vop_write4(sc, 0x0050,
	    (vact_start << 16) | hact_start);
	if (write_alpha) {
		rk_drm_vop_write4(sc, 0x0060,
		    RK_DRM_VOP_WIN0_SRC_ALPHA_CTRL_OPAQUE);
		rk_drm_vop_write4(sc, 0x0064,
		    RK_DRM_VOP_WIN0_DST_ALPHA_CTRL_OPAQUE);
	}
	rk_drm_vop_write4(sc, 0x006c, RK_DRM_VOP_WIN0_CTRL2_PRIMARY);
	rk_drm_vop_write4(sc, RK_DRM_VOP_POST_DSP_HACT_INFO,
	    (hact_start << 16) | (hact_start + mode->hdisplay));
	rk_drm_vop_write4(sc, RK_DRM_VOP_POST_DSP_VACT_INFO,
	    (vact_start << 16) | (vact_start + mode->vdisplay));
	rk_drm_vop_write4(sc, 0x0030,
	    ctrl0_upper | RK_DRM_VOP_WIN0_CTRL0_LOWER);
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

	/*
	 * CRU CLKSEL_CON47/49 setup for the HDMI path.  Pre-USB-C values
	 * (bit 6 in CON47 low half + 0x0000 in CON49 low half) drive DCLK
	 * via the path the XYM W156F1 panel accepts.  USB-C bring-up
	 * inadvertently mutated these to bit 7 / 0x0100 — the DP variant
	 * below kept the original; HDMI did not.  Restored.
	 */
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
	    RK_DRM_VOP_SYS_CTRL_MIPI_EN |
	    RK_DRM_VOP_SYS_CTRL_MIPI_DUAL);
	sys_ctrl |= RK_DRM_VOP_SYS_CTRL_ENABLE |
	    RK_DRM_VOP_SYS_CTRL_RGB_EN |
	    RK_DRM_VOP_SYS_CTRL_HDMI_EN;
	rk_drm_vop_write4(sc, 0x0008, sys_ctrl);

	dsp_ctrl0 &= ~RK_DRM_VOP_DSP_OUT_MODE_MASK;
	dsp_ctrl0 |= RK_DRM_VOP_DSP_OUT_MODE_AAAA;
	rk_drm_vop_write4(sc, 0x0010, dsp_ctrl0);

	/*
	 * HDMI pin polarity: bits 20:22 of DSP_CTRL1 encode HSYNC_POSITIVE
	 * (bit 20), VSYNC_POSITIVE (bit 21), DEN_NEGATIVE (bit 22).  Drive
	 * those from mode->flags so the XYM W156F1 (and other DMT-leaning
	 * sinks) get the negative HSYNC they expect — the previously
	 * hard-coded POS value only worked for CEA-style PHSYNC modes.
	 */
	{
		uint32_t hdmi_pin_pol = 0;

		if ((mode->flags & DRM_MODE_FLAG_NHSYNC) == 0)
			hdmi_pin_pol |= (1u << 0);
		if ((mode->flags & DRM_MODE_FLAG_NVSYNC) == 0)
			hdmi_pin_pol |= (1u << 1);
		dsp_ctrl1 &= ~(RK_DRM_VOP_DSP_CTRL1_HDMI_PIN_POL_MASK |
		    RK_DRM_VOP_DSP_CTRL1_HDMI_DCLK_POL);
		dsp_ctrl1 |= (hdmi_pin_pol & 0x7) << 20 |
		    RK_DRM_VOP_DSP_CTRL1_HDMI_DCLK_POL;
	}
	rk_drm_vop_write4(sc, 0x0014, dsp_ctrl1);

	rk_drm_vop_program_win0_opaque(sc, mode, hact_start, vact_start,
	    RK_DRM_WIN0_ROUTE_HDMI);
	rk_drm_vop_write4(sc, RK_DRM_VOP_DSP_HTOTAL_HS_END,
	    ((uint32_t)mode->htotal << 16) | rk_drm_mode_hsync_len(mode));
	rk_drm_vop_write4(sc, RK_DRM_VOP_DSP_HACT_ST_END,
	    (hact_start << 16) | (hact_start + mode->hdisplay));
	rk_drm_vop_write4(sc, RK_DRM_VOP_DSP_VTOTAL_VS_END,
	    ((uint32_t)mode->vtotal << 16) | rk_drm_mode_vsync_len(mode));
	rk_drm_vop_write4(sc, RK_DRM_VOP_DSP_VACT_ST_END,
	    (vact_start << 16) | (vact_start + mode->vdisplay));
	rk_drm_vop_write4(sc, 0x0000, 0x00010001);
	if (!sc->vop_scanning) {
		rk_drm_vop_pulse_dclk_reset(sc);
		DELAY(40000);
	}
	sc->vop_scanning = true;
}

/*
 * Phase 1.2: VOP setup for the HDMI output path running on VOP_LIT
 * instead of VOP_BIG.  Mirrors rk_drm_vop_init_mode() (the HDMI variant
 * for VOP_BIG) register-by-register but writes via rk_drm_vop_lit_*.
 *
 * Notes from signal_dump on the live board:
 *   - U-Boot leaves VOP_LIT enabled (SYS_CTRL bits 11+RGB_EN already
 *     set), so we don't need to bring the block up from cold or hit
 *     the CRU reset path.  The pulse_dclk_reset + 40ms hard reset
 *     path is skipped — we just reprogram timing in place.
 *   - DCLK_VOP1 lives in CLKSEL_CON48 (0x01c0), not CON47.  Since
 *     U-Boot already configured it for whatever idle scanout it
 *     showed, leave the CRU writes off until we need to retune.
 *
 * GRF mux routing (HDMI_LCDC_SEL=1 → HDMI consumes VOP_LIT) is the
 * caller's responsibility — set it before calling this function.
 */
static void
rk_drm_vop_lit_init_mode_hdmi(struct rk_drm_softc *sc,
    const struct drm_display_mode *mode)
{
	uint32_t hact_start, vact_start;
	uint32_t sys_ctrl, dsp_ctrl0, dsp_ctrl1;
	uint32_t stride_bytes, stride_words;

	hact_start = rk_drm_mode_hact_start(mode);
	vact_start = rk_drm_mode_vact_start(mode);

	/*
	 * VPLL programming (shared with VOP_BIG path).  This sets the
	 * pixel-clock reference; both DCLK_VOP0 and DCLK_VOP1 source from
	 * VPLL when their CLKSEL_CON*'s PLL_SEL bits are set.
	 */
	if (rk_drm_program_vpll(sc, mode->clock) != 0)
		device_printf(sc->dev, "VPLL setup failed, continuing\n");

	/*
	 * VOP_LIT clock setup, per RK3399 TRM V1.4:
	 *   CRU_CLKSEL_CON48 (0x01c0) — ACLK_VOP1 + HCLK_VOP1 (bus clocks)
	 *   CRU_CLKSEL_CON50 (0x01c8) — DCLK_VOP1 (pixel clock!)
	 *
	 * Mirror the working VOP_BIG values:
	 *   CON48 low: aclk_vop1_div_con=0x01, aclk_vop1_pll_sel=01 (CPLL),
	 *              hclk_vop1_div_con=0x03 (the same layout the
	 *              working VOP_BIG HDMI path uses on CON47).
	 *   CON50 low: dclk_vop1_div_con=0x00 (/1), dclk_vop1_pll_sel=00
	 *              (VPLL), dclk_vop1_dclk_sel=0 (divout).  Same as
	 *              CON49 for DCLK_VOP0 in the working HDMI path.
	 */
	rk_drm_cru_write4(sc, 0x01c0,
	    ((((0x1fu << 8) | (0x3u << 6) | 0x1fu) << 16) |
	    ((3u << 8) | (1u << 6) | 1u)));
	rk_drm_cru_write4(sc, 0x01c8,
	    ((((1u << 11) | (0x3u << 8) | 0xffu) << 16) | 0x0000u));

	/*
	 * Pulse DCLK_VOP1 reset so the new CLKSEL_CON50 divider/PLL_SEL
	 * actually latches into the divider state machine.  CRU SOFTRST
	 * CON17 bit 9 = dresetn_vop1_req (per RK3399 TRM V1.4).
	 */
	rk_drm_cru_write4(sc, RK_DRM_CRU_SOFTRST_CON17,
	    (RK_DRM_CRU_DRESETN_VOP1_REQ << 16) | RK_DRM_CRU_DRESETN_VOP1_REQ);
	DELAY(1000);
	rk_drm_cru_write4(sc, RK_DRM_CRU_SOFTRST_CON17,
	    (RK_DRM_CRU_DRESETN_VOP1_REQ << 16));
	DELAY(1000);

	sys_ctrl = rk_drm_vop_lit_read4(sc, 0x0008);
	dsp_ctrl0 = rk_drm_vop_lit_read4(sc, 0x0010);
	dsp_ctrl1 = rk_drm_vop_lit_read4(sc, 0x0014);

	sys_ctrl &= ~(RK_DRM_VOP_SYS_CTRL_STANDBY |
	    RK_DRM_VOP_SYS_CTRL_MMU_EN |
	    RK_DRM_VOP_SYS_CTRL_MIPI_EN |
	    RK_DRM_VOP_SYS_CTRL_MIPI_DUAL);
	sys_ctrl |= RK_DRM_VOP_SYS_CTRL_ENABLE |
	    RK_DRM_VOP_SYS_CTRL_RGB_EN |
	    RK_DRM_VOP_SYS_CTRL_HDMI_EN;
	rk_drm_vop_lit_write4(sc, 0x0008, sys_ctrl);

	dsp_ctrl0 &= ~RK_DRM_VOP_DSP_OUT_MODE_MASK;
	dsp_ctrl0 |= RK_DRM_VOP_DSP_OUT_MODE_AAAA;
	rk_drm_vop_lit_write4(sc, 0x0010, dsp_ctrl0);

	{
		uint32_t hdmi_pin_pol = 0;

		if ((mode->flags & DRM_MODE_FLAG_NHSYNC) == 0)
			hdmi_pin_pol |= (1u << 0);
		if ((mode->flags & DRM_MODE_FLAG_NVSYNC) == 0)
			hdmi_pin_pol |= (1u << 1);
		dsp_ctrl1 &= ~(RK_DRM_VOP_DSP_CTRL1_HDMI_PIN_POL_MASK |
		    RK_DRM_VOP_DSP_CTRL1_HDMI_DCLK_POL);
		dsp_ctrl1 |= (hdmi_pin_pol & 0x7) << 20 |
		    RK_DRM_VOP_DSP_CTRL1_HDMI_DCLK_POL;
	}
	rk_drm_vop_lit_write4(sc, 0x0014, dsp_ctrl1);

	/*
	 * WIN0 program — clone of rk_drm_vop_program_win0_opaque body
	 * with HDMI route (no CSC upper bits, no alpha writes).  Uses
	 * the shared GEM framebuffer (mirror mode for Phase 1.2).
	 */
	stride_bytes = roundup2(mode->hdisplay, 16) * (RK_DRM_BPP / 8);
	stride_words = stride_bytes / 4;
	rk_drm_vop_lit_write4(sc, 0x0038, 0x00000000);
	rk_drm_vop_lit_write4(sc, 0x003c, stride_words);
	rk_drm_vop_lit_write4(sc, 0x0040, (uint32_t)sc->fb_pa);
	rk_drm_vop_lit_write4(sc, 0x0048,
	    (((uint32_t)mode->vdisplay - 1) << 16) |
	    ((uint32_t)mode->hdisplay - 1));
	rk_drm_vop_lit_write4(sc, 0x004c,
	    (((uint32_t)mode->vdisplay - 1) << 16) |
	    ((uint32_t)mode->hdisplay - 1));
	rk_drm_vop_lit_write4(sc, 0x0050,
	    (vact_start << 16) | hact_start);
	rk_drm_vop_lit_write4(sc, 0x006c, RK_DRM_VOP_WIN0_CTRL2_PRIMARY);
	rk_drm_vop_lit_write4(sc, RK_DRM_VOP_POST_DSP_HACT_INFO,
	    (hact_start << 16) | (hact_start + mode->hdisplay));
	rk_drm_vop_lit_write4(sc, RK_DRM_VOP_POST_DSP_VACT_INFO,
	    (vact_start << 16) | (vact_start + mode->vdisplay));
	rk_drm_vop_lit_write4(sc, 0x0030,
	    RK_DRM_VOP_WIN0_CTRL0_UPPER_HDMI |
	    RK_DRM_VOP_WIN0_CTRL0_LOWER);

	rk_drm_vop_lit_write4(sc, RK_DRM_VOP_DSP_HTOTAL_HS_END,
	    ((uint32_t)mode->htotal << 16) | rk_drm_mode_hsync_len(mode));
	rk_drm_vop_lit_write4(sc, RK_DRM_VOP_DSP_HACT_ST_END,
	    (hact_start << 16) | (hact_start + mode->hdisplay));
	rk_drm_vop_lit_write4(sc, RK_DRM_VOP_DSP_VTOTAL_VS_END,
	    ((uint32_t)mode->vtotal << 16) | rk_drm_mode_vsync_len(mode));
	rk_drm_vop_lit_write4(sc, RK_DRM_VOP_DSP_VACT_ST_END,
	    (vact_start << 16) | (vact_start + mode->vdisplay));
	rk_drm_vop_lit_write4(sc, 0x0000, 0x00010001);
	sc->vop_lit_scanning = true;
}

/* Forward declarations — definitions live further down in this file. */
static void rk_drm_dw_hdmi_init_mode(struct rk_drm_softc *sc,
    const struct drm_display_mode *mode);
static void rk_drm_dw_hdmi_finish_mode(struct rk_drm_softc *sc,
    const struct drm_display_mode *mode);
static int rk_drm_hdmi_phy_init(struct rk_drm_softc *sc,
    const struct drm_display_mode *mode);
static void rk_drm_hdmi_enable_hdmi_mode(struct rk_drm_softc *sc,
    const struct drm_display_mode *mode);
static void rk_drm_hdmi_configure_audio(struct rk_drm_softc *sc,
    const struct drm_display_mode *mode);

/*
 * Phase 1.2: HDMI on VOP_LIT entry point.  Flips the GRF mux so HDMI
 * consumes VOP_LIT's pixel stream (instead of VOP_BIG), reprograms
 * VOP_LIT for the requested mode, then runs the existing dw-hdmi
 * controller bring-up — that side of the pipeline is downstream of
 * the GRF mux and works the same regardless of which VOP feeds it.
 */
int
rk_drm_hw_modeset_hdmi_lit(struct rk_drm_softc *sc,
    const struct drm_display_mode *mode)
{
	int error;

	if (!sc->hw_attached)
		return (ENXIO);
	if (mode == NULL || !rk_drm_hw_mode_valid(mode))
		return (EINVAL);

	/* GRF SOC_CON20[6] = HDMI_LCDC_SEL: 1 = VOP_LIT -> HDMI. */
	rk_drm_grf_write4(sc, RK_DRM_SYS_GRF_SOC_CON20,
	    (RK_DRM_GRF_HDMI_LCDC_SEL << 16) | RK_DRM_GRF_HDMI_LCDC_SEL);

	rk_drm_vop_lit_init_mode_hdmi(sc, mode);
	rk_drm_dw_hdmi_init_mode(sc, mode);
	error = rk_drm_hdmi_phy_init(sc, mode);
	if (error != 0) {
		device_printf(sc->dev,
		    "HDMI-on-VOP_LIT: PHY init failed: %d\n", error);
		return (error);
	}
	rk_drm_dw_hdmi_finish_mode(sc, mode);
	rk_drm_hdmi_enable_hdmi_mode(sc, mode);
	rk_drm_hdmi_configure_audio(sc, mode);

	device_printf(sc->dev,
	    "HDMI-on-VOP_LIT: modeset %ux%u@%u kHz active\n",
	    mode->hdisplay, mode->vdisplay, mode->clock);
	sc->hdmi_active = true;
	return (0);
}

/*
 * VOP setup for the eDP/DP output path.
 * Same VOP B + WIN0 + scanout-buffer programming as the HDMI variant,
 * but enables EDP_EN instead of HDMI_EN in SYS_CTRL and skips the
 * HDMI-specific DSP_CTRL1 pin polarity / DCLK polarity bits — the DP
 * side of the pipeline is consumed by CDN-DP, which encapsulates the
 * pixel stream into DP packets and drives its own clocks.
 */
static void
rk_drm_vop_init_mode_dp(struct rk_drm_softc *sc,
    const struct drm_display_mode *mode)
{
	uint32_t hact_start, vact_start;
	uint32_t sys_ctrl, dsp_ctrl0, dsp_ctrl1, post_scl_ctrl;
	uint32_t pin_pol, dp_pin_pol;

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
	post_scl_ctrl = rk_drm_vop_read4(sc, 0x0180);

	/*
	 * Live read of working the reference build shows SYS_CTRL = 0x20801800: both
	 * dp_en (bit 11) and rgb_en (bit 12) set, plus reserved bits 23/29.
	 * Despite the reference driver's output_type-switch in rockchip_drm_vop.c looking
	 * mutually-exclusive, the actual settled state on RK3399 vop_big with
	 * a working Cadence DP path carries both bits — rgb_en here is the
	 * internal parallel-RGB output-formatter the Cadence framer consumes,
	 * not an external mux selector.
	 */
	sys_ctrl &= ~(RK_DRM_VOP_SYS_CTRL_STANDBY |
	    RK_DRM_VOP_SYS_CTRL_MMU_EN |
	    RK_DRM_VOP_SYS_CTRL_EDP_EN |
	    RK_DRM_VOP_SYS_CTRL_MIPI_EN |
	    RK_DRM_VOP_SYS_CTRL_MIPI_DUAL);
	sys_ctrl |= RK_DRM_VOP_SYS_CTRL_ENABLE |
	    RK_DRM_VOP_SYS_CTRL_RGB_EN;
	rk_drm_vop_write4(sc, 0x0008, sys_ctrl);

	/*
	 * the reference driver cdn_dp_atomic_check (cdn-dp-core.c:973) forces
	 * `ROCKCHIP_OUT_MODE_AAAA` for the DP connector regardless of
	 * pixel bus_format — the Cadence framer's input bus is 32-bit
	 * with the alpha lane unused. P888 (24-bit packed) cycles VOP
	 * output differently and breaks link sync (verified empirically
	 * 2026-05-10: trying P888 dropped LANE0_1_STATUS 0x77→0x00 and
	 * SINK_STATUS 0x01→0x00).
	 */
	dsp_ctrl0 &= ~RK_DRM_VOP_DSP_OUT_MODE_MASK;
	dsp_ctrl0 |= RK_DRM_VOP_DSP_OUT_MODE_AAAA;
	/*
	 * On RK3399 vop_big (VOP version 3.5) DSP_CTRL0 bits 4..6 are NOT
	 * pin polarity — the reference driver's `.pin_pol = VOP_REG_VER(... bits 4..6, 3, 0, 1)`
	 * (rockchip_vop_reg.c:222) restricts that field to VOP 3.0/3.1.  On
	 * 3.5, bit 5 is `p2i_en` and bits 4/6 are other functions.  DP pin
	 * polarity lives in DSP_CTRL1[16:18] (handled below).
	 *
	 * Live read of working the reference build shows DSP_CTRL0 = 0x0f — only OUT_MODE
	 * bits set; bit 7 (DCLK_POL) is also 0.  Clear all the legacy
	 * pin_pol/dclk_pol/p2i/interlace bits and don't set DCLK_POL.
	 */
	pin_pol = 0;
	if ((mode->flags & DRM_MODE_FLAG_NHSYNC) == 0)
		pin_pol |= (1u << 0);
	if ((mode->flags & DRM_MODE_FLAG_NVSYNC) == 0)
		pin_pol |= (1u << 1);
	dsp_ctrl0 &= ~(RK_DRM_VOP_DSP_CTRL0_PIN_POL_MASK |
	    RK_DRM_VOP_DSP_CTRL0_DCLK_POL |
	    RK_DRM_VOP_DSP_CTRL0_P2I_EN |
	    RK_DRM_VOP_DSP_CTRL0_INTERLACE);
	rk_drm_vop_write4(sc, 0x0010, dsp_ctrl0);

	/*
	 * DSP_CTRL1 polarities for Cadence DP path (RK3399).  Per the reference driver
	 * rockchip_vop_reg.c and rockchip_drm_vop.c:
	 *   bits 18:16 = dp_pin_pol  (bit 0 = HSYNC_POSITIVE,
	 *                             bit 1 = VSYNC_POSITIVE,
	 *                             bit 2 = DEN_NEGATIVE)
	 *   bit  19    = dp_dclk_pol (the reference driver's DP encoder_enable forces 0)
	 *   bits 20-31 are hdmi/edp/mipi pin+dclk polarity fields — clear
	 *                them so prior HDMI/eDP state can't leak through.
	 * For PHSYNC mode → set HSYNC_POSITIVE; for PVSYNC → set
	 * VSYNC_POSITIVE.  NHSYNC/NVSYNC leave the bit cleared (the reference driver
	 * rockchip_drm_vop.c:2885).
	 */
	dp_pin_pol = pin_pol;
	dsp_ctrl1 &= ~(RK_DRM_VOP_DSP_CTRL1_DP_PIN_POL_MASK |
	    RK_DRM_VOP_DSP_CTRL1_DP_DCLK_POL |
	    RK_DRM_VOP_DSP_CTRL1_HDMI_PIN_POL_MASK |
	    RK_DRM_VOP_DSP_CTRL1_HDMI_DCLK_POL);
	dsp_ctrl1 |= (dp_pin_pol & 0x7) << 16;	/* dp_dclk_pol forced low */
	/*
	 * Match the reference driver's RGB888 setup: no dither-down or pre-dither on the
	 * 32-bit AAAA Cadence path, but leave the dither selector at Allegro.
	 */
	dsp_ctrl1 &= ~((1u << 3) | (1u << 2) | (1u << 1));
	dsp_ctrl1 |= (1u << 4);
	rk_drm_vop_write4(sc, 0x0014, dsp_ctrl1);

	/*
	 * Keep the VOP->Cadence interface in plain RGB mode. the reference driver clears
	 * data-swap and post-scaler YUV output state here; stale values can
	 * yield a trained link that never produces visible pixels.
	 */
	dsp_ctrl0 &= ~(0x1fu << 12);	/* dsp_data_swap = 0 */
	rk_drm_vop_write4(sc, 0x0010, dsp_ctrl0);
	post_scl_ctrl &= ~(1u << 2);	/* dsp_out_yuv = 0 */
	rk_drm_vop_write4(sc, 0x0180, post_scl_ctrl);
	rk_drm_vop_write4(sc, 0x0018, 0x00000000);	/* dsp_background = 0 */

	rk_drm_vop_program_win0_opaque(sc, mode, hact_start, vact_start,
	    RK_DRM_WIN0_ROUTE_DP);
	rk_drm_vop_write4(sc, RK_DRM_VOP_DSP_HTOTAL_HS_END,
	    ((uint32_t)mode->htotal << 16) | rk_drm_mode_hsync_len(mode));
	rk_drm_vop_write4(sc, RK_DRM_VOP_DSP_HACT_ST_END,
	    (hact_start << 16) | (hact_start + mode->hdisplay));
	rk_drm_vop_write4(sc, RK_DRM_VOP_DSP_VTOTAL_VS_END,
	    ((uint32_t)mode->vtotal << 16) | rk_drm_mode_vsync_len(mode));
	rk_drm_vop_write4(sc, RK_DRM_VOP_DSP_VACT_ST_END,
	    (vact_start << 16) | (vact_start + mode->vdisplay));
	/*
	 * REG_CFG_DONE (offset 0x0000) is hiword-update: mask in upper 16
	 * bits, value in lower. Writing 0x1 alone (mask bit clear) is a
	 * silent no-op — the commit never fires, so all the staged VOP
	 * register writes above stay in the shadow bank and never reach
	 * the live registers. Correct form: 0x10001.
	 */
	rk_drm_vop_write4(sc, 0x0000, 0x00010001);
	if (!sc->vop_scanning) {
		rk_drm_vop_pulse_dclk_reset(sc);
		DELAY(40000);
	}
	sc->vop_scanning = true;
}

/*
 * Public DP modeset entry point. Caller must have:
 * - rk_drm attached (hw_attached = true)
 * - CDN-DP firmware framer enabled (rk_cdn_dp stages 1..19)
 *
 * Programs VOP B for the requested mode and routes its output to the
 * eDP path that CDN-DP consumes. Fills the framebuffer with a solid
 * color so we can visually confirm pixels are flowing.
 */
int
rk_drm_hw_modeset_dp(struct rk_drm_softc *sc,
    const struct drm_display_mode *mode)
{
	if (!sc->hw_attached)
		return (ENXIO);
	if (mode == NULL || !rk_drm_hw_mode_valid(mode))
		return (EINVAL);

	rk_drm_fb_fill(sc, RK_DRM_FB_BOOT_COLOR);
	rk_drm_display_domain_sanity(sc);
	rk_drm_route_vop_to_dp(sc);
	rk_drm_vop_init_mode_dp(sc, mode);
	sc->output_enabled = true;
	device_printf(sc->dev,
	    "DP modeset: VOP B -> eDP path %ux%u@%u kHz, fb_pa=0x%jx\n",
	    mode->hdisplay, mode->vdisplay, mode->clock,
	    (uintmax_t)sc->fb_pa);
	return (0);
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
	/*
	 * REG_CFG_DONE (offset 0x0000) is hiword-update: mask in upper 16
	 * bits, value in lower. Writing 0x1 alone (mask bit clear) is a
	 * silent no-op — the commit never fires, so all the staged VOP
	 * register writes above stay in the shadow bank and never reach
	 * the live registers. Correct form: 0x10001.
	 */
	rk_drm_vop_write4(sc, 0x0000, 0x00010001);
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
	    RK_DRM_VOP_SYS_CTRL_HDMI_EN |
	    RK_DRM_VOP_SYS_CTRL_EDP_EN);
	sys_ctrl |= RK_DRM_VOP_SYS_CTRL_STANDBY;
	rk_drm_vop_write4(sc, 0x0008, sys_ctrl);
	/*
	 * REG_CFG_DONE (offset 0x0000) is hiword-update: mask in upper 16
	 * bits, value in lower. Writing 0x1 alone (mask bit clear) is a
	 * silent no-op — the commit never fires, so all the staged VOP
	 * register writes above stay in the shadow bank and never reach
	 * the live registers. Correct form: 0x10001.
	 */
	rk_drm_vop_write4(sc, 0x0000, 0x00010001);

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
	sc->vop_scanning = false;
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

/*
 * rk_drm_hdmi_configure_audio
 *
 * HDMI audio block setup, called at the end of every modeset so the
 * audio path tracks the active pixel clock.  Programs the Designware
 * HDMI TX audio sub-block to receive 48 kHz / 16-bit / 2-channel I2S
 * input and wrap each sample frame in a properly-CRCed Audio Sample
 * Packet that's interleaved with the video data islands on TMDS.
 *
 * Six register groups get touched, in order:
 *
 *   1. AUD_N1/N2/N3 -- the N divider that, paired with hardware-
 *      computed CTS, produces the audio recovery clock at the sink.
 *      Spec table values vary per pixel clock; we cover the common
 *      modes (25.17/27.02/74.17/148.5 MHz) and fall back to 6144 for
 *      unknown clocks (the Designware default for 48 kHz).
 *
 *   2. AUD_CTS3 -- clears N_SHIFT/CTS_MANUAL so hardware computes CTS
 *      automatically instead of expecting software to write it.
 *
 *   3. AUD_CONF0 -- selects I2S as input source (vs SPDIF) and enables
 *      one I2S input pair (CH2 = stereo).
 *
 *   4. AUD_CONF1 -- I2S left-justified, 16-bit sample width.
 *
 *   5. AUD_INPUTCLKFS -- BCLK : LRCK ratio = 64fs (matches the I2S2
 *      master we drive from rk_i2s).
 *
 *   6. FC_AUDICONF{0..3} + FC_AUDSV -- Audio InfoFrame: CC=1 (2 ch),
 *      default speaker mapping, both subpackets valid.  This is what
 *      the sink uses to detect "audio is present" and which channels
 *      are live.
 *
 * Finally MC_CLKDIS bit 3 is cleared to ungate the audio domain.
 *
 * No PCM samples flow until the I2S2 side is driving BCLK/LRCK and a
 * userland writer is producing samples to /dev/dsp0; that path is
 * provided by rk_i2s + audio_soc + the sound framework.  This routine
 * just makes the HDMI TX side ready to accept whatever shows up on
 * its I2S input pins.
 */
static void
rk_drm_hdmi_configure_audio(struct rk_drm_softc *sc,
    const struct drm_display_mode *mode)
{
	uint32_t n;
	uint8_t val;

	switch (mode->clock) {
	case 25170:
		n = 6864;
		break;
	case 27020:
		n = 6144;
		break;
	case 74170:
		n = 11648;
		break;
	case 148350:
	case 148500:
		n = 5824;
		break;
	default:
		n = 6144;
		break;
	}

	rk_drm_hdmi_write1(sc, RK_DRM_HDMI_AUD_N1, n & 0xff);
	rk_drm_hdmi_write1(sc, RK_DRM_HDMI_AUD_N2, (n >> 8) & 0xff);
	rk_drm_hdmi_write1(sc, RK_DRM_HDMI_AUD_N3, (n >> 16) & 0xff);

	val = rk_drm_hdmi_read1(sc, RK_DRM_HDMI_AUD_CTS3);
	val &= ~(RK_DRM_HDMI_AUD_CTS3_N_SHIFT_MASK |
	    RK_DRM_HDMI_AUD_CTS3_CTS_MANUAL);
	rk_drm_hdmi_write1(sc, RK_DRM_HDMI_AUD_CTS3, val);

	val = rk_drm_hdmi_read1(sc, RK_DRM_HDMI_AUD_CONF0);
	val &= ~RK_DRM_HDMI_AUD_CONF0_INTERFACE_MASK;
	val |= RK_DRM_HDMI_AUD_CONF0_INTERFACE_IIS;
	val &= ~RK_DRM_HDMI_AUD_CONF0_I2SINEN_MASK;
	val |= RK_DRM_HDMI_AUD_CONF0_I2SINEN_CH2;
	rk_drm_hdmi_write1(sc, RK_DRM_HDMI_AUD_CONF0, val);

	val = rk_drm_hdmi_read1(sc, RK_DRM_HDMI_AUD_CONF1);
	val &= ~RK_DRM_HDMI_AUD_CONF1_DATAMODE_MASK;
	val |= RK_DRM_HDMI_AUD_CONF1_DATAMODE_IIS;
	val &= ~RK_DRM_HDMI_AUD_CONF1_DATWIDTH_MASK;
	val |= RK_DRM_HDMI_AUD_CONF1_DATWIDTH_16BIT;
	rk_drm_hdmi_write1(sc, RK_DRM_HDMI_AUD_CONF1, val);

	rk_drm_hdmi_write1(sc, RK_DRM_HDMI_AUD_INPUTCLKFS,
	    RK_DRM_HDMI_AUD_INPUTCLKFS_64);

	/* Audio InfoFrame: 2 channels, default speaker mapping */
	rk_drm_hdmi_write1(sc, RK_DRM_HDMI_FC_AUDICONF0,
	    RK_DRM_HDMI_FC_AUDICONF0_CC_2CH);
	rk_drm_hdmi_write1(sc, RK_DRM_HDMI_FC_AUDICONF1, 0);
	rk_drm_hdmi_write1(sc, RK_DRM_HDMI_FC_AUDICONF2, 0);
	rk_drm_hdmi_write1(sc, RK_DRM_HDMI_FC_AUDICONF3, 0);
	rk_drm_hdmi_write1(sc, RK_DRM_HDMI_FC_AUDSV,
	    RK_DRM_HDMI_FC_AUDSV_LR_VALID);

	/* Ungate the audio clock domain in the main controller */
	val = rk_drm_hdmi_read1(sc, RK_DRM_HDMI_MC_CLKDIS);
	val &= ~RK_DRM_HDMI_MC_CLKDIS_AUDCLK_DISABLE;
	rk_drm_hdmi_write1(sc, RK_DRM_HDMI_MC_CLKDIS, val);

	device_printf(sc->dev,
	    "HDMI audio: N=%u 48kHz I2S 2ch 16bit (no PCM source yet)\n", n);
}

/*
 * rk_drm_hw_audio_i2s_probe
 *
 * Stage 2 reconnaissance helper: transiently map the RK3399 I2S2
 * controller (0xff8a0000, the on-chip-only I2S that feeds the HDMI
 * TX's audio input) and read back its core registers.  Used to
 * decide whether the I2S2 clock gate + reset are already in a state
 * that lets us program it from rk_drm, or whether we need to do CRU
 * work first.
 *
 * Reads return all-1s if the APB clock to the I2S2 block is gated;
 * sensible values (TXCR/RXCR/CKR/XFER reset defaults) if the block
 * is reachable.  No side effects beyond the temporary VA mapping.
 *
 * Output is logged via device_printf(); caller does not need to hold
 * any locks because we touch I2S2 registers only, not shared rk_drm
 * state.
 */
void
rk_drm_hw_audio_i2s_probe(struct rk_drm_softc *sc)
{
	vm_offset_t va;
	const vm_paddr_t i2s2_pa = 0xff8a0000;
	const size_t i2s2_size = 0x1000;
	uint32_t txcr, rxcr, ckr, dmacr, intcr, intsr, xfer;

	va = (vm_offset_t)pmap_mapdev(i2s2_pa, i2s2_size);
	if (va == 0) {
		device_printf(sc->dev, "i2s2_probe: pmap_mapdev failed\n");
		return;
	}

	txcr  = *(volatile uint32_t *)(va + 0x0000);
	rxcr  = *(volatile uint32_t *)(va + 0x0004);
	ckr   = *(volatile uint32_t *)(va + 0x0008);
	dmacr = *(volatile uint32_t *)(va + 0x0010);
	intcr = *(volatile uint32_t *)(va + 0x0014);
	intsr = *(volatile uint32_t *)(va + 0x0018);
	xfer  = *(volatile uint32_t *)(va + 0x001c);

	device_printf(sc->dev,
	    "i2s2_probe: TXCR=0x%08x RXCR=0x%08x CKR=0x%08x\n",
	    txcr, rxcr, ckr);
	device_printf(sc->dev,
	    "i2s2_probe: DMACR=0x%08x INTCR=0x%08x INTSR=0x%08x XFER=0x%08x\n",
	    dmacr, intcr, intsr, xfer);
	if (txcr == 0xffffffff && ckr == 0xffffffff) {
		device_printf(sc->dev,
		    "i2s2_probe: all-1s reads -> APB clock gated, "
		    "need CRU CLK_I2S2/HCLK_I2S2 ungate before programming\n");
	} else if (txcr == 0 && ckr == 0) {
		device_printf(sc->dev,
		    "i2s2_probe: reset defaults -> APB alive, "
		    "ready to program TXCR/CKR/XFER\n");
	} else {
		device_printf(sc->dev,
		    "i2s2_probe: non-default values -> already programmed by "
		    "another driver, do not double-init\n");
	}

	pmap_unmapdev((void *)va, i2s2_size);
}

/*
 * rk_drm_hw_audio_i2s_refill
 *
 * Callout body invoked periodically while continuous-silence refill
 * is active.  Each invocation reads I2S2's current TX FIFO fill level
 * (TXFIFOLR), computes free space, writes zero samples to fill it,
 * and schedules itself to run again before the FIFO would underrun.
 *
 * Timing: at 48 kHz stereo 16-bit, two 16-bit samples pack into one
 * 32-bit FIFO word, so 32 FIFO entries = 64 samples = ~1333 us of
 * audio.  We re-arm at 500 us to stay well ahead of underrun even if
 * the system briefly defers the callout.
 *
 * Caller (sysctl handler) must serialize start/stop so this routine
 * does not race with callout teardown.
 */
/*
 * 100-entry sine table, signed 16-bit, amplitude 8000 (~ -12 dBFS).
 * At 48 kHz sample rate one cycle / 100 samples = 480 Hz test tone.
 * Kept here so the refill callout never touches floats / m_sin().
 */
static const int16_t rk_drm_sine_table[100] = {
	     0,   502,  1003,  1502,  1996,  2486,  2969,  3445,
	  3912,  4370,  4818,  5253,  5677,  6086,  6481,  6860,
	  7223,  7568,  7896,  8204,  8493,  8763,  9012,  9239,
	  9446,  9630,  9793,  9933, 10050, 10145, 10217, 10266,
	 10291, 10293, 10272, 10227, 10160, 10069,  9956,  9821,
	  9663,  9484,  9285,  9065,  8826,  8568,  8292,  7999,
	  7689,  7363,  7022,  6668,  6300,  5919,  5527,  5125,
	  4713,  4292,  3863,  3428,  2987,  2541,  2092,  1640,
	  1186,   731,   276,  -180,  -634, -1088, -1539, -1988,
	 -2433, -2873, -3308, -3736, -4156, -4569, -4972, -5365,
	 -5747, -6117, -6474, -6818, -7148, -7462, -7761, -8042,
	 -8307, -8552, -8779, -8985, -9171, -9335, -9477, -9595,
	 -9690, -9760, -9805, -9826
};

static void
rk_drm_hw_audio_i2s_refill(void *arg)
{
	struct rk_drm_softc *sc = arg;
	uint32_t level;
	int free, i;

	if (!sc->audio_refill_running || sc->i2s2_va == 0)
		return;

	level = *(volatile uint32_t *)(sc->i2s2_va + 0x000c) & 0x3f;
	free = 32 - (int)level;
	sc->audio_refill_calls++;
	if (sc->audio_sine_running) {
		for (i = 0; i < free; i++) {
			uint32_t phase = sc->audio_sine_phase % 100;
			int32_t boosted = (int32_t)rk_drm_sine_table[phase] * 3;
			if (boosted > 32760) boosted = 32760;
			else if (boosted < -32760) boosted = -32760;
			int16_t s = (int16_t)boosted;
			/* Pack stereo: same sample on L and R. */
			uint32_t word = ((uint32_t)(uint16_t)s) |
			    ((uint32_t)(uint16_t)s << 16);
			*(volatile uint32_t *)(sc->i2s2_va + 0x0024) = word;
			sc->audio_sine_phase = phase + 1;
		}
	} else {
		for (i = 0; i < free; i++)
			*(volatile uint32_t *)(sc->i2s2_va + 0x0024) = 0;
	}
	sc->audio_refill_words += (uint64_t)free;

	callout_reset_sbt(&sc->audio_refill_co, SBT_1MS / 2, 0,
	    rk_drm_hw_audio_i2s_refill, sc, 0);
}

/*
 * rk_drm_hw_audio_i2s_refill_start
 *
 * Begin continuous-silence refill of I2S2 so the HDMI TX sees a
 * stable BCLK / LRCK / SDATA stream forever (or until stop is
 * called).  Maps I2S2 once into sc->i2s2_va if not already mapped,
 * pre-seeds the FIFO, asserts XFER = TXS_START, and arms the callout.
 *
 * Idempotent: subsequent calls while already running are no-ops.
 * Returns 0 on success, ENXIO on map failure.
 */
int
rk_drm_hw_audio_i2s_refill_start(struct rk_drm_softc *sc)
{
	const vm_paddr_t i2s2_pa = 0xff8a0000;
	const size_t i2s2_size = 0x1000;
	int i;

	if (sc->audio_refill_running)
		return (0);

	if (sc->i2s2_va == 0) {
		sc->i2s2_va =
		    (vm_offset_t)pmap_mapdev(i2s2_pa, i2s2_size);
		if (sc->i2s2_va == 0) {
			device_printf(sc->dev,
			    "audio_i2s_refill_start: pmap_mapdev failed\n");
			return (ENXIO);
		}
	}

	/*
	 * Force I2S2 clock-divider + bidirectional XFER to match
	 * Armbian BSP 4.4 values that the cdn-dp audio packetizer
	 * actually accepts.  Empirically:
	 *   CKR  = 0x00033f3f   (64-fs BCLK divider)
	 *   XFER = 0x00000003   (start both TX and RX state machines)
	 * Without these, DP audio packets reach the panel but never
	 * decode.  Discovered via /dev/mem diff against working
	 * Armbian (kernel 4.4.213-rockchip64) on the same board.
	 */
	*(volatile uint32_t *)(sc->i2s2_va + 0x0008) = 0x00033f3f;
	/* Pre-seed FIFO with 32 zero samples */
	for (i = 0; i < 32; i++)
		*(volatile uint32_t *)(sc->i2s2_va + 0x0024) = 0;
	/* Start TX + RX state machines */
	*(volatile uint32_t *)(sc->i2s2_va + 0x001c) = 0x00000003;
	__asm volatile("dsb sy" ::: "memory");

	sc->audio_refill_running = true;
	callout_reset_sbt(&sc->audio_refill_co, SBT_1MS / 2, 0,
	    rk_drm_hw_audio_i2s_refill, sc, 0);
	device_printf(sc->dev,
	    "audio_i2s_refill: started (500us callout, silence)\n");
	return (0);
}

/*
 * rk_drm_hw_audio_i2s_refill_stop
 *
 * Cancel the continuous refill callout, drain any in-flight callback,
 * and clear XFER so I2S2 stops driving clocks.  Safe to call when
 * not running.
 */
void
rk_drm_hw_audio_i2s_refill_stop(struct rk_drm_softc *sc)
{
	if (!sc->audio_refill_running)
		return;
	sc->audio_refill_running = false;
	callout_drain(&sc->audio_refill_co);
	if (sc->i2s2_va != 0)
		*(volatile uint32_t *)(sc->i2s2_va + 0x001c) = 0;
	device_printf(sc->dev, "audio_i2s_refill: stopped\n");
}

/*
 * rk_drm_hw_audio_i2s_start
 *
 * Stage 2 active path: kick I2S2 into transmitting silence so the
 * HDMI TX gets real BCLK / LRCK / SDATA at its on-chip audio input.
 * Without this, the HDMI audio block packetizes against a stopped
 * I2S clock and the sink either ignores audio or reports the link
 * as silent.
 *
 * Sequence:
 *   1. Map I2S2 (0xff8a0000) transiently.
 *   2. Snapshot existing TXCR/CKR — the rk_i2s driver may have
 *      already applied a sensible 48 kHz / 16-bit / 2-channel master
 *      mode programming on attach, in which case we leave it alone.
 *      If TXCR has not been programmed at all (zero value) we apply
 *      our defaults.
 *   3. Pre-seed the TX FIFO with up to 32 zero samples (the FIFO is
 *      32 entries deep) so the first XFER cycle has data to drain.
 *   4. Write XFER = TXS_START to begin clocking.  The FIFO will
 *      empty in ~666 us at 48 kHz 16-bit stereo; clocks then stop on
 *      underrun.  This is sufficient for HDMI-side observation —
 *      the audio packetizer locks to the brief BCLK burst and the
 *      sink reports "audio detected."  Continuous clocking requires
 *      a DMA or interrupt-driven refill which is out of scope here.
 *
 * Reports the post-start XFER and INTSR values to dmesg so we can
 * see the START bit took, and whether any underrun fired during the
 * burst.
 */
void
rk_drm_hw_audio_i2s_start(struct rk_drm_softc *sc)
{
	vm_offset_t va;
	const vm_paddr_t i2s2_pa = 0xff8a0000;
	const size_t i2s2_size = 0x1000;
	uint32_t txcr, xfer_before, xfer_after, intsr;
	int i;

	va = (vm_offset_t)pmap_mapdev(i2s2_pa, i2s2_size);
	if (va == 0) {
		device_printf(sc->dev, "i2s2_start: pmap_mapdev failed\n");
		return;
	}

	txcr = *(volatile uint32_t *)(va + 0x0000);

	/* If TXCR is unprogrammed, set 16-bit I2S, 2-channel, normal IBM */
	if (txcr == 0) {
		*(volatile uint32_t *)(va + 0x0000) = 0x0000000f;
		device_printf(sc->dev,
		    "i2s2_start: TXCR was 0, programmed 0x0f (I2S 16-bit)\n");
	}
	/*
	 * CKR = 0x00033f3f matches what BSP 4.4 Armbian programs while
	 * playing audio out of cdn-dp's I2S input.  We initially used
	 * 0x00071f1f (32-fs BCLK divider) and the cdn-dp audio
	 * packetizer accepted samples without complaint but the panel
	 * speakers never decoded a thing.  Matching the slower (64-fs)
	 * Armbian BCLK divider — captured via live /dev/mem diff —
	 * unblocked audio on the XYM panel.  Always force this value
	 * even if a prior driver left a different CKR programmed.
	 */
	*(volatile uint32_t *)(va + 0x0008) = 0x00033f3f;

	/* Seed TX FIFO with 32 zero samples */
	for (i = 0; i < 32; i++)
		*(volatile uint32_t *)(va + 0x0024) = 0;

	xfer_before = *(volatile uint32_t *)(va + 0x001c);

	/*
	 * XFER = 0x3 starts BOTH TX and RX state machines.  Armbian's
	 * I2S2 driver does this; ours used to start TX only (0x1).
	 * The cdn-dp audio packetizer needs the RX-side state machine
	 * to be alive (apparently for clock-distribution reasons —
	 * undocumented, found empirically) for it to actually
	 * propagate audio packets the sink will decode.  TX-only is
	 * fine for HDMI audio but silent on the DP path.
	 */
	*(volatile uint32_t *)(va + 0x001c) = 0x00000003;
	__asm volatile("dsb sy" ::: "memory");

	/* Brief settle — XFER status reflects current state immediately */
	DELAY(100);

	xfer_after = *(volatile uint32_t *)(va + 0x001c);
	intsr      = *(volatile uint32_t *)(va + 0x0018);

	device_printf(sc->dev,
	    "i2s2_start: XFER %#x -> %#x, INTSR=%#x %s%s\n",
	    xfer_before, xfer_after, intsr,
	    (intsr & 0x2) ? "TX_UNDERRUN " : "",
	    (intsr & 0x1) ? "TX_FIFO_EMPTY" : "");

	pmap_unmapdev((void *)va, i2s2_size);
}

/*
 * rk_drm_hw_audio_dump
 *
 * Read back the HDMI TX audio block registers and print them to
 * dmesg.  Used as a diagnostic to confirm rk_drm_hdmi_configure_audio()
 * actually committed the values we wrote (registers can fail to write
 * if the HDMI APB bus is gated, the controller is in reset, or the
 * audio clock domain is off).  The dump covers the full Stage 1 path:
 *   - N divider (n1/n2/n3) and CTS3 control
 *   - Audio interface config (CONF0/CONF1/INPUTCLKFS)
 *   - Main controller clock disable mask (audclk gate state)
 *   - Audio InfoFrame fields (FC_AUDICONF0..3, FC_AUDSV)
 *
 * Caller may hold sc->hw_lock to serialize against modeset.  Safe to
 * call when the HDMI is not yet attached — returns early in that case.
 */
/*
 * Dump VOP B + relevant CRU registers to dmesg. Used to verify the modeset
 * actually programmed the hardware, distinguishing "register writes happened"
 * (should be visible in our dmesg printf logs) from "registers actually hold
 * those values now" (only direct readback proves).
 */
void
rk_drm_hw_vop_dump(struct rk_drm_softc *sc)
{
	uint32_t cfg_done, sys_ctrl, dsp_ctrl0, dsp_ctrl1, win0_ctrl0;
	uint32_t win0_ystride, win0_yrgb_mst, win0_act, win0_dsp_inf;
	uint32_t win0_dsp_st, win0_src_alpha, win0_dst_alpha;
	uint32_t htotal, hact, vtotal, vact, intr_status;

	if (!sc->hw_attached) {
		device_printf(sc->dev, "vop_dump: not attached\n");
		return;
	}

	cfg_done    = rk_drm_vop_read4(sc, 0x0000);
	sys_ctrl    = rk_drm_vop_read4(sc, 0x0008);
	dsp_ctrl0   = rk_drm_vop_read4(sc, 0x0010);
	dsp_ctrl1   = rk_drm_vop_read4(sc, 0x0014);
	win0_ctrl0  = rk_drm_vop_read4(sc, 0x0030);
	win0_ystride= rk_drm_vop_read4(sc, 0x003c);
	win0_yrgb_mst = rk_drm_vop_read4(sc, 0x0040);
	win0_act    = rk_drm_vop_read4(sc, 0x0048);
	win0_dsp_inf= rk_drm_vop_read4(sc, 0x004c);
	win0_dsp_st = rk_drm_vop_read4(sc, 0x0050);
	win0_src_alpha = rk_drm_vop_read4(sc, 0x0060);
	win0_dst_alpha = rk_drm_vop_read4(sc, 0x0064);
	htotal      = rk_drm_vop_read4(sc, 0x0188);
	hact        = rk_drm_vop_read4(sc, 0x018c);
	vtotal      = rk_drm_vop_read4(sc, 0x0190);
	vact        = rk_drm_vop_read4(sc, 0x0194);
	intr_status = rk_drm_vop_read4(sc, 0x00ac);

	device_printf(sc->dev,
	    "vop_dump: CFG_DONE=0x%08x SYS_CTRL=0x%08x DSP_CTRL0=0x%08x DSP_CTRL1=0x%08x\n",
	    cfg_done, sys_ctrl, dsp_ctrl0, dsp_ctrl1);
	device_printf(sc->dev,
	    "vop_dump: WIN0 CTRL0=0x%08x YSTRIDE=0x%08x YRGB_MST=0x%08x ACT=0x%08x DSP_INF=0x%08x DSP_ST=0x%08x SRC_ALPHA=0x%08x DST_ALPHA=0x%08x\n",
	    win0_ctrl0, win0_ystride, win0_yrgb_mst, win0_act, win0_dsp_inf,
	    win0_dsp_st, win0_src_alpha, win0_dst_alpha);
	device_printf(sc->dev,
	    "vop_dump: HTOTAL=0x%08x HACT=0x%08x VTOTAL=0x%08x VACT=0x%08x INTR_STATUS=0x%08x\n",
	    htotal, hact, vtotal, vact, intr_status);
	{
		uint32_t soc_con9, soc_con20;
		soc_con9  = rk_drm_grf_read4(sc, RK_DRM_SYS_GRF_SOC_CON9);
		soc_con20 = rk_drm_grf_read4(sc, RK_DRM_SYS_GRF_SOC_CON20);
		device_printf(sc->dev,
		    "vop_dump: GRF SOC_CON9=0x%08x (DP_SEL_VOP_LIT bit12=%u → %s),"
		    " SOC_CON20=0x%08x (EDP_LCDC_SEL bit5=%u → %s)\n",
		    soc_con9, !!(soc_con9 & RK_DRM_GRF_DP_SEL_VOP_LIT),
		    (soc_con9 & RK_DRM_GRF_DP_SEL_VOP_LIT) ? "VOP_LIT" : "VOP_BIG",
		    soc_con20, !!(soc_con20 & RK_DRM_GRF_EDP_LCDC_SEL),
		    (soc_con20 & RK_DRM_GRF_EDP_LCDC_SEL) ? "VOP_LIT" : "VOP_BIG");
	}
	{
		/*
		 * VPLL state + DCLK_VOP0 source/divisor decode.
		 * VPLL Fout = (24 MHz * FBDIV) / (REFDIV * POSTDIV1 * POSTDIV2)
		 * CON0 [11:0]  = FBDIV
		 * CON1 [5:0]   = REFDIV
		 * CON1 [10:8]  = POSTDIV1
		 * CON1 [14:12] = POSTDIV2
		 * CON2 [31]    = LOCK
		 * CON3 [9:8]   = PLL_MODE  (0=slow, 1=normal, 2=deepslow)
		 *
		 * Per the reference driver clk-rk3399.c:1267 `COMPOSITE(DCLK_VOP0_DIV, ...,
		 * RK3399_CLKSEL_CON(49), 8, 2, MFLAGS, 0, 8, DFLAGS, ...)`:
		 *   CRU CLKSEL_CON49 = offset 0x01c4
		 *     [7:0]  DCLK_VOP0_DIV  (8-bit divider, rockchip composite
		 *                            convention: actual = field+1)
		 *     [9:8]  DCLK_VOP0_SEL  (0=VPLL, 1=CPLL, 2=GPLL)
		 */
		uint32_t con0 = rk_drm_cru_read4(sc, RK_DRM_CRU_VPLL_CON0);
		uint32_t con1 = rk_drm_cru_read4(sc, RK_DRM_CRU_VPLL_CON1);
		uint32_t con2 = rk_drm_cru_read4(sc, RK_DRM_CRU_VPLL_CON2);
		uint32_t con3 = rk_drm_cru_read4(sc, RK_DRM_CRU_VPLL_CON3);
		uint32_t cksel_con47 = rk_drm_cru_read4(sc, 0x01bc);
		uint32_t cksel_con49 = rk_drm_cru_read4(sc, 0x01c4);
		uint32_t fbdiv = con0 & 0xfff;
		uint32_t refdiv = con1 & 0x3f;
		uint32_t postdiv1 = (con1 >> 8) & 0x7;
		uint32_t postdiv2 = (con1 >> 12) & 0x7;
		uint64_t fout_khz = 0;
		if (refdiv && postdiv1 && postdiv2)
			fout_khz = ((uint64_t)24000 * fbdiv) /
			    ((uint64_t)refdiv * postdiv1 * postdiv2);
		uint32_t dclk_div = (cksel_con49 & 0xff) + 1;
		uint32_t dclk_pll_sel = (cksel_con49 >> 8) & 0x3;
		static const char * const pll_names[] = {"VPLL","CPLL","GPLL","?"};
		device_printf(sc->dev,
		    "vop_dump: VPLL CON0=0x%08x CON1=0x%08x CON2=0x%08x CON3=0x%08x\n",
		    con0, con1, con2, con3);
		device_printf(sc->dev,
		    "vop_dump: VPLL fbdiv=%u refdiv=%u pd1=%u pd2=%u Fout=%llu kHz "
		    "LOCK=%u mode=%u\n",
		    fbdiv, refdiv, postdiv1, postdiv2,
		    (unsigned long long)fout_khz,
		    !!(con2 & RK_DRM_CRU_VPLL_CON2_LOCK),
		    (con3 >> 8) & 0x3);
		device_printf(sc->dev,
		    "vop_dump: CRU CLKSEL_CON47=0x%08x CLKSEL_CON49=0x%08x "
		    "DCLK_VOP0_DIV=%u SEL=%u (%s) -> dclk_vop0 ~%llu kHz\n",
		    cksel_con47, cksel_con49, dclk_div, dclk_pll_sel,
		    pll_names[dclk_pll_sel], (unsigned long long)(fout_khz / dclk_div));
	}
	device_printf(sc->dev,
	    "vop_dump: SYS_CTRL decode: ENABLE=%d STANDBY=%d RGB_EN=%d HDMI_EN=%d EDP_EN=%d MIPI_EN=%d MMU_EN=%d\n",
	    !!(sys_ctrl & RK_DRM_VOP_SYS_CTRL_ENABLE),
	    !!(sys_ctrl & RK_DRM_VOP_SYS_CTRL_STANDBY),
	    !!(sys_ctrl & RK_DRM_VOP_SYS_CTRL_RGB_EN),
	    !!(sys_ctrl & RK_DRM_VOP_SYS_CTRL_HDMI_EN),
	    !!(sys_ctrl & RK_DRM_VOP_SYS_CTRL_EDP_EN),
	    !!(sys_ctrl & RK_DRM_VOP_SYS_CTRL_MIPI_EN),
	    !!(sys_ctrl & RK_DRM_VOP_SYS_CTRL_MMU_EN));
	device_printf(sc->dev,
	    "vop_dump: WIN0 ENABLE=%d (ctrl0 bit 0)\n", win0_ctrl0 & 1);
}

void
rk_drm_hw_audio_dump(struct rk_drm_softc *sc)
{
	uint8_t n1, n2, n3, cts3, conf0, conf1, clkfs, clkdis;
	uint8_t aic0, aic1, aic2, aic3, audsv;
	uint32_t n;

	if (!sc->hw_attached || sc->hdmi_va == 0) {
		device_printf(sc->dev, "audio_dump: HDMI not attached\n");
		return;
	}

	n1    = rk_drm_hdmi_read1(sc, RK_DRM_HDMI_AUD_N1);
	n2    = rk_drm_hdmi_read1(sc, RK_DRM_HDMI_AUD_N2);
	n3    = rk_drm_hdmi_read1(sc, RK_DRM_HDMI_AUD_N3);
	cts3  = rk_drm_hdmi_read1(sc, RK_DRM_HDMI_AUD_CTS3);
	conf0 = rk_drm_hdmi_read1(sc, RK_DRM_HDMI_AUD_CONF0);
	conf1 = rk_drm_hdmi_read1(sc, RK_DRM_HDMI_AUD_CONF1);
	clkfs = rk_drm_hdmi_read1(sc, RK_DRM_HDMI_AUD_INPUTCLKFS);
	clkdis = rk_drm_hdmi_read1(sc, RK_DRM_HDMI_MC_CLKDIS);
	aic0 = rk_drm_hdmi_read1(sc, RK_DRM_HDMI_FC_AUDICONF0);
	aic1 = rk_drm_hdmi_read1(sc, RK_DRM_HDMI_FC_AUDICONF1);
	aic2 = rk_drm_hdmi_read1(sc, RK_DRM_HDMI_FC_AUDICONF2);
	aic3 = rk_drm_hdmi_read1(sc, RK_DRM_HDMI_FC_AUDICONF3);
	audsv = rk_drm_hdmi_read1(sc, RK_DRM_HDMI_FC_AUDSV);
	n = ((uint32_t)n3 << 16) | ((uint32_t)n2 << 8) | n1;

	device_printf(sc->dev,
	    "audio_dump: N=%u (n1=0x%02x n2=0x%02x n3=0x%02x) cts3=0x%02x\n",
	    n, n1, n2, n3, cts3);
	device_printf(sc->dev,
	    "audio_dump: conf0=0x%02x conf1=0x%02x inputclkfs=0x%02x clkdis=0x%02x audclk=%s\n",
	    conf0, conf1, clkfs, clkdis,
	    (clkdis & RK_DRM_HDMI_MC_CLKDIS_AUDCLK_DISABLE) ? "GATED" : "running");
	device_printf(sc->dev,
	    "audio_dump: aic0=0x%02x aic1=0x%02x aic2=0x%02x aic3=0x%02x audsv=0x%02x\n",
	    aic0, aic1, aic2, aic3, audsv);
}

/*
 * Briefly drop HDMI TMDS output: clear PDDQ (which gates the PHY data
 * lanes off) and TXPWRON, return the previous PHY_CONF0 in *prev.
 */
int
rk_drm_hw_scanout_blank(struct rk_drm_softc *sc)
{
	if (sc->vop_va == 0 || sc->fb_va == 0)
		return (ENXIO);
	/* Save the current scanout DMA address. */
	sc->dpms_yrgb_mst_save = rk_drm_vop_read4(sc, 0x0040);
	/* Zero our local stash framebuffer and point VOP at it. */
	memset((void *)sc->fb_va, 0, sc->fb_size);
	rk_drm_vop_write4(sc, 0x0040, (uint32_t)sc->fb_pa);
	rk_drm_vop_write4(sc, 0x0000, 0x00010001);	/* CFG_DONE commit */
	return (0);
}

int
rk_drm_hw_scanout_unblank(struct rk_drm_softc *sc)
{
	if (sc->vop_va == 0 || sc->dpms_yrgb_mst_save == 0)
		return (ENXIO);
	rk_drm_vop_write4(sc, 0x0040, sc->dpms_yrgb_mst_save);
	rk_drm_vop_write4(sc, 0x0000, 0x00010001);
	sc->dpms_yrgb_mst_save = 0;
	return (0);
}

int
rk_drm_hw_fb_save(struct rk_drm_softc *sc)
{
	if (sc->fb_va == 0 || sc->fb_size == 0)
		return (ENXIO);
	if (sc->fb_stash == NULL) {
		sc->fb_stash = malloc(sc->fb_size, M_DEVBUF, M_WAITOK);
		if (sc->fb_stash == NULL)
			return (ENOMEM);
	}
	memcpy(sc->fb_stash, (void *)sc->fb_va, sc->fb_size);
	return (0);
}

int
rk_drm_hw_fb_restore(struct rk_drm_softc *sc)
{
	if (sc->fb_va == 0 || sc->fb_size == 0)
		return (ENXIO);
	if (sc->fb_stash == NULL)
		return (ENOENT);
	memcpy((void *)sc->fb_va, sc->fb_stash, sc->fb_size);
	return (0);
}

void
rk_drm_hw_signal_dump(struct rk_drm_softc *sc)
{
	uint32_t big_sys, big_win, big_ht, big_va;
	uint32_t lit_sys, lit_win, lit_ht, lit_va;
	uint32_t grf_con9, grf_con20;

	if (sc->vop_va == 0) {
		device_printf(sc->dev,
		    "signal_dump: VOP_BIG not mapped\n");
		return;
	}
	big_sys = rk_drm_vop_read4(sc, 0x0008);
	big_win = rk_drm_vop_read4(sc, 0x0030);
	big_ht  = rk_drm_vop_read4(sc, RK_DRM_VOP_DSP_HTOTAL_HS_END);
	big_va  = rk_drm_vop_read4(sc, RK_DRM_VOP_DSP_VACT_ST_END);
	device_printf(sc->dev,
	    "signal_dump: VOP_BIG SYS_CTRL=%#x WIN0=%#x HTOTAL=%#x VACT=%#x\n",
	    big_sys, big_win, big_ht, big_va);
	device_printf(sc->dev,
	    "signal_dump: VOP_BIG decode ENABLE=%d STANDBY=%d RGB_EN=%d "
	    "HDMI_EN=%d EDP_EN=%d\n",
	    !!(big_sys & RK_DRM_VOP_SYS_CTRL_ENABLE),
	    !!(big_sys & RK_DRM_VOP_SYS_CTRL_STANDBY),
	    !!(big_sys & RK_DRM_VOP_SYS_CTRL_RGB_EN),
	    !!(big_sys & RK_DRM_VOP_SYS_CTRL_HDMI_EN),
	    !!(big_sys & RK_DRM_VOP_SYS_CTRL_EDP_EN));

	if (sc->vop_lit_va != 0) {
		lit_sys = rk_drm_vop_lit_read4(sc, 0x0008);
		lit_win = rk_drm_vop_lit_read4(sc, 0x0030);
		lit_ht  = rk_drm_vop_lit_read4(sc,
		    RK_DRM_VOP_DSP_HTOTAL_HS_END);
		lit_va  = rk_drm_vop_lit_read4(sc,
		    RK_DRM_VOP_DSP_VACT_ST_END);
		device_printf(sc->dev,
		    "signal_dump: VOP_LIT SYS_CTRL=%#x WIN0=%#x "
		    "HTOTAL=%#x VACT=%#x\n",
		    lit_sys, lit_win, lit_ht, lit_va);
		device_printf(sc->dev,
		    "signal_dump: VOP_LIT decode ENABLE=%d STANDBY=%d "
		    "RGB_EN=%d HDMI_EN=%d EDP_EN=%d\n",
		    !!(lit_sys & RK_DRM_VOP_SYS_CTRL_ENABLE),
		    !!(lit_sys & RK_DRM_VOP_SYS_CTRL_STANDBY),
		    !!(lit_sys & RK_DRM_VOP_SYS_CTRL_RGB_EN),
		    !!(lit_sys & RK_DRM_VOP_SYS_CTRL_HDMI_EN),
		    !!(lit_sys & RK_DRM_VOP_SYS_CTRL_EDP_EN));
	} else {
		device_printf(sc->dev,
		    "signal_dump: VOP_LIT not mapped\n");
	}

	grf_con9  = rk_drm_grf_read4(sc, RK_DRM_SYS_GRF_SOC_CON9);
	grf_con20 = rk_drm_grf_read4(sc, RK_DRM_SYS_GRF_SOC_CON20);
	device_printf(sc->dev,
	    "signal_dump: GRF SOC_CON9=%#x SOC_CON20=%#x "
	    "(DP_SEL_VOP_LIT=%d EDP_LCDC_SEL=%d HDMI_LCDC_SEL=%d)\n",
	    grf_con9, grf_con20,
	    !!(grf_con9 & RK_DRM_GRF_DP_SEL_VOP_LIT),
	    !!(grf_con20 & RK_DRM_GRF_EDP_LCDC_SEL),
	    !!(grf_con20 & RK_DRM_GRF_HDMI_LCDC_SEL));
}

uint8_t
rk_drm_hw_hdmi_phy_blank(struct rk_drm_softc *sc)
{
	uint8_t conf0, off;

	if (sc->hdmi_va == 0)
		return (0);
	conf0 = rk_drm_hdmi_read1(sc, RK_DRM_HDMI_PHY_CONF0);
	off = (uint8_t)((conf0 & ~RK_DRM_HDMI_PHY_CONF0_TXPWRON) |
	    RK_DRM_HDMI_PHY_CONF0_PDDQ);
	rk_drm_hdmi_write1(sc, RK_DRM_HDMI_PHY_CONF0, off);
	return (conf0);
}

void
rk_drm_hw_hdmi_phy_restore(struct rk_drm_softc *sc, uint8_t prev)
{
	if (sc->hdmi_va == 0)
		return;
	rk_drm_hdmi_write1(sc, RK_DRM_HDMI_PHY_CONF0, prev);
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
	rk_drm_hdmi_configure_audio(sc, mode);
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
	if (sc->vop_lit_va != 0)
		bus_space_unmap(fdtbus_bs_tag, sc->vop_lit_bsh, sc->vop_lit_size);

	sc->hdmi_va = 0;
	sc->cru_va = 0;
	sc->pmucru_va = 0;
	sc->pmu_va = 0;
	sc->grf_va = 0;
	sc->vop_va = 0;
	sc->vop_lit_va = 0;
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

	/*
	 * VOP_LIT @ 0xff8f0000 — second VOP, register layout identical to
	 * VOP_BIG.  Used by the dual-VOP coexistence path so HDMI can run
	 * on VOP_LIT while USB-C DP runs on VOP_BIG independently.
	 * Mapping unconditionally is cheap (one page) even when the
	 * tunable hw.rk_drm.dual_vop is 0.
	 */
	sc->vop_lit_pa = 0xff8f0000;
	sc->vop_lit_size = 0x10000;
	if (bus_space_map(fdtbus_bs_tag, sc->vop_lit_pa, sc->vop_lit_size, 0,
	    &sc->vop_lit_bsh) != 0) {
		error = ENXIO;
		goto fail;
	}
	sc->vop_lit_va = (vm_offset_t)sc->vop_lit_bsh;

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
