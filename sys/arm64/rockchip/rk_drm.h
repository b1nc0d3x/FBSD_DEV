/*-
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Copyright (c) 2026 Kyle Crenshaw
 * All rights reserved.
 */

#ifndef _ARM64_ROCKCHIP_RK_DRM_H_
#define _ARM64_ROCKCHIP_RK_DRM_H_

#include <sys/mutex.h>
#include <sys/taskqueue.h>
#include <sys/callout.h>

#define RK_DRM_DRIVER_NAME        "rk_drm"
#define RK_DRM_DRIVER_DESC        "RK3399 DRM/KMS EDID-bounded modeset"
#define RK_DRM_DRIVER_DATE        "20260419"
#define RK_DRM_DRIVER_MAJOR       0
#define RK_DRM_DRIVER_MINOR       3
#define RK_DRM_DRIVER_PATCHLEVEL  0

/*
 * The initial hardware bring-up still boots in the known-good 1080p60 mode,
 * but runtime modeset is now driven from EDID for the subset of modes whose
 * clocks and HDMI PHY settings are implemented in rk_drm_hw.c.
 */
#define RK_DRM_DEFAULT_CLOCK_KHZ     148500
#define RK_DRM_DEFAULT_WIDTH         1920
#define RK_DRM_DEFAULT_HEIGHT        1080
/*
 * Hardcoded XYM-display EDID DTD timing (test 2026-05-10).
 * Cadence framer's stage-18 config-video parses these same values from
 * EDID and writes them into the MSA. If VOP supplies different
 * hsync/vsync placement the panel sees inconsistent stream-vs-MSA and
 * reports "No Signal" even with a healthy link.
 *   CEA 1080p60 (was):  HSYNC 2008..2052 (pulse 44), VSYNC 1084..1089
 *   XYM panel DTD:      HSYNC 1968..2000 (pulse 32), VSYNC 1083..1088
 * Long term: derive from cached EDID DTD via rk_cdn_dp_get_cached_edid.
 */
#define RK_DRM_DEFAULT_HSYNC_START   1968
#define RK_DRM_DEFAULT_HSYNC_END     2000
#define RK_DRM_DEFAULT_HTOTAL        2200
#define RK_DRM_DEFAULT_VSYNC_START   1083
#define RK_DRM_DEFAULT_VSYNC_END     1088
#define RK_DRM_DEFAULT_VTOTAL        1125

#define RK_DRM_MAX_WIDTH             1920
#define RK_DRM_MAX_HEIGHT            1080

#define RK_DRM_BPP                32
#define RK_DRM_FB_BOOT_COLOR      0xff202040u
#define RK_DRM_FB_DMA_LOWADDR_TEST 0x0fffffffu
#define RK_DRM_OUTPUT_AUTO        0
#define RK_DRM_OUTPUT_HDMI        1
#define RK_DRM_OUTPUT_USBC_DP     2

struct drm_display_mode;
struct rk_drm_fbdev;

/*
 * Per-instance softc.  One of these exists for each rk_drm device
 * (only one on RockPro64, since there's a single VOP+HDMI pipeline).
 *
 * Field grouping:
 *   - DRM/KMS objects (drm_dev, crtc, encoder, connector, fbdev) -- the
 *     handles we register with FreeBSD's drm2 stack.
 *   - Lifecycle flags -- whether DRM is registered, whether HW is up,
 *     whether the output is currently scanning out, etc.
 *   - HPD/vblank tasking state for the timeout_task callbacks.
 *   - Pending atomic-flip state (pending_fb, pending_flip_event) for
 *     async page flips coordinated with vblank.
 *   - Framebuffer DMA bookkeeping -- the boot framebuffer is allocated
 *     once at attach and reused; fb_va/fb_pa/fb_size describe it.
 *   - Audio refill state (callout + I2S2 mapping) used when the
 *     stage-2 silence-refill sysctl is enabled (superseded by the
 *     audio_soc + rk_i2s path; kept for diagnostics).
 *   - Block MMIO mappings for VOP, GRF, PMU, PMUCRU, CRU, HDMI -- both
 *     bus_space handles (legacy bus accessors) and raw VAs (used by
 *     the volatile-pointer fast path) plus PA/size for unmap.
 */
struct rk_drm_softc {
	device_t		dev;
	struct drm_device	drm_dev;
	struct drm_crtc		crtc;
	struct drm_encoder	encoder;
	struct drm_connector	connector;
	struct rk_drm_fbdev	*fbdev;
	struct mtx		hw_lock;
	bool			drm_registered;
	bool			hw_attached;
	bool			output_enabled;
	bool			hpd_task_running;
	bool			vblank_task_running;
	bool			hpd_state_valid;
	bool			hpd_last_status;
	bool			hpd_squelch;
	int			output_select;
	bool			dp_autobring_done;
	bool			dp_hotplug_reprobe_done;
	uint32_t		dp_last_altmode_status;	/* for HPD_IRQ rising-edge */
	bool			pending_flip_put;
	int			vblank_ticks;
	struct drm_framebuffer	*pending_fb;
	struct drm_pending_vblank_event *pending_flip_event;
	struct timeout_task	vblank_task;
	struct timeout_task	hpd_task;

	bus_dma_tag_t		fb_dma_tag;
	bus_dmamap_t		fb_dma_map;
	vm_offset_t		fb_va;
	vm_paddr_t		fb_pa;
	size_t			fb_size;
	uint32_t		stride;

	/*
	 * Stage-2 silence refill: callout reseeds I2S2's TX FIFO with
	 * zeros every 500 us so the HDMI TX gets stable BCLK/LRCK even
	 * when no userland audio source is feeding data.  Largely
	 * obsolete now that audio_soc + rk_i2s drive I2S2 properly,
	 * but kept available behind the audio_refill sysctl for
	 * standalone HDMI-audio diagnostics.
	 */
	struct callout		audio_refill_co;
	vm_offset_t		i2s2_va;
	bool			audio_refill_running;

	bus_space_handle_t	vop_bsh;
	bus_space_handle_t	grf_bsh;
	bus_space_handle_t	pmu_bsh;
	bus_space_handle_t	pmucru_bsh;
	bus_space_handle_t	cru_bsh;
	vm_offset_t		vop_va;
	vm_offset_t		grf_va;
	vm_offset_t		pmu_va;
	vm_offset_t		pmucru_va;
	vm_offset_t		cru_va;
	vm_offset_t		hdmi_va;
	vm_paddr_t		vop_pa;
	vm_paddr_t		grf_pa;
	vm_paddr_t		pmu_pa;
	vm_paddr_t		pmucru_pa;
	vm_paddr_t		cru_pa;
	vm_paddr_t		hdmi_pa;
	size_t			vop_size;
	size_t			grf_size;
	size_t			pmu_size;
	size_t			pmucru_size;
	size_t			cru_size;
	size_t			hdmi_size;
};

/*
 * rk_drm_hw_* -- hardware-side entry points exported by rk_drm_hw.c.
 *
 * These are the only symbols rk_drm.c (the DRM/KMS glue) is allowed
 * to call into the hardware layer.  Everything below this line is
 * implemented in rk_drm_hw.c; the split keeps DRM-framework code free
 * of register pokes and keeps register-level code free of DRM types.
 */

/* Lifecycle: map registers, allocate framebuffer, leave HW idle. */
int	rk_drm_hw_attach(struct rk_drm_softc *sc);

/* Lifecycle: tear down everything attach allocated. */
void	rk_drm_hw_detach(struct rk_drm_softc *sc);

/* Mode validation -- true if rk_drm_hw.c has clocks/PHY for this mode. */
bool	rk_drm_hw_mode_valid(const struct drm_display_mode *mode);

/* Drive VOP -> HDMI for the given mode (the standard HDMI output path). */
int	rk_drm_hw_modeset(struct rk_drm_softc *sc,
	    const struct drm_display_mode *mode);

/* Drive VOP -> eDP/DP for the given mode (USB-C DP-altmode path). */
int	rk_drm_hw_modeset_dp(struct rk_drm_softc *sc,
	    const struct drm_display_mode *mode);

/* Fill `mode` with our 1080p60 baseline -- used as the default at boot. */
void	rk_drm_default_mode_fill(struct drm_display_mode *mode);

/* Stop scanout (blanks the output but keeps clocks ready for re-enable). */
void	rk_drm_hw_disable(struct rk_drm_softc *sc);

/* Point the VOP scanout at a new framebuffer DMA address + stride. */
int	rk_drm_hw_set_scanout(struct rk_drm_softc *sc, vm_paddr_t paddr,
	    uint32_t stride);

/* Sample HDMI hot-plug-detect status (true = sink connected). */
bool	rk_drm_hw_hpd(struct rk_drm_softc *sc);

/* HDMI audio diagnostics -- dump the relevant TX register state to dmesg. */
void	rk_drm_hw_audio_dump(struct rk_drm_softc *sc);
void	rk_drm_hw_vop_dump(struct rk_drm_softc *sc);

/* I2S2 controller register dump (for verifying clock/reset state). */
void	rk_drm_hw_audio_i2s_probe(struct rk_drm_softc *sc);

/* One-shot: fill I2S2 FIFO with silence and assert XFER (brief burst). */
void	rk_drm_hw_audio_i2s_start(struct rk_drm_softc *sc);

/* Continuous-silence refill (callout-driven).  Returns 0 or errno. */
int	rk_drm_hw_audio_i2s_refill_start(struct rk_drm_softc *sc);

/* Stop the continuous refill callout and clear XFER. */
void	rk_drm_hw_audio_i2s_refill_stop(struct rk_drm_softc *sc);

#endif /* _ARM64_ROCKCHIP_RK_DRM_H_ */
