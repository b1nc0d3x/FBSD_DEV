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
#define RK_DRM_DEFAULT_HSYNC_START   2008
#define RK_DRM_DEFAULT_HSYNC_END     2052
#define RK_DRM_DEFAULT_HTOTAL        2200
#define RK_DRM_DEFAULT_VSYNC_START   1084
#define RK_DRM_DEFAULT_VSYNC_END     1089
#define RK_DRM_DEFAULT_VTOTAL        1125

#define RK_DRM_MAX_WIDTH             1920
#define RK_DRM_MAX_HEIGHT            1080

#define RK_DRM_BPP                32
#define RK_DRM_FB_BOOT_COLOR      0xff202040u
#define RK_DRM_FB_DMA_LOWADDR_TEST 0x0fffffffu

struct drm_display_mode;
struct rk_drm_fbdev;

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

int	rk_drm_hw_attach(struct rk_drm_softc *sc);
void	rk_drm_hw_detach(struct rk_drm_softc *sc);
bool	rk_drm_hw_mode_valid(const struct drm_display_mode *mode);
int	rk_drm_hw_modeset(struct rk_drm_softc *sc,
	    const struct drm_display_mode *mode);
int	rk_drm_hw_modeset_dp(struct rk_drm_softc *sc,
	    const struct drm_display_mode *mode);
void	rk_drm_default_mode_fill(struct drm_display_mode *mode);
void	rk_drm_hw_disable(struct rk_drm_softc *sc);
int	rk_drm_hw_set_scanout(struct rk_drm_softc *sc, vm_paddr_t paddr,
	    uint32_t stride);
bool	rk_drm_hw_hpd(struct rk_drm_softc *sc);
void	rk_drm_hw_audio_dump(struct rk_drm_softc *sc);
void	rk_drm_hw_audio_i2s_probe(struct rk_drm_softc *sc);
void	rk_drm_hw_audio_i2s_start(struct rk_drm_softc *sc);
int	rk_drm_hw_audio_i2s_refill_start(struct rk_drm_softc *sc);
void	rk_drm_hw_audio_i2s_refill_stop(struct rk_drm_softc *sc);

#endif /* _ARM64_ROCKCHIP_RK_DRM_H_ */
