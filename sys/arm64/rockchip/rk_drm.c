/*-
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Copyright (c) 2026 Kyle Crenshaw
 * All rights reserved.
 *
 * Phase-2 DRM/KMS bounded-dynamic bring-up for RK3399 on FreeBSD's drm2 stack.
 *
 * This phase is still intentionally narrow:
 * - one HDMI-A connector
 * - one TMDS encoder
 * - one CRTC
 * - EDID-backed mode discovery
 * - runtime modes limited to the RK3399-safe clocks implemented in rk_drm_hw.c
 *
 * It is no longer just an object-model scaffold:
 * - hardware bring-up is now owned by rk_drm
 * - one internal scanout buffer is allocated and programmed into WIN0
 *
 * It is still not a full display driver:
 * - hotplug is a native HPD task, not a full IRQ-driven connector stack yet
 * - fbdev/vt is currently a helper-backed bridge over the boot scanout buffer
 *
 * The purpose of this stage is to move the Rockchip DRM bring-up into the
 * FreeBSD kernel tree so it can link against drm2 through the normal kernel
 * build, instead of trying to load as an out-of-tree KLD.
 */

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/bus.h>
#include <sys/fbio.h>
#include <sys/kernel.h>
#include <sys/malloc.h>
#include <sys/module.h>
#include <sys/pctrie.h>
#include <sys/taskqueue.h>
#include <sys/vmem.h>

#include <machine/bus.h>
#include <vm/vm.h>
#include <vm/pmap.h>
#include <vm/vm_pageout.h>
#include <vm/vm_radix.h>

#include <dev/drm2/drmP.h>
#include <dev/drm2/drm_crtc.h>
#include <dev/drm2/drm_crtc_helper.h>
#include <dev/drm2/drm_edid.h>
#include <dev/drm2/drm_fb_helper.h>
#include <dev/ofw/openfirm.h>
#include <dev/ofw/ofw_bus.h>
#include <dev/ofw/ofw_bus_subr.h>

#include "rk_drm.h"
#include "fb_if.h"

static struct ofw_compat_data rk_drm_compat_data[] = {
	{ "rockchip,display-subsystem", 1 },
	{ NULL, 0 }
};

struct rk_drm_fbdev {
	struct drm_framebuffer	drm_fb;
	struct drm_fb_helper	fb_helper;
};

struct rk_drm_bo {
	struct drm_gem_object	gem_obj;
	vm_paddr_t		pbase;
	vm_offset_t		vbase;
	size_t			npages;
	vm_page_t		*m;
	vm_object_t		cdev_pager;
};

struct rk_drm_fb {
	struct drm_framebuffer	drm_fb;
	struct rk_drm_bo	**planes;
	int			nplanes;
};

static int rk_drm_connector_add_fixed_mode(struct drm_connector *connector);
static void rk_drm_fbdev_destroy(struct rk_drm_softc *sc);
static int rk_drm_fb_get_paddr_stride(struct rk_drm_softc *sc,
    struct drm_framebuffer *drm_fb, vm_paddr_t *paddr, uint32_t *stride);
static int rk_drm_crtc_mode_set_base(struct drm_crtc *crtc, int x, int y,
    struct drm_framebuffer *old_fb);
static int rk_drm_crtc_page_flip(struct drm_crtc *crtc,
    struct drm_framebuffer *drm_fb, struct drm_pending_vblank_event *event);
static void rk_drm_hpd_task(void *arg, int pending);
static void rk_drm_vblank_task(void *arg, int pending);
static int rk_drm_crtc_index(struct drm_crtc *crtc);
static void rk_drm_cancel_page_flip(struct rk_drm_softc *sc,
    struct drm_file *file_priv);
static int rk_drm_vblank_ticks_from_mode(const struct drm_display_mode *mode);
static int rk_drm_hw_modeset_locked(struct rk_drm_softc *sc,
    const struct drm_display_mode *mode);
static void rk_drm_hw_disable_locked(struct rk_drm_softc *sc);
static int rk_drm_hw_set_scanout_locked(struct rk_drm_softc *sc,
    vm_paddr_t paddr, uint32_t stride);
static bool rk_drm_hw_hpd_locked(struct rk_drm_softc *sc);
static bool rk_drm_output_enabled_locked(struct rk_drm_softc *sc);
static void rk_drm_lastclose(struct drm_device *drm_dev);

/*
 * rk_drm_mode_fill_default
 *
 * Stamp `mode` with our 1080p60 baseline (the same clock the HDMI PHY
 * is initialized for at attach).  Used wherever we need to hand DRM a
 * mode but don't have one from EDID yet -- the initial fb_helper
 * setup, the scanout-not-ready fallback, etc.  Callers that have a
 * real EDID-bounded mode should use that instead.
 */
static void
rk_drm_mode_fill_default(struct drm_display_mode *mode)
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
	mode->type = DRM_MODE_TYPE_DRIVER | DRM_MODE_TYPE_PREFERRED;
}

/*
 * rk_drm_output_poll_changed
 *
 * DRM mode-config callback fired when the HPD poller decides the
 * output state may have changed.  We forward the event into the
 * fb_helper layer so the kernel framebuffer console resizes /
 * redraws in response to a hot-plug event without userland help.
 */
static void
rk_drm_output_poll_changed(struct drm_device *drm_dev)
{
	struct rk_drm_softc *sc;

	sc = device_get_softc(drm_dev->dev);
	if (sc->fbdev != NULL)
		drm_fb_helper_hotplug_event(&sc->fbdev->fb_helper);
}

/*
 * rk_drm_crtc_index
 *
 * Translate a drm_crtc pointer to its index in the device's CRTC
 * list.  vblank APIs are pipe-indexed (0, 1, ...) rather than crtc-
 * pointer-keyed, so we have to walk the list to convert.  Returns
 * -1 if the crtc isn't on the list (defensive; shouldn't happen).
 */
static int
rk_drm_crtc_index(struct drm_crtc *crtc)
{
	struct drm_device *drm_dev;
	struct drm_crtc *iter;
	int index;

	drm_dev = crtc->dev;
	index = 0;
	list_for_each_entry(iter, &drm_dev->mode_config.crtc_list, head) {
		if (iter == crtc)
			return (index);
		index++;
	}
	return (-1);
}

/*
 * rk_drm_vblank_ticks_from_mode
 *
 * Compute the number of system ticks (1/hz seconds) per refresh frame
 * for the given mode.  We use this to schedule the vblank task at the
 * correct cadence: ticks = hz * (htotal * vtotal) / (pixel_clock_Hz).
 * Returns at least 1 tick so the callout always advances; falls back
 * to ~16.6 ms (60 Hz) if the mode is missing or has zero clock.
 */
static int
rk_drm_vblank_ticks_from_mode(const struct drm_display_mode *mode)
{
	uint64_t numerator, denominator;

	if (mode == NULL || mode->clock <= 0 || mode->htotal <= 0 ||
	    mode->vtotal <= 0)
		return (MAX(hz / 60, 1));

	numerator = (uint64_t)hz * (uint64_t)mode->htotal *
	    (uint64_t)mode->vtotal;
	denominator = (uint64_t)mode->clock * 1000ULL;
	if (denominator == 0)
		return (MAX(hz / 60, 1));

	return ((int)MAX((numerator + denominator - 1) / denominator, 1ULL));
}

/*
 * rk_drm_hw_modeset_locked
 *
 * Thin wrapper: take sc->hw_lock around rk_drm_hw_modeset() (the
 * register-level routine), so callers in a non-locked context (sysctl
 * handlers, DRM callbacks) can safely re-program the VOP/HDMI without
 * racing concurrent register access.
 */
static int
rk_drm_hw_modeset_locked(struct rk_drm_softc *sc,
    const struct drm_display_mode *mode)
{
	int error;

	mtx_lock(&sc->hw_lock);
	error = rk_drm_hw_modeset(sc, mode);
	mtx_unlock(&sc->hw_lock);
	return (error);
}

/*
 * rk_drm_hw_modeset_dp_locked
 *
 * As above, but for the eDP/USB-C-DP path (rk_drm_hw_modeset_dp).
 * Keeps modeset_dp serialized with the rest of the HW operations.
 */
static int
rk_drm_hw_modeset_dp_locked(struct rk_drm_softc *sc,
    const struct drm_display_mode *mode)
{
	int error;

	mtx_lock(&sc->hw_lock);
	error = rk_drm_hw_modeset_dp(sc, mode);
	mtx_unlock(&sc->hw_lock);
	return (error);
}

/*
 * Sysctl handler: write 1 to dev.rk_drm.0.dp_modeset_now to drive the
 * VOP -> eDP path with a hardcoded 1080p60 timing. Caller is expected
 * to have run rk_cdn_dp stages 1..19 first so the firmware framer is
 * ready to consume pixels.
 */
static int
rk_drm_sysctl_dp_modeset_now(SYSCTL_HANDLER_ARGS)
{
	struct rk_drm_softc *sc;
	struct drm_display_mode mode;
	int error, val = 0;

	sc = arg1;
	error = sysctl_handle_int(oidp, &val, 0, req);
	if (error != 0 || req->newptr == NULL)
		return (error);
	if (val != 1)
		return (EINVAL);

	rk_drm_default_mode_fill(&mode);	/* 1920x1080@60 baseline */
	return (rk_drm_hw_modeset_dp_locked(sc, &mode));
}

/*
 * rk_drm_sysctl_audio_dump
 *
 * Sysctl handler for `dev.rk_drm.<unit>.audio_dump`.  Writing 1 to
 * the sysctl triggers a one-shot read-back of the HDMI audio block
 * registers, with the result printed to dmesg via
 * rk_drm_hw_audio_dump().  Reads return 0 with no side effect.  Any
 * value other than 1 on write returns EINVAL so we do not overload
 * the sysctl with future expansion semantics.
 *
 * Holds sc->hw_lock for the duration of the read so we do not race
 * with a concurrent modeset that may be reprogramming the same
 * registers.
 */
static int
rk_drm_sysctl_audio_dump(SYSCTL_HANDLER_ARGS)
{
	struct rk_drm_softc *sc;
	int error, val = 0;

	sc = arg1;
	error = sysctl_handle_int(oidp, &val, 0, req);
	if (error != 0 || req->newptr == NULL)
		return (error);
	if (val != 1)
		return (EINVAL);

	mtx_lock(&sc->hw_lock);
	rk_drm_hw_audio_dump(sc);
	mtx_unlock(&sc->hw_lock);
	return (0);
}

/*
 * rk_drm_sysctl_audio_i2s_probe
 *
 * Sysctl handler for `dev.rk_drm.<unit>.audio_i2s_probe`.  Stage 2
 * reconnaissance: writing 1 transiently maps the I2S2 controller and
 * reads its register file to determine whether the block is reachable
 * (clock gates ungated) or dormant.  Output goes to dmesg.
 *
 * No locking required — the probe touches I2S2 registers only, not
 * shared rk_drm state.  Used to decide whether subsequent Stage 2
 * code needs to do CRU work or can program I2S2 directly.
 */
static int
rk_drm_sysctl_audio_i2s_probe(SYSCTL_HANDLER_ARGS)
{
	struct rk_drm_softc *sc;
	int error, val = 0;

	sc = arg1;
	error = sysctl_handle_int(oidp, &val, 0, req);
	if (error != 0 || req->newptr == NULL)
		return (error);
	if (val != 1)
		return (EINVAL);

	rk_drm_hw_audio_i2s_probe(sc);
	return (0);
}

/*
 * rk_drm_sysctl_audio_refill
 *
 * Sysctl handler for `dev.rk_drm.<unit>.audio_refill`.  Writing 1
 * begins continuous-silence refill of I2S2 (a callout reseeds the TX
 * FIFO with zeros every 500us so the HDMI TX gets a stable BCLK /
 * LRCK / SDATA stream forever).  Writing 0 stops the callout and
 * clears XFER.  Reads return the current running state.
 *
 * Used to give the HDMI sink stable audio frames so it can detect
 * audio activity without us needing a full PCM source path.
 */
static int
rk_drm_sysctl_audio_refill(SYSCTL_HANDLER_ARGS)
{
	struct rk_drm_softc *sc;
	int error, val;

	sc = arg1;
	val = sc->audio_refill_running ? 1 : 0;
	error = sysctl_handle_int(oidp, &val, 0, req);
	if (error != 0 || req->newptr == NULL)
		return (error);
	if (val == 1)
		return (rk_drm_hw_audio_i2s_refill_start(sc));
	if (val == 0) {
		rk_drm_hw_audio_i2s_refill_stop(sc);
		return (0);
	}
	return (EINVAL);
}

/*
 * rk_drm_sysctl_audio_i2s_start
 *
 * Sysctl handler for `dev.rk_drm.<unit>.audio_i2s_start`.  Writing 1
 * pre-seeds the I2S2 TX FIFO with zero samples and toggles XFER to
 * begin clocking.  This drives the HDMI TX's audio input with real
 * BCLK / LRCK / SDATA for ~666 us (one FIFO depth at 48 kHz stereo
 * 16-bit) so the sink observes audio activity.  After the brief
 * burst the FIFO underruns and clocks stop — sustained playback is
 * out of scope and requires DMA refill.
 *
 * No locking required — touches I2S2 registers only.
 */
static int
rk_drm_sysctl_audio_i2s_start(SYSCTL_HANDLER_ARGS)
{
	struct rk_drm_softc *sc;
	int error, val = 0;

	sc = arg1;
	error = sysctl_handle_int(oidp, &val, 0, req);
	if (error != 0 || req->newptr == NULL)
		return (error);
	if (val != 1)
		return (EINVAL);

	rk_drm_hw_audio_i2s_start(sc);
	return (0);
}

/*
 * --- Locked-state wrappers ---------------------------------------
 *
 * Below this point are a series of small functions that all share
 * the same shape: take sc->hw_lock, read or update one piece of
 * softc state (or call a non-locked HW routine), release the lock.
 *
 * They exist so the rest of the driver -- which often runs in
 * non-locked contexts (DRM callbacks, sysctl handlers, vblank
 * tasks) -- doesn't have to remember which fields require sc->hw_lock
 * and which don't.  The "*_locked" suffix means "I take the lock for
 * you", NOT "you must hold the lock when calling me".
 */

/* Disable scanout while holding hw_lock. */
static void
rk_drm_hw_disable_locked(struct rk_drm_softc *sc)
{
	mtx_lock(&sc->hw_lock);
	rk_drm_hw_disable(sc);
	mtx_unlock(&sc->hw_lock);
}

/* Re-point VOP scanout at a new framebuffer DMA address + stride. */
static int
rk_drm_hw_set_scanout_locked(struct rk_drm_softc *sc, vm_paddr_t paddr,
    uint32_t stride)
{
	int error;

	mtx_lock(&sc->hw_lock);
	error = rk_drm_hw_set_scanout(sc, paddr, stride);
	mtx_unlock(&sc->hw_lock);
	return (error);
}

/* Sample HPD pin state under hw_lock. */
static bool
rk_drm_hw_hpd_locked(struct rk_drm_softc *sc)
{
	bool hpd;

	mtx_lock(&sc->hw_lock);
	hpd = rk_drm_hw_hpd(sc);
	mtx_unlock(&sc->hw_lock);
	return (hpd);
}

/* Read sc->output_enabled atomically (just a flag, but lock for memory order). */
static bool
rk_drm_output_enabled_locked(struct rk_drm_softc *sc)
{
	bool enabled;

	mtx_lock(&sc->hw_lock);
	enabled = sc->output_enabled;
	mtx_unlock(&sc->hw_lock);
	return (enabled);
}

/*
 * Snapshot the three HPD state fields atomically so the caller sees
 * a coherent view (e.g., not "valid && last_status" while squelch
 * just changed underneath them).  Any of the out-pointers may be NULL.
 */
static void
rk_drm_hpd_state_locked(struct rk_drm_softc *sc, bool *valid, bool *last_status,
    bool *squelch)
{
	mtx_lock(&sc->hw_lock);
	if (valid != NULL)
		*valid = sc->hpd_state_valid;
	if (last_status != NULL)
		*last_status = sc->hpd_last_status;
	if (squelch != NULL)
		*squelch = sc->hpd_squelch;
	mtx_unlock(&sc->hw_lock);
}

/* Atomic 3-tuple HPD state update -- mirror image of hpd_state_locked. */
static void
rk_drm_hpd_update_locked(struct rk_drm_softc *sc, bool valid, bool last_status,
    bool squelch)
{
	mtx_lock(&sc->hw_lock);
	sc->hpd_state_valid = valid;
	sc->hpd_last_status = last_status;
	sc->hpd_squelch = squelch;
	mtx_unlock(&sc->hw_lock);
}

/* Toggle HPD squelch (used to suppress events during deliberate disable). */
static void
rk_drm_hpd_set_squelch_locked(struct rk_drm_softc *sc, bool squelch)
{
	mtx_lock(&sc->hw_lock);
	sc->hpd_squelch = squelch;
	mtx_unlock(&sc->hw_lock);
}

/* True if the HPD poll task is currently scheduled. */
static bool
rk_drm_hpd_task_running_locked(struct rk_drm_softc *sc)
{
	bool running;

	mtx_lock(&sc->hw_lock);
	running = sc->hpd_task_running;
	mtx_unlock(&sc->hw_lock);
	return (running);
}

/* True if the vblank emulation task is currently scheduled. */
static bool
rk_drm_vblank_task_running_locked(struct rk_drm_softc *sc)
{
	bool running;

	mtx_lock(&sc->hw_lock);
	running = sc->vblank_task_running;
	mtx_unlock(&sc->hw_lock);
	return (running);
}

/*
 * Test-and-set: arm the vblank task if it isn't already.  Returns
 * true if the caller should actually enqueue (we transitioned
 * 0 -> 1), false if it was already running (idempotent enable).
 */
static bool
rk_drm_vblank_enable_locked(struct rk_drm_softc *sc)
{
	bool start;

	mtx_lock(&sc->hw_lock);
	start = !sc->vblank_task_running;
	sc->vblank_task_running = true;
	mtx_unlock(&sc->hw_lock);
	return (start);
}

/* Mark the vblank task as no longer scheduled. */
static void
rk_drm_vblank_disable_locked(struct rk_drm_softc *sc)
{
	mtx_lock(&sc->hw_lock);
	sc->vblank_task_running = false;
	mtx_unlock(&sc->hw_lock);
}

/* Initialize HPD task state at the start of polling. */
static void
rk_drm_hpd_start_locked(struct rk_drm_softc *sc, bool initial_hpd)
{
	mtx_lock(&sc->hw_lock);
	sc->hpd_task_running = true;
	sc->hpd_last_status = initial_hpd;
	sc->hpd_state_valid = true;
	sc->hpd_squelch = false;
	mtx_unlock(&sc->hw_lock);
}

/* Mark the HPD task as no longer scheduled (does not cancel the callout). */
static void
rk_drm_hpd_stop_locked(struct rk_drm_softc *sc)
{
	mtx_lock(&sc->hw_lock);
	sc->hpd_task_running = false;
	mtx_unlock(&sc->hw_lock);
}

/* Cancel a pending page flip, freeing its event and dropping vblank ref. */
static void
rk_drm_cancel_page_flip(struct rk_drm_softc *sc, struct drm_file *file_priv)
{
	struct drm_pending_vblank_event *event;
	struct drm_file *owner;
	int pipe;

	mtx_lock(&sc->drm_dev.event_lock);
	event = sc->pending_flip_event;
	if (event == NULL) {
		pipe = rk_drm_crtc_index(&sc->crtc);
		sc->pending_fb = NULL;
		if (sc->pending_flip_put && pipe >= 0)
			drm_vblank_put(&sc->drm_dev, pipe);
		sc->pending_flip_put = false;
		mtx_unlock(&sc->drm_dev.event_lock);
		return;
	}
	if (file_priv != NULL && event->base.file_priv != file_priv) {
		mtx_unlock(&sc->drm_dev.event_lock);
		return;
	}

	sc->pending_flip_event = NULL;
	sc->pending_fb = NULL;
	sc->pending_flip_put = false;
	owner = event->base.file_priv;
	pipe = event->pipe;
	if (owner != NULL) {
		owner->event_space += sizeof(event->event);
		wakeup(&owner->event_space);
	}
	mtx_unlock(&sc->drm_dev.event_lock);

	event->base.destroy(&event->base);
	drm_vblank_put(&sc->drm_dev, pipe);
}

/* Periodic taskqueue body that emulates vblank: handles pending flips, fires events, reschedules. */
static void
rk_drm_vblank_task(void *arg, int pending)
{
	struct rk_drm_softc *sc;
	struct drm_pending_vblank_event *event;
	struct drm_framebuffer *new_fb;
	vm_paddr_t paddr;
	uint32_t stride;
	bool put_vblank, running;
	int pipe, error;

	(void)pending;

	sc = arg;
	running = rk_drm_vblank_task_running_locked(sc);
	if (!running)
		return;

	pipe = rk_drm_crtc_index(&sc->crtc);
	if (pipe < 0)
		return;

	event = NULL;
	new_fb = NULL;
	put_vblank = false;

	mtx_lock(&sc->drm_dev.event_lock);
	if (sc->pending_fb != NULL) {
		new_fb = sc->pending_fb;
		sc->pending_fb = NULL;
	}
	mtx_unlock(&sc->drm_dev.event_lock);

	if (new_fb != NULL) {
		error = rk_drm_fb_get_paddr_stride(sc, new_fb, &paddr, &stride);
		if (error == 0)
			error = rk_drm_hw_set_scanout_locked(sc, paddr, stride);
		if (error == 0)
			sc->crtc.fb = new_fb;
		else {
			device_printf(sc->dev,
			    "Cannot flip scanout on vblank: %d\n", error);
			mtx_lock(&sc->drm_dev.event_lock);
			if (sc->pending_fb == NULL)
				sc->pending_fb = new_fb;
			mtx_unlock(&sc->drm_dev.event_lock);
		}
	}

	drm_handle_vblank(&sc->drm_dev, pipe);

	mtx_lock(&sc->drm_dev.event_lock);
	if (sc->pending_flip_event != NULL && sc->pending_fb == NULL) {
		event = sc->pending_flip_event;
		sc->pending_flip_event = NULL;
	}
	if (sc->pending_fb == NULL && sc->pending_flip_put) {
		put_vblank = true;
		sc->pending_flip_put = false;
	}
	if (event != NULL)
		drm_send_vblank_event(&sc->drm_dev, pipe, event);
	mtx_unlock(&sc->drm_dev.event_lock);

	if (event != NULL || put_vblank)
		drm_vblank_put(&sc->drm_dev, pipe);

	running = rk_drm_vblank_task_running_locked(sc);
	if (running)
		taskqueue_enqueue_timeout(taskqueue_thread, &sc->vblank_task,
		    sc->vblank_ticks);
}

/* DRM framebuffer .destroy hook for the boot-time "fixed" fb (no GEM ref). */
static void
rk_drm_fixedfb_destroy(struct drm_framebuffer *drm_fb)
{
	(void)drm_fb;
	drm_framebuffer_cleanup(drm_fb);
}

/* No userland export for the fixed fb -- always returns -ENODEV. */
static int
rk_drm_fixedfb_create_handle(struct drm_framebuffer *drm_fb,
    struct drm_file *file_priv, unsigned int *handle)
{
	(void)drm_fb;
	(void)file_priv;
	(void)handle;
	return (-ENODEV);
}

static const struct drm_framebuffer_funcs rk_drm_fixedfb_funcs = {
	.destroy = rk_drm_fixedfb_destroy,
	.create_handle = rk_drm_fixedfb_create_handle,
};

/* Initialize the boot fb's drm_framebuffer struct in-place (no GEM backing). */
static int
rk_drm_fixedfb_alloc(struct drm_device *drm_dev,
    struct drm_mode_fb_cmd2 *mode_cmd, struct rk_drm_fbdev *fbdev)
{
	int error;

	drm_helper_mode_fill_fb_struct(&fbdev->drm_fb, mode_cmd);
	error = drm_framebuffer_init(drm_dev, &fbdev->drm_fb,
	    &rk_drm_fixedfb_funcs);
	if (error != 0)
		device_printf(drm_dev->dev,
		    "Cannot initialize fixed DRM framebuffer: %d\n", error);
	return (error);
}

static void
rk_drm_bo_destruct(struct rk_drm_bo *bo)
{
	struct pctrie_iter pages;
	vm_page_t m;
	size_t size;
	int i;

	if (bo->cdev_pager == NULL)
		return;

	size = round_page(bo->gem_obj.size);
	if (bo->vbase != 0)
		pmap_qremove(bo->vbase, bo->npages);

	vm_page_iter_init(&pages, bo->cdev_pager);
	VM_OBJECT_WLOCK(bo->cdev_pager);
	for (i = 0; i < bo->npages; i++) {
		m = vm_radix_iter_lookup(&pages, i);
		vm_page_busy_acquire(m, 0);
		cdev_mgtdev_pager_free_page(&pages, m);
		m->flags &= ~PG_FICTITIOUS;
		vm_page_unwire_noq(m);
		vm_page_free(m);
	}
	VM_OBJECT_WUNLOCK(bo->cdev_pager);

	vm_object_deallocate(bo->cdev_pager);
	if (bo->vbase != 0)
		vmem_free(kernel_arena, bo->vbase, size);
}

static void
rk_drm_bo_free_object(struct drm_gem_object *gem_obj)
{
	struct rk_drm_bo *bo;

	bo = container_of(gem_obj, struct rk_drm_bo, gem_obj);
	drm_gem_free_mmap_offset(gem_obj);
	drm_gem_object_release(gem_obj);
	rk_drm_bo_destruct(bo);
	free(bo->m, DRM_MEM_DRIVER);
	free(bo, DRM_MEM_DRIVER);
}

static int
rk_drm_bo_alloc_contig(size_t npages, u_long alignment, vm_memattr_t memattr,
    vm_page_t **ret_page)
{
	vm_page_t m;
	int err, i, tries;
	vm_paddr_t low, high, boundary;

	low = 0;
	high = RK_DRM_FB_DMA_LOWADDR_TEST;
	boundary = 0;
	tries = 0;
retry:
	m = vm_page_alloc_noobj_contig(VM_ALLOC_WIRED | VM_ALLOC_ZERO, npages,
	    low, high, alignment, boundary, memattr);
	if (m == NULL) {
		if (tries < 3) {
			err = vm_page_reclaim_contig(0, npages, low, high,
			    alignment, boundary);
			if (err == ENOMEM)
				vm_wait(NULL);
			else if (err != 0)
				return (ENOMEM);
			tries++;
			goto retry;
		}
		return (ENOMEM);
	}

	for (i = 0; i < npages; i++, m++) {
		m->valid = VM_PAGE_BITS_ALL;
		(*ret_page)[i] = m;
	}

	return (0);
}

static int
rk_drm_bo_init_pager(struct rk_drm_bo *bo)
{
	struct pctrie_iter pages;
	vm_page_t m;
	size_t size;
	int i;

	size = round_page(bo->gem_obj.size);
	bo->pbase = VM_PAGE_TO_PHYS(bo->m[0]);
	if (vmem_alloc(kernel_arena, size, M_WAITOK | M_BESTFIT, &bo->vbase))
		return (ENOMEM);

	vm_page_iter_init(&pages, bo->cdev_pager);
	VM_OBJECT_WLOCK(bo->cdev_pager);
	for (i = 0; i < bo->npages; i++) {
		m = bo->m[i];
		m->oflags &= ~VPO_UNMANAGED;
		m->flags |= PG_FICTITIOUS;
		if (vm_page_iter_insert(m, bo->cdev_pager, i, &pages) != 0) {
			VM_OBJECT_WUNLOCK(bo->cdev_pager);
			return (EINVAL);
		}
	}
	VM_OBJECT_WUNLOCK(bo->cdev_pager);

	pmap_qenter(bo->vbase, bo->m, bo->npages);
	return (0);
}

static int
rk_drm_bo_alloc(struct drm_device *drm_dev, struct rk_drm_bo *bo)
{
	size_t size;
	int error;

	size = bo->gem_obj.size;
	bo->npages = atop(size);
	bo->m = malloc(sizeof(vm_page_t *) * bo->npages, DRM_MEM_DRIVER,
	    M_WAITOK | M_ZERO);

	error = rk_drm_bo_alloc_contig(bo->npages, PAGE_SIZE,
	    VM_MEMATTR_WRITE_COMBINING, &bo->m);
	if (error != 0) {
		device_printf(drm_dev->dev,
		    "Cannot allocate Rockchip GEM buffer: %d\n", error);
		return (error);
	}
	error = rk_drm_bo_init_pager(bo);
	if (error != 0) {
		device_printf(drm_dev->dev,
		    "Cannot initialize Rockchip GEM pager: %d\n", error);
		return (error);
	}
	return (0);
}

static int
rk_drm_bo_create(struct drm_device *drm_dev, size_t size,
    struct rk_drm_bo **res_bo)
{
	struct rk_drm_bo *bo;
	int error;

	if (size == 0)
		return (-EINVAL);

	bo = malloc(sizeof(*bo), DRM_MEM_DRIVER, M_WAITOK | M_ZERO);
	size = round_page(size);

	error = drm_gem_object_init(drm_dev, &bo->gem_obj, size);
	if (error != 0) {
		free(bo, DRM_MEM_DRIVER);
		return (error);
	}
	error = drm_gem_create_mmap_offset(&bo->gem_obj);
	if (error != 0) {
		drm_gem_object_release(&bo->gem_obj);
		free(bo, DRM_MEM_DRIVER);
		return (error);
	}

	bo->cdev_pager = cdev_pager_allocate(&bo->gem_obj, OBJT_MGTDEVICE,
	    drm_dev->driver->gem_pager_ops, size, 0, 0, NULL);
	error = rk_drm_bo_alloc(drm_dev, bo);
	if (error != 0) {
		rk_drm_bo_free_object(&bo->gem_obj);
		return (error);
	}

	*res_bo = bo;
	return (0);
}

static int
rk_drm_bo_create_with_handle(struct drm_file *file_priv,
    struct drm_device *drm_dev, size_t size, uint32_t *handle,
    struct rk_drm_bo **res_bo)
{
	struct rk_drm_bo *bo;
	int error;

	error = rk_drm_bo_create(drm_dev, size, &bo);
	if (error != 0)
		return (error);

	error = drm_gem_handle_create(file_priv, &bo->gem_obj, handle);
	if (error != 0) {
		rk_drm_bo_free_object(&bo->gem_obj);
		drm_gem_object_release(&bo->gem_obj);
		return (error);
	}

	drm_gem_object_unreference_unlocked(&bo->gem_obj);
	*res_bo = bo;
	return (0);
}

static int
rk_drm_bo_dumb_create(struct drm_file *file_priv, struct drm_device *drm_dev,
    struct drm_mode_create_dumb *args)
{
	struct rk_drm_bo *bo;

	args->pitch = (args->width * args->bpp + 7) / 8;
	args->pitch = roundup(args->pitch, 64);
	args->size = args->pitch * args->height;
	return (rk_drm_bo_create_with_handle(file_priv, drm_dev, args->size,
	    &args->handle, &bo));
}

static int
rk_drm_bo_dumb_map_offset(struct drm_file *file_priv,
    struct drm_device *drm_dev, uint32_t handle, uint64_t *offset)
{
	struct drm_gem_object *gem_obj;
	int error;

	DRM_LOCK(drm_dev);
	gem_obj = drm_gem_object_lookup(drm_dev, file_priv, handle);
	if (gem_obj == NULL) {
		DRM_UNLOCK(drm_dev);
		return (-EINVAL);
	}
	error = drm_gem_create_mmap_offset(gem_obj);
	if (error != 0) {
		drm_gem_object_unreference(gem_obj);
		DRM_UNLOCK(drm_dev);
		return (error);
	}

	*offset = DRM_GEM_MAPPING_OFF(gem_obj->map_list.key) |
	    DRM_GEM_MAPPING_KEY;
	drm_gem_object_unreference(gem_obj);
	DRM_UNLOCK(drm_dev);
	return (0);
}

static int
rk_drm_bo_dumb_destroy(struct drm_file *file_priv,
    struct drm_device *drm_dev, uint32_t handle)
{
	return (drm_gem_handle_delete(file_priv, handle));
}

static int
rk_drm_gem_pager_fault(vm_object_t vm_obj, vm_ooffset_t offset, int prot,
    vm_page_t *mres)
{
	(void)vm_obj;
	(void)offset;
	(void)prot;
	(void)mres;
	return (VM_PAGER_FAIL);
}

static int
rk_drm_gem_pager_ctor(void *handle, vm_ooffset_t size, vm_prot_t prot,
    vm_ooffset_t foff, struct ucred *cred, u_short *color)
{
	(void)handle;
	(void)size;
	(void)prot;
	(void)foff;
	(void)cred;
	if (color != NULL)
		*color = 0;
	return (0);
}

static void
rk_drm_gem_pager_dtor(void *handle)
{
	(void)handle;
}

static struct cdev_pager_ops rk_drm_gem_pager_ops = {
	.cdev_pg_fault = rk_drm_gem_pager_fault,
	.cdev_pg_ctor = rk_drm_gem_pager_ctor,
	.cdev_pg_dtor = rk_drm_gem_pager_dtor,
};

static void
rk_drm_gem_fb_destroy(struct drm_framebuffer *drm_fb)
{
	struct rk_drm_fb *fb;
	unsigned int i;

	fb = container_of(drm_fb, struct rk_drm_fb, drm_fb);
	for (i = 0; i < fb->nplanes; i++) {
		if (fb->planes[i] != NULL)
			drm_gem_object_unreference_unlocked(&fb->planes[i]->gem_obj);
	}
	drm_framebuffer_cleanup(drm_fb);
	free(fb->planes, DRM_MEM_DRIVER);
	free(fb, DRM_MEM_DRIVER);
}

static int
rk_drm_gem_fb_create_handle(struct drm_framebuffer *drm_fb,
    struct drm_file *file_priv, unsigned int *handle)
{
	struct rk_drm_fb *fb;

	fb = container_of(drm_fb, struct rk_drm_fb, drm_fb);
	return (drm_gem_handle_create(file_priv, &fb->planes[0]->gem_obj,
	    handle));
}

static const struct drm_framebuffer_funcs rk_drm_gem_fb_funcs = {
	.destroy = rk_drm_gem_fb_destroy,
	.create_handle = rk_drm_gem_fb_create_handle,
};

static int
rk_drm_gem_fb_alloc(struct drm_device *drm_dev,
    struct drm_mode_fb_cmd2 *mode_cmd, struct rk_drm_bo **planes,
    int num_planes, struct rk_drm_fb **res_fb)
{
	struct rk_drm_fb *fb;
	int i, error;

	fb = malloc(sizeof(*fb), DRM_MEM_DRIVER, M_WAITOK | M_ZERO);
	fb->planes = malloc(num_planes * sizeof(*fb->planes), DRM_MEM_DRIVER,
	    M_WAITOK | M_ZERO);
	fb->nplanes = num_planes;

	drm_helper_mode_fill_fb_struct(&fb->drm_fb, mode_cmd);
	for (i = 0; i < num_planes; i++)
		fb->planes[i] = planes[i];
	error = drm_framebuffer_init(drm_dev, &fb->drm_fb, &rk_drm_gem_fb_funcs);
	if (error != 0) {
		free(fb->planes, DRM_MEM_DRIVER);
		free(fb, DRM_MEM_DRIVER);
		device_printf(drm_dev->dev,
		    "Cannot initialize GEM framebuffer: %d\n", error);
		return (error);
	}

	*res_fb = fb;
	return (0);
}

static int
rk_drm_fb_get_paddr_stride(struct rk_drm_softc *sc,
    struct drm_framebuffer *drm_fb, vm_paddr_t *paddr, uint32_t *stride)
{
	struct rk_drm_fb *fb;

	if (drm_fb == NULL)
		return (ENXIO);
	if (sc->fbdev != NULL && drm_fb == &sc->fbdev->drm_fb) {
		*paddr = sc->fb_pa;
		*stride = sc->stride;
		return (0);
	}

	fb = container_of(drm_fb, struct rk_drm_fb, drm_fb);
	if (fb->nplanes < 1 || fb->planes[0] == NULL)
		return (ENXIO);

	*paddr = fb->planes[0]->pbase;
	*stride = fb->drm_fb.pitches[0];
	return (0);
}

static int
rk_drm_fb_probe(struct drm_fb_helper *helper,
    struct drm_fb_helper_surface_size *sizes)
{
	struct rk_drm_softc *sc;
	struct rk_drm_fbdev *fbdev;
	struct drm_mode_fb_cmd2 mode_cmd;
	struct fb_info *info;
	int error;

	if (helper->fb != NULL)
		return (0);

	sc = device_get_softc(helper->dev->dev);
	fbdev = sc->fbdev;
	if (fbdev == NULL)
		return (-ENXIO);

	info = framebuffer_alloc();
	if (info == NULL)
		return (-ENOMEM);

	memset(&mode_cmd, 0, sizeof(mode_cmd));
	mode_cmd.width = sizes->surface_width;
	mode_cmd.height = sizes->surface_height;
	mode_cmd.pitches[0] = sc->stride;
	mode_cmd.pixel_format = drm_mode_legacy_fb_format(
	    sizes->surface_bpp, sizes->surface_depth);

	error = rk_drm_fixedfb_alloc(helper->dev, &mode_cmd, fbdev);
	if (error != 0) {
		framebuffer_release(info);
		return (error);
	}

	helper->fb = &fbdev->drm_fb;
	helper->fbdev = info;

	info->fb_vbase = sc->fb_va;
	info->fb_pbase = sc->fb_pa;
	info->fb_size = sc->fb_size;
	info->fb_bpp = sizes->surface_bpp;
	info->fb_flags |= FB_FLAG_MEMATTR;
	info->fb_memattr = VM_MEMATTR_UNCACHEABLE;
	drm_fb_helper_fill_fix(info, fbdev->drm_fb.pitches[0],
	    fbdev->drm_fb.depth);
	drm_fb_helper_fill_var(info, helper, fbdev->drm_fb.width,
	    fbdev->drm_fb.height);

	return (1);
}

static struct drm_fb_helper_funcs rk_drm_fb_helper_funcs = {
	.fb_probe = rk_drm_fb_probe,
};

static struct fb_info *
rk_drm_fb_getinfo(struct drm_device *drm_dev)
{
	struct rk_drm_softc *sc;

	sc = device_get_softc(drm_dev->dev);
	if (sc->fbdev == NULL)
		return (NULL);
	return (sc->fbdev->fb_helper.fbdev);
}

static struct fb_info *
rk_drm_fb_helper_getinfo(device_t dev)
{
	struct rk_drm_softc *sc;

	sc = device_get_softc(dev);
	if (sc->fbdev == NULL)
		return (NULL);
	return (rk_drm_fb_getinfo(&sc->drm_dev));
}

static int
rk_drm_fbdev_init(struct rk_drm_softc *sc)
{
	struct rk_drm_fbdev *fbdev;
	int error;

	fbdev = malloc(sizeof(*fbdev), DRM_MEM_DRIVER, M_WAITOK | M_ZERO);
	sc->fbdev = fbdev;
	fbdev->fb_helper.funcs = &rk_drm_fb_helper_funcs;

	error = drm_fb_helper_init(&sc->drm_dev, &fbdev->fb_helper,
	    sc->drm_dev.mode_config.num_crtc,
	    sc->drm_dev.mode_config.num_connector);
	if (error != 0) {
		device_printf(sc->dev,
		    "Cannot initialize DRM fb helper: %d\n", error);
		free(fbdev, DRM_MEM_DRIVER);
		sc->fbdev = NULL;
		return (error);
	}

	error = drm_fb_helper_single_add_all_connectors(&fbdev->fb_helper);
	if (error != 0) {
		device_printf(sc->dev, "Cannot add connectors to fb helper: %d\n",
		    error);
		rk_drm_fbdev_destroy(sc);
		return (error);
	}

	error = drm_fb_helper_initial_config(&fbdev->fb_helper, RK_DRM_BPP);
	if (error != 0) {
		device_printf(sc->dev,
		    "Cannot set initial DRM fb helper config: %d\n", error);
		rk_drm_fbdev_destroy(sc);
		return (error);
	}

	return (0);
}

static void
rk_drm_fbdev_destroy(struct rk_drm_softc *sc)
{
	struct rk_drm_fbdev *fbdev;
	struct fb_info *info;

	fbdev = sc->fbdev;
	if (fbdev == NULL)
		return;

	info = fbdev->fb_helper.fbdev;
	if (info != NULL && info->fb_fbd_dev != NULL)
		device_delete_child(sc->dev, info->fb_fbd_dev);
	if (fbdev->fb_helper.fb != NULL)
		drm_framebuffer_remove(&fbdev->drm_fb);
	if (info != NULL)
		framebuffer_release(info);
	drm_fb_helper_fini(&fbdev->fb_helper);
	free(fbdev, DRM_MEM_DRIVER);
	sc->fbdev = NULL;
}

static int
rk_drm_fb_create(struct drm_device *drm_dev, struct drm_file *file_priv,
    struct drm_mode_fb_cmd2 *cmd, struct drm_framebuffer **fb)
{
	struct drm_gem_object *gem_obj;
	struct rk_drm_bo *planes[4];
	struct rk_drm_fb *rkfb;
	int hsub, vsub, nplanes;
	int width, height, size, cpp;
	int i, error;

	if (cmd->pixel_format != DRM_FORMAT_XRGB8888 &&
	    cmd->pixel_format != DRM_FORMAT_ARGB8888)
		return (-EINVAL);

	hsub = drm_format_horz_chroma_subsampling(cmd->pixel_format);
	vsub = drm_format_vert_chroma_subsampling(cmd->pixel_format);
	nplanes = drm_format_num_planes(cmd->pixel_format);
	memset(planes, 0, sizeof(planes));

	for (i = 0; i < nplanes; i++) {
		width = cmd->width;
		height = cmd->height;
		if (i != 0) {
			width /= hsub;
			height /= vsub;
		}

		gem_obj = drm_gem_object_lookup(drm_dev, file_priv,
		    cmd->handles[i]);
		if (gem_obj == NULL) {
			error = -ENXIO;
			goto fail;
		}

		cpp = drm_format_plane_cpp(cmd->pixel_format, i);
		size = (height - 1) * cmd->pitches[i] +
		    width * cpp + cmd->offsets[i];
		if (gem_obj->size < size) {
			drm_gem_object_unreference_unlocked(gem_obj);
			error = -EINVAL;
			goto fail;
		}
		planes[i] = container_of(gem_obj, struct rk_drm_bo, gem_obj);
	}

	error = rk_drm_gem_fb_alloc(drm_dev, cmd, planes, nplanes, &rkfb);
	if (error != 0)
		goto fail;

	*fb = &rkfb->drm_fb;
	return (0);

fail:
	while (i-- > 0) {
		if (planes[i] != NULL)
			drm_gem_object_unreference_unlocked(&planes[i]->gem_obj);
	}
	return (error);
}

static const struct drm_mode_config_funcs rk_drm_mode_config_funcs = {
	.fb_create = rk_drm_fb_create,
	.output_poll_changed = rk_drm_output_poll_changed,
};

static void
rk_drm_crtc_reset(struct drm_crtc *crtc)
{
}

static void
rk_drm_crtc_destroy(struct drm_crtc *crtc)
{
	drm_crtc_cleanup(crtc);
}

static int
rk_drm_crtc_set_config(struct drm_mode_set *set)
{
	return (drm_crtc_helper_set_config(set));
}

static const struct drm_crtc_funcs rk_drm_crtc_funcs = {
	.reset = rk_drm_crtc_reset,
	.destroy = rk_drm_crtc_destroy,
	.set_config = rk_drm_crtc_set_config,
	.page_flip = rk_drm_crtc_page_flip,
};

static bool
rk_drm_crtc_mode_fixup(struct drm_crtc *crtc,
    const struct drm_display_mode *mode, struct drm_display_mode *adjusted_mode)
{
	return (true);
}

static int
rk_drm_crtc_mode_set(struct drm_crtc *crtc, struct drm_display_mode *mode,
    struct drm_display_mode *adjusted_mode, int x, int y,
    struct drm_framebuffer *old_fb)
{
	struct rk_drm_softc *sc;
	const struct drm_display_mode *active_mode;
	int error;

	sc = device_get_softc(crtc->dev->dev);
	active_mode = adjusted_mode != NULL ? adjusted_mode : mode;
	error = rk_drm_hw_modeset_locked(sc, active_mode);
	if (error != 0)
		return (-error);
	sc->vblank_ticks = rk_drm_vblank_ticks_from_mode(active_mode);
	if (crtc->fb == NULL)
		return (0);
	return (rk_drm_crtc_mode_set_base(crtc, x, y, old_fb));
}

static int
rk_drm_crtc_mode_set_base(struct drm_crtc *crtc, int x, int y,
    struct drm_framebuffer *old_fb)
{
	struct rk_drm_softc *sc;
	vm_paddr_t paddr;
	uint32_t stride;
	int error;

	if (x != 0 || y != 0)
		return (-EINVAL);

	sc = device_get_softc(crtc->dev->dev);
	error = rk_drm_fb_get_paddr_stride(sc, crtc->fb, &paddr, &stride);
	if (error != 0)
		return (-error);
	error = rk_drm_hw_set_scanout_locked(sc, paddr, stride);
	if (error != 0)
		return (-error);
	return (0);
}

static int
rk_drm_crtc_page_flip(struct drm_crtc *crtc, struct drm_framebuffer *drm_fb,
    struct drm_pending_vblank_event *event)
{
	struct rk_drm_softc *sc;
	int pipe, error;

	sc = device_get_softc(crtc->dev->dev);
	pipe = rk_drm_crtc_index(crtc);
	if (pipe < 0)
		return (-ENODEV);
	if (!rk_drm_hw_hpd_locked(sc) || !rk_drm_output_enabled_locked(sc))
		return (-EBUSY);

	mtx_lock(&sc->drm_dev.event_lock);
	if (sc->pending_fb != NULL || sc->pending_flip_event != NULL) {
		mtx_unlock(&sc->drm_dev.event_lock);
		return (-EBUSY);
	}
	mtx_unlock(&sc->drm_dev.event_lock);

	if (event != NULL) {
		event->pipe = pipe;
	}
	error = drm_vblank_get(&sc->drm_dev, pipe);
	if (error != 0)
		return (error);

	mtx_lock(&sc->drm_dev.event_lock);
	if (sc->pending_fb != NULL || sc->pending_flip_event != NULL) {
		mtx_unlock(&sc->drm_dev.event_lock);
		drm_vblank_put(&sc->drm_dev, pipe);
		return (-EBUSY);
	}
	sc->pending_fb = drm_fb;
	sc->pending_flip_event = event;
	sc->pending_flip_put = true;
	mtx_unlock(&sc->drm_dev.event_lock);

	return (0);
}

static void
rk_drm_crtc_dpms(struct drm_crtc *crtc, int mode)
{
	struct rk_drm_softc *sc;
	const struct drm_display_mode *active_mode;
	struct drm_display_mode default_mode;
	int error;

	sc = device_get_softc(crtc->dev->dev);
	if (mode != DRM_MODE_DPMS_ON) {
		rk_drm_cancel_page_flip(sc, NULL);
		rk_drm_hw_disable_locked(sc);
		return;
	}
	if (rk_drm_output_enabled_locked(sc))
		return;

	active_mode = &crtc->hwmode;
	if (!rk_drm_hw_mode_valid(active_mode)) {
		active_mode = &crtc->mode;
		if (!rk_drm_hw_mode_valid(active_mode)) {
			rk_drm_mode_fill_default(&default_mode);
			active_mode = &default_mode;
		}
	}

	error = rk_drm_hw_modeset_locked(sc, active_mode);
	if (error != 0) {
		device_printf(sc->dev, "Cannot re-enable display pipe: %d\n",
		    error);
		return;
	}
	if (crtc->fb == NULL)
		return;
	error = rk_drm_crtc_mode_set_base(crtc, crtc->x, crtc->y, NULL);
	if (error != 0)
		device_printf(sc->dev, "Cannot restore scanout after DPMS on: %d\n",
		    -error);
}

static void
rk_drm_crtc_prepare(struct drm_crtc *crtc)
{
	struct rk_drm_softc *sc;
	int pipe;

	sc = device_get_softc(crtc->dev->dev);
	rk_drm_hpd_set_squelch_locked(sc, true);
	pipe = rk_drm_crtc_index(crtc);
	if (pipe >= 0)
		drm_vblank_pre_modeset(&sc->drm_dev, pipe);
	rk_drm_cancel_page_flip(sc, NULL);
	rk_drm_hw_disable_locked(sc);
}

static void
rk_drm_crtc_commit(struct drm_crtc *crtc)
{
	struct rk_drm_softc *sc;
	bool hpd;
	int pipe;

	sc = device_get_softc(crtc->dev->dev);
	pipe = rk_drm_crtc_index(crtc);
	if (pipe >= 0)
		drm_vblank_post_modeset(&sc->drm_dev, pipe);
	hpd = rk_drm_hw_hpd_locked(sc);
	rk_drm_hpd_update_locked(sc, true, hpd, false);
}

static void
rk_drm_crtc_disable(struct drm_crtc *crtc)
{
	struct rk_drm_softc *sc;

	sc = device_get_softc(crtc->dev->dev);
	rk_drm_cancel_page_flip(sc, NULL);
	rk_drm_hw_disable_locked(sc);
}

static const struct drm_crtc_helper_funcs rk_drm_crtc_helper_funcs = {
	.dpms = rk_drm_crtc_dpms,
	.prepare = rk_drm_crtc_prepare,
	.commit = rk_drm_crtc_commit,
	.mode_fixup = rk_drm_crtc_mode_fixup,
	.mode_set = rk_drm_crtc_mode_set,
	.mode_set_base = rk_drm_crtc_mode_set_base,
	.disable = rk_drm_crtc_disable,
};

static void
rk_drm_encoder_destroy(struct drm_encoder *encoder)
{
	drm_encoder_cleanup(encoder);
}

static const struct drm_encoder_funcs rk_drm_encoder_funcs = {
	.destroy = rk_drm_encoder_destroy,
};

static bool
rk_drm_encoder_mode_fixup(struct drm_encoder *encoder,
    const struct drm_display_mode *mode, struct drm_display_mode *adjusted_mode)
{
	return (true);
}

static void
rk_drm_encoder_dpms(struct drm_encoder *encoder, int mode)
{
}

static void
rk_drm_encoder_prepare(struct drm_encoder *encoder)
{
}

static void
rk_drm_encoder_commit(struct drm_encoder *encoder)
{
}

static void
rk_drm_encoder_mode_set(struct drm_encoder *encoder,
    struct drm_display_mode *mode, struct drm_display_mode *adjusted_mode)
{
}

static void
rk_drm_encoder_disable(struct drm_encoder *encoder)
{
	if (encoder->crtc != NULL)
		rk_drm_crtc_disable(encoder->crtc);
}

static const struct drm_encoder_helper_funcs rk_drm_encoder_helper_funcs = {
	.dpms = rk_drm_encoder_dpms,
	.mode_fixup = rk_drm_encoder_mode_fixup,
	.prepare = rk_drm_encoder_prepare,
	.commit = rk_drm_encoder_commit,
	.mode_set = rk_drm_encoder_mode_set,
	.disable = rk_drm_encoder_disable,
};

static enum drm_connector_status
rk_drm_connector_detect(struct drm_connector *connector, bool force)
{
	struct rk_drm_softc *sc;

	sc = device_get_softc(connector->dev->dev);
	return (rk_drm_hw_hpd_locked(sc) ? connector_status_connected :
	    connector_status_disconnected);
}

static int
rk_drm_connector_fill_modes(struct drm_connector *connector, uint32_t max_width,
    uint32_t max_height)
{
	return (drm_helper_probe_single_connector_modes(connector,
	    max_width, max_height));
}

static void
rk_drm_connector_destroy(struct drm_connector *connector)
{
	drm_connector_cleanup(connector);
}

static const struct drm_connector_funcs rk_drm_connector_funcs = {
	.dpms = drm_helper_connector_dpms,
	.detect = rk_drm_connector_detect,
	.fill_modes = rk_drm_connector_fill_modes,
	.destroy = rk_drm_connector_destroy,
};

static void
rk_drm_hpd_task(void *arg, int pending)
{
	struct rk_drm_softc *sc;
	bool hpd, changed, valid, squelch, last_status;

	(void)pending;

	sc = arg;
	if (!rk_drm_hpd_task_running_locked(sc))
		return;

	hpd = rk_drm_hw_hpd_locked(sc);
	rk_drm_hpd_state_locked(sc, &valid, &last_status, &squelch);
	changed = valid && !squelch && hpd != last_status;
	rk_drm_hpd_update_locked(sc, true, hpd, squelch);
	if (changed)
		drm_helper_hpd_irq_event(&sc->drm_dev);
	if (rk_drm_hpd_task_running_locked(sc))
		taskqueue_enqueue_timeout(taskqueue_thread, &sc->hpd_task, hz);
}

static int
rk_drm_connector_add_fixed_mode(struct drm_connector *connector)
{
	struct drm_device *drm_dev;
	struct drm_display_mode *mode;

	drm_dev = connector->dev;
	mode = drm_mode_create(drm_dev);
	if (mode == NULL)
		return (0);

	rk_drm_mode_fill_default(mode);
	drm_mode_set_name(mode);
	drm_mode_probed_add(connector, mode);

	return (1);
}

static int
rk_drm_connector_get_modes(struct drm_connector *connector)
{
	struct edid *edid;
	device_t ddc_dev;
	phandle_t node;
	pcell_t xref;
	int count;

	edid = NULL;
	ddc_dev = NULL;
	node = OF_finddevice("/hdmi@ff940000");
	if (node != -1) {
		if (OF_getencprop(node, "ddc-i2c-bus", &xref, sizeof(xref)) != -1)
			ddc_dev = OF_device_from_xref(xref);
		else if (OF_getencprop(node, "ddc", &xref, sizeof(xref)) != -1)
			ddc_dev = OF_device_from_xref(xref);
	}

	if (ddc_dev != NULL)
		edid = drm_get_edid(connector, ddc_dev);

	if (edid != NULL) {
		drm_mode_connector_update_edid_property(connector, edid);
		count = drm_add_edid_modes(connector, edid);
		drm_edid_to_eld(connector, edid);
		if (count > 0)
			return (count);
	}

	drm_mode_connector_update_edid_property(connector, NULL);
	return (rk_drm_connector_add_fixed_mode(connector));
}

static int
rk_drm_connector_mode_valid(struct drm_connector *connector,
    struct drm_display_mode *mode)
{
	(void)connector;

	if (!rk_drm_hw_mode_valid(mode))
		return (MODE_BAD);

	return (MODE_OK);
}

static struct drm_encoder *
rk_drm_connector_best_encoder(struct drm_connector *connector)
{
	struct rk_drm_softc *sc;

	sc = device_get_softc(connector->dev->dev);
	return (&sc->encoder);
}

static const struct drm_connector_helper_funcs rk_drm_connector_helper_funcs = {
	.get_modes = rk_drm_connector_get_modes,
	.mode_valid = rk_drm_connector_mode_valid,
	.best_encoder = rk_drm_connector_best_encoder,
};

static int
rk_drm_kms_init(struct rk_drm_softc *sc)
{
	struct drm_device *drm_dev;
	int error;

	drm_dev = &sc->drm_dev;

	drm_mode_config_init(drm_dev);
	drm_dev->mode_config.min_width = 640;
	drm_dev->mode_config.min_height = 480;
	drm_dev->mode_config.max_width = RK_DRM_MAX_WIDTH;
	drm_dev->mode_config.max_height = RK_DRM_MAX_HEIGHT;
	drm_dev->mode_config.funcs = &rk_drm_mode_config_funcs;

	error = drm_crtc_init(drm_dev, &sc->crtc, &rk_drm_crtc_funcs);
	if (error != 0)
		goto fail;
	drm_crtc_helper_add(&sc->crtc, &rk_drm_crtc_helper_funcs);

	error = drm_encoder_init(drm_dev, &sc->encoder, &rk_drm_encoder_funcs,
	    DRM_MODE_ENCODER_TMDS);
	if (error != 0)
		goto fail;
	drm_encoder_helper_add(&sc->encoder, &rk_drm_encoder_helper_funcs);
	sc->encoder.possible_crtcs = 0x1;

	error = drm_connector_init(drm_dev, &sc->connector,
	    &rk_drm_connector_funcs, DRM_MODE_CONNECTOR_HDMIA);
	if (error != 0)
		goto fail;
	drm_connector_helper_add(&sc->connector,
	    &rk_drm_connector_helper_funcs);
	sc->connector.interlace_allowed = false;
	sc->connector.doublescan_allowed = false;
	sc->connector.polled = DRM_CONNECTOR_POLL_HPD;
	drm_mode_connector_attach_encoder(&sc->connector, &sc->encoder);

	drm_mode_config_reset(drm_dev);

	return (0);

fail:
	drm_mode_config_cleanup(drm_dev);
	return (error);
}

static void
rk_drm_kms_fini(struct rk_drm_softc *sc)
{
	drm_mode_config_cleanup(&sc->drm_dev);
}

static void
rk_drm_preclose(struct drm_device *drm_dev, struct drm_file *file_priv)
{
	struct rk_drm_softc *sc;

	sc = device_get_softc(drm_dev->dev);
	rk_drm_cancel_page_flip(sc, file_priv);
}

static void
rk_drm_lastclose(struct drm_device *drm_dev)
{
	struct rk_drm_softc *sc;

	sc = device_get_softc(drm_dev->dev);
	if (sc->fbdev != NULL)
		drm_fb_helper_restore_fbdev_mode(&sc->fbdev->fb_helper);
}

static int
rk_drm_enable_vblank(struct drm_device *drm_dev, int pipe)
{
	struct rk_drm_softc *sc;
	bool start;

	sc = device_get_softc(drm_dev->dev);
	if (pipe != rk_drm_crtc_index(&sc->crtc))
		return (-ENODEV);
	start = rk_drm_vblank_enable_locked(sc);
	if (start)
		taskqueue_enqueue_timeout(taskqueue_thread, &sc->vblank_task, 1);
	return (0);
}

static void
rk_drm_disable_vblank(struct drm_device *drm_dev, int pipe)
{
	struct rk_drm_softc *sc;

	sc = device_get_softc(drm_dev->dev);
	if (pipe != rk_drm_crtc_index(&sc->crtc))
		return;
	rk_drm_vblank_disable_locked(sc);
}

static int
rk_drm_drm_load(struct drm_device *drm_dev, unsigned long flags)
{
	struct rk_drm_softc *sc;
	int error;

	sc = device_get_softc(drm_dev->dev);
	error = rk_drm_kms_init(sc);
	if (error != 0)
		return (error);

	drm_dev->irq_enabled = true;
	drm_dev->max_vblank_count = 0xffffffff;
	drm_dev->vblank_disable_allowed = true;
	error = drm_vblank_init(drm_dev, drm_dev->mode_config.num_crtc);
	if (error != 0) {
		rk_drm_kms_fini(sc);
		return (error);
	}
	sc->vblank_ticks = rk_drm_vblank_ticks_from_mode(&sc->crtc.hwmode);

	error = rk_drm_fbdev_init(sc);
	if (error != 0) {
		drm_vblank_cleanup(drm_dev);
		rk_drm_kms_fini(sc);
		return (error);
	}

	drm_kms_helper_poll_init(drm_dev);
	TIMEOUT_TASK_INIT(taskqueue_thread, &sc->vblank_task, 0,
	    rk_drm_vblank_task, sc);
	TIMEOUT_TASK_INIT(taskqueue_thread, &sc->hpd_task, 0, rk_drm_hpd_task,
	    sc);
	callout_init(&sc->audio_refill_co, 1);
	rk_drm_hpd_start_locked(sc, rk_drm_hw_hpd_locked(sc));
	taskqueue_enqueue_timeout(taskqueue_thread, &sc->hpd_task, hz);
	return (0);
}

static int
rk_drm_drm_unload(struct drm_device *drm_dev)
{
	struct rk_drm_softc *sc;

	sc = device_get_softc(drm_dev->dev);
	rk_drm_vblank_disable_locked(sc);
	taskqueue_cancel_timeout(taskqueue_thread, &sc->vblank_task, NULL);
	taskqueue_drain_timeout(taskqueue_thread, &sc->vblank_task);
	rk_drm_cancel_page_flip(sc, NULL);
	rk_drm_hpd_stop_locked(sc);
	taskqueue_cancel_timeout(taskqueue_thread, &sc->hpd_task, NULL);
	taskqueue_drain_timeout(taskqueue_thread, &sc->hpd_task);
	drm_kms_helper_poll_fini(drm_dev);
	rk_drm_fbdev_destroy(sc);
	drm_vblank_cleanup(drm_dev);
	rk_drm_kms_fini(sc);
	return (0);
}

static struct drm_ioctl_desc rk_drm_ioctls[] = {
};

static struct drm_driver rk_drm_driver = {
	.driver_features = DRIVER_MODESET | DRIVER_GEM,
	.load = rk_drm_drm_load,
	.unload = rk_drm_drm_unload,
	.preclose = rk_drm_preclose,
	.lastclose = rk_drm_lastclose,
	.get_vblank_counter = drm_vblank_count,
	.enable_vblank = rk_drm_enable_vblank,
	.disable_vblank = rk_drm_disable_vblank,
	.gem_free_object = rk_drm_bo_free_object,
	.gem_pager_ops = &rk_drm_gem_pager_ops,
	.dumb_create = rk_drm_bo_dumb_create,
	.dumb_map_offset = rk_drm_bo_dumb_map_offset,
	.dumb_destroy = rk_drm_bo_dumb_destroy,
	.ioctls = rk_drm_ioctls,
	.num_ioctls = nitems(rk_drm_ioctls),
	.name = RK_DRM_DRIVER_NAME,
	.desc = RK_DRM_DRIVER_DESC,
	.date = RK_DRM_DRIVER_DATE,
	.major = RK_DRM_DRIVER_MAJOR,
	.minor = RK_DRM_DRIVER_MINOR,
	.patchlevel = RK_DRM_DRIVER_PATCHLEVEL,
};

static int
rk_drm_probe(device_t dev)
{
	if (!ofw_bus_status_okay(dev))
		return (ENXIO);
	if (ofw_bus_search_compatible(dev, rk_drm_compat_data)->ocd_data == 0)
		return (ENXIO);

	device_set_desc(dev, RK_DRM_DRIVER_DESC);
	return (BUS_PROBE_DEFAULT);
}

static int
rk_drm_attach(device_t dev)
{
	struct rk_drm_softc *sc;
	int error;

	sc = device_get_softc(dev);
	bzero(sc, sizeof(*sc));
	sc->dev = dev;
	mtx_init(&sc->hw_lock, "rk_drm hw", NULL, MTX_DEF);

	error = rk_drm_hw_attach(sc);
	if (error != 0) {
		device_printf(dev, "hardware attach failed: %d\n", error);
		mtx_destroy(&sc->hw_lock);
		return (error);
	}

	bzero(&sc->drm_dev, sizeof(sc->drm_dev));
	sc->drm_dev.dev = dev;
	sc->drm_dev.dev_private = sc;

	error = drm_get_platform_dev(dev, &sc->drm_dev, &rk_drm_driver);
	if (error != 0) {
		device_printf(dev, "drm_get_platform_dev failed: %d\n", error);
		rk_drm_hw_detach(sc);
		mtx_destroy(&sc->hw_lock);
		return (error);
	}

	sc->drm_registered = true;
	device_printf(dev, "registered DRM device with EDID-bounded modeset\n");

	{
		struct sysctl_ctx_list *ctx = device_get_sysctl_ctx(dev);
		struct sysctl_oid *tree = device_get_sysctl_tree(dev);

		SYSCTL_ADD_PROC(ctx, SYSCTL_CHILDREN(tree), OID_AUTO,
		    "dp_modeset_now",
		    CTLTYPE_INT | CTLFLAG_RW | CTLFLAG_MPSAFE,
		    sc, 0, rk_drm_sysctl_dp_modeset_now, "I",
		    "Write 1 to drive VOP -> eDP path with 1080p60 (run rk_cdn_dp stages 1..19 first)");

		SYSCTL_ADD_PROC(ctx, SYSCTL_CHILDREN(tree), OID_AUTO,
		    "audio_dump",
		    CTLTYPE_INT | CTLFLAG_RW | CTLFLAG_MPSAFE,
		    sc, 0, rk_drm_sysctl_audio_dump, "I",
		    "Write 1 to dump HDMI audio register state to dmesg");

		SYSCTL_ADD_PROC(ctx, SYSCTL_CHILDREN(tree), OID_AUTO,
		    "audio_i2s_probe",
		    CTLTYPE_INT | CTLFLAG_RW | CTLFLAG_MPSAFE,
		    sc, 0, rk_drm_sysctl_audio_i2s_probe, "I",
		    "Write 1 to probe I2S2 (HDMI audio source) register state");

		SYSCTL_ADD_PROC(ctx, SYSCTL_CHILDREN(tree), OID_AUTO,
		    "audio_i2s_start",
		    CTLTYPE_INT | CTLFLAG_RW | CTLFLAG_MPSAFE,
		    sc, 0, rk_drm_sysctl_audio_i2s_start, "I",
		    "Write 1 to seed I2S2 FIFO and start TX (brief silence burst)");

		SYSCTL_ADD_PROC(ctx, SYSCTL_CHILDREN(tree), OID_AUTO,
		    "audio_refill",
		    CTLTYPE_INT | CTLFLAG_RW | CTLFLAG_MPSAFE,
		    sc, 0, rk_drm_sysctl_audio_refill, "I",
		    "Write 1 to start continuous I2S2 silence refill, 0 to stop");
	}

	return (0);
}

static int
rk_drm_detach(device_t dev)
{
	struct rk_drm_softc *sc;

	sc = device_get_softc(dev);
	if (sc->drm_registered) {
		drm_put_dev(&sc->drm_dev);
		sc->drm_registered = false;
	}
	rk_drm_hw_audio_i2s_refill_stop(sc);
	rk_drm_hw_detach(sc);
	mtx_destroy(&sc->hw_lock);
	return (0);
}

static device_method_t rk_drm_methods[] = {
	DEVMETHOD(device_probe,		rk_drm_probe),
	DEVMETHOD(device_attach,	rk_drm_attach),
	DEVMETHOD(device_detach,	rk_drm_detach),
	DEVMETHOD(fb_getinfo,		rk_drm_fb_helper_getinfo),
	DEVMETHOD_END
};

static driver_t rk_drm_driver_kobj = {
	RK_DRM_DRIVER_NAME,
	rk_drm_methods,
	sizeof(struct rk_drm_softc),
};

DRIVER_MODULE(rk_drm, simplebus, rk_drm_driver_kobj, 0, 0);
DRIVER_MODULE(rk_drm, ofwbus, rk_drm_driver_kobj, 0, 0);
extern driver_t fbd_driver;
DRIVER_MODULE(fbd, rk_drm, fbd_driver, 0, 0);
MODULE_VERSION(rk_drm, 1);
