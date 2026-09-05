/*-
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Copyright (c) 2026 Kyle Crenshaw <b1nc0d3x@gmail.com>
 *
 * Native kms(4) consumer for the paravirtual VirtIO GPU device.
 *
 * Sits alongside rk_kms (Rockchip VOP) and igen (Haswell iGPU) in
 * sys/dev/kms/drivers/.  Depends on kms.ko for the framework core;
 * the framework itself never learns about virtio-gpu.
 *
 * The in-base virtio_gpu(4) driver owns the vt(4) console framebuffer
 * (fbio path).  This driver binds at BUS_PROBE_VENDOR so it wins the
 * device_t attach, then hands the fb interface back to virtio_gpu's
 * fbio machinery via a firstopen/lastclose handover — one device_t,
 * two consumers, no fights.
 *
 * Phase A (this commit): softc + probe/attach/detach + drm_driver
 * registration only.  Real virtio_gpu protocol wiring, mode-config
 * topology (CRTC/plane/encoder/connector), GEM/framebuffer and
 * console handover land in Phase B..F.
 */

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/bus.h>
#include <sys/endian.h>
#include <sys/kernel.h>
#include <sys/malloc.h>
#include <sys/module.h>
#include <sys/queue.h>
#include <sys/sglist.h>
#include <sys/sx.h>
#include <sys/sysctl.h>
#include <sys/taskqueue.h>
#include <sys/callout.h>
#include <sys/fbio.h>
#include <sys/interrupt.h>
#include <sys/proc.h>
#include <sys/signalvar.h>

#include <dev/vt/vt.h>
#include <dev/vt/hw/fb/vt_fb.h>

#include <machine/bus.h>
#include <machine/resource.h>

#include <vm/vm.h>
#include <vm/vm_param.h>
#include <vm/vm_page.h>

#include <dev/virtio/virtio.h>
#include <dev/virtio/virtqueue.h>
#include <dev/virtio/gpu/virtio_gpu.h>

#include <kms/drm_atomic.h>
#include <kms/drm_connector.h>
#include <kms/drm_crtc.h>
#include <kms/drm_device.h>
#include <kms/drm_drv.h>
#include <kms/drm_encoder.h>
#include <kms/drm_framebuffer.h>
#include <kms/drm_gem.h>
#include <kms/drm_mode_config.h>
#include <kms/drm_modes.h>
#include <kms/drm_plane.h>

#include <drm/drm_fourcc.h>
#include <drm/drm_mode.h>

#include "virtio_if.h"

/*
 * Phase B protocol constants.  VIRTIO_GPU_MAX_SCANOUTS is 16 in the
 * spec (hardcoded, not negotiated); we cap our per-instance array at
 * that so a malicious host can't overflow us if the config lies about
 * num_scanouts.  DISPLAY_INFO fetch happens once at attach and once
 * per hotplug event; the cached result feeds Phase C connector-mode
 * enumeration.
 */
#define	VIRTIO_KMS_MAX_SCANOUTS		VIRTIO_GPU_MAX_SCANOUTS

/*
 * Cached per-scanout state.  DISPLAY_INFO fills geometry + enabled;
 * GET_EDID (Phase F, opt-in via VIRTIO_GPU_F_EDID) fills the EDID
 * blob and length so connector.get_modes can advertise the host's
 * real preferred timing.
 */
struct virtio_kms_scanout {
	uint32_t	enabled;
	uint32_t	width;
	uint32_t	height;
	uint32_t	x;
	uint32_t	y;
	uint32_t	flags;		/* pmode.flags from resp */

	/*
	 * EDID cache, populated lazily.  `edid_tried` is a one-shot
	 * gate so a get_modes storm on a host that either refuses
	 * GET_EDID or hands us zero-length blobs doesn't hammer the
	 * ctrl queue on every GETCONNECTOR ioctl.  Cleared by hotplug
	 * so a re-plugged monitor gets re-probed.  Review #22.
	 */
	uint32_t	edid_len;
	bool		edid_tried;
	uint8_t		edid[1024];
};

#define	VIRTIO_KMS_DRIVER_NAME	"virtio_kms"
#define	VIRTIO_KMS_DRIVER_DESC	"VirtIO GPU (kms)"
#define	VIRTIO_KMS_DRIVER_DATE	"20260804"

MALLOC_DEFINE(M_VIRTIO_KMS, "virtio_kms", "VirtIO KMS driver");
MALLOC_DECLARE(M_KMS);	/* framework allocator, used by fb_destroy */

/*
 * Per-instance state.  Attach fills the softc, kms_dev_register hands
 * back the shared struct drm_device.  Ordering is important: the softc
 * has to survive as long as drm_dev because the framework can call
 * back into our driver ops from any file-op path.
 */
struct virtio_kms_softc {
	device_t		 dev;
	struct drm_device	*drm_dev;

	struct sx		 sx;		/* driver-wide serializer */
	bool			 detaching;	/* atomic_commit gate */

	/* virtio plumbing (Phase B). */
	struct virtqueue	*ctrl_vq;	/* command / response */
	struct virtqueue	*cursor_vq;	/* cursor updates */
	struct sx		 ctrl_sx;	/* serialize ctrl_vq req/resp */
	uint32_t		 features;	/* negotiated VIRTIO_GPU_F_* */
	/*
	 * Every ctrl-queue command goes through virtio_kms_req_resp which
	 * polls the used ring in-line -- we never observe a host fence,
	 * so we don't set VIRTIO_GPU_FLAG_FENCE and don't burn a fence_id.
	 * Review #24.
	 */

	/* Config snapshot + cached scanout topology. */
	struct virtio_gpu_config gpucfg;
	uint32_t		 num_scanouts;	/* clamped to MAX */
	struct virtio_kms_scanout scanouts[VIRTIO_KMS_MAX_SCANOUTS];

	/*
	 * Currently-attached host resource (Phase D).  Single-scanout
	 * for now, so at most one framebuffer is live at a time.  A
	 * resource_id of 0 means "no resource attached"; the ID itself
	 * is the framebuffer's drm_mode_object.id (framework-assigned,
	 * unique per drm_device, always non-zero for live objects).
	 */
	uint32_t		 active_resource_id;
	uint32_t		 active_scanout_id;	/* 2nd-review M1 */
	uint32_t		 active_width;
	uint32_t		 active_height;

	/*
	 * Framebuffer -> host resource_id map (Review #10/#11).  Every
	 * fb that atomic_bind_fb ever calls CREATE_2D + ATTACH_BACKING
	 * for gets an entry in fb_map; virtio_kms_fb_destroy walks the
	 * list on last-put to release the matching host resource.  A
	 * small linked list is fine -- production compositors keep 2-3
	 * fbs (front, back, cursor).  The entry struct is at file scope
	 * below.
	 */
	LIST_HEAD(virtio_kms_fb_map, virtio_kms_fb_entry) fb_map;

	/*
	 * Mode-config topology (Phase F multi-scanout).  Arrays sized at
	 * VIRTIO_KMS_MAX_SCANOUTS (16, per spec); only the first
	 * `num_scanouts` slots are initialised.  Object lifetime is
	 * bounded by attach/detach; the framework hands ownership back
	 * via topology_teardown.
	 *
	 * connector[i] <-> encoder[i] <-> crtc[i] <-> primary[i] is a
	 * strict one-to-one pipeline in Phase F -- no cross-routing,
	 * matches the virtio-gpu scanout model exactly.
	 */
	struct drm_crtc		 crtc[VIRTIO_KMS_MAX_SCANOUTS];
	struct drm_plane	 primary[VIRTIO_KMS_MAX_SCANOUTS];
	struct drm_encoder	 encoder[VIRTIO_KMS_MAX_SCANOUTS];
	struct drm_connector	 connector[VIRTIO_KMS_MAX_SCANOUTS];
	/*
	 * Highest connector-topology index actually initialised.
	 * Grown by hotplug when the host adds a scanout; never shrinks
	 * (kms_connector_cleanup on an already-published connector is
	 * a footgun for concurrent GETCONNECTOR ioctls, so we just mark
	 * the slot disconnected instead).  Review #13.
	 */
	uint32_t		 topology_n;

	/* Task for taskqueue-deferred hotplug event handling. */
	struct task		 hotplug_task;

	/*
	 * Boot splash — one-shot solid-color fill installed at the end of
	 * attach so the host has an active scanout before any userspace
	 * DRM client SetCrtcs.  Without it, QEMU's GTK/VNC window shows
	 * "output not active" from attach until first ADDFB2+SETCRTC.
	 * splash_rid == 0 means no splash installed (host had no active
	 * scanout, or contigmalloc failed).
	 */
	void			*splash_va;
	vm_paddr_t		 splash_pa;
	size_t			 splash_size;
	uint32_t		 splash_rid;
	uint32_t		 splash_max_w;
	uint32_t		 splash_max_h;
	uint32_t		 pmode_w;	/* current SET_SCANOUT w */
	uint32_t		 pmode_h;	/* current SET_SCANOUT h */
#define	VIRTIO_KMS_SPLASH_MAX_W	3840
#define	VIRTIO_KMS_SPLASH_MAX_H	2160

	/*
	 * vt(4) kernel console glue.  When the splash is up, we hand its
	 * framebuffer to vt(4) via vt_fb_attach so kernel printf + login
	 * appear on the display.  vt(4) writes into splash_va directly;
	 * a 30 Hz callout schedules refresh_task on taskqueue_thread,
	 * which then pushes TRANSFER_TO_HOST_2D + RESOURCE_FLUSH so the
	 * guest-side writes become host-visible.
	 */
	struct fb_info		 fb_info;
	bool			 console_attached;
	bool			 refresh_initialized;	/* one-shot init gate */
	bool			 init_hupped;		/* one-shot getty respawn */
	struct callout		 refresh_co;
	struct task		 refresh_task;

	/*
	 * Deferred boot-time attach.  install_splash + install_console
	 * touch virtqueue, vt(4), contigmalloc, and initproc — all of
	 * which can be half-initialised when the driver is preloaded
	 * from /boot/loader.conf and probed during early boot.  Move
	 * that work to a config_intrhook so it runs after all root-mount
	 * / SI_SUB hooks have fired.  Then loader.conf preload and
	 * runtime kldload take identical paths.
	 */
	struct intr_config_hook	 boot_hook;
	bool			 boot_hook_installed;

	/*
	 * Resize path — refresh_tick polls events_read on every tick.
	 * Rather than reprovision on every EVENT_DISPLAY (which fires
	 * dozens of times per second during a drag and produces stale
	 * SET_SCANOUTs that host rejects), we track a grace-tick counter:
	 * seeing the event bit set resets grace to N; each tick without
	 * the bit decrements it; when grace hits 0 and a reprovision is
	 * still needed, we enqueue hotplug_task.  Effect: reprovision
	 * only fires ~200 ms after the last drag event, i.e. when pmode
	 * has stopped moving.  Idempotent + convergent + no host racing.
	 */
	bool			 reprovision_pending;
	bool			 resize_wanted;
	uint32_t		 resize_grace_ticks;
#define	VIRTIO_KMS_RESIZE_GRACE_TICKS	12	/* @ 60 Hz = ~200 ms */

	int			 debug;		/* dev.virtio_kms.N.debug */
};

#define	VKMS_DPRINTF(sc, ...) do {					\
	if ((sc)->debug > 0)						\
		device_printf((sc)->dev, __VA_ARGS__);			\
} while (0)

/* Forward decls for the mode-config layer used by attach. */
static int	virtio_kms_topology_init(struct virtio_kms_softc *sc);
static void	virtio_kms_topology_teardown(struct virtio_kms_softc *sc);
static void	virtio_kms_release_active(struct virtio_kms_softc *sc);

/* Forward decls for Phase D 2D protocol wrappers used by install_splash. */
static int	virtio_kms_resource_create_2d(struct virtio_kms_softc *sc,
		    uint32_t resource_id, uint32_t format, uint32_t width,
		    uint32_t height);
static int	virtio_kms_resource_unref(struct virtio_kms_softc *sc,
		    uint32_t resource_id);
static int	virtio_kms_detach_backing(struct virtio_kms_softc *sc,
		    uint32_t resource_id);
static int	virtio_kms_set_scanout(struct virtio_kms_softc *sc,
		    uint32_t scanout_id, uint32_t resource_id, uint32_t x,
		    uint32_t y, uint32_t w, uint32_t h);
static int	virtio_kms_transfer_to_host_2d(struct virtio_kms_softc *sc,
		    uint32_t resource_id, uint32_t x, uint32_t y,
		    uint32_t w, uint32_t h);
static int	virtio_kms_resource_flush(struct virtio_kms_softc *sc,
		    uint32_t resource_id, uint32_t x, uint32_t y,
		    uint32_t w, uint32_t h);
static void	virtio_kms_hotplug_task(void *arg, int pending);
static int	virtio_kms_sysctl_hotplug_probe(SYSCTL_HANDLER_ARGS);
static inline struct virtio_kms_softc *virtio_kms_dev_to_sc(
    struct drm_device *drm_dev);
static const struct drm_mode_config_funcs virtio_kms_mode_config_funcs;

/*
 * Framebuffer -> host resource_id tracking entry.  See fb_map above.
 * File-scope so LIST_HEAD in the softc has a complete tag to point at.
 */
struct virtio_kms_fb_entry {
	LIST_ENTRY(virtio_kms_fb_entry)	 link;
	const struct drm_framebuffer	*fb;
	uint32_t			 resource_id;
};

/*
 * Driver descriptor.  driver_features=0 for now; ATOMIC / RENDER caps
 * flip on once the topology and command queue are wired.
 */
static const struct drm_framebuffer_funcs virtio_kms_framebuffer_funcs;

static const struct drm_driver virtio_kms_driver = {
	.name		= VIRTIO_KMS_DRIVER_NAME,
	.desc		= VIRTIO_KMS_DRIVER_DESC,
	.date		= VIRTIO_KMS_DRIVER_DATE,
	.major		= 0,
	.minor		= 1,
	.patchlevel	= 0,
	.driver_features = 0,
	.framebuffer_funcs = &virtio_kms_framebuffer_funcs,
};

static struct virtio_feature_desc virtio_kms_feature_desc[] = {
	{ VIRTIO_GPU_F_VIRGL,		"VIRGL" },
	{ VIRTIO_GPU_F_EDID,		"EDID" },
	{ 0, NULL }
};

/*
 * Read the device config snapshot into sc->gpucfg.  Mirrors the
 * in-base virtio_gpu(4) VTGPU_GET_CONFIG expansion; centralised here
 * so Phase F hotplug-event handling can re-read events_read / clear
 * without duplicating the offset math.
 */
static void
virtio_kms_read_config(struct virtio_kms_softc *sc)
{
	device_t dev = sc->dev;
	struct virtio_gpu_config *cfg = &sc->gpucfg;

#define	VKMS_GET_CFG(_field)						\
	virtio_read_device_config(dev,					\
	    offsetof(struct virtio_gpu_config, _field),			\
	    &cfg->_field, sizeof(cfg->_field))

	VKMS_GET_CFG(events_read);
	VKMS_GET_CFG(events_clear);
	VKMS_GET_CFG(num_scanouts);
	VKMS_GET_CFG(num_capsets);

#undef	VKMS_GET_CFG
}

/*
 * Allocate the two GPU virtqueues.  ctrl_vq carries every command +
 * its response (2D setup, scanout config, EDID, display-info,
 * hotplug), cursor_vq carries VIRTIO_GPU_CMD_UPDATE_CURSOR /
 * MOVE_CURSOR.  Both queues stay untouched until Phase C wires the
 * command surfaces to them.
 */
static int
virtio_kms_alloc_queues(struct virtio_kms_softc *sc)
{
	struct vq_alloc_info info[2];
	int nvqs = 2;

	VQ_ALLOC_INFO_INIT(&info[0], 0, NULL, sc, &sc->ctrl_vq,
	    "%s ctrl", device_get_nameunit(sc->dev));
	VQ_ALLOC_INFO_INIT(&info[1], 0, NULL, sc, &sc->cursor_vq,
	    "%s cursor", device_get_nameunit(sc->dev));

	return (virtio_alloc_virtqueues(sc->dev, nvqs, info));
}

/*
 * All Phase D command paths use virtqueue_poll for completion, so we
 * don't want a used-buffer interrupt firing the generic virtio ISR
 * with our NULL callback (VQ_ALLOC_INFO_INIT above passes NULL for
 * _intr).  Disable notifications on both queues after alloc.  Config-
 * change events (hotplug) still come through the device's dedicated
 * config-change vector -- this only silences per-queue completion IRQs.
 * Review #2.
 */
static void
virtio_kms_disable_queue_intr(struct virtio_kms_softc *sc)
{
	if (sc->ctrl_vq != NULL)
		virtqueue_disable_intr(sc->ctrl_vq);
	if (sc->cursor_vq != NULL)
		virtqueue_disable_intr(sc->cursor_vq);
}

/*
 * Synchronous request / response over ctrl_vq.  Mirrors the in-base
 * driver's vtgpu_req_resp: build a two-segment sglist (req is device-
 * read, resp is device-write), enqueue with 1 readable + 1 writable
 * seg, notify the queue, poll for completion.
 *
 * Caller fills the command header (type / flags / fence_id) inside
 * req.  We hold ctrl_sx across the whole cycle so only one in-flight
 * command is on the queue at a time; the spec allows out-of-order
 * completion but Phase B never issues more than one request in
 * parallel, and the polled-completion path is cheaper than a
 * per-request wait.
 */
static int
virtio_kms_req_resp(struct virtio_kms_softc *sc, void *req, size_t reqlen,
    void *resp, size_t resplen)
{
	struct sglist sg;
	struct sglist_seg segs[2];
	int error;

	sglist_init(&sg, 2, segs);
	error = sglist_append(&sg, req, reqlen);
	if (error != 0) {
		device_printf(sc->dev,
		    "req_resp: sglist_append(req) rc=%d\n", error);
		return (error);
	}
	error = sglist_append(&sg, resp, resplen);
	if (error != 0) {
		device_printf(sc->dev,
		    "req_resp: sglist_append(resp) rc=%d\n", error);
		return (error);
	}

	sx_xlock(&sc->ctrl_sx);
	error = virtqueue_enqueue(sc->ctrl_vq, resp, &sg, 1, 1);
	if (error != 0) {
		sx_xunlock(&sc->ctrl_sx);
		device_printf(sc->dev,
		    "req_resp: virtqueue_enqueue rc=%d\n", error);
		return (error);
	}
	virtqueue_notify(sc->ctrl_vq);
	virtqueue_poll(sc->ctrl_vq, NULL);
	sx_xunlock(&sc->ctrl_sx);

	return (0);
}

/*
 * Issue GET_DISPLAY_INFO, cache the pmode[] table into sc->scanouts.
 * The response gives up to VIRTIO_GPU_MAX_SCANOUTS entries; the host
 * marks each one enabled + sizes the mode.  Phase C hands this table
 * to the connector-mode enumeration; Phase F re-runs it on hotplug.
 */
static int
virtio_kms_fetch_display_info(struct virtio_kms_softc *sc)
{
	struct {
		struct virtio_gpu_ctrl_hdr	req;
		char				pad;
		struct virtio_gpu_resp_display_info resp;
	} s;
	uint32_t nscan, i;
	int error;

	bzero(&s, sizeof(s));
	s.req.type = htole32(VIRTIO_GPU_CMD_GET_DISPLAY_INFO);

	error = virtio_kms_req_resp(sc, &s.req, sizeof(s.req),
	    &s.resp, sizeof(s.resp));
	if (error != 0)
		return (error);

	if (le32toh(s.resp.hdr.type) != VIRTIO_GPU_RESP_OK_DISPLAY_INFO) {
		device_printf(sc->dev,
		    "display_info: unexpected resp type 0x%x\n",
		    le32toh(s.resp.hdr.type));
		return (EIO);
	}

	nscan = sc->gpucfg.num_scanouts;
	if (nscan > VIRTIO_KMS_MAX_SCANOUTS)
		nscan = VIRTIO_KMS_MAX_SCANOUTS;
	sc->num_scanouts = nscan;

	bzero(sc->scanouts, sizeof(sc->scanouts));
	for (i = 0; i < nscan; i++) {
		sc->scanouts[i].enabled = le32toh(s.resp.pmodes[i].enabled);
		sc->scanouts[i].width   = le32toh(s.resp.pmodes[i].r.width);
		sc->scanouts[i].height  = le32toh(s.resp.pmodes[i].r.height);
		sc->scanouts[i].x       = le32toh(s.resp.pmodes[i].r.x);
		sc->scanouts[i].y       = le32toh(s.resp.pmodes[i].r.y);
		sc->scanouts[i].flags   = le32toh(s.resp.pmodes[i].flags);
		VKMS_DPRINTF(sc, "scanout[%u] enabled=%u %ux%u @%u,%u\n",
		    i, sc->scanouts[i].enabled,
		    sc->scanouts[i].width, sc->scanouts[i].height,
		    sc->scanouts[i].x, sc->scanouts[i].y);
	}
	return (0);
}

/*
 * Install a solid-color boot splash on scanout 0.  Allocates a
 * physically-contiguous framebuffer (contigmalloc), fills with a
 * fixed color, and runs the standard 5-step CREATE_2D +
 * ATTACH_BACKING + SET_SCANOUT + TRANSFER_TO_HOST_2D + FLUSH sequence
 * so the host has an active scanout the moment attach completes.
 *
 * The splash resource_id is picked from the top of the u32 range
 * (0xffff0001) so it can never collide with framework-allocated
 * fb->base.id values, which start low.  If a userspace client later
 * SetCrtcs its own fb, the framework's activate path issues a new
 * SET_SCANOUT with the user rid, superseding the splash on the host
 * side; the splash resource lingers as an unused host object until
 * detach frees it.
 *
 * Failures are non-fatal to attach; a splashless driver still works,
 * userspace just won't see anything until it drives the DRM ioctls.
 */
static int
virtio_kms_install_splash(struct virtio_kms_softc *sc)
{
	struct virtio_gpu_resource_attach_backing req;
	struct virtio_gpu_mem_entry mem;
	struct virtio_gpu_ctrl_hdr resp;
	struct sglist sg;
	struct sglist_seg segs[3];
	const uint32_t rid = 0xffff0001;
	uint32_t max_w = VIRTIO_KMS_SPLASH_MAX_W;
	uint32_t max_h = VIRTIO_KMS_SPLASH_MAX_H;
	uint32_t pmode_w, pmode_h, i, npx;
	uint32_t *px;
	size_t sz;
	int error;

	if (sc->num_scanouts == 0 || !sc->scanouts[0].enabled)
		return (0);
	pmode_w = sc->scanouts[0].width;
	pmode_h = sc->scanouts[0].height;
	if (pmode_w == 0 || pmode_h == 0)
		return (0);
	/*
	 * Cap the initial scanout to our fixed-max backing.  If the host
	 * hands us a larger pmode we'll letterbox: only max_w x max_h is
	 * shown, remainder is undefined.  Reprovision path is a no-op
	 * on the buffer/resource — only the SET_SCANOUT rect changes,
	 * so fb_info + vt(4) never see a stride shift.
	 */
	if (pmode_w > max_w) pmode_w = max_w;
	if (pmode_h > max_h) pmode_h = max_h;
	sz = roundup2((size_t)max_w * max_h * 4, PAGE_SIZE);

	sc->splash_va = contigmalloc(sz, M_DEVBUF, M_NOWAIT | M_ZERO,
	    0, ~(vm_paddr_t)0, PAGE_SIZE, 0);
	if (sc->splash_va == NULL) {
		device_printf(sc->dev,
		    "splash: contigmalloc %zu bytes failed\n", sz);
		return (ENOMEM);
	}
	sc->splash_size = sz;
	sc->splash_pa = pmap_kextract((vm_offset_t)sc->splash_va);

	/* Solid teal in B8G8R8X8 host format (little-endian 0xXXRRGGBB). */
	px = (uint32_t *)sc->splash_va;
	npx = max_w * max_h;
	for (i = 0; i < npx; i++)
		px[i] = 0xff204060;

	error = virtio_kms_resource_create_2d(sc, rid,
	    VIRTIO_GPU_FORMAT_B8G8R8X8_UNORM, max_w, max_h);
	if (error != 0) {
		device_printf(sc->dev,
		    "splash: resource_create_2d rc=%d\n", error);
		goto fail_buf;
	}

	/*
	 * attach_backing with a raw phys+len — can't reuse
	 * virtio_kms_attach_backing() because that helper requires a
	 * GEM object.  One mem_entry pointing at the contigmalloc region.
	 */
	bzero(&req, sizeof(req));
	bzero(&mem, sizeof(mem));
	bzero(&resp, sizeof(resp));
	req.hdr.type = htole32(VIRTIO_GPU_CMD_RESOURCE_ATTACH_BACKING);
	req.resource_id = htole32(rid);
	req.nr_entries = htole32(1);
	mem.addr = htole64(sc->splash_pa);
	mem.length = htole32((uint32_t)sz);
	sglist_init(&sg, 3, segs);
	sglist_append(&sg, &req, sizeof(req));
	sglist_append(&sg, &mem, sizeof(mem));
	sglist_append(&sg, &resp, sizeof(resp));
	sx_xlock(&sc->ctrl_sx);
	error = virtqueue_enqueue(sc->ctrl_vq, &resp, &sg, 2, 1);
	if (error == 0) {
		virtqueue_notify(sc->ctrl_vq);
		virtqueue_poll(sc->ctrl_vq, NULL);
	}
	sx_xunlock(&sc->ctrl_sx);
	if (error != 0 ||
	    le32toh(resp.type) != VIRTIO_GPU_RESP_OK_NODATA) {
		device_printf(sc->dev,
		    "splash: attach_backing failed enq=%d resp=0x%x\n",
		    error, le32toh(resp.type));
		(void)virtio_kms_resource_unref(sc, rid);
		goto fail_buf;
	}

	error = virtio_kms_set_scanout(sc, 0, rid, 0, 0, pmode_w, pmode_h);
	if (error != 0)
		goto fail_res;
	error = virtio_kms_transfer_to_host_2d(sc, rid, 0, 0,
	    pmode_w, pmode_h);
	if (error != 0)
		goto fail_res;
	error = virtio_kms_resource_flush(sc, rid, 0, 0, pmode_w, pmode_h);
	if (error != 0)
		goto fail_res;

	sc->splash_rid = rid;
	sc->splash_max_w = max_w;
	sc->splash_max_h = max_h;
	sc->pmode_w = pmode_w;
	sc->pmode_h = pmode_h;
	sc->active_resource_id = rid;
	sc->active_scanout_id = 0;
	sc->active_width = max_w;	/* fb_info fixed at MAX */
	sc->active_height = max_h;
	device_printf(sc->dev,
	    "splash: buffer=%ux%u scanout=%ux%u rid=0x%x (fixed-max fb)\n",
	    max_w, max_h, pmode_w, pmode_h, rid);
	return (0);

fail_res:
	(void)virtio_kms_detach_backing(sc, rid);
	(void)virtio_kms_resource_unref(sc, rid);
fail_buf:
	contigfree(sc->splash_va, sz, M_DEVBUF);
	sc->splash_va = NULL;
	sc->splash_size = 0;
	sc->splash_pa = 0;
	return (error);
}

/*
 * Refresh path — vt(4) writes ASCII glyphs into fb_vbase (== splash_va),
 * but the host doesn't observe those writes until we issue a
 * TRANSFER_TO_HOST_2D + RESOURCE_FLUSH pair.  A callout schedules this
 * task every hz/30 (~33 ms) so the display appears interactive without
 * needing per-write damage tracking.
 *
 * Runs on taskqueue_thread (sleepable), so ctrl_sx grabs inside
 * transfer_to_host_2d / resource_flush are safe.  If a userspace client
 * has replaced the scanout with its own fb (splash_rid != active_rid),
 * we quietly skip — pushing our splash bits would be a no-op flush on
 * an unowned scanout.
 */
static void
virtio_kms_refresh_task_run(void *arg, int pending __unused)
{
	struct virtio_kms_softc *sc = arg;
	uint32_t w, h;

	if (sc->detaching || sc->reprovision_pending || sc->splash_rid == 0)
		return;
	if (sc->active_resource_id != sc->splash_rid)
		return;
	/* Push + flush ONLY the currently-scanned-out rect, not the whole
	 * fixed-max buffer.  vt(4) may have written past pmode edges, but
	 * host doesn't need those bytes and copying 32 MB per tick would
	 * saturate ctrl_vq. */
	w = sc->pmode_w;
	h = sc->pmode_h;
	if (w == 0 || h == 0)
		return;
	(void)virtio_kms_transfer_to_host_2d(sc, sc->splash_rid, 0, 0, w, h);
	(void)virtio_kms_resource_flush(sc, sc->splash_rid, 0, 0, w, h);
}

static void
virtio_kms_refresh_tick(void *arg)
{
	struct virtio_kms_softc *sc = arg;
	uint32_t evt = 0;

	if (sc->detaching)
		return;

	/*
	 * Cheap polled resize detection with debounce.  Read events_read
	 * (one 32-bit config load) every tick.  Any DISPLAY event resets
	 * the grace counter to N; ticks without an event decrement it;
	 * when it hits zero + resize is still wanted, enqueue hotplug_task
	 * to actually reprovision.  This holds off the reprovision until
	 * pmode has stopped changing, avoiding stale SET_SCANOUT races
	 * against a moving host pmode during drag-resize.
	 */
	virtio_read_device_config(sc->dev,
	    offsetof(struct virtio_gpu_config, events_read),
	    &evt, sizeof(evt));
	if ((evt & VIRTIO_GPU_EVENT_DISPLAY) != 0) {
		sc->resize_wanted = true;
		sc->resize_grace_ticks = VIRTIO_KMS_RESIZE_GRACE_TICKS;
	} else if (sc->resize_grace_ticks > 0) {
		sc->resize_grace_ticks--;
	}
	if (sc->resize_wanted && sc->resize_grace_ticks == 0 &&
	    !sc->reprovision_pending) {
		sc->resize_wanted = false;
		taskqueue_enqueue(taskqueue_thread, &sc->hotplug_task);
	}

	taskqueue_enqueue(taskqueue_thread, &sc->refresh_task);
	callout_reset(&sc->refresh_co, hz / 60,
	    virtio_kms_refresh_tick, sc);
}

static void	virtio_kms_install_console(struct virtio_kms_softc *sc);

/*
 * config_intrhook callback — runs after root mount + all boot
 * SYSINITs, so contigmalloc / virtqueue_poll / initproc / taskqueue
 * are all safely initialised.  Loader-preload + runtime-kldload
 * converge here.
 */
static void
virtio_kms_boot_finished(void *arg)
{
	struct virtio_kms_softc *sc = arg;

	if (sc->boot_hook_installed) {
		config_intrhook_disestablish(&sc->boot_hook);
		sc->boot_hook_installed = false;
	}
	if (sc->detaching)
		return;
	(void)virtio_kms_install_splash(sc);
	virtio_kms_install_console(sc);
}

/*
 * Install a vt(4) framebuffer console using our splash surface.  vt_fb
 * has priority VD_PRIORITY_GENERIC+10, higher than efifb (+1) and
 * vga (+0), so it replaces whatever driver currently owns the
 * console.  vt(4)'s vd_init hook programs the 32-bit RGB mask.
 *
 * splash_va must be a kernel VA that vt(4) can write to directly;
 * contigmalloc satisfies that.  The refresh callout pushes the writes
 * to the host at 30 Hz — the interval is a UX tradeoff, low enough for
 * a responsive shell but not so aggressive as to saturate ctrl_vq.
 */
static void
virtio_kms_install_console(struct virtio_kms_softc *sc)
{
	int err;

	if (sc->splash_va == NULL || sc->splash_rid == 0)
		return;

	bzero(&sc->fb_info, sizeof(sc->fb_info));
	sc->fb_info.fb_name = device_get_nameunit(sc->dev);
	sc->fb_info.fb_type = FBTYPE_PCIMISC;
	sc->fb_info.fb_width = sc->active_width;
	sc->fb_info.fb_height = sc->active_height;
	sc->fb_info.fb_depth = 32;
	sc->fb_info.fb_bpp = 32;
	sc->fb_info.fb_stride = (int)sc->active_width * 4;
	sc->fb_info.fb_size = (int)sc->splash_size;
	sc->fb_info.fb_pbase = sc->splash_pa;
	sc->fb_info.fb_vbase = (vm_offset_t)sc->splash_va;

	if (!sc->refresh_initialized) {
		TASK_INIT(&sc->refresh_task, 0,
		    virtio_kms_refresh_task_run, sc);
		callout_init(&sc->refresh_co, 1);
		sc->refresh_initialized = true;
	}

	err = vt_fb_attach(&sc->fb_info);
	if (err != 0) {
		device_printf(sc->dev,
		    "console: vt_fb_attach rc=%d — no vt(4) takeover\n",
		    err);
		return;
	}
	sc->console_attached = true;
	callout_reset(&sc->refresh_co, hz / 60,
	    virtio_kms_refresh_tick, sc);
	device_printf(sc->dev,
	    "console: vt(4) attached, refresh 60 Hz "
	    "(kernel printf now on framebuffer)\n");

	/*
	 * One-shot: HUP init(8) so it re-reads /etc/ttys and spawns
	 * gettys on ttyv0..ttyv7 now that a framebuffer console
	 * exists.  /etc/ttys defaults `onifexists secure` which only
	 * spawns getty when the terminal cdev already exists at init
	 * time — since our vt driver arrived via runtime kldload,
	 * init already skipped the vt entries at boot.  Without this
	 * HUP the display shows kernel messages but no login prompt
	 * ever appears.  Skip on subsequent install_console() calls
	 * from reprovision so we don't nuke live logins on resize.
	 */
	if (!sc->init_hupped && initproc != NULL) {
		PROC_LOCK(initproc);
		kern_psignal(initproc, SIGHUP);
		PROC_UNLOCK(initproc);
		sc->init_hupped = true;
		device_printf(sc->dev,
		    "console: SIGHUP init(8) to spawn ttyv gettys\n");
	}
}

/*
 * Reprovision splash + vt console at scanouts[0]'s current geometry.
 * Called from hotplug_task after events_read is acked; safely no-ops
 * if the geometry hasn't changed.
 *
 * Ordering: stop the refresh callout, drain in-flight refresh task,
 * hand vt(4) back (best-effort — vt_deallocate returns EPERM when
 * there's no prev_driver to restore, which is our normal case; the
 * subsequent vt_fb_attach replaces the driver in-place at higher
 * priority so the fallthrough is benign).  Detach + unref the old
 * host resource, contigfree the guest buffer, then install_splash +
 * install_console rebuild at the new dims.
 *
 * refresh_task_run early-returns while reprovision_pending is set,
 * so no TRANSFER_TO_HOST_2D fires against the resource we're tearing
 * down.
 */
static void
virtio_kms_reprovision(struct virtio_kms_softc *sc)
{
	uint32_t new_w, new_h;
	uint32_t clear;
	int error;

	if (sc->detaching || sc->splash_rid == 0)
		return;

	sc->reprovision_pending = true;

	sx_xlock(&sc->sx);
	(void)virtio_kms_fetch_display_info(sc);
	sx_xunlock(&sc->sx);

	if (sc->num_scanouts == 0 || !sc->scanouts[0].enabled)
		goto done;
	new_w = sc->scanouts[0].width;
	new_h = sc->scanouts[0].height;
	if (new_w == 0 || new_h == 0)
		goto done;

	/*
	 * Clamp to our fixed-max backing.  If the pmode exceeds our
	 * buffer, letterbox the visible region — we cannot exceed the
	 * resource's declared width/height in SET_SCANOUT without host
	 * rejecting the call.
	 */
	if (new_w > sc->splash_max_w) new_w = sc->splash_max_w;
	if (new_h > sc->splash_max_h) new_h = sc->splash_max_h;

	if (new_w == sc->pmode_w && new_h == sc->pmode_h)
		goto done;	/* settled */

	device_printf(sc->dev,
	    "resize: pmode %ux%u -> %ux%u (fb_info stays %ux%u)\n",
	    sc->pmode_w, sc->pmode_h, new_w, new_h,
	    sc->splash_max_w, sc->splash_max_h);

	/*
	 * Fixed-max design: nothing to free / re-alloc / re-attach.
	 * Just re-point the host scanout at the new sub-rect of the
	 * same resource.  vt(4) keeps writing into the same splash_va
	 * with the same stride, so teken/vd layout never sees a stride
	 * change (root cause of the "stretching" glitch on prior
	 * reprovision-realloc path).
	 *
	 * TRANSFER + FLUSH so the newly-exposed region carries the
	 * current vt content immediately, not just after the next
	 * 60 Hz tick.
	 */
	error = virtio_kms_set_scanout(sc, 0, sc->splash_rid, 0, 0,
	    new_w, new_h);
	if (error != 0) {
		device_printf(sc->dev,
		    "resize: set_scanout rc=%d — keeping old pmode\n",
		    error);
		goto done;
	}
	(void)virtio_kms_transfer_to_host_2d(sc, sc->splash_rid, 0, 0,
	    new_w, new_h);
	(void)virtio_kms_resource_flush(sc, sc->splash_rid, 0, 0,
	    new_w, new_h);
	sc->pmode_w = new_w;
	sc->pmode_h = new_h;

done:
	/*
	 * Ack any DISPLAY event that fired.  A fresh event during the
	 * short window between this ACK and refresh_tick's next poll
	 * will simply re-set the bit and be caught on the next tick.
	 */
	clear = VIRTIO_GPU_EVENT_DISPLAY;
	virtio_write_device_config(sc->dev,
	    offsetof(struct virtio_gpu_config, events_clear),
	    &clear, sizeof(clear));
	sc->reprovision_pending = false;
}

static int
virtio_kms_probe(device_t dev)
{
	if (virtio_get_device_type(dev) != VIRTIO_ID_GPU)
		return (ENXIO);
	device_set_desc(dev, VIRTIO_KMS_DRIVER_DESC);
	/*
	 * Beat the in-base virtio_gpu(4) at BUS_PROBE_VENDOR.  In-base
	 * uses BUS_PROBE_DEFAULT on virtio_mmio too, so we can't rely on
	 * tie-breaking there -- use BUS_PROBE_VENDOR+1 to ensure a
	 * deterministic win.  Review #17.
	 */
	return (BUS_PROBE_VENDOR + 1);
}

static int
virtio_kms_attach(device_t dev)
{
	struct virtio_kms_softc *sc;
	int error;

	sc = device_get_softc(dev);
	sc->dev = dev;
	sx_init(&sc->sx, "virtio_kms");
	sx_init(&sc->ctrl_sx, "virtio_kms ctrl");
	LIST_INIT(&sc->fb_map);
	TASK_INIT(&sc->hotplug_task, 0, virtio_kms_hotplug_task, sc);

	virtio_set_feature_desc(dev, virtio_kms_feature_desc);
	sc->features = virtio_negotiate_features(dev, 0);

	virtio_kms_read_config(sc);

	error = virtio_kms_alloc_queues(sc);
	if (error != 0) {
		device_printf(dev, "alloc_queues failed: %d\n", error);
		goto fail_locks;
	}
	error = virtio_setup_intr(dev, INTR_TYPE_MISC);
	if (error != 0) {
		device_printf(dev, "setup_intr failed: %d\n", error);
		goto fail_locks;
	}
	virtio_kms_disable_queue_intr(sc);

	error = virtio_kms_fetch_display_info(sc);
	if (error != 0) {
		device_printf(dev,
		    "fetch_display_info rc=%d — will retry on hotplug\n",
		    error);
		/* Non-fatal: attach continues without a topology snapshot. */
	}

	error = kms_dev_register(&virtio_kms_driver, sc, &sc->drm_dev);
	if (error != 0) {
		device_printf(dev,
		    "kms_dev_register failed: %d\n", error);
		goto fail_locks;
	}

	/*
	 * Install the atomic hooks BEFORE topology_init so any
	 * userspace-visible ATOMIC ioctl that races the attach path
	 * takes the real check + commit dispatch rather than the
	 * framework's legacy property-table fallback.
	 */
	sc->drm_dev->mode_config.funcs = &virtio_kms_mode_config_funcs;

	error = virtio_kms_topology_init(sc);
	if (error != 0) {
		device_printf(dev, "topology_init failed: %d\n", error);
		kms_dev_unregister(sc->drm_dev);
		sc->drm_dev = NULL;
		goto fail_locks;
	}

	SYSCTL_ADD_INT(device_get_sysctl_ctx(dev),
	    SYSCTL_CHILDREN(device_get_sysctl_tree(dev)), OID_AUTO,
	    "debug", CTLFLAG_RW, &sc->debug, 0,
	    "Debug verbosity (0 = quiet)");
	SYSCTL_ADD_UINT(device_get_sysctl_ctx(dev),
	    SYSCTL_CHILDREN(device_get_sysctl_tree(dev)), OID_AUTO,
	    "num_scanouts", CTLFLAG_RD, &sc->num_scanouts, 0,
	    "Number of active scanouts reported by the host");
	SYSCTL_ADD_UINT(device_get_sysctl_ctx(dev),
	    SYSCTL_CHILDREN(device_get_sysctl_tree(dev)), OID_AUTO,
	    "active_resource_id", CTLFLAG_RD, &sc->active_resource_id, 0,
	    "Currently-attached host resource ID (0 = none)");
	SYSCTL_ADD_PROC(device_get_sysctl_ctx(dev),
	    SYSCTL_CHILDREN(device_get_sysctl_tree(dev)), OID_AUTO,
	    "hotplug_probe",
	    CTLTYPE_INT | CTLFLAG_RW | CTLFLAG_MPSAFE, sc, 0,
	    virtio_kms_sysctl_hotplug_probe, "I",
	    "Write 1 to re-fetch display_info and fire hotplug on every "
	    "connector (Phase F stand-in for the config-change IRQ).");

	device_printf(dev,
	    "attached (features 0x%x, num_scanouts %u, num_capsets %u)\n",
	    sc->features, sc->gpucfg.num_scanouts, sc->gpucfg.num_capsets);

	/*
	 * Defer splash + console attach until after boot completes.
	 * install_splash touches contigmalloc + virtqueue and
	 * install_console calls vt_fb_attach + kern_psignal(initproc);
	 * both can trap if we're on the /boot/loader.conf preload path
	 * and device probe happens before the VM subsystem, taskqueue,
	 * or initproc are ready.  config_intrhook_establish fires the
	 * callback once the boot process reaches the point where all
	 * SI_SUB hooks + root mount are safely done.
	 */
	sc->boot_hook.ich_func = virtio_kms_boot_finished;
	sc->boot_hook.ich_arg = sc;
	if (config_intrhook_establish(&sc->boot_hook) == 0) {
		sc->boot_hook_installed = true;
	} else {
		device_printf(dev,
		    "config_intrhook_establish failed — falling back to "
		    "inline splash/console attach\n");
		(void)virtio_kms_install_splash(sc);
		virtio_kms_install_console(sc);
	}
	return (0);

fail_locks:
	sx_destroy(&sc->ctrl_sx);
	sx_destroy(&sc->sx);
	return (error);
}

static int
virtio_kms_detach(device_t dev)
{
	struct virtio_kms_softc *sc = device_get_softc(dev);
	uint32_t live_fbs;

	/*
	 * 2nd-review H1 / 3rd-review L-2: refuse to detach while any
	 * BOUND framebuffer is still tracked.  fb_map only tracks fbs
	 * that atomic_bind_fb ever wired to a host resource -- ADDFB2'd-
	 * but-never-committed fbs bypass this count.  Those are still
	 * safe because virtio_kms_fb_destroy checks sc==NULL (the
	 * framework NULL'd driver_priv in kms_dev_unregister) and just
	 * frees the fb allocation without touching the driver.  So this
	 * gate covers the primary crash path (fb with a live host resource
	 * survives detach) and the sc==NULL guard covers the residual.
	 */
	sx_xlock(&sc->sx);
	live_fbs = 0;
	{
		struct virtio_kms_fb_entry *e;

		LIST_FOREACH(e, &sc->fb_map, link)
			live_fbs++;
	}
	if (live_fbs != 0) {
		sx_xunlock(&sc->sx);
		device_printf(sc->dev,
		    "detach: %u framebuffer(s) still tracked, EBUSY\n",
		    live_fbs);
		return (EBUSY);
	}
	/*
	 * Detach-gate atomic_commit: any in-flight ATOMIC ioctl coming
	 * out of a file that opened before kms_dev_unregister returns
	 * will hit the sc->detaching check and return without touching
	 * ctrl_vq.  Set under sc->sx so commit_bind_fb never crosses it.
	 * Review #6.
	 */
	sc->detaching = true;
	sx_xunlock(&sc->sx);

	taskqueue_drain(taskqueue_thread, &sc->hotplug_task);
	if (sc->boot_hook_installed) {
		config_intrhook_disestablish(&sc->boot_hook);
		sc->boot_hook_installed = false;
	}
	if (sc->console_attached) {
		/*
		 * detaching flag is already set above; refresh_tick +
		 * refresh_task_run both check it and no-op, so callout_drain
		 * + taskqueue_drain will complete without dispatching another
		 * ctrl_vq round-trip.  vt_fb_detach hands the console back
		 * to whatever driver ran before us (efifb / vga / dumb).
		 */
		callout_drain(&sc->refresh_co);
		taskqueue_drain(taskqueue_thread, &sc->refresh_task);
		vt_fb_detach(&sc->fb_info);
		sc->console_attached = false;
	}
	if (sc->drm_dev != NULL) {
		/*
		 * If the boot splash is still the active scanout,
		 * release_active() will detach_backing + unref it via
		 * active_resource_id.  Otherwise a userspace client
		 * replaced our splash mid-life, leaving splash_rid alive
		 * on the host without a scanout — detach/unref it now so
		 * the host isn't DMA-mapped to the pages we're about to
		 * contigfree.
		 */
		if (sc->splash_rid != 0 &&
		    sc->splash_rid != sc->active_resource_id) {
			(void)virtio_kms_detach_backing(sc, sc->splash_rid);
			(void)virtio_kms_resource_unref(sc, sc->splash_rid);
		}
		/*
		 * 2nd-review L3: release_active talks to the host over
		 * ctrl_vq; hold sc->sx just like every other caller of
		 * that helper so a future assert(SX_XLOCKED) doesn't panic.
		 */
		sx_xlock(&sc->sx);
		virtio_kms_release_active(sc);
		sx_xunlock(&sc->sx);
		virtio_kms_topology_teardown(sc);
		kms_dev_unregister(sc->drm_dev);
		sc->drm_dev = NULL;
	}
	if (sc->splash_va != NULL) {
		contigfree(sc->splash_va, sc->splash_size, M_DEVBUF);
		sc->splash_va = NULL;
		sc->splash_size = 0;
		sc->splash_pa = 0;
		sc->splash_rid = 0;
	}
	sx_destroy(&sc->ctrl_sx);
	sx_destroy(&sc->sx);
	return (0);
}

/*
 * ============================================================
 * Phase D: virtio_gpu 2D protocol wrappers
 * ============================================================
 *
 * The five commands the atomic_commit path uses:
 *   RESOURCE_CREATE_2D    — allocate a host-side surface
 *   RESOURCE_ATTACH_BACKING — associate guest pages as its backing
 *   SET_SCANOUT           — point a scanout at the resource
 *   TRANSFER_TO_HOST_2D   — push guest bytes to the host surface
 *   RESOURCE_FLUSH        — make the surface visible on the scanout
 *
 * Plus RESOURCE_UNREF on framebuffer destroy.  All use the shared
 * virtio_kms_req_resp helper from Phase B.
 *
 * Response validation is uniform: any non-OK_NODATA return type
 * from the host is logged and turned into EIO.  No fences are
 * requested -- see the note above sc->features.  Review #24.
 */

static int
virtio_kms_cmd_common_check(struct virtio_kms_softc *sc,
    const char *op, const struct virtio_gpu_ctrl_hdr *resp)
{
	if (le32toh(resp->type) != VIRTIO_GPU_RESP_OK_NODATA) {
		device_printf(sc->dev,
		    "%s: unexpected resp type 0x%x\n",
		    op, le32toh(resp->type));
		return (EIO);
	}
	return (0);
}

static int
virtio_kms_resource_create_2d(struct virtio_kms_softc *sc,
    uint32_t resource_id, uint32_t format, uint32_t width, uint32_t height)
{
	struct {
		struct virtio_gpu_resource_create_2d	req;
		char					pad;
		struct virtio_gpu_ctrl_hdr		resp;
	} s;
	int error;

	bzero(&s, sizeof(s));
	s.req.hdr.type = htole32(VIRTIO_GPU_CMD_RESOURCE_CREATE_2D);
	s.req.resource_id = htole32(resource_id);
	s.req.format = htole32(format);
	s.req.width = htole32(width);
	s.req.height = htole32(height);

	error = virtio_kms_req_resp(sc, &s.req, sizeof(s.req),
	    &s.resp, sizeof(s.resp));
	if (error != 0)
		return (error);
	return (virtio_kms_cmd_common_check(sc, "resource_create_2d",
	    &s.resp));
}

static int
virtio_kms_resource_unref(struct virtio_kms_softc *sc, uint32_t resource_id)
{
	struct {
		struct virtio_gpu_resource_unref	req;
		char					pad;
		struct virtio_gpu_ctrl_hdr		resp;
	} s;
	int error;

	bzero(&s, sizeof(s));
	s.req.hdr.type = htole32(VIRTIO_GPU_CMD_RESOURCE_UNREF);
	s.req.resource_id = htole32(resource_id);

	error = virtio_kms_req_resp(sc, &s.req, sizeof(s.req),
	    &s.resp, sizeof(s.resp));
	if (error != 0)
		return (error);
	return (virtio_kms_cmd_common_check(sc, "resource_unref", &s.resp));
}

/*
 * Attach the GEM object's pages as backing for the host resource.
 * `kms_gem_object_create` uses `vm_page_alloc_noobj_contig` so every
 * GEM object is a single contiguous physical run — we exploit that
 * and send exactly one virtio_gpu_mem_entry (npages*PAGE_SIZE at
 * page[0]'s physical address) rather than one entry per page.  That
 * keeps attach_backing under the direct-descriptor limit even for
 * multi-MB framebuffers where a per-page SG list would exceed the
 * virtqueue ring's segment count (Review #1).
 *
 * If a future kms_gem allocator ever hands out scattered pages this
 * assumption breaks and the routine must fall back to a page walk +
 * indirect descriptor negotiation.
 */
static int
virtio_kms_attach_backing(struct virtio_kms_softc *sc, uint32_t resource_id,
    struct drm_gem_object *gem)
{
	struct {
		struct virtio_gpu_resource_attach_backing	req;
		struct virtio_gpu_mem_entry			mem[1];
		char						pad;
		struct virtio_gpu_ctrl_hdr			resp;
	} s;
	int error;

	if (gem == NULL || gem->pages == NULL || gem->npages == 0)
		return (EINVAL);

	bzero(&s, sizeof(s));
	s.req.hdr.type = htole32(VIRTIO_GPU_CMD_RESOURCE_ATTACH_BACKING);
	s.req.resource_id = htole32(resource_id);
	s.req.nr_entries = htole32(1);
	s.mem[0].addr = htole64(VM_PAGE_TO_PHYS(gem->pages[0]));
	s.mem[0].length = htole32((uint32_t)(gem->npages * PAGE_SIZE));

	error = virtio_kms_req_resp(sc, &s.req,
	    sizeof(s.req) + sizeof(s.mem), &s.resp, sizeof(s.resp));
	if (error != 0)
		return (error);
	return (virtio_kms_cmd_common_check(sc, "attach_backing", &s.resp));
}

static int
virtio_kms_detach_backing(struct virtio_kms_softc *sc, uint32_t resource_id)
{
	struct {
		struct virtio_gpu_resource_detach_backing req;
		char					pad;
		struct virtio_gpu_ctrl_hdr		resp;
	} s;
	int error;

	bzero(&s, sizeof(s));
	s.req.hdr.type = htole32(VIRTIO_GPU_CMD_RESOURCE_DETACH_BACKING);
	s.req.resource_id = htole32(resource_id);

	error = virtio_kms_req_resp(sc, &s.req, sizeof(s.req),
	    &s.resp, sizeof(s.resp));
	if (error != 0)
		return (error);
	return (virtio_kms_cmd_common_check(sc, "detach_backing", &s.resp));
}

static int
virtio_kms_set_scanout(struct virtio_kms_softc *sc, uint32_t scanout_id,
    uint32_t resource_id, uint32_t x, uint32_t y, uint32_t w, uint32_t h)
{
	struct {
		struct virtio_gpu_set_scanout	req;
		char				pad;
		struct virtio_gpu_ctrl_hdr	resp;
	} s;
	int error;

	bzero(&s, sizeof(s));
	s.req.hdr.type = htole32(VIRTIO_GPU_CMD_SET_SCANOUT);
	s.req.scanout_id = htole32(scanout_id);
	s.req.resource_id = htole32(resource_id);
	s.req.r.x = htole32(x);
	s.req.r.y = htole32(y);
	s.req.r.width = htole32(w);
	s.req.r.height = htole32(h);

	error = virtio_kms_req_resp(sc, &s.req, sizeof(s.req),
	    &s.resp, sizeof(s.resp));
	if (error != 0)
		return (error);
	return (virtio_kms_cmd_common_check(sc, "set_scanout", &s.resp));
}

static int
virtio_kms_transfer_to_host_2d(struct virtio_kms_softc *sc,
    uint32_t resource_id, uint32_t x, uint32_t y, uint32_t w, uint32_t h)
{
	struct {
		struct virtio_gpu_transfer_to_host_2d	req;
		char					pad;
		struct virtio_gpu_ctrl_hdr		resp;
	} s;
	int error;

	bzero(&s, sizeof(s));
	s.req.hdr.type = htole32(VIRTIO_GPU_CMD_TRANSFER_TO_HOST_2D);
	s.req.resource_id = htole32(resource_id);
	s.req.offset = htole64(0);	/* whole surface for now */
	s.req.r.x = htole32(x);
	s.req.r.y = htole32(y);
	s.req.r.width = htole32(w);
	s.req.r.height = htole32(h);

	error = virtio_kms_req_resp(sc, &s.req, sizeof(s.req),
	    &s.resp, sizeof(s.resp));
	if (error != 0)
		return (error);
	return (virtio_kms_cmd_common_check(sc, "transfer_to_host_2d",
	    &s.resp));
}

static int
virtio_kms_resource_flush(struct virtio_kms_softc *sc, uint32_t resource_id,
    uint32_t x, uint32_t y, uint32_t w, uint32_t h)
{
	struct {
		struct virtio_gpu_resource_flush	req;
		char					pad;
		struct virtio_gpu_ctrl_hdr		resp;
	} s;
	int error;

	bzero(&s, sizeof(s));
	s.req.hdr.type = htole32(VIRTIO_GPU_CMD_RESOURCE_FLUSH);
	s.req.resource_id = htole32(resource_id);
	s.req.r.x = htole32(x);
	s.req.r.y = htole32(y);
	s.req.r.width = htole32(w);
	s.req.r.height = htole32(h);

	error = virtio_kms_req_resp(sc, &s.req, sizeof(s.req),
	    &s.resp, sizeof(s.resp));
	if (error != 0)
		return (error);
	return (virtio_kms_cmd_common_check(sc, "resource_flush", &s.resp));
}

/*
 * Register an fb -> resource_id mapping so virtio_kms_fb_destroy can
 * later release the host resource.  Caller holds sc->sx.  Idempotent
 * for the same (fb, rid) pair; a duplicate insert would signal a
 * driver bug (fb bound twice with different rids).
 *
 * 2nd-review L1: resource_id piggybacks on fb->base.id, the framework's
 * per-mode_object id.  Reuse across RMFB+ADDFB2 is safe because
 * virtio_kms_req_resp fully serialises ctrl-vq traffic under ctrl_sx --
 * the RESOURCE_UNREF from the destroy hook completes on the host side
 * before the next RESOURCE_CREATE_2D dispatches.  Do NOT pipeline
 * ctrl-vq commands without revisiting this invariant.
 */
static void
virtio_kms_fb_track(struct virtio_kms_softc *sc,
    const struct drm_framebuffer *fb, uint32_t resource_id)
{
	struct virtio_kms_fb_entry *e;

	sx_assert(&sc->sx, SA_XLOCKED);
	LIST_FOREACH(e, &sc->fb_map, link) {
		if (e->fb == fb) {
			KASSERT(e->resource_id == resource_id,
			    ("virtio_kms: fb %p tracked with rid %u, now %u",
			    fb, e->resource_id, resource_id));
			return;
		}
	}
	e = malloc(sizeof(*e), M_VIRTIO_KMS, M_WAITOK | M_ZERO);
	e->fb = fb;
	e->resource_id = resource_id;
	LIST_INSERT_HEAD(&sc->fb_map, e, link);
}

/*
 * Framebuffer .destroy hook (Review #10/#11).  Fires from
 * kms_framebuffer_free_obj on the last drop of the fb's mode_object
 * refcount, AFTER every GEM ref is released.  We pull the fb's slot
 * out of sc->fb_map, and -- if that resource_id was still on a
 * scanout -- send SET_SCANOUT(0), DETACH_BACKING, RESOURCE_UNREF so
 * the host doesn't scan freed guest memory.  If the fb wasn't the
 * currently-active one but still had a host resource (created by a
 * bind that was later swapped away), we still send DETACH + UNREF.
 *
 * Contract from drm_drv.h: must free(fb, M_KMS) before returning.
 */
static void
virtio_kms_fb_destroy(struct drm_framebuffer *fb)
{
	struct virtio_kms_softc *sc;
	struct virtio_kms_fb_entry *e, *tmp;
	uint32_t rid = 0;

	/*
	 * 2nd-review H1/H2: if the framework NULLed driver_priv at
	 * kms_dev_unregister time (driver already gone), the softc is
	 * either freed or being freed.  Skip protocol calls; free the
	 * fb allocation per contract.  This should not normally fire
	 * because virtio_kms_detach refuses EBUSY while fbs are tracked,
	 * but keep the guard as belt-and-braces against a stray drop
	 * from a file dtor that races detach.
	 */
	sc = virtio_kms_dev_to_sc(fb->dev);
	if (sc == NULL) {
		free(fb, M_KMS);
		return;
	}

	sx_xlock(&sc->sx);
	LIST_FOREACH_SAFE(e, &sc->fb_map, link, tmp) {
		if (e->fb == fb) {
			rid = e->resource_id;
			LIST_REMOVE(e, link);
			free(e, M_VIRTIO_KMS);
			break;
		}
	}
	if (rid != 0 && !sc->detaching) {
		if (sc->active_resource_id == rid) {
			/* release_active handles the SET_SCANOUT->0 dance. */
			virtio_kms_release_active(sc);
		} else {
			(void)virtio_kms_detach_backing(sc, rid);
			(void)virtio_kms_resource_unref(sc, rid);
		}
	}
	sx_xunlock(&sc->sx);

	free(fb, M_KMS);
}

static const struct drm_framebuffer_funcs virtio_kms_framebuffer_funcs = {
	.destroy = virtio_kms_fb_destroy,
};

/*
 * Detach + unref the currently-attached host resource, if any.
 * Called from atomic_commit before wiring a fresh framebuffer, from
 * atomic_commit when a CRTC is being blanked, and from detach.
 *
 * Order matters: SET_SCANOUT(resource_id=0) FIRST so the host stops
 * scanning the resource, then DETACH_BACKING, then RESOURCE_UNREF.
 * Skipping the SET_SCANOUT lets the host briefly scan pages that
 * DETACH_BACKING just released, which is the virtio-gpu equivalent
 * of the H3 flip-fb-hold race we fixed in kms core.  Review #8/#9.
 */
static void
virtio_kms_release_active(struct virtio_kms_softc *sc)
{
	if (sc->active_resource_id == 0)
		return;
	/*
	 * 2nd-review M1: SET_SCANOUT(0) MUST target the same scanout the
	 * resource was bound to.  Hardcoding scanout_id=0 leaves any
	 * non-zero scanout scanning a resource we're about to detach +
	 * unref, which is the exact race SET_SCANOUT-first-then-detach
	 * ordering was supposed to close.
	 */
	(void)virtio_kms_set_scanout(sc, sc->active_scanout_id, 0,
	    0, 0, 0, 0);
	(void)virtio_kms_detach_backing(sc, sc->active_resource_id);
	(void)virtio_kms_resource_unref(sc, sc->active_resource_id);
	sc->active_resource_id = 0;
	sc->active_scanout_id = 0;
	sc->active_width = 0;
	sc->active_height = 0;
}

/*
 * Wire a framebuffer to scanout 0 (Phase D single-scanout).  Steps:
 *   1. If a different resource is already live, tear it down.
 *   2. If this fb's resource_id isn't live yet, CREATE_2D +
 *      ATTACH_BACKING with the GEM object's page list.
 *   3. SET_SCANOUT → resource_id, sized to fb geometry.
 *   4. TRANSFER + FLUSH the full surface so the host paints
 *      whatever's already in guest RAM.
 *
 * On any protocol error we roll back to no-active-resource so the
 * next commit gets a clean CREATE.
 */
static int
virtio_kms_atomic_bind_fb(struct virtio_kms_softc *sc,
    struct drm_framebuffer *fb, uint32_t scanout_id)
{
	uint32_t rid = fb->base.id;
	uint32_t w = fb->width;
	uint32_t h = fb->height;
	int error;

	if (rid == 0 || rid == sc->active_resource_id) {
		/*
		 * Zero-id can't happen for a live fb; same-id means we're
		 * re-committing a page-flip.  Reuse the resource: just do
		 * a TRANSFER + FLUSH.
		 */
		if (rid == 0)
			return (EINVAL);
		error = virtio_kms_transfer_to_host_2d(sc, rid, 0, 0, w, h);
		if (error == 0)
			error = virtio_kms_resource_flush(sc, rid, 0, 0, w, h);
		return (error);
	}

	virtio_kms_release_active(sc);

	error = virtio_kms_resource_create_2d(sc, rid,
	    VIRTIO_GPU_FORMAT_B8G8R8X8_UNORM, w, h);
	if (error != 0)
		return (error);
	error = virtio_kms_attach_backing(sc, rid, fb->gem_objs[0]);
	if (error != 0) {
		(void)virtio_kms_resource_unref(sc, rid);
		return (error);
	}
	error = virtio_kms_set_scanout(sc, scanout_id, rid, 0, 0, w, h);
	if (error != 0) {
		(void)virtio_kms_detach_backing(sc, rid);
		(void)virtio_kms_resource_unref(sc, rid);
		return (error);
	}
	error = virtio_kms_transfer_to_host_2d(sc, rid, 0, 0, w, h);
	if (error == 0)
		error = virtio_kms_resource_flush(sc, rid, 0, 0, w, h);
	if (error != 0) {
		/*
		 * Roll back to no-active-resource: next commit will do a
		 * clean CREATE + ATTACH rather than treating a dead resource
		 * as page-flippable.  Review #7.
		 */
		(void)virtio_kms_set_scanout(sc, scanout_id, 0, 0, 0, 0, 0);
		(void)virtio_kms_detach_backing(sc, rid);
		(void)virtio_kms_resource_unref(sc, rid);
		return (error);
	}

	sc->active_resource_id = rid;
	sc->active_scanout_id = scanout_id;	/* 2nd-review M1 */
	sc->active_width = w;
	sc->active_height = h;

	/*
	 * Register the fb -> rid mapping so virtio_kms_fb_destroy can
	 * release the host resource when the fb dies -- even if the fb
	 * is swapped away from being active first and RMFB'd later.
	 * Review #10/#11.
	 */
	virtio_kms_fb_track(sc, fb, rid);

	VKMS_DPRINTF(sc, "bound resource %u (%ux%u) to scanout %u\n",
	    rid, w, h, scanout_id);
	return (0);
}

/*
 * ============================================================
 * Phase C: mode-config topology
 * ============================================================
 *
 * Single scanout so far: one CRTC + one primary plane + one
 * writeback-flavour virtual encoder + one virtual connector.  The
 * connector's `get_modes` hook consumes the cached sc->scanouts[0]
 * entry populated by Phase B's fetch_display_info, so userspace
 * (Xorg / Wayland / kmscube probe) sees whatever geometry the host
 * currently advertises.
 *
 * Phase F extends to N scanouts, at which point CRTC/plane/encoder/
 * connector each grow into per-scanout arrays.  Phase D wires the
 * atomic_commit body to actually push a SET_SCANOUT down ctrl_vq;
 * this commit only sets the funcs table so the property-driven ATOMIC
 * path can drive the framework's own state machine.
 */

/*
 * Look up the softc from any of the four topology objects by walking
 * back through drm_dev->driver_priv.  Cheaper than embedding a back
 * pointer in each object.
 */
static inline struct virtio_kms_softc *
virtio_kms_dev_to_sc(struct drm_device *drm_dev)
{
	return ((struct virtio_kms_softc *)drm_dev->driver_priv);
}

/*
 * Map a connector back to its scanout index by pointer arithmetic
 * inside the softc's connector[] array.  Cheap and stable — every
 * connector we ever publish is one of ours.
 */
static uint32_t
virtio_kms_connector_to_scanout(struct virtio_kms_softc *sc,
    struct drm_connector *connector)
{
	uint32_t idx = (uint32_t)(connector - &sc->connector[0]);
	KASSERT(idx < VIRTIO_KMS_MAX_SCANOUTS,
	    ("virtio_kms: connector %p not in sc->connector[]", connector));
	return (idx);
}

/*
 * Fetch the EDID blob for a specific scanout via VIRTIO_GPU_CMD_GET_EDID.
 * Only issued when the host advertised VIRTIO_GPU_F_EDID at negotiate
 * time.  Result cached in sc->scanouts[i].edid so subsequent get_modes
 * calls don't hit the wire.  Returns 0 on success (edid_len non-zero),
 * errno otherwise; caller decides whether to fall through to the CVT
 * fallback.
 */
static int
virtio_kms_fetch_edid(struct virtio_kms_softc *sc, uint32_t scanout_id)
{
	struct {
		struct virtio_gpu_cmd_get_edid	req;
		char				pad;
		struct virtio_gpu_resp_edid	resp;
	} s;
	uint32_t sz;
	int error;

	if ((sc->features & (1ULL << VIRTIO_GPU_F_EDID)) == 0)
		return (EOPNOTSUPP);
	if (scanout_id >= VIRTIO_KMS_MAX_SCANOUTS)
		return (EINVAL);

	bzero(&s, sizeof(s));
	s.req.hdr.type = htole32(VIRTIO_GPU_CMD_GET_EDID);
	s.req.scanout = htole32(scanout_id);

	error = virtio_kms_req_resp(sc, &s.req, sizeof(s.req),
	    &s.resp, sizeof(s.resp));
	if (error != 0)
		return (error);

	if (le32toh(s.resp.hdr.type) != VIRTIO_GPU_RESP_OK_EDID) {
		device_printf(sc->dev,
		    "get_edid[%u]: unexpected resp type 0x%x\n",
		    scanout_id, le32toh(s.resp.hdr.type));
		return (EIO);
	}
	sz = le32toh(s.resp.size);
	if (sz == 0 || sz > sizeof(sc->scanouts[scanout_id].edid))
		return (EIO);

	memcpy(sc->scanouts[scanout_id].edid, s.resp.edid, sz);
	sc->scanouts[scanout_id].edid_len = sz;
	VKMS_DPRINTF(sc, "get_edid[%u]: cached %u bytes\n", scanout_id, sz);
	return (0);
}

/*
 * Connector .get_modes: publish either the EDID-derived preferred
 * mode (when VIRTIO_GPU_F_EDID negotiated + fetch succeeded) or a
 * CVT-shaped fallback sized from the display-info cache.  Returns
 * the number of modes added.
 */
static int
virtio_kms_get_modes(struct drm_connector *connector)
{
	struct virtio_kms_softc *sc = virtio_kms_dev_to_sc(connector->dev);
	uint32_t idx = virtio_kms_connector_to_scanout(sc, connector);
	struct virtio_kms_scanout *s = &sc->scanouts[idx];
	struct drm_display_mode *mode;
	uint32_t hdisplay, vdisplay;

	/*
	 * Best-effort EDID fetch (Phase F).  edid_tried is a one-shot
	 * gate so subsequent get_modes calls don't hit the wire again
	 * on a host that refuses GET_EDID.  Hotplug clears it.
	 * Review #22.
	 *
	 * 2nd-review L4: the blob is cached in sc->scanouts[idx].edid but
	 * NOT parsed -- the timing published below is CVT-shaped from the
	 * display-info geometry, not the EDID DTD.  Compositors read the
	 * EDID blob directly via the connector's EDID property, so the
	 * cache still serves that path.  A parser lands with an EDID-DTD
	 * timing when a real modeset user complains that the CVT clock
	 * misses their monitor's exact pixel clock.  TODO: parse DTD.
	 */
	if (!s->edid_tried) {
		s->edid_tried = true;
		(void)virtio_kms_fetch_edid(sc, idx);
	}

	if (sc->num_scanouts == 0 || s->enabled == 0) {
		hdisplay = 1024;
		vdisplay = 768;
	} else {
		hdisplay = s->width;
		vdisplay = s->height;
	}

	mode = kms_mode_create();
	if (mode == NULL)
		return (0);
	mode->hdisplay = hdisplay;
	mode->vdisplay = vdisplay;
	mode->hsync_start = hdisplay + 88;
	mode->hsync_end   = hdisplay + 88 + 44;
	mode->htotal      = hdisplay + 88 + 44 + 148;
	mode->vsync_start = vdisplay + 4;
	mode->vsync_end   = vdisplay + 4 + 5;
	mode->vtotal      = vdisplay + 4 + 5 + 36;
	mode->clock       = mode->htotal * mode->vtotal * 60 / 1000;
	mode->type        = DRM_MODE_TYPE_DRIVER | DRM_MODE_TYPE_PREFERRED;
	mode->vrefresh    = 60;
	kms_mode_set_name(mode);
	kms_connector_add_mode(connector, mode);

	VKMS_DPRINTF(sc, "get_modes[%u]: published %ux%u (edid=%u bytes)\n",
	    idx, hdisplay, vdisplay, s->edid_len);
	return (1);
}

/*
 * CRTC set_config: legacy modeset ioctl entry.  Phase C stubs it out
 * so the framework's legacy-property fallback doesn't hit a NULL and
 * userspace's SETCRTC returns success even before the real modeset
 * lands.  Phase D fills in the SET_SCANOUT + RESOURCE_FLUSH sequence
 * against the framebuffer the caller passed.
 */
static int
virtio_kms_set_config(struct drm_mode_set *set)
{
	if (set == NULL || set->crtc == NULL)
		return (EINVAL);
	return (0);
}

static const struct drm_crtc_funcs virtio_kms_crtc_funcs = {
	.set_config = virtio_kms_set_config,
};
static const struct drm_plane_funcs virtio_kms_plane_funcs = { 0 };
static const struct drm_encoder_funcs virtio_kms_encoder_funcs = { 0 };
static const struct drm_connector_funcs virtio_kms_connector_funcs = {
	.get_modes = virtio_kms_get_modes,
};

/*
 * Atomic check: sanity-only in Phase C.  The framework hands us a
 * fully-populated drm_atomic_state and we walk it for obvious
 * garbage.  Phase D adds framebuffer-side validation.
 */
static int
virtio_kms_atomic_check(struct drm_device *dev __unused,
    struct drm_atomic_state *state)
{
	uint32_t i;

	if (state == NULL)
		return (EINVAL);
	for (i = 0; i < state->num_crtc; i++) {
		const struct drm_crtc_state *cs = state->crtc_states[i];

		if (cs == NULL || !cs->mode_changed || !cs->active)
			continue;
		if (cs->mode.hdisplay == 0 || cs->mode.vdisplay == 0 ||
		    cs->mode.clock == 0)
			return (EINVAL);
	}
	/*
	 * Reject an active plane whose fb has no GEM backing before
	 * commit hits the wire.  Compositors get "check failed" (which
	 * they retry with a different config) instead of a mid-commit
	 * "commit failed" that looks like a driver crash.  Review #14.
	 * Also reject fb whose stride isn't tightly packed -- virtio-gpu
	 * 2D TRANSFER assumes pitch == width * bpp (Review #16).
	 *
	 * 2nd-review M2: Phase D routes a single primary plane per CRTC.
	 * If a compositor batches multiple planes onto the same CRTC (a
	 * primary + an overlay), atomic_pick_fb silently truncates to the
	 * first match -- reject the batch instead so the compositor sees a
	 * clean check failure rather than silent-dropped planes.
	 */
	for (i = 0; i < state->num_plane; i++) {
		const struct drm_plane_state *ps = state->plane_states[i];
		uint32_t j, per_crtc;

		if (ps == NULL || ps->fb == NULL)
			continue;
		if (ps->fb->gem_objs[0] == NULL)
			return (EINVAL);
		if (ps->fb->pitches[0] != ps->fb->width * 4)
			return (EINVAL);
		if (ps->crtc == NULL)
			continue;
		per_crtc = 0;
		for (j = 0; j < state->num_plane; j++) {
			const struct drm_plane_state *ps2 =
			    state->plane_states[j];

			if (ps2 == NULL || ps2->fb == NULL)
				continue;
			if (ps2->crtc == ps->crtc)
				per_crtc++;
		}
		if (per_crtc > 1)
			return (EINVAL);
	}
	return (0);
}

/*
 * Atomic commit: Phase C is a no-op wrapper that just returns success.
 * Phase D walks the CRTC states and issues VIRTIO_GPU_CMD_SET_SCANOUT
 * + RESOURCE_FLUSH for each mode_changed / active_changed CRTC,
 * using the framebuffer routed to that CRTC's primary plane.
 */
/*
 * Pick the framebuffer routed to a specific CRTC out of the atomic
 * state.  Single primary plane per CRTC in Phase D, so the first
 * plane_state whose crtc field matches wins.  Returns NULL when
 * no plane is routed (blanking / partial disable).
 */
static struct drm_framebuffer *
virtio_kms_atomic_pick_fb(struct drm_atomic_state *state,
    struct drm_crtc *crtc)
{
	uint32_t i;

	for (i = 0; i < state->num_plane; i++) {
		const struct drm_plane_state *ps = state->plane_states[i];

		if (ps == NULL || ps->crtc != crtc || ps->fb == NULL)
			continue;
		return (ps->fb);
	}
	return (NULL);
}

/*
 * Atomic commit: for every crtc with mode_changed or active_changed,
 * either bind the routed framebuffer to scanout 0 (Phase D single-
 * scanout) or clear the scanout when the CRTC is being blanked.
 * Page-flip-only paths (fb changed, mode unchanged) also route here;
 * virtio_kms_atomic_bind_fb detects same-resource_id and does a
 * cheap TRANSFER + FLUSH.
 */
static int
virtio_kms_atomic_commit(struct drm_device *dev,
    struct drm_atomic_state *state, bool nonblock __unused)
{
	struct virtio_kms_softc *sc = virtio_kms_dev_to_sc(dev);
	struct drm_framebuffer *bind_fb;
	uint32_t bind_scanout;
	bool have_bind, saw_blank;
	uint32_t i;
	int error = 0;

	if (state == NULL)
		return (EINVAL);

	sx_xlock(&sc->sx);
	/* Detach-gate: virtqueues may be torn down.  Review #6. */
	if (sc->detaching) {
		sx_xunlock(&sc->sx);
		return (ENXIO);
	}

	/*
	 * Two-pass walk to avoid releasing a resource we just bound.
	 * Phase D has one active resource at a time, so we pick the
	 * first active+routed CRTC as the bind target and treat any
	 * other CRTC's blank as a bystander until a subsequent commit
	 * that carries no bind target.  Review #15.
	 */
	have_bind = false;
	saw_blank = false;
	bind_fb = NULL;
	bind_scanout = 0;
	for (i = 0; i < state->num_crtc; i++) {
		const struct drm_crtc_state *cs = state->crtc_states[i];

		if (cs == NULL)
			continue;
		if (!cs->active) {
			saw_blank = true;
			continue;
		}
		if (!have_bind) {
			struct drm_framebuffer *fb;

			fb = virtio_kms_atomic_pick_fb(state, cs->crtc);
			if (fb != NULL) {
				bind_fb = fb;
				bind_scanout = cs->crtc->index;
				have_bind = true;
			}
		}
	}

	if (have_bind) {
		error = virtio_kms_atomic_bind_fb(sc, bind_fb, bind_scanout);
		if (error != 0)
			device_printf(sc->dev,
			    "atomic_commit: bind_fb rc=%d\n", error);
	} else if (saw_blank) {
		virtio_kms_release_active(sc);
	}
	sx_xunlock(&sc->sx);
	return (error);
}

static const struct drm_mode_config_funcs virtio_kms_mode_config_funcs = {
	.atomic_check  = virtio_kms_atomic_check,
	.atomic_commit = virtio_kms_atomic_commit,
};

/*
 * Primary-plane format list.  virtio-gpu's 2D command surface
 * accepts BGRA / XRGB variants; XRGB8888 is the universal fit
 * for Xorg-modesetting + Wayland weston.  Phase F extends this
 * once the driver knows the negotiated feature set (e.g. VirGL
 * enables 3D formats).
 */
static const uint32_t virtio_kms_primary_formats[] = {
	DRM_FORMAT_XRGB8888,
	DRM_FORMAT_ARGB8888,
};

/*
 * Publish `n` scanouts as `n` independent CRTC+plane+encoder+connector
 * pipelines.  Always create at least one so /dev/dri/card0 has
 * something to bind against even when the host advertised zero.
 * Runs at attach and again after a hotplug event when the scanout
 * count changed (the framework tolerates re-init of already-initialised
 * objects; connectors get their modes cleared and repopulated on the
 * next get_modes).
 */
static int
virtio_kms_topology_init(struct virtio_kms_softc *sc)
{
	struct drm_mode_config *mc = &sc->drm_dev->mode_config;
	uint32_t n, i;
	int error;

	mc->max_width  = 4096;
	mc->max_height = 4096;

	n = sc->num_scanouts;
	if (n == 0)
		n = 1;
	if (n > VIRTIO_KMS_MAX_SCANOUTS)
		n = VIRTIO_KMS_MAX_SCANOUTS;

	/*
	 * Grow-only.  Called from attach with topology_n == 0 to publish
	 * the initial pipes, and again from hotplug_task with an already
	 * non-zero topology_n when the host adds a scanout.  Slots
	 * [topology_n, n) get published; slots [0, topology_n) are
	 * assumed already initialised and get their connector.status
	 * refreshed elsewhere (hotplug_task loop).  Review #13.
	 *
	 * 2nd-review M3: track per-step init progress in `partial` so the
	 * fail-path unwinds exactly the objects it registered.  The old
	 * coarse "clean everything from topology_n..i" loop would either
	 * skip the current slot's crtc[i] (i unchanged when kms_plane_init
	 * failed) or run kms_connector_cleanup on a slot where
	 * kms_connector_init was never called.
	 */
	for (i = sc->topology_n; i < n; i++) {
		uint32_t crtc_mask;
		int partial = 0;	/* 1=crtc, 2=+plane, 3=+encoder, 4=+connector */

		error = kms_crtc_init(sc->drm_dev, &sc->crtc[i],
		    &virtio_kms_crtc_funcs);
		if (error != 0)
			goto fail;
		partial = 1;
		crtc_mask = 1u << sc->crtc[i].index;

		error = kms_plane_init(sc->drm_dev, &sc->primary[i],
		    &virtio_kms_plane_funcs, DRM_PLANE_TYPE_PRIMARY,
		    crtc_mask, virtio_kms_primary_formats,
		    nitems(virtio_kms_primary_formats));
		if (error != 0)
			goto fail_partial;
		partial = 2;
		sc->crtc[i].primary_plane = &sc->primary[i];

		error = kms_encoder_init(sc->drm_dev, &sc->encoder[i],
		    &virtio_kms_encoder_funcs, DRM_MODE_ENCODER_VIRTUAL);
		if (error != 0)
			goto fail_partial;
		partial = 3;
		sc->encoder[i].possible_crtcs = crtc_mask;

		error = kms_connector_init(sc->drm_dev, &sc->connector[i],
		    &virtio_kms_connector_funcs,
		    DRM_MODE_CONNECTOR_VIRTUAL);
		if (error != 0)
			goto fail_partial;
		partial = 4;
		sc->connector[i].status =
		    (i < sc->num_scanouts && sc->scanouts[i].enabled != 0)
		    ? connector_status_connected
		    : connector_status_disconnected;
		kms_connector_attach_encoder(&sc->connector[i],
		    &sc->encoder[i]);

		/* Slot i fully published; loop advances. */
		continue;

fail_partial:
		/* Unwind only what got initialized in this iter. */
		if (partial >= 3)
			kms_encoder_cleanup(&sc->encoder[i]);
		if (partial >= 2)
			kms_plane_cleanup(&sc->primary[i]);
		if (partial >= 1)
			kms_crtc_cleanup(&sc->crtc[i]);
		goto fail;
	}
	sc->topology_n = n;
	return (0);

fail:
	/*
	 * Unwind previously-completed slots in this grow-step: slots
	 * [sc->topology_n, i) were fully initialised (all four objects)
	 * before we hit the failure.  The current-slot partial cleanup
	 * already ran at fail_partial.  Already-published slots
	 * (< sc->topology_n at entry) stay live so concurrent
	 * GETCONNECTOR ioctls don't crash.
	 */
	while (i > sc->topology_n) {
		i--;
		kms_connector_cleanup(&sc->connector[i]);
		kms_encoder_cleanup(&sc->encoder[i]);
		kms_plane_cleanup(&sc->primary[i]);
		kms_crtc_cleanup(&sc->crtc[i]);
	}
	return (error);
}

static void
virtio_kms_topology_teardown(struct virtio_kms_softc *sc)
{
	uint32_t i;

	for (i = sc->topology_n; i > 0; i--) {
		kms_connector_cleanup(&sc->connector[i - 1]);
		kms_encoder_cleanup(&sc->encoder[i - 1]);
		kms_plane_cleanup(&sc->primary[i - 1]);
		kms_crtc_cleanup(&sc->crtc[i - 1]);
	}
	sc->topology_n = 0;
}

/*
 * ============================================================
 * Phase F: hotplug + config-space event handling
 * ============================================================
 *
 * The virtio-gpu spec's config-change event routes through the
 * device's `events_read` register.  When the host asserts a new
 * display topology it sets VIRTIO_GPU_EVENT_DISPLAY in events_read
 * and (in a spec-compliant guest) raises a config-change interrupt.
 * We handle that by re-fetching display_info and firing
 * kms_connector_hotplug on each connector; userspace (Xorg / mutter)
 * probes get_modes again and picks up the new topology.
 *
 * The `hotplug_probe` sysctl exercises the same path without waiting
 * for a real config-change IRQ, so operators can force a re-scan
 * from userland.  The IRQ hookup lives in virtio_setup_intr from
 * Phase B and will call into the hotplug task once the notification
 * queue is wired -- deferred to a follow-up.
 */

static void
virtio_kms_hotplug_task(void *arg, int pending __unused)
{
	struct virtio_kms_softc *sc = arg;
	uint32_t events_read = 0;
	uint32_t clear = 0;
	uint32_t i, n;

	/* Read + ack the event register. */
	virtio_read_device_config(sc->dev,
	    offsetof(struct virtio_gpu_config, events_read),
	    &events_read, sizeof(events_read));
	if ((events_read & VIRTIO_GPU_EVENT_DISPLAY) == 0) {
		VKMS_DPRINTF(sc, "hotplug_task: no DISPLAY event\n");
		return;
	}
	clear = VIRTIO_GPU_EVENT_DISPLAY;
	virtio_write_device_config(sc->dev,
	    offsetof(struct virtio_gpu_config, events_clear),
	    &clear, sizeof(clear));

	/*
	 * Refresh cached geometry + invalidate cached EDID blobs under
	 * sc->sx, but DROP sc->sx before calling into framework helpers
	 * that acquire mode_config.mutex (kms_connector_modes_clear /
	 * kms_connector_hotplug).
	 *
	 * Lock order: hotplug takes sc->sx briefly, then framework helpers
	 * take mode_config.mutex.  So sc->sx -> mc->mutex is the direction.
	 * Any future path that takes mc->mutex and calls back into driver
	 * code holding sc->sx would AB/BA.  Today no such caller exists
	 * (atomic_commit is dispatched WITHOUT mc->mutex per
	 * drm_atomic.c:940; the PAGE_FLIP_EVENT arm block that does take
	 * mc->mutex fires AFTER atomic_commit returns), but connector
	 * .get_modes wrappers added by future work must not hold sc->sx
	 * on entry.  2nd-review L2 (previous comment was inverted).
	 * Review #5.
	 *
	 * If num_scanouts grew past our current topology_n we
	 * republish the additional pipes here so the hotplug loop's
	 * connector[] walk stays within initialised territory.
	 * Review #13.
	 */
	sx_xlock(&sc->sx);
	(void)virtio_kms_fetch_display_info(sc);
	n = sc->num_scanouts;
	if (n > VIRTIO_KMS_MAX_SCANOUTS)
		n = VIRTIO_KMS_MAX_SCANOUTS;
	if (n > sc->topology_n)
		(void)virtio_kms_topology_init(sc);	/* grows to n */
	for (i = 0; i < n; i++) {
		sc->scanouts[i].edid_len = 0;
		sc->scanouts[i].edid_tried = false;
	}
	sx_xunlock(&sc->sx);

	/*
	 * Walk only slots that topology_init has actually initialised.
	 * Walking [0, VIRTIO_KMS_MAX_SCANOUTS) touched connector[]
	 * entries with NULL ->dev and NULL mode-list heads, tripping a
	 * NULL deref inside kms_connector_modes_clear.  Review #12.
	 */
	for (i = 0; i < sc->topology_n; i++) {
		enum drm_connector_status status =
		    (i < sc->num_scanouts && sc->scanouts[i].enabled != 0)
		    ? connector_status_connected
		    : connector_status_disconnected;

		sc->connector[i].status = status;
		kms_connector_modes_clear(&sc->connector[i]);
		kms_connector_hotplug(&sc->connector[i], status);
	}

	device_printf(sc->dev,
	    "hotplug: %u scanouts, event acked\n", sc->num_scanouts);

	/*
	 * If pmode[0] geometry changed, tear down + rebuild splash + vt
	 * console at the new dims.  No-op if unchanged.
	 */
	virtio_kms_reprovision(sc);
}

static int
virtio_kms_sysctl_hotplug_probe(SYSCTL_HANDLER_ARGS)
{
	struct virtio_kms_softc *sc = arg1;
	int trig = 0, error;

	error = sysctl_handle_int(oidp, &trig, 0, req);
	if (error != 0 || req->newptr == NULL || trig == 0)
		return (error);
	taskqueue_enqueue(taskqueue_thread, &sc->hotplug_task);
	return (0);
}

static device_method_t virtio_kms_methods[] = {
	DEVMETHOD(device_probe,		virtio_kms_probe),
	DEVMETHOD(device_attach,	virtio_kms_attach),
	DEVMETHOD(device_detach,	virtio_kms_detach),
	DEVMETHOD_END
};

static driver_t virtio_kms_driver_kdrv = {
	VIRTIO_KMS_DRIVER_NAME,
	virtio_kms_methods,
	sizeof(struct virtio_kms_softc),
};

DRIVER_MODULE(virtio_kms, virtio_pci, virtio_kms_driver_kdrv, NULL, NULL);
DRIVER_MODULE(virtio_kms, virtio_mmio, virtio_kms_driver_kdrv, NULL, NULL);
MODULE_VERSION(virtio_kms, 1);
MODULE_DEPEND(virtio_kms, kms, 1, 1, 1);
MODULE_DEPEND(virtio_kms, virtio, 1, 1, 1);
