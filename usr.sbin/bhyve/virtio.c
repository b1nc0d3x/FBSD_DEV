/*-
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Copyright (c) 2013  Chris Torek <torek @ torek net>
 * All rights reserved.
 * Copyright (c) 2019 Joyent, Inc.
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

#include <sys/param.h>
#include <sys/uio.h>

#include <machine/atomic.h>

#include <dev/virtio/pci/virtio_pci_legacy_var.h>

#include <stddef.h>
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <pthread.h>
#include <pthread_np.h>

#include "bhyverun.h"
#include "debug.h"
#include "pci_emul.h"
#ifdef BHYVE_SNAPSHOT
#include "snapshot.h"
#endif
#include "virtio.h"

/*
 * Functions for dealing with generalized "virtual devices" as
 * defined by <https://www.google.com/#output=search&q=virtio+spec>
 */

/*
 * In case we decide to relax the "virtio softc comes at the
 * front of virtio-based device softc" constraint, let's use
 * this to convert.
 */
#define	DEV_SOFTC(vs) ((void *)(vs))

/*
 * Bumped whenever the virtio_softc / vqueue_info snapshot layout changes.
 */
#define	VI_SNAPSHOT_VERSION	2

/*
 * Link a virtio_softc to its constants, the device softc, and
 * the PCI emulation.
 */
void
vi_softc_linkup(struct virtio_softc *vs, struct virtio_consts *vc,
		void *dev_softc, struct pci_devinst *pi,
		struct vqueue_info *queues)
{
	int i;

	/* vs and dev_softc addresses must match */
	assert((void *)vs == dev_softc);
	vs->vs_vc = vc;
	vs->vs_pi = pi;
	vs->vs_modern_bar = -1;		/* legacy-only until modern caps added */
	pi->pi_arg = vs;

	vs->vs_queues = queues;
	for (i = 0; i < vc->vc_nvq; i++) {
		queues[i].vq_vs = vs;
		queues[i].vq_num = i;
	}
}

/*
 * Reset device (device-wide).  This erases all queues, i.e.,
 * all the queues become invalid (though we don't wipe out the
 * internal pointers, we just clear the VQ_ALLOC flag).
 *
 * It resets negotiated features to "none".
 *
 * If MSI-X is enabled, this also resets all the vectors to NO_VECTOR.
 */
void
vi_reset_dev(struct virtio_softc *vs)
{
	struct vqueue_info *vq;
	int i, nvq;

	if (vs->vs_mtx)
		assert(pthread_mutex_isowned_np(vs->vs_mtx));

	nvq = vs->vs_vc->vc_nvq;
	for (vq = vs->vs_queues, i = 0; i < nvq; vq++, i++) {
		vq->vq_flags = 0;
		vq->vq_last_avail = 0;
		vq->vq_next_used = 0;
		vq->vq_save_used = 0;
		vq->vq_pfn = 0;
		vq->vq_msix_idx = VIRTIO_MSI_NO_VECTOR;
		vq->vq_desc_addr = 0;
		vq->vq_avail_addr = 0;
		vq->vq_used_addr = 0;
		vq->vq_enable = 0;
		/*
		 * Drop the driver's size selection so the device
		 * re-advertises its real size to the next driver.  Do NOT
		 * move vq_qsize itself: this function deliberately leaves
		 * vq_desc/vq_avail/vq_used mapped, and an I/O still in
		 * flight completes into vq_relchain()/vq_endchains(), which
		 * mask with vq_qsize and index the event-index slot at
		 * vq_qsize without checking VQ_ALLOC.  Growing it here would
		 * run those past the extent paddr_guest2host() validated.
		 * vq_qsize is only ever moved by vi_modern_vq_map(), where
		 * the new value and its mapping are established together.
		 *
		 * What this buys is host-side safety only: whatever a late
		 * completion touches is a mapping that was validated for the
		 * size in effect at the time.  It does not make the request
		 * itself survive.  A guest that resets with I/O outstanding
		 * and then programs a fresh ring will have the completion
		 * publish a stale descriptor head into the new used ring --
		 * guest-visible corruption of the guest's own making, which
		 * the device does not try to prevent.
		 */
		vq->vq_qsize_neg = 0;
	}
	vs->vs_negotiated_caps = 0;
	vs->vs_curq = 0;
	vs->vs_feature_select = 0;
	vs->vs_guest_feature_select = 0;
	/* vs->vs_status = 0; -- redundant */
	if (vs->vs_isr)
		pci_lintr_deassert(vs->vs_pi);
	vs->vs_isr = 0;
	vs->vs_msix_cfg_idx = VIRTIO_MSI_NO_VECTOR;
}

/*
 * Set I/O BAR (usually 0) to map PCI config registers.
 */
void
vi_set_io_bar(struct virtio_softc *vs, int barnum)
{
	size_t size;

	/*
	 * ??? should we use VIRTIO_PCI_CONFIG_OFF(0) if MSI-X is disabled?
	 * Existing code did not...
	 */
	size = VIRTIO_PCI_CONFIG_OFF(1) + vs->vs_vc->vc_cfgsize;
	pci_emul_alloc_bar(vs->vs_pi, barnum, PCIBAR_IO, size);
}

/*
 * Modern virtio-pci transport (spec 1.0 §4.1.4).  BAR layout:
 *
 *   0x0000  COMMON_CFG    4K  virtio_pci_common_cfg
 *   0x1000  ISR_CFG       4K  1-byte ISR-status
 *   0x2000  NOTIFY_CFG    4K  per-queue doorbells (multiplier = 4)
 *   0x3000  DEVICE_CFG    4K  driver-provided vc_cfgsize bytes
 *
 * Total 16 KiB MEM32 BAR.  All four regions live inside the same BAR
 * so a driver's PCI-cap walk finds them via one allocation.  Read/
 * write dispatch across the four regions lands in iter 0.5-c.
 */
#define	VTCFG_MODERN_BAR_SIZE		0x4000
#define	VTCFG_MODERN_COMMON_OFF		0x0000
#define	VTCFG_MODERN_ISR_OFF		0x1000
#define	VTCFG_MODERN_NOTIFY_OFF		0x2000
#define	VTCFG_MODERN_DEVICE_OFF		0x3000
#define	VTCFG_MODERN_REGION_SIZE	0x1000
#define	VTCFG_MODERN_NOTIFY_MULT	4

static int
vi_emit_modern_cap(struct pci_devinst *pi, uint8_t cfg_type,
    uint8_t bar, uint32_t offset, uint32_t length)
{
	struct virtio_pci_cap cap;

	memset(&cap, 0, sizeof(cap));
	cap.cap_vndr = PCIY_VENDOR;
	cap.cap_len = sizeof(cap);
	cap.cfg_type = cfg_type;
	cap.bar = bar;
	cap.offset = offset;
	cap.length = length;
	return (pci_emul_add_capability(pi, (u_char *)&cap, sizeof(cap)));
}

int
vi_add_modern_capabilities(struct virtio_softc *vs, int barnum)
{
	struct pci_devinst *pi = vs->vs_pi;
	struct virtio_pci_notify_cap ncap;
	int err, i;

	/*
	 * VIRTIO_F_VERSION_1 is the backend's decision, not the transport's:
	 * a 1.0 device changes wire contracts a legacy backend may not
	 * implement (virtio-net, for one, must use the 12-byte v1 header
	 * unconditionally once it is negotiated).  Requiring it here turns
	 * a device that could never complete the 1.0 handshake into a loud
	 * failure at init instead of a silent one at probe.
	 */
	if ((vs->vs_vc->vc_hv_caps & VIRTIO_F_VERSION_1) == 0)
		return (-1);

	/*
	 * The DEVICE_CFG region is one VTCFG_MODERN_REGION_SIZE slot at the
	 * top of the BAR, and vi_pci_modern_read() masks the offset to that
	 * width, so a larger config could never be served.
	 *
	 * Check it here, with the other virtio_consts invariants, and
	 * before anything is allocated.  Failing after pci_emul_alloc_bar()
	 * would leave a registered mem_range whose arg1 points at the
	 * pci_devinst that pci_emul_init() frees on a non-zero pe_init
	 * return -- there is no unregister on that path, so the first guest
	 * access to the BAR would be a use-after-free.
	 */
	if (vs->vs_vc->vc_cfgsize > VTCFG_MODERN_REGION_SIZE)
		return (-1);

	/*
	 * The modern BAR must be its own.  vi_pci_read()/vi_pci_write()
	 * test baridx against vs_modern_bar before the legacy
	 * assert(baridx == 0), so handing this the legacy I/O BAR or an
	 * MSI-X BAR would silently divert those accesses into the modern
	 * handlers with nothing to catch it.
	 */
	if (barnum == 0 || barnum == pci_msix_table_bar(pi) ||
	    barnum == pci_msix_pba_bar(pi))
		return (-1);

	/*
	 * Emit the capabilities first.  pci_emul_add_capability() can fail
	 * on a full config space, and every fallible step must complete
	 * before anything is allocated or armed: a non-zero return from
	 * here leaves pci_emul_init() to free the pci_devinst, and it does
	 * not unregister a BAR, so a half-configured device would leave a
	 * queued pci_bar_allocation and a live mem_range pointing at freed
	 * memory.  Capabilities are just bytes in config space and need no
	 * unwinding.
	 */
	err = vi_emit_modern_cap(pi, VIRTIO_PCI_CAP_COMMON_CFG, barnum,
	    VTCFG_MODERN_COMMON_OFF,
	    sizeof(struct virtio_pci_common_cfg));
	if (err != 0)
		return (err);

	err = vi_emit_modern_cap(pi, VIRTIO_PCI_CAP_ISR_CFG, barnum,
	    VTCFG_MODERN_ISR_OFF, 1);
	if (err != 0)
		return (err);

	memset(&ncap, 0, sizeof(ncap));
	ncap.cap.cap_vndr = PCIY_VENDOR;
	ncap.cap.cap_len = sizeof(ncap);
	ncap.cap.cfg_type = VIRTIO_PCI_CAP_NOTIFY_CFG;
	ncap.cap.bar = barnum;
	ncap.cap.offset = VTCFG_MODERN_NOTIFY_OFF;
	ncap.cap.length = VTCFG_MODERN_REGION_SIZE;
	ncap.notify_off_multiplier = VTCFG_MODERN_NOTIFY_MULT;
	err = pci_emul_add_capability(pi, (u_char *)&ncap, sizeof(ncap));
	if (err != 0)
		return (err);

	/*
	 * Not emitted: VIRTIO_PCI_CAP_PCI_CFG.  Spec 1.0 §4.1.4.9 says a
	 * device MUST offer the PCI-config access window, but serving it
	 * means intercepting PCI config space for the device, which the
	 * virtio layer is not wired into.  Neither the Linux nor the
	 * FreeBSD modern driver looks for it -- both probe on
	 * COMMON/ISR/NOTIFY only -- so this is a conformance gap, not a
	 * boot failure.  A driver or validation suite that uses the window
	 * will need it implemented.
	 */

	/*
	 * Only publish DEVICE_CFG when the backend has a config region.
	 * virtio-rnd sets vc_cfgsize to 0, and a zero-length capability is
	 * rejected by the Linux and FreeBSD modern drivers, which fails the
	 * whole probe rather than just skipping the region.
	 */
	if (vs->vs_vc->vc_cfgsize != 0) {
		err = vi_emit_modern_cap(pi, VIRTIO_PCI_CAP_DEVICE_CFG, barnum,
		    VTCFG_MODERN_DEVICE_OFF, vs->vs_vc->vc_cfgsize);
		if (err != 0)
			return (err);
	}

	err = pci_emul_alloc_bar(pi, barnum, PCIBAR_MEM32,
	    VTCFG_MODERN_BAR_SIZE);
	if (err != 0)
		return (err);

	/*
	 * Latch the backend's queue sizes as the ceiling the driver may
	 * negotiate down from.  Spec 1.0 §4.1.4.3.2 lets the driver write
	 * queue_size, but only to shrink the queue; without a remembered
	 * maximum there is nothing to validate a write against.  Callers
	 * must therefore set vq_qsize before calling this function -- the
	 * same ordering vi_set_io_bar() already requires.
	 */
	for (i = 0; i < vs->vs_vc->vc_nvq; i++)
		vs->vs_queues[i].vq_qsize_max = vs->vs_queues[i].vq_qsize;

	/* Nothing below can fail; arm modern dispatch last. */
	vs->vs_modern_bar = barnum;
	return (0);
}

/*
 * Note a change to the device-specific configuration region.
 *
 * Spec 1.0 §4.1.4.3.1 requires config_generation to change whenever the
 * device config does, so a driver reading a multi-word config can detect
 * that it raced an update and retry.  Backends that mutate their config
 * (display resize, link status, capacity change) call this, which bumps
 * the generation and raises a configuration-change interrupt.
 *
 * Must be called WITHOUT vs_mtx held.  vi_interrupt() takes VS_LOCK on
 * the non-MSI-X path and vs_mtx is not recursive, so a caller holding it
 * would deadlock on a guest booted without MSI-X.  This matches
 * pci_vtblk_resized(), the one config-change interrupt caller in the
 * tree, which raises VIRTIO_PCI_ISR_CONFIG from a blockif callback with
 * no lock held.  (The queue path is not a precedent: pci_vtblk_done()
 * calls vq_interrupt() under vsc_mtx, which has the same non-MSI-X
 * hazard -- pre-existing, not something to copy.)
 *
 * Because the bump therefore cannot happen inside the backend's own
 * critical section, this closes the common case but not the whole race:
 * a driver can still read gen, read a config the backend has already
 * begun updating, and read the same gen again before the increment
 * lands.  A backend that needs the window fully closed should mutate
 * its config and call vi_config_generation_bump() in the same region
 * under vs_mtx, then call this afterwards for the interrupt; the extra
 * bump is harmless, since drivers compare generations rather than
 * counting them.
 */
void
vi_config_changed(struct virtio_softc *vs)
{
	uint16_t msix_idx;

	/*
	 * vs_msix_cfg_idx is written by the guest through common_cfg under
	 * vs_mtx, so sample it here rather than racing a concurrent
	 * msix_config write and delivering to a stale vector.
	 */
	VS_LOCK(vs);
	vi_config_generation_bump(vs);
	msix_idx = vs->vs_msix_cfg_idx;
	VS_UNLOCK(vs);
	vi_interrupt(vs, VIRTIO_PCI_ISR_CONFIG, msix_idx);
}

/*
 * Initialize MSI-X vector capabilities if we're to use MSI-X,
 * or MSI capabilities if not.
 *
 * We assume we want one MSI-X vector per queue, here, plus one
 * for the config vec.
 */
int
vi_intr_init(struct virtio_softc *vs, int barnum, int use_msix)
{
	int nvec;

	if (use_msix) {
		vs->vs_flags |= VIRTIO_USE_MSIX;
		VS_LOCK(vs);
		vi_reset_dev(vs); /* set all vectors to NO_VECTOR */
		VS_UNLOCK(vs);
		nvec = vs->vs_vc->vc_nvq + 1;
		if (pci_emul_add_msixcap(vs->vs_pi, nvec, barnum))
			return (1);
	} else
		vs->vs_flags &= ~VIRTIO_USE_MSIX;

	/* Only 1 MSI vector for bhyve */
	pci_emul_add_msicap(vs->vs_pi, 1);

	/* Legacy interrupts are mandatory for virtio devices */
	pci_lintr_request(vs->vs_pi);

	return (0);
}

/*
 * Initialize the currently-selected virtio queue (vs->vs_curq).
 * The guest just gave us a page frame number, from which we can
 * calculate the addresses of the queue.
 */
/*
 * The queue size the device advertises: the ceiling latched when the
 * modern capabilities were published, or the backend's own size on a
 * device that never published them.
 */
static uint16_t
vi_vq_advertised_size(struct vqueue_info *vq)
{

	return (vq->vq_qsize_max != 0 ? vq->vq_qsize_max : vq->vq_qsize);
}

static void
vi_vq_init(struct virtio_softc *vs, uint32_t pfn)
{
	struct vqueue_info *vq;
	uint64_t phys;
	size_t size;
	char *base;

	vq = &vs->vs_queues[vs->vs_curq];
	/*
	 * The legacy transport has no driver-selectable size: the ring is
	 * whatever QUEUE_NUM advertised.  Commit that here, alongside the
	 * mapping it sizes, so a size a previous modern driver negotiated
	 * down to cannot outlive it.
	 */
	vq->vq_qsize = vi_vq_advertised_size(vq);
	vq->vq_pfn = pfn;
	phys = (uint64_t)pfn << VRING_PFN;
	size = vring_size_aligned(vq->vq_qsize);
	base = paddr_guest2host(vs->vs_pi->pi_vmctx, phys, size);

	/* First page(s) are descriptors... */
	vq->vq_desc = (struct vring_desc *)base;
	base += vq->vq_qsize * sizeof(struct vring_desc);

	/* ... immediately followed by "avail" ring (entirely uint16_t's) */
	vq->vq_avail = (struct vring_avail *)base;
	base += (2 + vq->vq_qsize + 1) * sizeof(uint16_t);

	/* Then it's rounded up to the next page... */
	base = (char *)roundup2((uintptr_t)base, VRING_ALIGN);

	/* ... and the last page(s) are the used ring. */
	vq->vq_used = (struct vring_used *)base;

	/* Mark queue as allocated, and start at 0 when we use it. */
	vq->vq_flags = VQ_ALLOC;
	vq->vq_last_avail = 0;
	vq->vq_next_used = 0;
	vq->vq_save_used = 0;
}

/*
 * Helper inline for vq_getchain(): record the i'th "real"
 * descriptor.
 */
static inline void
_vq_record(int i, struct vring_desc *vd, struct vmctx *ctx, struct iovec *iov,
    int n_iov, struct vi_req *reqp)
{
	uint32_t len;
	uint64_t addr;

	if (i >= n_iov)
		return;
	len = atomic_load_32(&vd->len);
	addr = atomic_load_64(&vd->addr);
	iov[i].iov_len = len;
	iov[i].iov_base = paddr_guest2host(ctx, addr, len);
	if ((vd->flags & VRING_DESC_F_WRITE) == 0)
		reqp->readable++;
	else
		reqp->writable++;
}
#define	VQ_MAX_DESCRIPTORS	512	/* see below */

/*
 * Examine the chain of descriptors starting at the "next one" to
 * make sure that they describe a sensible request.  If so, return
 * the number of "real" descriptors that would be needed/used in
 * acting on this request.  This may be smaller than the number of
 * available descriptors, e.g., if there are two available but
 * they are two separate requests, this just returns 1.  Or, it
 * may be larger: if there are indirect descriptors involved,
 * there may only be one descriptor available but it may be an
 * indirect pointing to eight more.  We return 8 in this case,
 * i.e., we do not count the indirect descriptors, only the "real"
 * ones.
 *
 * Basically, this vets the "flags" and "next" field of each
 * descriptor and tells you how many are involved.  Since some may
 * be indirect, this also needs the vmctx (in the pci_devinst
 * at vs->vs_pi) so that it can find indirect descriptors.
 *
 * As we process each descriptor, we copy and adjust it (guest to
 * host address wise, also using the vmtctx) into the given iov[]
 * array (of the given size).  If the array overflows, we stop
 * placing values into the array but keep processing descriptors,
 * up to VQ_MAX_DESCRIPTORS, before giving up and returning -1.
 * So you, the caller, must not assume that iov[] is as big as the
 * return value (you can process the same thing twice to allocate
 * a larger iov array if needed, or supply a zero length to find
 * out how much space is needed).
 *
 * If some descriptor(s) are invalid, this prints a diagnostic message
 * and returns -1.  If no descriptors are ready now it simply returns 0.
 *
 * You are assumed to have done a vq_ring_ready() if needed (note
 * that vq_has_descs() does one).
 */
int
vq_getchain(struct vqueue_info *vq, struct iovec *iov, int niov,
	    struct vi_req *reqp)
{
	int i;
	u_int ndesc, n_indir;
	u_int idx, next;
	struct vi_req req;
	struct vring_desc *vdir, *vindir, *vp;
	struct vmctx *ctx;
	struct virtio_softc *vs;
	const char *name;

	vs = vq->vq_vs;
	name = vs->vs_vc->vc_name;
	memset(&req, 0, sizeof(req));

	/*
	 * Note: it's the responsibility of the guest not to
	 * update vq->vq_avail->idx until all of the descriptors
         * the guest has written are valid (including all their
         * "next" fields and "flags").
	 *
	 * Compute (vq_avail->idx - last_avail) in integers mod 2**16.  This is
	 * the number of descriptors the device has made available
	 * since the last time we updated vq->vq_last_avail.
	 *
	 * We just need to do the subtraction as an unsigned int,
	 * then trim off excess bits.
	 */
	idx = vq->vq_last_avail;
	ndesc = (uint16_t)((u_int)vq->vq_avail->idx - idx);
	if (ndesc == 0)
		return (0);
	if (ndesc > vq->vq_qsize) {
		/* XXX need better way to diagnose issues */
		EPRINTLN(
		    "%s: ndesc (%u) out of range, driver confused?",
		    name, (u_int)ndesc);
		return (-1);
	}

	/*
	 * Now count/parse "involved" descriptors starting from
	 * the head of the chain.
	 *
	 * To prevent loops, we could be more complicated and
	 * check whether we're re-visiting a previously visited
	 * index, but we just abort if the count gets excessive.
	 */
	ctx = vs->vs_pi->pi_vmctx;
	req.idx = next = vq->vq_avail->ring[idx & (vq->vq_qsize - 1)];
	vq->vq_last_avail++;
	for (i = 0; i < VQ_MAX_DESCRIPTORS; next = vdir->next) {
		if (next >= vq->vq_qsize) {
			EPRINTLN(
			    "%s: descriptor index %u out of range, "
			    "driver confused?",
			    name, next);
			return (-1);
		}
		vdir = &vq->vq_desc[next];
		if ((vdir->flags & VRING_DESC_F_INDIRECT) == 0) {
			_vq_record(i, vdir, ctx, iov, niov, &req);
			i++;
		} else if ((vs->vs_negotiated_caps &
		    VIRTIO_RING_F_INDIRECT_DESC) == 0) {
			EPRINTLN(
			    "%s: descriptor has forbidden INDIRECT flag, "
			    "driver confused?",
			    name);
			return (-1);
		} else {
			n_indir = vdir->len / 16;
			if ((vdir->len & 0xf) || n_indir == 0) {
				EPRINTLN(
				    "%s: invalid indir len 0x%x, "
				    "driver confused?",
				    name, (u_int)vdir->len);
				return (-1);
			}
			vindir = paddr_guest2host(ctx,
			    vdir->addr, vdir->len);
			/*
			 * Indirects start at the 0th, then follow
			 * their own embedded "next"s until those run
			 * out.  Each one's indirect flag must be off
			 * (we don't really have to check, could just
			 * ignore errors...).
			 */
			next = 0;
			for (;;) {
				vp = &vindir[next];
				if (vp->flags & VRING_DESC_F_INDIRECT) {
					EPRINTLN(
					    "%s: indirect desc has INDIR flag,"
					    " driver confused?",
					    name);
					return (-1);
				}
				_vq_record(i, vp, ctx, iov, niov, &req);
				if (++i > VQ_MAX_DESCRIPTORS)
					goto loopy;
				if ((vp->flags & VRING_DESC_F_NEXT) == 0)
					break;
				next = vp->next;
				if (next >= n_indir) {
					EPRINTLN(
					    "%s: invalid next %u > %u, "
					    "driver confused?",
					    name, (u_int)next, n_indir);
					return (-1);
				}
			}
		}
		if ((vdir->flags & VRING_DESC_F_NEXT) == 0)
			goto done;
	}

loopy:
	EPRINTLN(
	    "%s: descriptor loop? count > %d - driver confused?",
	    name, i);
	return (-1);

done:
	*reqp = req;
	return (i);
}

/*
 * Return the first n_chain request chains back to the available queue.
 *
 * (These chains are the ones you handled when you called vq_getchain()
 * and used its positive return value.)
 */
void
vq_retchains(struct vqueue_info *vq, uint16_t n_chains)
{

	vq->vq_last_avail -= n_chains;
}

void
vq_relchain_prepare(struct vqueue_info *vq, uint16_t idx, uint32_t iolen)
{
	struct vring_used *vuh;
	struct vring_used_elem *vue;
	uint16_t mask;

	/*
	 * Notes:
	 *  - mask is N-1 where N is a power of 2 so computes x % N
	 *  - vuh points to the "used" data shared with guest
	 *  - vue points to the "used" ring entry we want to update
	 */
	mask = vq->vq_qsize - 1;
	vuh = vq->vq_used;

	vue = &vuh->ring[vq->vq_next_used++ & mask];
	vue->id = idx;
	vue->len = iolen;
}

void
vq_relchain_publish(struct vqueue_info *vq)
{
	/*
	 * Ensure the used descriptor is visible before updating the index.
	 * This is necessary on ISAs with memory ordering less strict than x86
	 * (and even on x86 to act as a compiler barrier).
	 */
	atomic_thread_fence_rel();
	vq->vq_used->idx = vq->vq_next_used;
}

/*
 * Return specified request chain to the guest, setting its I/O length
 * to the provided value.
 *
 * (This chain is the one you handled when you called vq_getchain()
 * and used its positive return value.)
 */
void
vq_relchain(struct vqueue_info *vq, uint16_t idx, uint32_t iolen)
{
	vq_relchain_prepare(vq, idx, iolen);
	vq_relchain_publish(vq);
}

/*
 * Driver has finished processing "available" chains and calling
 * vq_relchain on each one.  If driver used all the available
 * chains, used_all should be set.
 *
 * If the "used" index moved we may need to inform the guest, i.e.,
 * deliver an interrupt.  Even if the used index did NOT move we
 * may need to deliver an interrupt, if the avail ring is empty and
 * we are supposed to interrupt on empty.
 *
 * Note that used_all_avail is provided by the caller because it's
 * a snapshot of the ring state when he decided to finish interrupt
 * processing -- it's possible that descriptors became available after
 * that point.  (It's also typically a constant 1/True as well.)
 */
void
vq_endchains(struct vqueue_info *vq, int used_all_avail)
{
	struct virtio_softc *vs;
	uint16_t event_idx, new_idx, old_idx;
	int intr;

	/*
	 * Interrupt generation: if we're using EVENT_IDX,
	 * interrupt if we've crossed the event threshold.
	 * Otherwise interrupt is generated if we added "used" entries,
	 * but suppressed by VRING_AVAIL_F_NO_INTERRUPT.
	 *
	 * In any case, though, if NOTIFY_ON_EMPTY is set and the
	 * entire avail was processed, we need to interrupt always.
	 */
	vs = vq->vq_vs;
	old_idx = vq->vq_save_used;
	vq->vq_save_used = new_idx = vq->vq_used->idx;

	/*
	 * Use full memory barrier between "idx" store from preceding
	 * vq_relchain() call and the loads from VQ_USED_EVENT_IDX() or
	 * "flags" field below.
	 */
	atomic_thread_fence_seq_cst();
	if (used_all_avail &&
	    (vs->vs_negotiated_caps & VIRTIO_F_NOTIFY_ON_EMPTY))
		intr = 1;
	else if (vs->vs_negotiated_caps & VIRTIO_RING_F_EVENT_IDX) {
		event_idx = VQ_USED_EVENT_IDX(vq);
		/*
		 * This calculation is per docs and the kernel
		 * (see src/sys/dev/virtio/virtio_ring.h).
		 */
		intr = (uint16_t)(new_idx - event_idx - 1) <
			(uint16_t)(new_idx - old_idx);
	} else {
		intr = new_idx != old_idx &&
		    !(vq->vq_avail->flags & VRING_AVAIL_F_NO_INTERRUPT);
	}
	if (intr)
		vq_interrupt(vs, vq);
}

/* Note: these are in sorted order to make for a fast search */
static struct config_reg {
	uint16_t	cr_offset;	/* register offset */
	uint8_t		cr_size;	/* size (bytes) */
	uint8_t		cr_ro;		/* true => reg is read only */
	const char	*cr_name;	/* name of reg */
} config_regs[] = {
	{ VIRTIO_PCI_HOST_FEATURES,	4, 1, "HOST_FEATURES" },
	{ VIRTIO_PCI_GUEST_FEATURES,	4, 0, "GUEST_FEATURES" },
	{ VIRTIO_PCI_QUEUE_PFN,		4, 0, "QUEUE_PFN" },
	{ VIRTIO_PCI_QUEUE_NUM,		2, 1, "QUEUE_NUM" },
	{ VIRTIO_PCI_QUEUE_SEL,		2, 0, "QUEUE_SEL" },
	{ VIRTIO_PCI_QUEUE_NOTIFY,	2, 0, "QUEUE_NOTIFY" },
	{ VIRTIO_PCI_STATUS,		1, 0, "STATUS" },
	{ VIRTIO_PCI_ISR,		1, 0, "ISR" },
	{ VIRTIO_MSI_CONFIG_VECTOR,	2, 0, "CONFIG_VECTOR" },
	{ VIRTIO_MSI_QUEUE_VECTOR,	2, 0, "QUEUE_VECTOR" },
};

static inline struct config_reg *
vi_find_cr(int offset) {
	u_int hi, lo, mid;
	struct config_reg *cr;

	lo = 0;
	hi = sizeof(config_regs) / sizeof(*config_regs) - 1;
	while (hi >= lo) {
		mid = (hi + lo) >> 1;
		cr = &config_regs[mid];
		if (cr->cr_offset == offset)
			return (cr);
		if (cr->cr_offset < offset)
			lo = mid + 1;
		else
			hi = mid - 1;
	}
	return (NULL);
}

/*
 * Modern virtio-pci transport BAR read/write (spec 1.0 §4.1.4).
 * The BAR is carved into four 4-KiB regions installed by
 * vi_add_modern_capabilities().  See VTCFG_MODERN_*_OFF above.
 */

/*
 * Width of each common_cfg register, so an access that spans two of them
 * can be rejected rather than silently matching the first and zeroing the
 * rest: a 4-byte read at offset 16 covers msix_config and num_queues, and
 * would otherwise report the device as having no queues.
 */
static int
vi_modern_common_width(uint64_t off)
{
#define	VTCFG_CC(field)							\
	case offsetof(struct virtio_pci_common_cfg, field):		\
		return (sizeof(((struct virtio_pci_common_cfg *)0)->field))

	switch (off) {
	VTCFG_CC(device_feature_select);
	VTCFG_CC(device_feature);
	VTCFG_CC(guest_feature_select);
	VTCFG_CC(guest_feature);
	VTCFG_CC(msix_config);
	VTCFG_CC(num_queues);
	VTCFG_CC(device_status);
	VTCFG_CC(config_generation);
	VTCFG_CC(queue_select);
	VTCFG_CC(queue_size);
	VTCFG_CC(queue_msix_vector);
	VTCFG_CC(queue_enable);
	VTCFG_CC(queue_notify_off);
	VTCFG_CC(queue_desc_lo);
	VTCFG_CC(queue_desc_hi);
	VTCFG_CC(queue_avail_lo);
	VTCFG_CC(queue_avail_hi);
	VTCFG_CC(queue_used_lo);
	VTCFG_CC(queue_used_hi);
	}
	return (0);
#undef	VTCFG_CC
}

static uint64_t
vi_modern_common_read(struct virtio_softc *vs, uint64_t off, int size)
{
	struct vqueue_info *vq;
	uint64_t v = 0;

	switch (off) {
	case offsetof(struct virtio_pci_common_cfg, device_feature_select):
		v = vs->vs_feature_select;
		break;
	case offsetof(struct virtio_pci_common_cfg, device_feature):
		/*
		 * Spec 1.0 §4.1.4.3.1: selects beyond the implemented range
		 * read as 0, rather than aliasing onto the high word and
		 * letting a driver probing for 64+ feature bits mis-negotiate.
		 */
		if (vs->vs_feature_select == 0)
			v = vs->vs_vc->vc_hv_caps & 0xffffffff;
		else if (vs->vs_feature_select == 1)
			v = vs->vs_vc->vc_hv_caps >> 32;
		else
			v = 0;
		break;
	case offsetof(struct virtio_pci_common_cfg, guest_feature_select):
		v = vs->vs_guest_feature_select;
		break;
	case offsetof(struct virtio_pci_common_cfg, guest_feature):
		if (vs->vs_guest_feature_select == 0)
			v = vs->vs_negotiated_caps & 0xffffffff;
		else if (vs->vs_guest_feature_select == 1)
			v = vs->vs_negotiated_caps >> 32;
		else
			v = 0;
		break;
	case offsetof(struct virtio_pci_common_cfg, msix_config):
		v = vs->vs_msix_cfg_idx;
		break;
	case offsetof(struct virtio_pci_common_cfg, num_queues):
		v = vs->vs_vc->vc_nvq;
		break;
	case offsetof(struct virtio_pci_common_cfg, device_status):
		v = vs->vs_status;
		break;
	case offsetof(struct virtio_pci_common_cfg, config_generation):
		v = vs->vs_config_generation;
		break;
	case offsetof(struct virtio_pci_common_cfg, queue_select):
		v = vs->vs_curq;
		break;
	default:
		if (vs->vs_curq >= vs->vs_vc->vc_nvq)
			break;
		vq = &vs->vs_queues[vs->vs_curq];
		switch (off) {
		case offsetof(struct virtio_pci_common_cfg, queue_size):
			v = vq->vq_qsize_neg != 0 ? vq->vq_qsize_neg :
			    (vq->vq_qsize_max != 0 ? vq->vq_qsize_max :
			    vq->vq_qsize);
			break;
		case offsetof(struct virtio_pci_common_cfg, queue_msix_vector):
			v = vq->vq_msix_idx; break;
		case offsetof(struct virtio_pci_common_cfg, queue_enable):
			v = vq->vq_enable; break;
		case offsetof(struct virtio_pci_common_cfg, queue_notify_off):
			v = vs->vs_curq; break;
		case offsetof(struct virtio_pci_common_cfg, queue_desc_lo):
			v = vq->vq_desc_addr & 0xffffffff; break;
		case offsetof(struct virtio_pci_common_cfg, queue_desc_hi):
			v = vq->vq_desc_addr >> 32; break;
		case offsetof(struct virtio_pci_common_cfg, queue_avail_lo):
			v = vq->vq_avail_addr & 0xffffffff; break;
		case offsetof(struct virtio_pci_common_cfg, queue_avail_hi):
			v = vq->vq_avail_addr >> 32; break;
		case offsetof(struct virtio_pci_common_cfg, queue_used_lo):
			v = vq->vq_used_addr & 0xffffffff; break;
		case offsetof(struct virtio_pci_common_cfg, queue_used_hi):
			v = vq->vq_used_addr >> 32; break;
		}
	}
	(void)size;
	return (v);
}

/*
 * Translate the three guest-physical ring addresses the driver programmed
 * through COMMON_CFG into host pointers.
 *
 * Every input here is guest-controlled, so validate before handing the
 * results to the vq_getchain() fast path: a zero address means the driver
 * never programmed that ring, and paddr_guest2host() yields NULL for any
 * range not wholly inside guest memory.  On failure the queue is left
 * unmapped and disabled rather than half-initialised.
 */
static int
vi_modern_vq_map(struct virtio_softc *vs, struct vqueue_info *vq)
{
	struct vmctx *ctx = vs->vs_pi->pi_vmctx;
	struct vring_desc *desc;
	struct vring_avail *avail;
	struct vring_used *used;
	size_t nq;

	/*
	 * The size the driver selected, defaulting to what the device
	 * advertised if it never wrote queue_size.  vq_qsize is committed
	 * from this only once the mapping below succeeds, so the ring size
	 * and the validated extents can never disagree.
	 */
	nq = vq->vq_qsize_neg != 0 ? vq->vq_qsize_neg :
	    vi_vq_advertised_size(vq);
	if (nq == 0 || (nq & (nq - 1)) != 0 || nq > vq->vq_qsize_max)
		return (-1);
	if (vq->vq_desc_addr == 0 || vq->vq_avail_addr == 0 ||
	    vq->vq_used_addr == 0)
		return (-1);

	/*
	 * avail is flags + idx + ring[nq] + used_event, all uint16_t.
	 * used is flags + idx + avail_event as uint16_t, plus ring[nq].
	 */
	desc = paddr_guest2host(ctx, vq->vq_desc_addr,
	    nq * sizeof(struct vring_desc));
	avail = paddr_guest2host(ctx, vq->vq_avail_addr,
	    (3 + nq) * sizeof(uint16_t));
	used = paddr_guest2host(ctx, vq->vq_used_addr,
	    3 * sizeof(uint16_t) + nq * sizeof(struct vring_used_elem));
	if (desc == NULL || avail == NULL || used == NULL)
		return (-1);

	vq->vq_qsize = nq;
	vq->vq_desc = desc;
	vq->vq_avail = avail;
	vq->vq_used = used;

	/*
	 * A ring that has just become live starts at index 0, as
	 * vi_vq_init() does for the legacy transport.  Without this a
	 * disable/re-enable cycle short of a full device reset would
	 * resume consuming a freshly programmed ring from the old
	 * position and emit bogus used entries.
	 */
	vq->vq_last_avail = 0;
	vq->vq_next_used = 0;
	vq->vq_save_used = 0;
	vq->vq_flags |= VQ_ALLOC;
	return (0);
}

static void
vi_modern_common_write(struct virtio_softc *vs, uint64_t off, int size,
    uint64_t v)
{
	struct vqueue_info *vq;
	uint64_t caps;
	int fok;

	switch (off) {
	case offsetof(struct virtio_pci_common_cfg, device_feature_select):
		vs->vs_feature_select = v;
		return;
	case offsetof(struct virtio_pci_common_cfg, guest_feature_select):
		vs->vs_guest_feature_select = v;
		return;
	case offsetof(struct virtio_pci_common_cfg, guest_feature):
		/*
		 * Mask against what the device actually offers, as the
		 * legacy VIRTIO_PCI_GUEST_FEATURES path does.  A driver
		 * must not be able to negotiate a feature the backend
		 * cannot implement: vc_apply_features() would then switch
		 * the backend to a layout it cannot produce.  Because the
		 * value is masked here it can never carry an unoffered
		 * bit, so FEATURES_OK below needs no separate refusal.
		 */
		/*
		 * Spec 1.0 §2.2.2: the device ignores feature writes after
		 * FEATURES_OK.  vc_apply_features() only fires on the
		 * FEATURES_OK edge, so a late write would leave the backend
		 * configured for one feature set while vq_endchains() reads
		 * another live out of vs_negotiated_caps -- enough to switch
		 * on event-index suppression against a ring the driver does
		 * not maintain an event index for.
		 */
		if (vs->vs_status & VIRTIO_CONFIG_S_FEATURES_OK)
			return;
		caps = vs->vs_vc->vc_hv_caps;
		if (vs->vs_guest_feature_select == 0)
			vs->vs_negotiated_caps =
			    (vs->vs_negotiated_caps & ~0xffffffffULL) |
			    (v & caps & 0xffffffffULL);
		else if (vs->vs_guest_feature_select == 1)
			vs->vs_negotiated_caps =
			    (vs->vs_negotiated_caps & 0xffffffffULL) |
			    ((v << 32) & caps & ~0xffffffffULL);
		return;
	case offsetof(struct virtio_pci_common_cfg, msix_config):
		vs->vs_msix_cfg_idx = v;
		return;
	case offsetof(struct virtio_pci_common_cfg, device_status):
		if (v == 0) {
			vs->vs_status = 0;
			/*
			 * A backend without vc_reset must still get the
			 * transport reset, or VQ_ALLOC stays set forever and
			 * the geometry freeze below rejects every subsequent
			 * queue register write -- the driver could never
			 * re-initialise the device.
			 */
			if (vs->vs_vc->vc_reset != NULL)
				(*vs->vs_vc->vc_reset)(DEV_SOFTC(vs));
			else
				vi_reset_dev(vs);
			return;
		}
		/*
		 * Spec 1.0 §3.1.1: a driver that does not accept
		 * VIRTIO_F_VERSION_1 is not a 1.0 driver, and the device
		 * must refuse to latch FEATURES_OK so it finds out.  The
		 * guest_feature masking above stops it adding features the
		 * backend never offered, but cannot stop it dropping this
		 * one.
		 */
		if ((v & VIRTIO_CONFIG_S_FEATURES_OK) &&
		    (vs->vs_negotiated_caps & VIRTIO_F_VERSION_1) == 0)
			v &= ~VIRTIO_CONFIG_S_FEATURES_OK;
		/*
		 * device_status is cumulative and the 1.0 handshake writes
		 * it repeatedly, so fire vc_apply_features() on the 0->1
		 * edge only.  The legacy path calls it exactly once, from
		 * the GUEST_FEATURES write; backends are not required to be
		 * idempotent.
		 */
		fok = (v & VIRTIO_CONFIG_S_FEATURES_OK) != 0 &&
		    (vs->vs_status & VIRTIO_CONFIG_S_FEATURES_OK) == 0;
		vs->vs_status = v;
		if (fok && vs->vs_vc->vc_apply_features != NULL)
			(*vs->vs_vc->vc_apply_features)(DEV_SOFTC(vs),
			    vs->vs_negotiated_caps);
		return;
	case offsetof(struct virtio_pci_common_cfg, queue_select):
		/*
		 * Read-write per spec 1.0 §4.1.4.3, so store what was
		 * written rather than clamping -- a driver must read back
		 * what it wrote.  Every use as an array subscript is guarded
		 * by vs_curq >= vc_nvq, and the width check in
		 * vi_pci_modern_write() limits this to a 2-byte value, so it
		 * cannot truncate to a negative int.
		 */
		vs->vs_curq = (int)(v & 0xffff);
		return;
	}

	if (vs->vs_curq >= vs->vs_vc->vc_nvq)
		return;
	vq = &vs->vs_queues[vs->vs_curq];

	/*
	 * Ring geometry is latched when the queue is mapped, and the mapped
	 * extents are validated against the size in effect at that moment.
	 * Growing queue_size afterwards (still within vq_qsize_max, so
	 * otherwise acceptable) would let vq_getchain() index past the end
	 * of the region paddr_guest2host() actually validated, and the
	 * event-index macros address at vq_qsize itself.
	 *
	 * Key this on VQ_ALLOC rather than vq_enable.  A transitional
	 * device can have a live ring with vq_enable == 0, because the
	 * legacy vi_vq_init() path sets VQ_ALLOC without ever touching
	 * vq_enable; and the disable path below intentionally leaves the
	 * mapping in place for I/O still in flight.  Only vi_reset_dev()
	 * clears VQ_ALLOC, which is the point at which a driver is allowed
	 * to reprogram the queue.
	 */
	if (vq->vq_enable != 0 || (vq->vq_flags & VQ_ALLOC) != 0) {
		switch (off) {
		case offsetof(struct virtio_pci_common_cfg, queue_size):
		case offsetof(struct virtio_pci_common_cfg, queue_desc_lo):
		case offsetof(struct virtio_pci_common_cfg, queue_desc_hi):
		case offsetof(struct virtio_pci_common_cfg, queue_avail_lo):
		case offsetof(struct virtio_pci_common_cfg, queue_avail_hi):
		case offsetof(struct virtio_pci_common_cfg, queue_used_lo):
		case offsetof(struct virtio_pci_common_cfg, queue_used_hi):
			return;
		}
	}

	switch (off) {
	case offsetof(struct virtio_pci_common_cfg, queue_size):
		/*
		 * Spec 1.0 §4.1.4.3.2 lets the driver shrink the queue but
		 * not grow it past the size the device published, and the
		 * ring index arithmetic requires a power of two.  Ignore
		 * anything else rather than carry it into vi_modern_vq_map().
		 */
		if (v == 0 || v > vq->vq_qsize_max || (v & (v - 1)) != 0)
			break;
		vq->vq_qsize_neg = v;
		break;
	case offsetof(struct virtio_pci_common_cfg, queue_msix_vector):
		vq->vq_msix_idx = v; break;
	case offsetof(struct virtio_pci_common_cfg, queue_enable):
		if (v != 1) {
			/*
			 * Stop accepting notifications, but leave the
			 * mapping intact: an I/O already in flight completes
			 * asynchronously into vq_relchain()/vq_endchains(),
			 * which dereference vq_desc/vq_avail/vq_used and
			 * mask with vq_qsize without checking VQ_ALLOC.
			 * Tearing any of that down here turns a queue
			 * disable into a guest-triggerable bhyve segfault or
			 * an out-of-bounds ring write.  vi_reset_dev() is
			 * what releases the queue.
			 */
			vq->vq_enable = 0;
			break;
		}
		/*
		 * Re-enabling a queue that is already mapped must not rewind
		 * vq_last_avail/vq_next_used under the driver, or the device
		 * re-consumes avail entries and republishes used entries
		 * from index 0.  Just reopen the gate.
		 *
		 * A consequence: disable, reprogram the addresses, re-enable
		 * reuses the old ring, because the geometry freeze above
		 * ignored the address writes.  That is deliberate -- keeping
		 * the mapping is what makes a late completion safe -- and no
		 * 1.0 driver does it, since resetting the device is the
		 * defined way to reprogram a queue and VIRTIO_F_RING_RESET
		 * is not offered.
		 */
		if (vq->vq_flags & VQ_ALLOC) {
			vq->vq_enable = 1;
			break;
		}
		if (vi_modern_vq_map(vs, vq) != 0)
			break;
		vq->vq_enable = 1;
		break;
	case offsetof(struct virtio_pci_common_cfg, queue_desc_lo):
		vq->vq_desc_addr = (vq->vq_desc_addr & ~0xffffffffULL) |
		    (v & 0xffffffffULL); break;
	case offsetof(struct virtio_pci_common_cfg, queue_desc_hi):
		vq->vq_desc_addr =
		    (vq->vq_desc_addr & 0xffffffffULL) | (v << 32); break;
	case offsetof(struct virtio_pci_common_cfg, queue_avail_lo):
		vq->vq_avail_addr = (vq->vq_avail_addr & ~0xffffffffULL) |
		    (v & 0xffffffffULL); break;
	case offsetof(struct virtio_pci_common_cfg, queue_avail_hi):
		vq->vq_avail_addr =
		    (vq->vq_avail_addr & 0xffffffffULL) | (v << 32); break;
	case offsetof(struct virtio_pci_common_cfg, queue_used_lo):
		vq->vq_used_addr = (vq->vq_used_addr & ~0xffffffffULL) |
		    (v & 0xffffffffULL); break;
	case offsetof(struct virtio_pci_common_cfg, queue_used_hi):
		vq->vq_used_addr =
		    (vq->vq_used_addr & 0xffffffffULL) | (v << 32); break;
	}
	(void)size;
}

uint64_t
vi_pci_modern_read(struct pci_devinst *pi, uint64_t offset, int size)
{
	struct virtio_softc *vs = pi->pi_arg;
	uint64_t region_off = offset & (VTCFG_MODERN_REGION_SIZE - 1);
	uint64_t max;
	uint32_t v;

	VS_LOCK(vs);
	if (offset >= VTCFG_MODERN_DEVICE_OFF) {
		/*
		 * Bound the driver-supplied offset before handing it to the
		 * backend.  region_off spans the whole 4 KiB region, while
		 * the config struct is vc_cfgsize bytes; backends copy out
		 * of it without validating because the legacy path already
		 * guards this for them, so an unchecked offset reads past
		 * the struct and leaks host heap to the guest.
		 */
		max = vs->vs_vc->vc_cfgsize;
		if (size < 1 || size > 4 || region_off + size > max) {
			VS_UNLOCK(vs);
			/* Same all-ones convention as the paths below. */
			return (size == 1 ? 0xff :
			    (size == 2 ? 0xffff : 0xffffffff));
		}
		/*
		 * Seed the result the way the legacy path does: a backend
		 * fills only `size' bytes, so an unseeded v would return
		 * stack garbage in the remainder.
		 */
		v = size == 1 ? 0xff : (size == 2 ? 0xffff : 0xffffffff);
		if (vs->vs_vc->vc_cfgread != NULL &&
		    vs->vs_vc->vc_cfgread(DEV_SOFTC(vs), region_off,
		    size, &v) != 0) {
			/*
			 * The backend rejected the read.  Return the seed,
			 * which is what the legacy path's bad: label yields,
			 * rather than whatever the callee may have left in v.
			 */
			v = size == 1 ? 0xff :
			    (size == 2 ? 0xffff : 0xffffffff);
		}
		VS_UNLOCK(vs);
		return (v);
	}
	if (offset >= VTCFG_MODERN_NOTIFY_OFF) {
		VS_UNLOCK(vs);
		return (0);
	}
	if (offset >= VTCFG_MODERN_ISR_OFF) {
		/*
		 * The ISR capability advertises one byte at offset 0.
		 * Clearing on a read anywhere in the 4 KiB window would let
		 * a stray access swallow an interrupt the driver's real read
		 * at offset 0 then never sees.
		 */
		if (region_off != 0) {
			VS_UNLOCK(vs);
			return (0);
		}
		v = vs->vs_isr;
		vs->vs_isr = 0;		/* a read clears this flag */
		if (v)
			pci_lintr_deassert(pi);
		VS_UNLOCK(vs);
		return (v);
	}
	/*
	 * The decode below keys on offset alone, so an access that does not
	 * match the register exactly would either drop part of a wider
	 * neighbour or report an adjacent register as zero.  Reject rather
	 * than guess.
	 */
	if (vi_modern_common_width(region_off) != size) {
		VS_UNLOCK(vs);
		return (0);
	}
	v = vi_modern_common_read(vs, region_off, size);
	VS_UNLOCK(vs);
	return (v);
}

void
vi_pci_modern_write(struct pci_devinst *pi, uint64_t offset, int size,
    uint64_t value)
{
	struct virtio_softc *vs = pi->pi_arg;
	uint64_t region_off = offset & (VTCFG_MODERN_REGION_SIZE - 1);
	uint64_t max;
	uint32_t idx;

	VS_LOCK(vs);
	if (offset >= VTCFG_MODERN_DEVICE_OFF) {
		/*
		 * Bound as in vi_pci_modern_read().  Note the fallback the
		 * legacy path uses for vc_cfgsize == 0 would be wrong here:
		 * legacy offsets are already bounded by the I/O BAR size,
		 * whereas region_off spans a full 4 KiB, so a backend with
		 * no config region must reject everything.
		 */
		max = vs->vs_vc->vc_cfgsize;
		if (size < 1 || size > 4 || region_off + size > max) {
			VS_UNLOCK(vs);
			return;
		}
		if (vs->vs_vc->vc_cfgwrite != NULL &&
		    vs->vs_vc->vc_cfgwrite(DEV_SOFTC(vs), region_off,
		    size, value) != 0)
			EPRINTLN("%s: write to bad device-config offset/size "
			    "%jd/%d", vs->vs_vc->vc_name,
			    (uintmax_t)region_off, size);
		VS_UNLOCK(vs);
		return;
	}
	if (offset >= VTCFG_MODERN_NOTIFY_OFF) {
		idx = region_off / VTCFG_MODERN_NOTIFY_MULT;
		if (idx < (uint32_t)vs->vs_vc->vc_nvq) {
			struct vqueue_info *vq = &vs->vs_queues[idx];

			/*
			 * Only ring the backend for a queue that is both
			 * mapped and enabled: vq_has_descs() dereferences
			 * vq_avail, which is NULL on a queue the driver
			 * never programmed.
			 *
			 * The handler runs with vs_mtx held, matching the
			 * legacy VIRTIO_PCI_QUEUE_NOTIFY path: backends rely
			 * on that serialisation for vq_last_avail and the
			 * used ring when two vCPUs ring the same doorbell.
			 */
			if (vq->vq_enable && (vq->vq_flags & VQ_ALLOC)) {
				if (vq->vq_notify)
					vq->vq_notify(DEV_SOFTC(vs), vq);
				else if (vs->vs_vc->vc_qnotify)
					vs->vs_vc->vc_qnotify(DEV_SOFTC(vs),
					    vq);
			}
		}
		VS_UNLOCK(vs);
		return;
	}
	if (offset >= VTCFG_MODERN_ISR_OFF) {
		VS_UNLOCK(vs);
		return;
	}
	/* See the width note in vi_pci_modern_read(). */
	if (vi_modern_common_width(region_off) != size) {
		VS_UNLOCK(vs);
		return;
	}
	vi_modern_common_write(vs, region_off, size, value);
	VS_UNLOCK(vs);
}

/*
 * Handle pci config space reads.
 * If it's to the MSI-X info, do that.
 * If it's part of the virtio standard stuff, do that.
 * Otherwise dispatch to the actual driver.
 */
uint64_t
vi_pci_read(struct pci_devinst *pi, int baridx, uint64_t offset, int size)
{
	struct virtio_softc *vs = pi->pi_arg;
	struct virtio_consts *vc;
	struct config_reg *cr;
	uint64_t virtio_config_size, max;
	const char *name;
	uint32_t newoff;
	uint32_t value;
	int error;

	if (vs->vs_flags & VIRTIO_USE_MSIX) {
		if (baridx == pci_msix_table_bar(pi) ||
		    baridx == pci_msix_pba_bar(pi)) {
			return (pci_emul_msix_tread(pi, offset, size));
		}
	}

	if (baridx == vs->vs_modern_bar)
		return (vi_pci_modern_read(pi, offset, size));
	/* XXX probably should do something better than just assert() */
	assert(baridx == 0);

	if (vs->vs_mtx)
		pthread_mutex_lock(vs->vs_mtx);

	vc = vs->vs_vc;
	name = vc->vc_name;
	value = size == 1 ? 0xff : size == 2 ? 0xffff : 0xffffffff;

	if (size != 1 && size != 2 && size != 4)
		goto bad;

	virtio_config_size = VIRTIO_PCI_CONFIG_OFF(pci_msix_enabled(pi));

	if (offset >= virtio_config_size) {
		/*
		 * Subtract off the standard size (including MSI-X
		 * registers if enabled) and dispatch to underlying driver.
		 * If that fails, fall into general code.
		 */
		newoff = offset - virtio_config_size;
		max = vc->vc_cfgsize ? vc->vc_cfgsize : 0x100000000;
		if (newoff + size > max)
			goto bad;
		if (vc->vc_cfgread != NULL)
			error = (*vc->vc_cfgread)(DEV_SOFTC(vs), newoff, size, &value);
		else
			error = 0;
		if (!error)
			goto done;
	}

bad:
	cr = vi_find_cr(offset);
	if (cr == NULL || cr->cr_size != size) {
		if (cr != NULL) {
			/* offset must be OK, so size must be bad */
			EPRINTLN(
			    "%s: read from %s: bad size %d",
			    name, cr->cr_name, size);
		} else {
			EPRINTLN(
			    "%s: read from bad offset/size %jd/%d",
			    name, (uintmax_t)offset, size);
		}
		goto done;
	}

	switch (offset) {
	case VIRTIO_PCI_HOST_FEATURES:
		value = vc->vc_hv_caps;
		break;
	case VIRTIO_PCI_GUEST_FEATURES:
		value = vs->vs_negotiated_caps;
		break;
	case VIRTIO_PCI_QUEUE_PFN:
		if (vs->vs_curq < vc->vc_nvq)
			value = vs->vs_queues[vs->vs_curq].vq_pfn;
		break;
	case VIRTIO_PCI_QUEUE_NUM:
		/*
		 * QUEUE_NUM is read-only and always reports what the device
		 * offers.  On a transitional device vq_qsize may still hold
		 * a size a previous modern driver negotiated down to, so
		 * prefer the advertised ceiling when there is one.
		 */
		value = vs->vs_curq < vc->vc_nvq ?
		    vi_vq_advertised_size(&vs->vs_queues[vs->vs_curq]) : 0;
		break;
	case VIRTIO_PCI_QUEUE_SEL:
		value = vs->vs_curq;
		break;
	case VIRTIO_PCI_QUEUE_NOTIFY:
		value = 0;	/* XXX */
		break;
	case VIRTIO_PCI_STATUS:
		value = vs->vs_status;
		break;
	case VIRTIO_PCI_ISR:
		value = vs->vs_isr;
		vs->vs_isr = 0;		/* a read clears this flag */
		if (value)
			pci_lintr_deassert(pi);
		break;
	case VIRTIO_MSI_CONFIG_VECTOR:
		value = vs->vs_msix_cfg_idx;
		break;
	case VIRTIO_MSI_QUEUE_VECTOR:
		value = vs->vs_curq < vc->vc_nvq ?
		    vs->vs_queues[vs->vs_curq].vq_msix_idx :
		    VIRTIO_MSI_NO_VECTOR;
		break;
	}
done:
	if (vs->vs_mtx)
		pthread_mutex_unlock(vs->vs_mtx);
	return (value);
}

/*
 * Handle pci config space writes.
 * If it's to the MSI-X info, do that.
 * If it's part of the virtio standard stuff, do that.
 * Otherwise dispatch to the actual driver.
 */
void
vi_pci_write(struct pci_devinst *pi, int baridx, uint64_t offset, int size,
    uint64_t value)
{
	struct virtio_softc *vs = pi->pi_arg;
	struct vqueue_info *vq;
	struct virtio_consts *vc;
	struct config_reg *cr;
	uint64_t virtio_config_size, max;
	const char *name;
	uint32_t newoff;
	int error;

	if (vs->vs_flags & VIRTIO_USE_MSIX) {
		if (baridx == pci_msix_table_bar(pi) ||
		    baridx == pci_msix_pba_bar(pi)) {
			pci_emul_msix_twrite(pi, offset, size, value);
			return;
		}
	}

	if (baridx == vs->vs_modern_bar) {
		vi_pci_modern_write(pi, offset, size, value);
		return;
	}
	/* XXX probably should do something better than just assert() */
	assert(baridx == 0);

	if (vs->vs_mtx)
		pthread_mutex_lock(vs->vs_mtx);

	vc = vs->vs_vc;
	name = vc->vc_name;

	if (size != 1 && size != 2 && size != 4)
		goto bad;

	virtio_config_size = VIRTIO_PCI_CONFIG_OFF(pci_msix_enabled(pi));

	if (offset >= virtio_config_size) {
		/*
		 * Subtract off the standard size (including MSI-X
		 * registers if enabled) and dispatch to underlying driver.
		 */
		newoff = offset - virtio_config_size;
		max = vc->vc_cfgsize ? vc->vc_cfgsize : 0x100000000;
		if (newoff + size > max)
			goto bad;
		if (vc->vc_cfgwrite != NULL)
			error = (*vc->vc_cfgwrite)(DEV_SOFTC(vs), newoff, size, value);
		else
			error = 0;
		if (!error)
			goto done;
	}

bad:
	cr = vi_find_cr(offset);
	if (cr == NULL || cr->cr_size != size || cr->cr_ro) {
		if (cr != NULL) {
			/* offset must be OK, wrong size and/or reg is R/O */
			if (cr->cr_size != size)
				EPRINTLN(
				    "%s: write to %s: bad size %d",
				    name, cr->cr_name, size);
			if (cr->cr_ro)
				EPRINTLN(
				    "%s: write to read-only reg %s",
				    name, cr->cr_name);
		} else {
			EPRINTLN(
			    "%s: write to bad offset/size %jd/%d",
			    name, (uintmax_t)offset, size);
		}
		goto done;
	}

	switch (offset) {
	case VIRTIO_PCI_GUEST_FEATURES:
		vs->vs_negotiated_caps = value & vc->vc_hv_caps;
		if (vc->vc_apply_features)
			(*vc->vc_apply_features)(DEV_SOFTC(vs),
			    vs->vs_negotiated_caps);
		break;
	case VIRTIO_PCI_QUEUE_PFN:
		if (vs->vs_curq >= vc->vc_nvq)
			goto bad_qindex;
		vi_vq_init(vs, value);
		break;
	case VIRTIO_PCI_QUEUE_SEL:
		/*
		 * Note that the guest is allowed to select an
		 * invalid queue; we just need to return a QNUM
		 * of 0 while the bad queue is selected.
		 */
		vs->vs_curq = value;
		break;
	case VIRTIO_PCI_QUEUE_NOTIFY:
		if (value >= (unsigned int)vc->vc_nvq) {
			EPRINTLN("%s: queue %d notify out of range",
				name, (int)value);
			goto done;
		}
		vq = &vs->vs_queues[value];
		if (vq->vq_notify)
			(*vq->vq_notify)(DEV_SOFTC(vs), vq);
		else if (vc->vc_qnotify)
			(*vc->vc_qnotify)(DEV_SOFTC(vs), vq);
		else
			EPRINTLN(
			    "%s: qnotify queue %d: missing vq/vc notify",
				name, (int)value);
		break;
	case VIRTIO_PCI_STATUS:
		vs->vs_status = value;
		if (value == 0)
			(*vc->vc_reset)(DEV_SOFTC(vs));
		break;
	case VIRTIO_MSI_CONFIG_VECTOR:
		vs->vs_msix_cfg_idx = value;
		break;
	case VIRTIO_MSI_QUEUE_VECTOR:
		if (vs->vs_curq >= vc->vc_nvq)
			goto bad_qindex;
		vq = &vs->vs_queues[vs->vs_curq];
		vq->vq_msix_idx = value;
		break;
	}
	goto done;

bad_qindex:
	EPRINTLN(
	    "%s: write config reg %s: curq %d >= max %d",
	    name, cr->cr_name, vs->vs_curq, vc->vc_nvq);
done:
	if (vs->vs_mtx)
		pthread_mutex_unlock(vs->vs_mtx);
}

#ifdef BHYVE_SNAPSHOT
int
vi_pci_pause(struct pci_devinst *pi)
{
	struct virtio_softc *vs;
	struct virtio_consts *vc;

	vs = pi->pi_arg;
	vc = vs->vs_vc;

	vc = vs->vs_vc;
	assert(vc->vc_pause != NULL);
	(*vc->vc_pause)(DEV_SOFTC(vs));

	return (0);
}

int
vi_pci_resume(struct pci_devinst *pi)
{
	struct virtio_softc *vs;
	struct virtio_consts *vc;

	vs = pi->pi_arg;
	vc = vs->vs_vc;

	vc = vs->vs_vc;
	assert(vc->vc_resume != NULL);
	(*vc->vc_resume)(DEV_SOFTC(vs));

	return (0);
}

static int
vi_pci_snapshot_softc(struct virtio_softc *vs, struct vm_snapshot_meta *meta)
{
	uint32_t vers = VI_SNAPSHOT_VERSION;
	int ret;

	/*
	 * SNAPSHOT_VAR_OR_LEAVE is a raw sizeof-based blob with no format
	 * version of its own, so a layout change deserialises misaligned
	 * rather than failing.  This series widened vs_negotiated_caps and
	 * added softc and per-queue fields, and vq_qsize -- previously a
	 * compared invariant that would have caught the drift -- is now
	 * restored state.  Compare a version first so a mismatch is loud.
	 */
	SNAPSHOT_VAR_CMP_OR_LEAVE(vers, meta, ret, done);

	SNAPSHOT_VAR_OR_LEAVE(vs->vs_flags, meta, ret, done);
	SNAPSHOT_VAR_OR_LEAVE(vs->vs_negotiated_caps, meta, ret, done);
	SNAPSHOT_VAR_OR_LEAVE(vs->vs_curq, meta, ret, done);
	SNAPSHOT_VAR_OR_LEAVE(vs->vs_status, meta, ret, done);
	SNAPSHOT_VAR_OR_LEAVE(vs->vs_isr, meta, ret, done);
	SNAPSHOT_VAR_OR_LEAVE(vs->vs_msix_cfg_idx, meta, ret, done);

	/* Modern transport state; inert when vs_modern_bar is -1. */
	SNAPSHOT_VAR_OR_LEAVE(vs->vs_feature_select, meta, ret, done);
	SNAPSHOT_VAR_OR_LEAVE(vs->vs_guest_feature_select, meta, ret, done);
	SNAPSHOT_VAR_OR_LEAVE(vs->vs_config_generation, meta, ret, done);

done:
	return (ret);
}

static int
vi_pci_snapshot_consts(struct virtio_consts *vc, struct vm_snapshot_meta *meta)
{
	int ret;

	SNAPSHOT_VAR_CMP_OR_LEAVE(vc->vc_nvq, meta, ret, done);
	SNAPSHOT_VAR_CMP_OR_LEAVE(vc->vc_cfgsize, meta, ret, done);
	SNAPSHOT_VAR_CMP_OR_LEAVE(vc->vc_hv_caps, meta, ret, done);

done:
	return (ret);
}

static int
vi_pci_snapshot_queues(struct virtio_softc *vs, struct vm_snapshot_meta *meta)
{
	int i;
	int ret;
	struct virtio_consts *vc;
	struct vqueue_info *vq;
	struct vmctx *ctx;
	uint64_t addr_size;

	ctx = vs->vs_pi->pi_vmctx;
	vc = vs->vs_vc;

	/* Save virtio queue info */
	for (i = 0; i < vc->vc_nvq; i++) {
		vq = &vs->vs_queues[i];

		/*
		 * vq_qsize used to be device-constant, which is why it was a
		 * compared invariant.  The modern transport lets the driver
		 * shrink it, so a guest that negotiated a smaller queue would
		 * fail the compare against a freshly initialised device on
		 * restore.  vq_qsize_max is the invariant now; vq_qsize is
		 * saved state.
		 */
		SNAPSHOT_VAR_CMP_OR_LEAVE(vq->vq_qsize_max, meta, ret, done);
		SNAPSHOT_VAR_OR_LEAVE(vq->vq_qsize, meta, ret, done);
		SNAPSHOT_VAR_OR_LEAVE(vq->vq_qsize_neg, meta, ret, done);
		SNAPSHOT_VAR_CMP_OR_LEAVE(vq->vq_num, meta, ret, done);

		SNAPSHOT_VAR_OR_LEAVE(vq->vq_flags, meta, ret, done);
		SNAPSHOT_VAR_OR_LEAVE(vq->vq_last_avail, meta, ret, done);
		SNAPSHOT_VAR_OR_LEAVE(vq->vq_next_used, meta, ret, done);
		SNAPSHOT_VAR_OR_LEAVE(vq->vq_save_used, meta, ret, done);
		SNAPSHOT_VAR_OR_LEAVE(vq->vq_msix_idx, meta, ret, done);

		SNAPSHOT_VAR_OR_LEAVE(vq->vq_pfn, meta, ret, done);

		/*
		 * Modern transport ring addresses.  The vq_desc / vq_avail /
		 * vq_used pointers themselves are re-translated below by the
		 * GUEST2HOST macros, which serve both transports; these are
		 * the GPAs the driver programmed, kept so the queue can be
		 * re-enabled coherently after restore.
		 */
		SNAPSHOT_VAR_OR_LEAVE(vq->vq_desc_addr, meta, ret, done);
		SNAPSHOT_VAR_OR_LEAVE(vq->vq_avail_addr, meta, ret, done);
		SNAPSHOT_VAR_OR_LEAVE(vq->vq_used_addr, meta, ret, done);
		SNAPSHOT_VAR_OR_LEAVE(vq->vq_enable, meta, ret, done);

		if (!vq_ring_ready(vq))
			continue;

		addr_size = vq->vq_qsize * sizeof(struct vring_desc);
		SNAPSHOT_GUEST2HOST_ADDR_OR_LEAVE(ctx, vq->vq_desc, addr_size,
			false, meta, ret, done);

		addr_size = (2 + vq->vq_qsize + 1) * sizeof(uint16_t);
		SNAPSHOT_GUEST2HOST_ADDR_OR_LEAVE(ctx, vq->vq_avail, addr_size,
			false, meta, ret, done);

		/*
		 * flags + idx + avail_event as uint16_t, plus ring[qsize] of
		 * vring_used_elem (8 bytes each).  The previous expression
		 * counted a used element as two uint16_t and so validated
		 * only half the ring; the modern branch below copies the
		 * full extent through this pointer, which would then run off
		 * the end of a mapping for a used ring placed near the end
		 * of a guest memory segment.
		 */
		addr_size = 3 * sizeof(uint16_t) +
		    vq->vq_qsize * sizeof(struct vring_used_elem);
		SNAPSHOT_GUEST2HOST_ADDR_OR_LEAVE(ctx, vq->vq_used, addr_size,
			false, meta, ret, done);

		/*
		 * Under the legacy transport desc/avail/used are one
		 * contiguous guest allocation, so a single buffer starting
		 * at vq_desc covers all three.  The modern transport lets
		 * the driver place them independently, so saving
		 * vring_size_aligned() bytes from vq_desc would copy
		 * unrelated guest memory on save and, worse, clobber the
		 * pages after the descriptor table on restore while missing
		 * the real avail and used rings.  Save each region on its
		 * own extent instead.
		 */
		if (vs->vs_modern_bar >= 0) {
			SNAPSHOT_BUF_OR_LEAVE(vq->vq_desc,
				vq->vq_qsize * sizeof(struct vring_desc),
				meta, ret, done);
			SNAPSHOT_BUF_OR_LEAVE(vq->vq_avail,
				(3 + vq->vq_qsize) * sizeof(uint16_t),
				meta, ret, done);
			SNAPSHOT_BUF_OR_LEAVE(vq->vq_used,
				3 * sizeof(uint16_t) +
				vq->vq_qsize * sizeof(struct vring_used_elem),
				meta, ret, done);
		} else {
			SNAPSHOT_BUF_OR_LEAVE(vq->vq_desc,
				vring_size_aligned(vq->vq_qsize), meta, ret,
				done);
		}
	}

done:
	return (ret);
}

int
vi_pci_snapshot(struct vm_snapshot_meta *meta)
{
	int ret;
	struct pci_devinst *pi;
	struct virtio_softc *vs;
	struct virtio_consts *vc;

	pi = meta->dev_data;
	vs = pi->pi_arg;
	vc = vs->vs_vc;

	/* Save virtio softc */
	ret = vi_pci_snapshot_softc(vs, meta);
	if (ret != 0)
		goto done;

	/* Save virtio consts */
	ret = vi_pci_snapshot_consts(vc, meta);
	if (ret != 0)
		goto done;

	/* Save virtio queue info */
	ret = vi_pci_snapshot_queues(vs, meta);
	if (ret != 0)
		goto done;

	/* Save device softc, if needed */
	if (vc->vc_snapshot != NULL) {
		ret = (*vc->vc_snapshot)(DEV_SOFTC(vs), meta);
		if (ret != 0)
			goto done;
	}

done:
	return (ret);
}
#endif
