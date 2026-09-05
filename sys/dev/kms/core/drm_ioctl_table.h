/*-
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Table-driven DRM ioctl dispatch for kms.ko.
 *
 * Design goal: one entry per DRM ioctl NR drawn from drm/drm.h and
 * drm/drm_mode.h.  Every command gets a slot from day one — unimplemented
 * commands point at a shared stub that returns a clean ENOTTY.  Real
 * handlers get wired one slot at a time as they're built.  The key-to-slot
 * mapping never changes; only what the slot points at.
 *
 * Every dispatch does the same sequence:
 *   1. look up slot by NR (cmd & 0xff)
 *   2. bounds-check + null-slot check — else clean reject (ENOTTY)
 *   3. credentials vs permission flags (MASTER / AUTH / RENDER_ALLOW)
 *   4. copyin exactly arg_size bytes (implicit via ioctl(2)'s IOC_INOUT;
 *      table's arg_size KASSERT'd against IOCPARM_LEN(cmd) as a build-
 *      time-shape / build-error catcher for header drift)
 *   5. run handler
 *   6. copyout results (implicit via IOC_OUT bit)
 *
 * The three security guards:
 *   - arg size enforced on copy (IOC_INOUT is honored by the kernel;
 *     KASSERT catches divergence from our table)
 *   - permission flags checked BEFORE the handler runs
 *   - bounds-and-null check on every lookup so a bad NR never jumps
 *     through a garbage pointer
 *
 * Migration model: today's kms_ioctl still runs the legacy switch.  The
 * dev.kms.table_dispatch sysctl flips per-slot dispatch on.  Slots that
 * have not yet been migrated to a dedicated handler point at the shared
 * kms_ioctl_delegate_marker, which the dispatcher detects by pointer
 * identity — permission gate runs, dispatcher returns -1, and kms_ioctl
 * falls through to the legacy switch so behavior is unchanged.
 *
 * Per-slot migration order that stays safe for both tunable=0 and =1:
 *   1. Implement the per-NR handler in its own function (or nearby .c).
 *   2. Update the table slot: replace kms_ioctl_delegate_marker with the
 *      new handler pointer.  DO NOT remove the switch case yet — with
 *      tunable=0 the switch is still the only path.
 *   3. Verify with tunable=1: the table's real handler runs and the
 *      switch case is now unreachable when the tunable is enabled.
 *   4. Only when tunable=1 has been the default for enough of a shakedown
 *      window (or when you're ready to delete the switch wholesale)
 *      remove the corresponding switch case.  Doing (4) before flipping
 *      the tunable default breaks the tunable=0 path (silent ENOTTY).
 *
 * Middle-window caveat: between step (2) and step (4), the tunable acts
 * as a per-slot behavior toggle.  If the table handler diverges from the
 * switch case (which is often the whole point of migrating — the table
 * version is supposed to be an improvement), then flipping the tunable
 * at runtime picks between the two implementations for THAT slot.  This
 * is the intended debugging surface — a maintainer can compare old vs
 * new behavior at a bug repro by flipping the sysctl.  But it also means
 * "old behavior with tunable=0" is a real live path until step (4)
 * lands — do not treat it as vestigial.
 *
 * When every slot has a real handler and the switch is empty, delete the
 * switch and the tunable together.
 */

#ifndef _KMS_DRM_IOCTL_TABLE_H_
#define	_KMS_DRM_IOCTL_TABLE_H_

#include <sys/types.h>
#include <sys/sysctl.h>			/* SYSCTL_DECL */

struct drm_file;
struct thread;

/*
 * Per-command permission flags.  Checked in step 3 of dispatch, BEFORE
 * the handler runs.  A slot with 0 flags is unrestricted (usable on
 * card + render nodes, no master or auth requirement) — e.g. VERSION,
 * GET_CAP, WAIT_VBLANK.
 */
#define	KMS_IOCTL_MASTER		(1u << 0)  /* file->is_master required */
#define	KMS_IOCTL_AUTH			(1u << 1)  /* is_master OR authenticated */
#define	KMS_IOCTL_RENDER_ALLOW		(1u << 2)  /* legal on /dev/dri/renderD* */
#define	KMS_IOCTL_UNLOCKED		(1u << 3)  /* pre-attach (reserved, no
						    * effect today) */

/*
 * Slots not yet migrated to a dedicated handler set their `.handler`
 * field to kms_ioctl_delegate_marker (via the DELEGATE() macro in the
 * .c file).  The dispatcher checks this pointer by identity, enforces
 * the permission gate, then returns -1 so the legacy switch runs.  A
 * slot migrates simply by replacing the marker with a real handler.
 */

/*
 * One entry per NR.  arg_size is the payload the ioctl(2) syscall will
 * copyin/copyout; keep it exactly matched to IOCPARM_LEN(cmd) or the
 * KASSERT trips during driver bring-up.
 */
struct kms_ioctl_desc {
	const char	*name;			/* trace + error strings */
	size_t		 arg_size;		/* EXACT payload bytes */
	uint32_t	 flags;			/* KMS_IOCTL_* bitmask */
	int		 (*handler)(struct drm_file *file, void *data,
			     struct thread *td);
};

/*
 * Shared stub for slots that exist only to give the dispatcher a clean
 * "known but unimplemented" answer.  Returns ENOTTY.
 */
int	kms_ioctl_stub_unsupported(struct drm_file *file, void *data,
	    struct thread *td);

/*
 * Delegation sentinel.  Slot entries whose handler pointer equals this
 * function trigger the "fall through to the legacy switch" branch in
 * the dispatcher.  Migrating a slot to a dedicated handler is a matter
 * of overwriting the pointer with the new handler.
 */
int	kms_ioctl_delegate_marker(struct drm_file *file, void *data,
	    struct thread *td);


/*
 * Table-driven entry point.  Returns >= 0 (errno) on hit, -1 if the
 * table has no slot for this NR (caller falls back to the legacy switch
 * for compatibility during migration).  When dev.kms.table_dispatch=0
 * this function returns -1 unconditionally so kms_ioctl runs the switch
 * as before.
 *
 * The DELEGATE flag on a matched slot is also reported as -1 after the
 * permission gate has been enforced — the legacy switch then runs with
 * the promise that master/auth/render-node checks have already fired.
 */
int	kms_ioctl_table_dispatch(struct drm_file *file, u_long cmd,
	    caddr_t data, struct thread *td);

/*
 * Tunable declared in drm_ioctl_table.c so drm_ioctl.c can gate the
 * dispatch call at runtime.  Default 0 (legacy switch runs unchanged);
 * flipped to 1 by a maintainer to enable table dispatch after per-slot
 * audit.
 */
extern int	kms_ioctl_table_dispatch_enabled;

/*
 * SYSCTL_DECL of the dev.kms parent node so per-GPU consumer .ko files
 * (igen, rk_kms, vfgpu, ...) can hang their own sub-nodes off it via
 *   SYSCTL_NODE(_dev_kms, OID_AUTO, <driver>, ...)
 * without redeclaring the parent (which would fail on module load with
 * a duplicate-node error).
 */
SYSCTL_DECL(_dev_kms);


/*
 * Table adapters, defined in drm_ioctl.c.  Each replaces DELEGATE in
 * its slot; the legacy switch case stays until the tunable defaults on.
 */
int	kms_tbl_version(struct drm_file *file, void *data,
	    struct thread *td);
int	kms_tbl_get_unique(struct drm_file *file, void *data,
	    struct thread *td);
int	kms_tbl_set_version(struct drm_file *file, void *data,
	    struct thread *td);
int	kms_tbl_get_cap(struct drm_file *file, void *data,
	    struct thread *td);
int	kms_tbl_mode_getresources(struct drm_file *file, void *data,
	    struct thread *td);
int	kms_tbl_mode_getcrtc(struct drm_file *file, void *data,
	    struct thread *td);
int	kms_tbl_mode_getencoder(struct drm_file *file, void *data,
	    struct thread *td);
int	kms_tbl_mode_getconnector(struct drm_file *file, void *data,
	    struct thread *td);
int	kms_tbl_mode_getplane_resources(struct drm_file *file, void *data,
	    struct thread *td);
int	kms_tbl_mode_getplane(struct drm_file *file, void *data,
	    struct thread *td);
int	kms_tbl_mode_create_dumb(struct drm_file *file, void *data,
	    struct thread *td);
int	kms_tbl_mode_map_dumb(struct drm_file *file, void *data,
	    struct thread *td);
int	kms_tbl_mode_destroy_dumb(struct drm_file *file, void *data,
	    struct thread *td);
int	kms_tbl_gem_close(struct drm_file *file, void *data,
	    struct thread *td);
int	kms_tbl_prime_handle_to_fd(struct drm_file *file, void *data,
	    struct thread *td);
int	kms_tbl_prime_fd_to_handle(struct drm_file *file, void *data,
	    struct thread *td);
int	kms_tbl_mode_addfb(struct drm_file *file, void *data,
	    struct thread *td);
int	kms_tbl_mode_addfb2(struct drm_file *file, void *data,
	    struct thread *td);
int	kms_tbl_mode_rmfb(struct drm_file *file, void *data,
	    struct thread *td);
int	kms_tbl_mode_closefb(struct drm_file *file, void *data,
	    struct thread *td);
int	kms_tbl_mode_getfb(struct drm_file *file, void *data,
	    struct thread *td);
int	kms_tbl_mode_getfb2(struct drm_file *file, void *data,
	    struct thread *td);
int	kms_tbl_mode_setcrtc(struct drm_file *file, void *data,
	    struct thread *td);
int	kms_tbl_mode_page_flip(struct drm_file *file, void *data,
	    struct thread *td);
int	kms_tbl_set_client_cap(struct drm_file *file, void *data,
	    struct thread *td);
int	kms_tbl_mode_getproperty(struct drm_file *file, void *data,
	    struct thread *td);
int	kms_tbl_mode_obj_getproperties(struct drm_file *file, void *data,
	    struct thread *td);
int	kms_tbl_mode_obj_setproperty(struct drm_file *file, void *data,
	    struct thread *td);
int	kms_tbl_mode_createpropblob(struct drm_file *file, void *data,
	    struct thread *td);
int	kms_tbl_mode_destroypropblob(struct drm_file *file, void *data,
	    struct thread *td);
int	kms_tbl_mode_getpropblob(struct drm_file *file, void *data,
	    struct thread *td);
int	kms_tbl_mode_atomic(struct drm_file *file, void *data,
	    struct thread *td);
int	kms_tbl_syncobj_create(struct drm_file *file, void *data,
	    struct thread *td);
int	kms_tbl_syncobj_destroy(struct drm_file *file, void *data,
	    struct thread *td);
int	kms_tbl_syncobj_wait(struct drm_file *file, void *data,
	    struct thread *td);
int	kms_tbl_syncobj_reset(struct drm_file *file, void *data,
	    struct thread *td);
int	kms_tbl_syncobj_signal(struct drm_file *file, void *data,
	    struct thread *td);
int	kms_tbl_syncobj_handle_to_fd(struct drm_file *file, void *data,
	    struct thread *td);
int	kms_tbl_wait_vblank(struct drm_file *file, void *data,
	    struct thread *td);
int	kms_tbl_mode_cursor(struct drm_file *file, void *data,
	    struct thread *td);
int	kms_tbl_mode_cursor2(struct drm_file *file, void *data,
	    struct thread *td);


/* Hoisted from the legacy switch; see drm_ioctl.c. */
int	kms_tbl_mode_list_lessees(struct drm_file *file, void *data,
	    struct thread *td);
int	kms_tbl_mode_get_lease(struct drm_file *file, void *data,
	    struct thread *td);
int	kms_tbl_mode_lease_unsupported(struct drm_file *file, void *data,
	    struct thread *td);
int	kms_tbl_set_master(struct drm_file *file, void *data,
	    struct thread *td);
int	kms_tbl_drop_master(struct drm_file *file, void *data,
	    struct thread *td);
int	kms_tbl_get_magic(struct drm_file *file, void *data,
	    struct thread *td);
int	kms_tbl_auth_magic(struct drm_file *file, void *data,
	    struct thread *td);
int	kms_tbl_mode_setgamma(struct drm_file *file, void *data,
	    struct thread *td);
int	kms_tbl_mode_dirtyfb(struct drm_file *file, void *data,
	    struct thread *td);


/* Newly implemented; previously DELEGATE with no implementation. */
int	kms_tbl_irq_busid(struct drm_file *file, void *data,
	    struct thread *td);
int	kms_tbl_mode_setproperty(struct drm_file *file, void *data,
	    struct thread *td);
int	kms_tbl_mode_setplane(struct drm_file *file, void *data,
	    struct thread *td);

#endif /* _KMS_DRM_IOCTL_TABLE_H_ */
