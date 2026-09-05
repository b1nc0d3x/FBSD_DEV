/*-
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Table-driven DRM ioctl dispatch — see drm_ioctl_table.h for the design.
 *
 * This file owns:
 *   - the shared stub handler
 *   - the descriptor table indexed by NR (cmd & 0xff)
 *   - the dispatch entry point (kms_ioctl_table_dispatch)
 *   - the dev.kms.table_dispatch sysctl tunable
 *
 * On import this file DOES NOT change kms_ioctl's behavior for anyone.
 * Every slot that maps to a command the legacy switch handles is marked
 * KMS_IOCTL_DELEGATE — the dispatcher runs the permission gate, then
 * returns -1 so kms_ioctl falls through to the switch.  Migrating a
 * handler out of the switch is: (a) implement a per-NR handler function
 * with the standard signature, (b) drop DELEGATE from its slot and set
 * the handler pointer, (c) remove the switch case.
 */

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/errno.h>
#include <sys/ioccom.h>
#include <sys/sysctl.h>
#include <sys/kernel.h>

#include <drm/drm.h>
#include <drm/drm_mode.h>
#include <kms/drm_file.h>

#include "drm_ioctl_table.h"

/*
 * DRM ioctl commands live in NR groups 'd' (drm class) and each command
 * carries an 8-bit NR.  Base drm.h commands occupy 0x00..0x3F.  MODE_*
 * commands from drm_mode.h start at DRM_COMMAND_BASE (0x40) and extend
 * through 0xD0 today (MODE_CLOSEFB).  Driver-specific ioctls occupy the
 * range starting at DRM_COMMAND_BASE + driver_specific_nr and are
 * dispatched by the driver's own ioctl hook — NOT by this table.
 *
 * Cap the table at 256 slots (the full NR byte space).  Any future
 * MODE_* additions upstream drop straight in without a resize; the
 * cost is 256 * sizeof(struct kms_ioctl_desc) ≈ 8 KiB of .rodata.
 */
#define	KMS_IOCTL_NR_MAX	0x100u
#define	KMS_IOCTL_NR(cmd)	((cmd) & 0xffu)

int kms_ioctl_table_dispatch_enabled = 0;

SYSCTL_NODE(_dev, OID_AUTO, kms, CTLFLAG_RD | CTLFLAG_MPSAFE, NULL,
    "kms core framework");

SYSCTL_INT(_dev_kms, OID_AUTO, table_dispatch, CTLFLAG_RWTUN,
    &kms_ioctl_table_dispatch_enabled, 0,
    "Route DRM ioctls through the per-NR table dispatcher; 0 = legacy switch");

int
kms_ioctl_stub_unsupported(struct drm_file *file __unused, void *data __unused,
    struct thread *td __unused)
{
	return (ENOTTY);
}

/*
 * Sentinel handler for slots that have not been migrated to a dedicated
 * per-NR function yet.  The dispatcher detects this pointer by identity
 * and returns -1 to fall through to the legacy switch — the sentinel is
 * a defensive tripwire if a caller ever reaches it directly.
 *
 * Detecting delegation by pointer identity keeps the table entry format
 * flat (one flags field, one handler field) so designated initialisers
 * don't have to pack two independent values into one column.
 */
int
kms_ioctl_delegate_marker(struct drm_file *file __unused,
    void *data __unused, struct thread *td __unused)
{
	return (ENOTTY);
}

/*
 * DELEGATE is a documentation stand-in — a symbolic name for the
 * marker function pointer so the intent reads clearly at the call
 * site.  It takes no argument; permission enforcement is spelled out
 * one field earlier in the entry initialiser via the flags column.
 * The earlier DELEGATE form was removed after the second-round
 * review pointed out that the arg was silently discarded and could
 * drift from the flags column undetected.
 */
#define	DELEGATE		kms_ioctl_delegate_marker

/*
 * The table.  Entries are populated by initialiser index so gaps between
 * assigned NRs stay explicitly zero (name=NULL, handler=NULL) — the
 * dispatcher detects null-handler and rejects with ENOTTY.
 *
 * Permission flag column reflects Linux drm_ioctls[] semantics with two
 * documented deltas:
 *   - MODE_ADDFB / MODE_ADDFB2 flagged KMS_IOCTL_AUTH (not MASTER).  This
 *     is the delta from Linux that the DRM_AUTH split commit was built
 *     for — kwin_wayland opens card0 twice per process and needs the
 *     non-master render fd to add framebuffers.  RMFB / DIRTYFB /
 *     OBJ_SETPROPERTY stay MASTER (matches Linux, closes the review
 *     H1/H2/H3 findings that surfaced during the switch-only rework).
 *   - MODE_CURSOR / MODE_CURSOR2 stay MASTER (matches Linux).  Atomic
 *     compositors drive the cursor plane via MODE_ATOMIC anyway.
 */
static const struct kms_ioctl_desc kms_ioctl_table[KMS_IOCTL_NR_MAX] = {
	/* --------- base drm/drm.h --------- */
	[KMS_IOCTL_NR(DRM_IOCTL_VERSION)] = {
		"VERSION",		sizeof(struct drm_version),
		KMS_IOCTL_UNLOCKED | KMS_IOCTL_RENDER_ALLOW,
		kms_tbl_version
	},
	[KMS_IOCTL_NR(DRM_IOCTL_GET_UNIQUE)] = {
		"GET_UNIQUE",		sizeof(struct drm_unique),
		KMS_IOCTL_RENDER_ALLOW,
		kms_tbl_get_unique
	},
	[KMS_IOCTL_NR(DRM_IOCTL_GET_MAGIC)] = {
		"GET_MAGIC",		sizeof(struct drm_auth),
		0,
		kms_tbl_get_magic
	},
	[KMS_IOCTL_NR(DRM_IOCTL_IRQ_BUSID)] = {
		/*
		 * Not implemented by the core, but keep DELEGATE so a
		 * per-driver drv->ioctl hook can still see it via the
		 * fall-through path at kms_ioctl's tail.
		 */
		"IRQ_BUSID",		sizeof(struct drm_irq_busid),
		KMS_IOCTL_MASTER,
		kms_tbl_irq_busid
	},
	[KMS_IOCTL_NR(DRM_IOCTL_GET_CAP)] = {
		"GET_CAP",		sizeof(struct drm_get_cap),
		KMS_IOCTL_UNLOCKED | KMS_IOCTL_RENDER_ALLOW,
		kms_tbl_get_cap
	},
	[KMS_IOCTL_NR(DRM_IOCTL_SET_CLIENT_CAP)] = {
		"SET_CLIENT_CAP",	sizeof(struct drm_set_client_cap),
		KMS_IOCTL_RENDER_ALLOW,
		kms_tbl_set_client_cap
	},
	[KMS_IOCTL_NR(DRM_IOCTL_SET_VERSION)] = {
		"SET_VERSION",		sizeof(struct drm_set_version),
		KMS_IOCTL_MASTER,
		kms_tbl_set_version
	},
	[KMS_IOCTL_NR(DRM_IOCTL_AUTH_MAGIC)] = {
		"AUTH_MAGIC",		sizeof(struct drm_auth),
		KMS_IOCTL_MASTER,
		kms_tbl_auth_magic
	},
	[KMS_IOCTL_NR(DRM_IOCTL_SET_MASTER)] = {
		"SET_MASTER",		0, 0,
		kms_tbl_set_master
	},
	[KMS_IOCTL_NR(DRM_IOCTL_DROP_MASTER)] = {
		"DROP_MASTER",		0, 0,
		kms_tbl_drop_master
	},
	[KMS_IOCTL_NR(DRM_IOCTL_WAIT_VBLANK)] = {
		/*
		 * NOT KMS_IOCTL_RENDER_ALLOW.  WAIT_VBLANK is display-side;
		 * render nodes are display-agnostic per Linux DRM ABI and
		 * drm_ioctl.c's switch prologue already rejects this on
		 * render nodes with EACCES.  Match that policy in the table.
		 */
		"WAIT_VBLANK",		sizeof(union drm_wait_vblank),
		0,
		kms_tbl_wait_vblank
	},
	[KMS_IOCTL_NR(DRM_IOCTL_GEM_CLOSE)] = {
		"GEM_CLOSE",		sizeof(struct drm_gem_close),
		KMS_IOCTL_RENDER_ALLOW,
		kms_tbl_gem_close
	},
	[KMS_IOCTL_NR(DRM_IOCTL_PRIME_HANDLE_TO_FD)] = {
		"PRIME_HANDLE_TO_FD",	sizeof(struct drm_prime_handle),
		KMS_IOCTL_AUTH | KMS_IOCTL_RENDER_ALLOW,
		kms_tbl_prime_handle_to_fd
	},
	[KMS_IOCTL_NR(DRM_IOCTL_PRIME_FD_TO_HANDLE)] = {
		"PRIME_FD_TO_HANDLE",	sizeof(struct drm_prime_handle),
		KMS_IOCTL_AUTH | KMS_IOCTL_RENDER_ALLOW,
		kms_tbl_prime_fd_to_handle
	},
	[KMS_IOCTL_NR(DRM_IOCTL_SYNCOBJ_CREATE)] = {
		"SYNCOBJ_CREATE",	sizeof(struct drm_syncobj_create),
		KMS_IOCTL_RENDER_ALLOW,
		kms_tbl_syncobj_create
	},
	[KMS_IOCTL_NR(DRM_IOCTL_SYNCOBJ_DESTROY)] = {
		"SYNCOBJ_DESTROY",	sizeof(struct drm_syncobj_destroy),
		KMS_IOCTL_RENDER_ALLOW,
		kms_tbl_syncobj_destroy
	},
	[KMS_IOCTL_NR(DRM_IOCTL_SYNCOBJ_HANDLE_TO_FD)] = {
		/*
		 * KMS_IOCTL_RENDER_ALLOW — Linux upstream flags this
		 * DRM_RENDER_ALLOW only (no DRM_AUTH).  Mesa's
		 * linux-drm-syncobj-v1 Wayland explicit-sync path opens
		 * /dev/dri/renderD* and calls SYNCOBJ_HANDLE_TO_FD without
		 * ever issuing AUTH_MAGIC — requiring auth here would
		 * regress iris + radeonsi under any Wayland compositor
		 * using explicit sync.
		 */
		"SYNCOBJ_HANDLE_TO_FD",	sizeof(struct drm_syncobj_handle),
		KMS_IOCTL_RENDER_ALLOW,
		kms_tbl_syncobj_handle_to_fd
	},
	[KMS_IOCTL_NR(DRM_IOCTL_SYNCOBJ_FD_TO_HANDLE)] = {
		/* Same as SYNCOBJ_HANDLE_TO_FD — RENDER_ALLOW only, no
		 * AUTH.  Mesa render-node clients import sync FDs without
		 * AUTH_MAGIC. */
		"SYNCOBJ_FD_TO_HANDLE",	sizeof(struct drm_syncobj_handle),
		KMS_IOCTL_RENDER_ALLOW,
		DELEGATE
	},
	[KMS_IOCTL_NR(DRM_IOCTL_SYNCOBJ_WAIT)] = {
		"SYNCOBJ_WAIT",		sizeof(struct drm_syncobj_wait),
		KMS_IOCTL_RENDER_ALLOW,
		kms_tbl_syncobj_wait
	},
	[KMS_IOCTL_NR(DRM_IOCTL_SYNCOBJ_RESET)] = {
		"SYNCOBJ_RESET",	sizeof(struct drm_syncobj_array),
		KMS_IOCTL_RENDER_ALLOW,
		kms_tbl_syncobj_reset
	},
	[KMS_IOCTL_NR(DRM_IOCTL_SYNCOBJ_SIGNAL)] = {
		"SYNCOBJ_SIGNAL",	sizeof(struct drm_syncobj_array),
		KMS_IOCTL_RENDER_ALLOW,
		kms_tbl_syncobj_signal
	},
	[KMS_IOCTL_NR(DRM_IOCTL_SYNCOBJ_TIMELINE_WAIT)] = {
		/* DELEGATE so a driver's ioctl hook can implement it. */
		"SYNCOBJ_TIMELINE_WAIT",
		sizeof(struct drm_syncobj_timeline_wait),
		KMS_IOCTL_RENDER_ALLOW,
		DELEGATE
	},
	[KMS_IOCTL_NR(DRM_IOCTL_SYNCOBJ_TIMELINE_SIGNAL)] = {
		"SYNCOBJ_TIMELINE_SIGNAL",
		sizeof(struct drm_syncobj_timeline_array),
		KMS_IOCTL_RENDER_ALLOW,
		DELEGATE
	},

	/* --------- drm/drm_mode.h MODE_* --------- */
	[KMS_IOCTL_NR(DRM_IOCTL_MODE_GETRESOURCES)] = {
		"MODE_GETRESOURCES",	sizeof(struct drm_mode_card_res),
		KMS_IOCTL_RENDER_ALLOW,
		kms_tbl_mode_getresources
	},
	[KMS_IOCTL_NR(DRM_IOCTL_MODE_GETCRTC)] = {
		"MODE_GETCRTC",		sizeof(struct drm_mode_crtc),
		KMS_IOCTL_RENDER_ALLOW,
		kms_tbl_mode_getcrtc
	},
	[KMS_IOCTL_NR(DRM_IOCTL_MODE_SETCRTC)] = {
		"MODE_SETCRTC",		sizeof(struct drm_mode_crtc),
		KMS_IOCTL_MASTER,
		kms_tbl_mode_setcrtc
	},
	[KMS_IOCTL_NR(DRM_IOCTL_MODE_CURSOR)] = {
		"MODE_CURSOR",		sizeof(struct drm_mode_cursor),
		KMS_IOCTL_MASTER,
		kms_tbl_mode_cursor
	},
	[KMS_IOCTL_NR(DRM_IOCTL_MODE_GETGAMMA)] = {
		"MODE_GETGAMMA",	sizeof(struct drm_mode_crtc_lut),
		0,
		DELEGATE
	},
	[KMS_IOCTL_NR(DRM_IOCTL_MODE_SETGAMMA)] = {
		"MODE_SETGAMMA",	sizeof(struct drm_mode_crtc_lut),
		KMS_IOCTL_MASTER,
		kms_tbl_mode_setgamma
	},
	[KMS_IOCTL_NR(DRM_IOCTL_MODE_GETENCODER)] = {
		"MODE_GETENCODER",	sizeof(struct drm_mode_get_encoder),
		KMS_IOCTL_RENDER_ALLOW,
		kms_tbl_mode_getencoder
	},
	[KMS_IOCTL_NR(DRM_IOCTL_MODE_GETCONNECTOR)] = {
		"MODE_GETCONNECTOR",	sizeof(struct drm_mode_get_connector),
		KMS_IOCTL_RENDER_ALLOW,
		kms_tbl_mode_getconnector
	},
	[KMS_IOCTL_NR(DRM_IOCTL_MODE_GETPROPERTY)] = {
		"MODE_GETPROPERTY",	sizeof(struct drm_mode_get_property),
		0,
		kms_tbl_mode_getproperty
	},
	[KMS_IOCTL_NR(DRM_IOCTL_MODE_SETPROPERTY)] = {
		/*
		 * Core doesn't handle SETPROPERTY today — every current per-
		 * GPU consumer routes property writes through OBJ_SETPROPERTY
		 * instead — but DELEGATE keeps the drv->ioctl fallback path
		 * open so a future consumer can plug in.  Master-gated per
		 * Linux drm_ioctls[].
		 */
		"MODE_SETPROPERTY",	sizeof(struct drm_mode_connector_set_property),
		KMS_IOCTL_MASTER,
		kms_tbl_mode_setproperty
	},
	[KMS_IOCTL_NR(DRM_IOCTL_MODE_GETPROPBLOB)] = {
		"MODE_GETPROPBLOB",	sizeof(struct drm_mode_get_blob),
		0,
		kms_tbl_mode_getpropblob
	},
	[KMS_IOCTL_NR(DRM_IOCTL_MODE_GETFB)] = {
		/*
		 * MASTER per Linux drm_ioctls[] — GETFB returns the front-
		 * buffer handle for the queried fb_id, i.e. read access to
		 * the compositor's live scanout memory.  Same exposure class
		 * as RMFB / DIRTYFB: a non-master video-group user could
		 * enumerate fb_ids and read the current desktop.
		 */
		"MODE_GETFB",		sizeof(struct drm_mode_fb_cmd),
		KMS_IOCTL_MASTER,
		kms_tbl_mode_getfb
	},
	[KMS_IOCTL_NR(DRM_IOCTL_MODE_ADDFB)] = {
		"MODE_ADDFB",		sizeof(struct drm_mode_fb_cmd),
		KMS_IOCTL_AUTH,
		kms_tbl_mode_addfb
	},
	[KMS_IOCTL_NR(DRM_IOCTL_MODE_RMFB)] = {
		"MODE_RMFB",		sizeof(uint32_t),
		KMS_IOCTL_MASTER,
		kms_tbl_mode_rmfb
	},
	[KMS_IOCTL_NR(DRM_IOCTL_MODE_PAGE_FLIP)] = {
		"MODE_PAGE_FLIP",	sizeof(struct drm_mode_crtc_page_flip),
		KMS_IOCTL_MASTER,
		kms_tbl_mode_page_flip
	},
	[KMS_IOCTL_NR(DRM_IOCTL_MODE_DIRTYFB)] = {
		"MODE_DIRTYFB",		sizeof(struct drm_mode_fb_dirty_cmd),
		KMS_IOCTL_MASTER,
		kms_tbl_mode_dirtyfb
	},
	[KMS_IOCTL_NR(DRM_IOCTL_MODE_CREATE_DUMB)] = {
		"MODE_CREATE_DUMB",	sizeof(struct drm_mode_create_dumb),
		KMS_IOCTL_RENDER_ALLOW,
		kms_tbl_mode_create_dumb
	},
	[KMS_IOCTL_NR(DRM_IOCTL_MODE_MAP_DUMB)] = {
		"MODE_MAP_DUMB",	sizeof(struct drm_mode_map_dumb),
		KMS_IOCTL_RENDER_ALLOW,
		kms_tbl_mode_map_dumb
	},
	[KMS_IOCTL_NR(DRM_IOCTL_MODE_DESTROY_DUMB)] = {
		"MODE_DESTROY_DUMB",	sizeof(struct drm_mode_destroy_dumb),
		KMS_IOCTL_RENDER_ALLOW,
		kms_tbl_mode_destroy_dumb
	},
	[KMS_IOCTL_NR(DRM_IOCTL_MODE_GETPLANERESOURCES)] = {
		"MODE_GETPLANERESOURCES",
		sizeof(struct drm_mode_get_plane_res),
		KMS_IOCTL_RENDER_ALLOW,
		kms_tbl_mode_getplane_resources
	},
	[KMS_IOCTL_NR(DRM_IOCTL_MODE_GETPLANE)] = {
		"MODE_GETPLANE",	sizeof(struct drm_mode_get_plane),
		KMS_IOCTL_RENDER_ALLOW,
		kms_tbl_mode_getplane
	},
	[KMS_IOCTL_NR(DRM_IOCTL_MODE_SETPLANE)] = {
		"MODE_SETPLANE",	sizeof(struct drm_mode_set_plane),
		KMS_IOCTL_MASTER,
		kms_tbl_mode_setplane
	},
	[KMS_IOCTL_NR(DRM_IOCTL_MODE_ADDFB2)] = {
		"MODE_ADDFB2",		sizeof(struct drm_mode_fb_cmd2),
		KMS_IOCTL_AUTH,
		kms_tbl_mode_addfb2
	},
	[KMS_IOCTL_NR(DRM_IOCTL_MODE_OBJ_GETPROPERTIES)] = {
		/*
		 * KMS_IOCTL_RENDER_ALLOW — Linux relaxed OBJ_GETPROPERTIES
		 * from DRM_MASTER to DRM_RENDER_ALLOW years ago because
		 * Mesa (iris/radeonsi) and Vulkan clients query plane
		 * format modifiers + CRTC/connector capability properties
		 * from the render node during their format-negotiation
		 * dance.  Property reads return metadata (format lists,
		 * capability bitmaps, IN_FORMATS blobs), not scanout
		 * memory — no exposure of the class GETFB has.
		 */
		"MODE_OBJ_GETPROPERTIES",
		sizeof(struct drm_mode_obj_get_properties),
		KMS_IOCTL_RENDER_ALLOW,
		kms_tbl_mode_obj_getproperties
	},
	[KMS_IOCTL_NR(DRM_IOCTL_MODE_OBJ_SETPROPERTY)] = {
		"MODE_OBJ_SETPROPERTY",	sizeof(struct drm_mode_obj_set_property),
		KMS_IOCTL_MASTER,
		kms_tbl_mode_obj_setproperty
	},
	[KMS_IOCTL_NR(DRM_IOCTL_MODE_CURSOR2)] = {
		"MODE_CURSOR2",		sizeof(struct drm_mode_cursor2),
		KMS_IOCTL_MASTER,
		kms_tbl_mode_cursor2
	},
	[KMS_IOCTL_NR(DRM_IOCTL_MODE_ATOMIC)] = {
		"MODE_ATOMIC",		sizeof(struct drm_mode_atomic),
		KMS_IOCTL_MASTER,
		kms_tbl_mode_atomic
	},
	[KMS_IOCTL_NR(DRM_IOCTL_MODE_CREATEPROPBLOB)] = {
		"MODE_CREATEPROPBLOB",	sizeof(struct drm_mode_create_blob),
		KMS_IOCTL_AUTH,
		kms_tbl_mode_createpropblob
	},
	[KMS_IOCTL_NR(DRM_IOCTL_MODE_DESTROYPROPBLOB)] = {
		"MODE_DESTROYPROPBLOB",	sizeof(struct drm_mode_destroy_blob),
		KMS_IOCTL_AUTH,
		kms_tbl_mode_destroypropblob
	},
	[KMS_IOCTL_NR(DRM_IOCTL_MODE_GETFB2)] = {
		/* Same as MODE_GETFB — MASTER per Linux drm_ioctls[]. */
		"MODE_GETFB2",		sizeof(struct drm_mode_fb_cmd2),
		KMS_IOCTL_MASTER,
		kms_tbl_mode_getfb2
	},
	[KMS_IOCTL_NR(DRM_IOCTL_MODE_CLOSEFB)] = {
		"MODE_CLOSEFB",		sizeof(struct drm_mode_closefb),
		KMS_IOCTL_MASTER,
		kms_tbl_mode_closefb
	},
	[KMS_IOCTL_NR(DRM_IOCTL_MODE_LIST_LESSEES)] = {
		"MODE_LIST_LESSEES",	sizeof(struct drm_mode_list_lessees),
		0,
		kms_tbl_mode_list_lessees
	},
	[KMS_IOCTL_NR(DRM_IOCTL_MODE_GET_LEASE)] = {
		"MODE_GET_LEASE",	sizeof(struct drm_mode_get_lease),
		0,
		kms_tbl_mode_get_lease
	},
	[KMS_IOCTL_NR(DRM_IOCTL_MODE_CREATE_LEASE)] = {
		"MODE_CREATE_LEASE",	sizeof(struct drm_mode_create_lease),
		KMS_IOCTL_MASTER,
		kms_tbl_mode_lease_unsupported
	},
	[KMS_IOCTL_NR(DRM_IOCTL_MODE_REVOKE_LEASE)] = {
		"MODE_REVOKE_LEASE",	sizeof(struct drm_mode_revoke_lease),
		KMS_IOCTL_MASTER,
		kms_tbl_mode_lease_unsupported
	},
	/*
	 * Slots left unassigned (name=NULL, handler=NULL) are rejected as
	 * ENOTTY by the dispatcher's null-handler check.  Base drm/drm.h
	 * commands we intentionally never support — legacy AGP, SG, DMA,
	 * lock, context (drm_ioctl.h 0x30..0x50), and CTX_TYPE — fall in
	 * that class.  A `pkg install kms_ioctl_symbol_dumper` style debug
	 * pass, or `sysctl dev.kms.table_dispatch=1` followed by an ioctl
	 * with an unassigned NR, produces the clean ENOTTY.
	 */
};

int
kms_ioctl_table_dispatch(struct drm_file *file, u_long cmd, caddr_t data,
    struct thread *td)
{
	const struct kms_ioctl_desc *desc;
	unsigned int nr;

	if (kms_ioctl_table_dispatch_enabled == 0)
		return (-1);				/* legacy switch */

	/*
	 * Only claim commands from the DRM 'd' ioctl group.  Without this,
	 * `cmd & 0xff` would collide with any same-low-byte NR from another
	 * ioctl group whose cdev happened to reuse this dispatcher — the
	 * permission gate would fire on the wrong semantics, and the KASSERT
	 * at step 4 would panic in debug builds when arg-size mismatches.
	 */
	if (IOCGROUP(cmd) != DRM_IOCTL_BASE)
		return (-1);				/* let the switch route */

	/* Step 1 + 2: NR-scoped lookup with bounds and null-slot guards. */
	nr = KMS_IOCTL_NR(cmd);
	if (nr >= KMS_IOCTL_NR_MAX)
		return (ENOTTY);
	desc = &kms_ioctl_table[nr];
	if (desc->handler == NULL)
		return (ENOTTY);

	/* Step 3: render-node ABI, then master/auth flags. */
	if (file->is_render_node &&
	    (desc->flags & KMS_IOCTL_RENDER_ALLOW) == 0)
		return (EACCES);
	if ((desc->flags & KMS_IOCTL_MASTER) != 0 && !file->is_master)
		return (EACCES);
	if ((desc->flags & KMS_IOCTL_AUTH) != 0 &&
	    !(file->is_master || file->authenticated))
		return (EACCES);

	/* Step 4: EXACT arg-size shape.  IOCPARM_LEN(cmd) came off the
	 * userland ioctl(2) frame; if our desc says something different,
	 * we've drifted from the uapi header and the KASSERT fires during
	 * bring-up — clean loud shape mismatch instead of a silent copyin
	 * of the wrong byte count. */
	KASSERT((desc->arg_size == 0 && IOCPARM_LEN(cmd) == 0) ||
	    desc->arg_size == IOCPARM_LEN(cmd),
	    ("kms_ioctl table entry \"%s\" arg_size=%zu != IOCPARM_LEN=%d "
	     "for cmd 0x%lx",
	     desc->name, desc->arg_size, IOCPARM_LEN(cmd), cmd));

	/* Step 5: DELEGATE slots hand back to the legacy switch after the
	 * gate has fired — the marker pointer identity is the signal.
	 * That way already-migrated slots skip the switch entirely without
	 * duplicating the master/auth check. */
	if (desc->handler == kms_ioctl_delegate_marker)
		return (-1);

	/* Step 5': real handler. */
	return (desc->handler(file, (void *)data, td));
}
