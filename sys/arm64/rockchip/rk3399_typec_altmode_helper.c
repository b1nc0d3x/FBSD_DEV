/*-
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Copyright (c) 2026 Kyle Crenshaw
 * All rights reserved.
 */

#include <sys/param.h>
#include <sys/bus.h>
#include <sys/kernel.h>
#include <sys/module.h>
#include <sys/systm.h>
#include <sys/sysctl.h>

#include "rk3399_typec_altmode_var.h"

static struct sysctl_ctx_list rk3399_typec_altmode_ctx;
static struct sysctl_oid *rk3399_typec_altmode_tree;

static int rk3399_typec_altmode_valid = 1;
static int rk3399_typec_altmode_dp_ready = 1;
static int rk3399_typec_altmode_usb_ss;
static uint32_t rk3399_typec_altmode_pin_assignment = 0x4;
static uint32_t rk3399_typec_altmode_dp_status = 0x8a;
static uint32_t rk3399_typec_altmode_get_count;

int
rk3399_typec_dp_altmode_get_state(device_t dev __unused,
    struct rk3399_typec_dp_altmode_status *status)
{

	if (status == NULL)
		return (EINVAL);

	status->valid = (rk3399_typec_altmode_valid != 0);
	status->dp_ready = (rk3399_typec_altmode_dp_ready != 0);
	status->usb_ss = rk3399_typec_altmode_usb_ss;
	status->pin_assignment = rk3399_typec_altmode_pin_assignment;
	status->dp_status = rk3399_typec_altmode_dp_status;
	rk3399_typec_altmode_get_count++;
	return (0);
}

static int
rk3399_typec_altmode_usb_ss_sysctl(SYSCTL_HANDLER_ARGS)
{
	int error, value;

	value = rk3399_typec_altmode_usb_ss;
	error = sysctl_handle_int(oidp, &value, 0, req);
	if (error != 0 || req->newptr == NULL)
		return (error);
	if (value < -1 || value > 1)
		return (EINVAL);
	rk3399_typec_altmode_usb_ss = value;
	return (0);
}

static int
rk3399_typec_altmode_modevent(module_t mod __unused, int what, void *arg __unused)
{
	switch (what) {
	case MOD_LOAD:
		sysctl_ctx_init(&rk3399_typec_altmode_ctx);
		rk3399_typec_altmode_tree = SYSCTL_ADD_NODE(
		    &rk3399_typec_altmode_ctx,
		    SYSCTL_STATIC_CHILDREN(_hw),
		    OID_AUTO, "rk3399_typec_altmode",
		    CTLFLAG_RW | CTLFLAG_MPSAFE, 0,
		    "Manual RK3399 Type-C DP Alt Mode bridge");
		if (rk3399_typec_altmode_tree == NULL)
			return (ENOMEM);

		SYSCTL_ADD_INT(&rk3399_typec_altmode_ctx,
		    SYSCTL_CHILDREN(rk3399_typec_altmode_tree), OID_AUTO,
		    "valid", CTLFLAG_RWTUN,
		    &rk3399_typec_altmode_valid, 0,
		    "Whether manual altmode state should be considered present");
		SYSCTL_ADD_INT(&rk3399_typec_altmode_ctx,
		    SYSCTL_CHILDREN(rk3399_typec_altmode_tree), OID_AUTO,
		    "dp_ready", CTLFLAG_RWTUN,
		    &rk3399_typec_altmode_dp_ready, 0,
		    "Manual DP Alt Mode ready state");
		SYSCTL_ADD_PROC(&rk3399_typec_altmode_ctx,
		    SYSCTL_CHILDREN(rk3399_typec_altmode_tree), OID_AUTO,
		    "usb_ss", CTLTYPE_INT | CTLFLAG_RWTUN | CTLFLAG_MPSAFE,
		    NULL, 0, rk3399_typec_altmode_usb_ss_sysctl, "I",
		    "Manual USB_SS property: -1 unknown, 0 DP-only, 1 USB3+DP");
		SYSCTL_ADD_U32(&rk3399_typec_altmode_ctx,
		    SYSCTL_CHILDREN(rk3399_typec_altmode_tree), OID_AUTO,
		    "pin_assignment", CTLFLAG_RWTUN,
		    &rk3399_typec_altmode_pin_assignment, 0,
		    "Manual DP Alt Mode pin assignment value");
		SYSCTL_ADD_U32(&rk3399_typec_altmode_ctx,
		    SYSCTL_CHILDREN(rk3399_typec_altmode_tree), OID_AUTO,
		    "dp_status", CTLFLAG_RWTUN,
		    &rk3399_typec_altmode_dp_status, 0,
		    "Manual DP Alt Mode status value");
		SYSCTL_ADD_U32(&rk3399_typec_altmode_ctx,
		    SYSCTL_CHILDREN(rk3399_typec_altmode_tree), OID_AUTO,
		    "get_count", CTLFLAG_RD,
		    &rk3399_typec_altmode_get_count, 0,
		    "Number of exported state reads");
		break;
	case MOD_UNLOAD:
		sysctl_ctx_free(&rk3399_typec_altmode_ctx);
		break;
	default:
		break;
	}

	return (0);
}

static moduledata_t rk3399_typec_altmode_mod = {
	"rk3399_typec_altmode_helper",
	rk3399_typec_altmode_modevent,
	NULL
};

DECLARE_MODULE(rk3399_typec_altmode_helper, rk3399_typec_altmode_mod,
    SI_SUB_DRIVERS, SI_ORDER_MIDDLE);
MODULE_VERSION(rk3399_typec_altmode_helper, 1);
