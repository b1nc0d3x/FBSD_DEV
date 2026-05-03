/*-
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Copyright (c) 2026 Kyle Crenshaw
 * All rights reserved.
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
#include <sys/endian.h>
#include <sys/systm.h>
#include <sys/kernel.h>

#include <dev/ofw/openfirm.h>
#include <dev/ofw/ofw_bus_subr.h>

#define	RK3399_DWC3_NODE		"/usb@fe800000"
#define	RK3399_DWC3_CHILD_NODE		"/usb@fe800000/usb@fe800000"
#define	RK3399_CDN_DP_NODE		"/dp@fec00000"
#define	RK3399_FUSB0_NODE		"/i2c@ff3d0000/typec-portc@22"
#define	RK3399_TCPHY0_DP_NODE		"/phy@ff7c0000/dp-port"
#define	RK3399_ROOT_FIXUP_PROP		"freebsd,rk3399-typec-dp-fixup"
#define	RK3399_STATUS_OKAY		"okay"
#define	RK3399_STATUS_DISABLED		"disabled"

void	rk3399_fdt_fixup(void);

static bool
rk3399_fdt_set_status(const char *path, const char *status)
{
	phandle_t node;
	char current[16];
	char updated[16];
	int len;
	int rv;

	node = OF_finddevice(path);
	if (node == -1) {
		printf("rk3399_fdt_fixup: node %s not found\n", path);
		return (false);
	}

	rv = OF_getprop(node, "status", current, sizeof(current));
	if (rv > 0)
		printf("rk3399_fdt_fixup: %s status before='%s'\n", path, current);
	else
		printf("rk3399_fdt_fixup: %s has no readable status before setprop (rv=%d)\n",
		    path, rv);

	len = strlen(status) + 1;
	rv = OF_setprop(node, "status", status, len);
	if (rv != len) {
		printf("rk3399_fdt_fixup: OF_setprop(%s,'%s') failed rv=%d len=%d\n",
		    path, status, rv, len);
		return (false);
	}

	rv = OF_getprop(node, "status", updated, sizeof(updated));
	if (rv > 0)
		printf("rk3399_fdt_fixup: %s status after='%s'\n", path, updated);
	else
		printf("rk3399_fdt_fixup: %s has no readable status after setprop (rv=%d)\n",
		    path, rv);

	return (true);
}

static bool
rk3399_fdt_set_u32_cells(const char *path, const char *propname,
    const pcell_t *cells, int ncells)
{
	phandle_t node;
	int len;
	int rv;

	node = OF_finddevice(path);
	if (node == -1) {
		printf("rk3399_fdt_fixup: node %s not found for %s\n",
		    path, propname);
		return (false);
	}

	len = ncells * sizeof(*cells);
	rv = OF_setprop(node, propname, cells, len);
	if (rv != len) {
		printf("rk3399_fdt_fixup: OF_setprop(%s,%s) failed rv=%d len=%d\n",
		    path, propname, rv, len);
		return (false);
	}

	return (true);
}

static bool
rk3399_fdt_set_xref_prop(const char *path, const char *propname,
    const char *target_path)
{
	phandle_t target;
	pcell_t cell;

	target = OF_finddevice(target_path);
	if (target == -1) {
		printf("rk3399_fdt_fixup: target %s not found for %s:%s\n",
		    target_path, path, propname);
		return (false);
	}

	cell = htobe32(OF_xref_from_node(target));
	return (rk3399_fdt_set_u32_cells(path, propname, &cell, 1));
}

void
rk3399_fdt_fixup(void)
{
	phandle_t root;
	int force_typec_dp;

	force_typec_dp = 0;
	if (!TUNABLE_INT_FETCH("hw.rk3399_typec_dp_force", &force_typec_dp)) {
		printf("rk3399_fdt_fixup: hw.rk3399_typec_dp_force not present\n");
		return;
	}
	printf("rk3399_fdt_fixup: hw.rk3399_typec_dp_force=%d\n",
	    force_typec_dp);
	if (force_typec_dp == 0)
		return;

	root = OF_finddevice("/");
	if (root == -1) {
		printf("rk3399_fdt_fixup: root node not found\n");
		return;
	}

	if (!ofw_bus_node_is_compatible(root, "pine64,rockpro64") &&
	    !ofw_bus_node_is_compatible(root, "pine64,rockpro64-v2.1")) {
		printf("rk3399_fdt_fixup: root is not RockPro64-compatible\n");
		return;
	}

	if (!rk3399_fdt_set_status(RK3399_CDN_DP_NODE, RK3399_STATUS_OKAY)) {
		printf("rk3399_fdt_fixup: unable to enable %s\n",
		    RK3399_CDN_DP_NODE);
		return;
	}

	if (!rk3399_fdt_set_xref_prop(RK3399_CDN_DP_NODE, "extcon",
	    RK3399_FUSB0_NODE))
		printf("rk3399_fdt_fixup: unable to set %s extcon\n",
		    RK3399_CDN_DP_NODE);
	if (!rk3399_fdt_set_xref_prop(RK3399_CDN_DP_NODE, "phys",
	    RK3399_TCPHY0_DP_NODE))
		printf("rk3399_fdt_fixup: unable to set %s phys\n",
		    RK3399_CDN_DP_NODE);

	if (!rk3399_fdt_set_status(RK3399_DWC3_CHILD_NODE,
	    RK3399_STATUS_DISABLED))
		printf("rk3399_fdt_fixup: unable to disable %s\n",
		    RK3399_DWC3_CHILD_NODE);
	if (!rk3399_fdt_set_status(RK3399_DWC3_NODE,
	    RK3399_STATUS_DISABLED))
		printf("rk3399_fdt_fixup: unable to disable %s\n",
		    RK3399_DWC3_NODE);
	(void)OF_setprop(root, RK3399_ROOT_FIXUP_PROP, "1", 2);

	printf("rk3399_fdt_fixup: forcing Type-C DP mode via %s\n",
	    "hw.rk3399_typec_dp_force=1");
}
