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

/*
 * RK3399 Power Domain Controller driver
 *
 * The RK3399 SoC groups its peripherals into power domains gated by the PMU
 * (Power Management Unit).  Before a peripheral's MMIO registers can be
 * accessed, two hardware interlocks must be cleared:
 *
 *   1. PMU_PWRDN_CON / PMU_PWRDN_ST  — the block-level power switch.
 *      Writing bit N of PWRDN_CON to 0 requests power-on; polling PWRDN_ST
 *      until bit N clears confirms the supply is stable.
 *
 *   2. PMU_BUS_IDLE_REQ / PMU_BUS_IDLE_ACK / PMU_BUS_IDLE_ST  — the AXI/AHB
 *      bus isolation gate.  While a domain is idle-gated the fabric presents a
 *      slave-error response to any master, which on ARM64 materialises as an
 *      SError (asynchronous external abort) and panics the kernel.  Writing the
 *      IDLE_REQ bit to 0 de-asserts isolation; polling ACK and ST until they
 *      clear confirms the bus is open.
 *
 * This driver currently exposes only domain 21 (HDCP / CDN-DP), which is the
 * domain required for USB-C DisplayPort Alt Mode.  Additional domains can be
 * added to rk3399_power_domains[] as needed.
 *
 * The driver attaches early (BUS_PASS_RESOURCE) on simple_mfd so that it is
 * ready before any consumer (rk_cdn_dp, hdmi) attempts to call
 * rk3399_power_enable_domain().  The parent bus is simple_mfd because the
 * power-controller node is a child of power-management@ff310000, which has
 * compatible "rockchip,rk3399-pmu","syscon","simple-mfd" and is handled by
 * the FreeBSD simple_mfd driver as simple_mfd0.
 */

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/bus.h>
#include <sys/kernel.h>
#include <sys/module.h>
#include <sys/mutex.h>

#include <dev/clk/clk.h>
#include <dev/ofw/ofw_bus.h>
#include <dev/ofw/ofw_bus_subr.h>
#include <dev/ofw/openfirm.h>
#include <dev/syscon/syscon.h>

#include "rk3399_power.h"
#include "syscon_if.h"

/*
 * PMU register offsets (RK3399 TRM §9.4, PMU register map).
 * All accessed via the parent syscon (pmugrf) handle; offsets are relative
 * to the PMU base mapped by the syscon driver.
 */
#define	RK3399_PMU_PWRDN_CON		0x0014	/* bit N=1: request power-off */
#define	RK3399_PMU_PWRDN_ST		0x0018	/* bit N=1: domain is powered down */
#define	RK3399_PMU_BUS_CLR		0x005c	/* write 1 to clear bus-idle sticky bits */
#define	RK3399_PMU_BUS_IDLE_REQ		0x0060	/* bit N=1: request bus isolation */
#define	RK3399_PMU_BUS_IDLE_ST		0x0064	/* bit N=1: bus is currently isolated */
#define	RK3399_PMU_BUS_IDLE_ACK		0x0068	/* bit N=1: PMU acknowledged idle request */

/*
 * Domain 21 (HDCP) is the only domain currently needed.  The CDN-DP
 * (USB-C DisplayPort) controller lives in this domain per the RK3399 DTS
 * power-domains property: <&power RK3399_PD_HDCP> == domain 21 (0x15).
 * bus_bit 11 is the corresponding AXI isolation bit in PMU_BUS_IDLE_*.
 */
#define	RK3399_POWER_DOMAIN_HDCP	21U
#define	RK3399_POWER_BUS_HDCP		11U
#define	RK3399_POWER_MAX_CLKS		8
#define	RK3399_POWER_POLL_RETRY		2000
#define	RK3399_POWER_POLL_DELAY_US	10

struct rk3399_power_domain_desc {
	uint32_t	id;
	uint32_t	bus_bit;
	const char	*name;
};

static const struct rk3399_power_domain_desc rk3399_power_domains[] = {
	{ RK3399_POWER_DOMAIN_HDCP, RK3399_POWER_BUS_HDCP, "hdcp" },
};

struct rk3399_power_clkset {
	clk_t		clks[RK3399_POWER_MAX_CLKS];
	int		nclks;
};

struct rk3399_power_softc {
	device_t		dev;
	phandle_t		node;
	struct syscon		*syscon;
	struct mtx		mtx;
};

static const struct ofw_compat_data rk3399_power_compat_data[] = {
	{ "rockchip,rk3399-power-controller", 1 },
	{ NULL, 0 }
};

/*
 * rk3399_power_lookup_domain
 *
 * Translates a numeric domain ID (as used in DTS power-domains phandle args)
 * into the driver's static descriptor table.  Returns NULL if the domain is
 * not implemented here, which causes rk3399_power_enable_domain() to return
 * ENOTSUP rather than silently succeeding or panicking.
 */
static const struct rk3399_power_domain_desc *
rk3399_power_lookup_domain(uint32_t id)
{
	size_t i;

	for (i = 0; i < nitems(rk3399_power_domains); i++) {
		if (rk3399_power_domains[i].id == id)
			return (&rk3399_power_domains[i]);
	}

	return (NULL);
}

/*
 * rk3399_power_find_domain_node
 *
 * The RK3399 DTS nests each power-domain as a child node of the
 * power-controller node, identified by a "reg" property equal to the domain
 * ID.  Some domains list transition clocks on their child node that must be
 * running during the power-on sequence (TRM §9.3.4).  This function walks
 * the FDT tree to locate the right child node so those clocks can be fetched.
 * Returns 0 if the node is absent (caller must handle gracefully — not all
 * domains have clock entries in the DTS).
 */
static phandle_t
rk3399_power_find_domain_node(phandle_t node, uint32_t id)
{
	phandle_t child, found;
	pcell_t reg;

	for (child = OF_child(node); child > 0; child = OF_peer(child)) {
		if (OF_getencprop(child, "reg", &reg, sizeof(reg)) > 0 &&
		    reg == id)
			return (child);
		found = rk3399_power_find_domain_node(child, id);
		if (found > 0)
			return (found);
	}

	return (0);
}

/*
 * rk3399_power_release_clocks
 *
 * Drops every clock reference acquired by rk3399_power_get_domain_clocks().
 * Called on both the success and failure paths of the enable sequence to
 * avoid leaking clock handles — these are only needed transiently during the
 * PMU handshake and should not be held for the lifetime of the domain.
 */
static void
rk3399_power_release_clocks(struct rk3399_power_clkset *clkset)
{
	int i;

	for (i = 0; i < clkset->nclks; i++) {
		if (clkset->clks[i] != NULL) {
			clk_release(clkset->clks[i]);
			clkset->clks[i] = NULL;
		}
	}
	clkset->nclks = 0;
}

/*
 * rk3399_power_disable_clocks
 *
 * Gates all transition clocks after the PMU power-on handshake completes.
 * The TRM requires these clocks to be running during the PMU state change
 * (so the PMU finite-state machine can propagate the request through the
 * clock tree), but they do not need to remain running once the domain is
 * fully powered and bus-isolated.  Disabling them reduces idle power draw.
 * Iterates in reverse-acquisition order as a precaution against dependency
 * ordering in the clock tree.
 */
static void
rk3399_power_disable_clocks(struct rk3399_power_clkset *clkset)
{
	int i;

	for (i = clkset->nclks - 1; i >= 0; i--) {
		if (clkset->clks[i] != NULL)
			(void)clk_disable(clkset->clks[i]);
	}
}

/*
 * rk3399_power_get_node_clocks
 *
 * Fetches every clock listed on an arbitrary DTS node into a transient
 * clkset.  The PMU power-domain binding uses this for per-domain transition
 * clocks (ACLK/HCLK/PCLK for HDCP), while CDN-DP uses it for the consumer
 * clocks that keep the HDCP/CDN fabric alive during the bus-idle handshake.
 * If the node has no "clocks" property the caller gets an empty clkset and
 * can skip the enable step.  The RK3399_POWER_MAX_CLKS cap guards against
 * malformed DTS that lists an unreasonably large clock array.
 */
static int
rk3399_power_get_node_clocks(struct rk3399_power_softc *sc, phandle_t node,
    struct rk3399_power_clkset *clkset)
{
	int error, i, nclks;

	bzero(clkset, sizeof(*clkset));

	if (node <= 0)
		return (0);

	error = ofw_bus_parse_xref_list_get_length(node, "clocks",
	    "#clock-cells", &nclks);
	if (error == ENOENT)
		return (0);
	if (error != 0)
		return (error);
	if (nclks > RK3399_POWER_MAX_CLKS) {
		device_printf(sc->dev,
		    "domain node has too many transition clocks: %d\n", nclks);
		return (E2BIG);
	}

	for (i = 0; i < nclks; i++) {
		error = clk_get_by_ofw_index(sc->dev, node, i, &clkset->clks[i]);
		if (error != 0) {
			device_printf(sc->dev,
			    "cannot get transient clock %d from node %#x\n", i,
			    node);
			rk3399_power_release_clocks(clkset);
			return (error);
		}
		clkset->nclks++;
	}

	return (0);
}

/*
 * rk3399_power_get_domain_clocks
 *
 * Fetches the transition clocks declared on the power-domain child node.
 * These clocks keep the PMU finite-state machine and domain fabric alive
 * while the HDCP power switch and bus-idle handshake run.
 */
static int
rk3399_power_get_domain_clocks(struct rk3399_power_softc *sc, phandle_t dnode,
    struct rk3399_power_clkset *clkset)
{

	return (rk3399_power_get_node_clocks(sc, dnode, clkset));
}

/*
 * rk3399_power_get_consumer_clocks
 *
 * Fetches clocks from the consumer node that is requesting the power domain.
 * CDN-DP lists its own functional clocks (core, APB, SPDIF_REC_DPTX, GRF)
 * separately from the HDCP power-domain node.  Bringing those clocks up
 * before polling BUS_IDLE_ST/BUS_IDLE_ACK matches the Rockchip bring-up
 * model more closely than relying on the domain clocks alone.
 */
static int
rk3399_power_get_consumer_clocks(struct rk3399_power_softc *sc,
    phandle_t consumer_node, struct rk3399_power_clkset *clkset)
{

	return (rk3399_power_get_node_clocks(sc, consumer_node, clkset));
}

/*
 * rk3399_power_enable_clocks
 *
 * Gates on every transition clock in the set before the PMU handshake begins.
 * Rolls back already-enabled clocks on the first failure so the clkset is
 * always left in a consistent all-on or all-off state.  This matters because
 * rk3399_power_disable_clocks() is always called after the handshake
 * regardless of success; a partial enable would leave some clocks running.
 */
static int
rk3399_power_enable_clocks(struct rk3399_power_softc *sc,
    struct rk3399_power_clkset *clkset)
{
	int error, i;

	for (i = 0; i < clkset->nclks; i++) {
		error = clk_enable(clkset->clks[i]);
		if (error != 0) {
			device_printf(sc->dev,
			    "cannot enable domain transition clock %d\n", i);
			while (--i >= 0)
				(void)clk_disable(clkset->clks[i]);
			return (error);
		}
	}

	return (0);
}

/*
 * rk3399_power_wait_for
 *
 * Busy-polls a PMU status register until a bitmask reaches the expected
 * state (set or clear).  Used in two places: waiting for PWRDN_ST to confirm
 * the power switch closed, and waiting for BUS_IDLE_ACK/BUS_IDLE_ST to
 * confirm the AXI bus isolation gate opened.  The 2000×10 µs budget (20 ms
 * total) is conservative — the TRM documents typical PMU transitions as
 * completing within a few microseconds — but matches what other Rockchip
 * platform drivers use for the same registers.  Returning ETIMEDOUT instead
 * of panicking gives the caller a chance to log diagnostics before failing.
 */
static int
rk3399_power_wait_for(struct rk3399_power_softc *sc, bus_size_t reg,
    uint32_t mask, bool set, const char *what)
{
	uint32_t val;
	int i;

	for (i = 0; i < RK3399_POWER_POLL_RETRY; i++) {
		val = SYSCON_READ_4(sc->syscon, reg);
		if (set) {
			if ((val & mask) == mask)
				return (0);
		} else {
			if ((val & mask) == 0)
				return (0);
		}
		DELAY(RK3399_POWER_POLL_DELAY_US);
	}

	device_printf(sc->dev,
	    "timeout waiting for %s (reg=%#jx mask=%#x val=%#x)\n",
	    what, (uintmax_t)reg, mask, SYSCON_READ_4(sc->syscon, reg));
	return (ETIMEDOUT);
}

/*
 * rk3399_power_enable_domain_locked
 *
 * Core power-on sequence for one PMU domain.  Must be called with sc->mtx
 * held to prevent concurrent enables of the same domain from interleaving
 * PMU register writes (the PMU has no hardware serialisation for concurrent
 * masters).
 *
 * Sequence (per RK3399 TRM §9.3.4 "Power Domain Turn-On Procedure"):
 *   1. Acquire and enable transition clocks plus any consumer-side clocks
 *      needed to keep the HDCP/CDN fabric alive during the handshake.
 *   2. If PWRDN_ST shows the domain is off, write PWRDN_CON to request
 *      power-on and poll PWRDN_ST until the bit clears.
 *   3. Deassert bus isolation: clear the IDLE_REQ bit, then poll IDLE_ST
 *      and IDLE_ACK until both clear — confirming the AXI fabric is
 *      accepting transactions.  Rockchip TF-A's rk3399 PMU path does not
 *      pulse BUS_CLR during the normal power-on/activate sequence.
 *   4. Disable and release transition clocks (no longer needed).
 *
 * Step 3 is the critical one for CDN-DP: if bus isolation is not cleared,
 * any MMIO access to the CDN-DP register block generates a slave-error on
 * the AXI bus, which ARM64 escalates to an SError and the kernel panics.
 */
static int
rk3399_power_enable_domain_locked(struct rk3399_power_softc *sc,
    uint32_t domain_id, phandle_t consumer_node)
{
	const struct rk3399_power_domain_desc *desc;
	struct rk3399_power_clkset consumer_clkset, domain_clkset;
	phandle_t dnode;
	uint32_t bit, bus_mask;
	int error;

	desc = rk3399_power_lookup_domain(domain_id);
	if (desc == NULL)
		return (ENOTSUP);

	dnode = rk3399_power_find_domain_node(sc->node, domain_id);
	if (dnode <= 0) {
		device_printf(sc->dev, "cannot find power-domain node %u\n",
		    domain_id);
		return (ENOENT);
	}

	error = rk3399_power_get_domain_clocks(sc, dnode, &domain_clkset);
	if (error != 0)
		return (error);

	error = rk3399_power_get_consumer_clocks(sc, consumer_node,
	    &consumer_clkset);
	if (error != 0) {
		rk3399_power_release_clocks(&domain_clkset);
		return (error);
	}

	error = rk3399_power_enable_clocks(sc, &domain_clkset);
	if (error != 0) {
		rk3399_power_release_clocks(&consumer_clkset);
		rk3399_power_release_clocks(&domain_clkset);
		return (error);
	}

	error = rk3399_power_enable_clocks(sc, &consumer_clkset);
	if (error != 0) {
		rk3399_power_disable_clocks(&domain_clkset);
		rk3399_power_release_clocks(&consumer_clkset);
		rk3399_power_release_clocks(&domain_clkset);
		return (error);
	}

	bit = (1U << domain_id);
	if ((SYSCON_READ_4(sc->syscon, RK3399_PMU_PWRDN_ST) & bit) != 0) {
		error = SYSCON_MODIFY_4(sc->syscon, RK3399_PMU_PWRDN_CON,
		    bit, 0);
		if (error == 0) {
			error = rk3399_power_wait_for(sc, RK3399_PMU_PWRDN_ST,
			    bit, false, desc->name);
		}
		if (error != 0)
			goto done;
	}

	bus_mask = (1U << desc->bus_bit);
	error = SYSCON_MODIFY_4(sc->syscon, RK3399_PMU_BUS_IDLE_REQ,
	    bus_mask, 0);
	if (error != 0)
		goto done;
	error = rk3399_power_wait_for(sc, RK3399_PMU_BUS_IDLE_ST, bus_mask,
	    false, "bus-idle-st");
	if (error != 0)
		goto done;
	error = rk3399_power_wait_for(sc, RK3399_PMU_BUS_IDLE_ACK, bus_mask,
	    false, "bus-idle-ack");

done:
	rk3399_power_disable_clocks(&consumer_clkset);
	rk3399_power_disable_clocks(&domain_clkset);
	rk3399_power_release_clocks(&consumer_clkset);
	rk3399_power_release_clocks(&domain_clkset);
	return (error);
}

/*
 * rk3399_power_enable_domain  (exported API, called by rk_cdn_dp et al.)
 *
 * Public entry point for consumer drivers to ungate a PMU power domain.
 * Validates that the provider handle is live (guards against races during
 * early boot where a consumer might call before rk3399_power0 has attached),
 * takes the serialisation mutex, and delegates to the locked helper.
 *
 * Consumers obtain the provider device_t via OF_device_from_xref() on the
 * phandle listed in their "power-domains" DTS property.  This works because
 * rk3399_power_attach() registers the device via OF_device_register_xref(),
 * which is why the attach path must complete before any consumer attaches —
 * hence the EARLY_DRIVER_MODULE ordering.
 */
int
rk3399_power_enable_domain(device_t provider, uint32_t domain_id)
{

	return (rk3399_power_enable_domain_for_node(provider, domain_id, 0));
}

/*
 * rk3399_power_enable_domain_for_node
 *
 * Public entry point that also accepts the consumer OFW node.  Supplying the
 * consumer node lets the provider transiently enable functional clocks that
 * live on the consumer (for example CDN-DP's core/APB/GRF clocks) before the
 * HDCP bus-idle handshake runs.
 */
int
rk3399_power_enable_domain_for_node(device_t provider, uint32_t domain_id,
    phandle_t consumer_node)
{
	struct rk3399_power_softc *sc;
	int error;

	if (provider == NULL)
		return (ENODEV);

	sc = device_get_softc(provider);
	if (sc == NULL || sc->syscon == NULL)
		return (ENXIO);

	mtx_lock(&sc->mtx);
	error = rk3399_power_enable_domain_locked(sc, domain_id,
	    consumer_node);
	mtx_unlock(&sc->mtx);

	return (error);
}

/*
 * rk3399_power_probe
 *
 * Matches only the "rockchip,rk3399-power-controller" compatible string.
 * The status check ensures we don't attach to nodes marked disabled in the
 * DTB (though in practice the PMU node is always enabled at boot).
 */
static int
rk3399_power_probe(device_t dev)
{
	if (!ofw_bus_status_okay(dev))
		return (ENXIO);
	if (ofw_bus_search_compatible(dev, rk3399_power_compat_data)->ocd_data == 0)
		return (ENXIO);

	device_set_desc(dev, "Rockchip RK3399 power-domain controller");
	return (BUS_PROBE_DEFAULT);
}

/*
 * rk3399_power_attach
 *
 * Attaches to the PMU syscon and registers this device as the OFW xref
 * provider for the power-controller node.  The xref registration is what
 * allows consumer drivers to call OF_device_from_xref() and obtain this
 * device_t — without it, rk_cdn_dp_get_power_domain() returns NULL and the
 * CDN-DP driver cannot enable its power domain.
 *
 * The syscon handle gives access to the PMU register block without needing
 * a separate bus_alloc_resources() call; the PMU parent node owns the MMIO
 * mapping and the syscon interface provides the read/modify/write primitives.
 *
 * Note: bus name in EARLY_DRIVER_MODULE must be "simple_mfd" — the PMU node
 * (power-management@ff310000) is a simple-mfd node and simple_mfd0 is the
 * parent bus of the power-controller child.  Using "simplebus" here silently
 * prevents attachment and leaves the CDN-DP power domain ungated.
 */
static int
rk3399_power_attach(device_t dev)
{
	struct rk3399_power_softc *sc;
	int error;

	sc = device_get_softc(dev);
	sc->dev = dev;
	sc->node = ofw_bus_get_node(dev);
	mtx_init(&sc->mtx, "rk3399_power", NULL, MTX_DEF);

	error = SYSCON_GET_HANDLE(dev, &sc->syscon);
	if (error != 0) {
		device_printf(dev, "cannot get parent syscon\n");
		mtx_destroy(&sc->mtx);
		return (error);
	}

	OF_device_register_xref(OF_xref_from_node(sc->node), dev);
	return (0);
}

/*
 * rk3399_power_detach
 *
 * Unregisters the OFW xref so that any consumer that calls
 * OF_device_from_xref() after detach receives NULL rather than a dangling
 * device_t.  The rk3399_power_enable_domain() NULL-check then returns ENODEV
 * cleanly instead of dereferencing freed memory.
 */
static int
rk3399_power_detach(device_t dev)
{
	struct rk3399_power_softc *sc;

	sc = device_get_softc(dev);
	OF_device_register_xref(OF_xref_from_node(sc->node), NULL);
	mtx_destroy(&sc->mtx);
	return (0);
}

static device_method_t rk3399_power_methods[] = {
	DEVMETHOD(device_probe,		rk3399_power_probe),
	DEVMETHOD(device_attach,	rk3399_power_attach),
	DEVMETHOD(device_detach,	rk3399_power_detach),

	DEVMETHOD_END
};

static driver_t rk3399_power_driver = {
	"rk3399_power",
	rk3399_power_methods,
	sizeof(struct rk3399_power_softc),
};

/*
 * EARLY_DRIVER_MODULE on simple_mfd at BUS_PASS_RESOURCE+MIDDLE ensures
 * rk3399_power0 attaches before any consumer (rk_cdn_dp, hdmi) tries to
 * resolve its "power-domains" phandle during their own attach passes.
 *
 * The bus name MUST be "simple_mfd" — in the RK3399 DTS the power-controller
 * node is a child of power-management@ff310000, which carries the compatible
 * string "rockchip,rk3399-pmu","syscon","simple-mfd".  That node is claimed by
 * the FreeBSD simple_mfd driver, making simple_mfd0 the direct parent bus of
 * the power-controller child.  Using "simplebus" here silently prevents attach.
 */
EARLY_DRIVER_MODULE(rk3399_power, simple_mfd, rk3399_power_driver, 0, 0,
    BUS_PASS_RESOURCE + BUS_PASS_ORDER_MIDDLE);
MODULE_VERSION(rk3399_power, 1);
MODULE_DEPEND(rk3399_power, clk, 1, 1, 1);
MODULE_DEPEND(rk3399_power, ofwbus, 1, 1, 1);
MODULE_DEPEND(rk3399_power, syscon, 1, 1, 1);
