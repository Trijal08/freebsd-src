/*-
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Copyright (c) 2025 Trijal Saha <trijalsaha2012@gmail.com>
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
 * ANY EXPRESS OR IMPLIED WARRANTIES ARE DISCLAIMED.  IN NO EVENT SHALL THE
 * AUTHOR OR CONTRIBUTORS BE LIABLE FOR ANY DAMAGES WHATSOEVER ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE.
 */

/*
 * Samsung/Google Exynos (gs101) watchdog timer.
 *
 * The block is the classic s3c2410 design: a down-counter (WTCNT) reloaded
 * from WTDAT and gated by WTCON.  When it underflows it asserts a SoC reset.
 * On gs101 the counter and its reset output are additionally gated by the
 * PMU, reached through a syscon: the reset only fires once the per-cluster
 * counter-enable and reset-unmask bits are set.
 *
 * Bring-up scope: CPU cluster 0 only.  No clock driver is required -- the
 * bootloader leaves the watchdog pclk running and the count source is the
 * fixed 24.5MHz oscillator (used here as a constant for the timeout math).
 */

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/bus.h>
#include <sys/kernel.h>
#include <sys/module.h>
#include <sys/mutex.h>
#include <sys/rman.h>
#include <sys/watchdog.h>
#include <sys/eventhandler.h>

#include <machine/bus.h>

#include <dev/ofw/openfirm.h>
#include <dev/ofw/ofw_bus.h>
#include <dev/ofw/ofw_bus_subr.h>
#include <dev/ofw/ofw_subr.h>

#include <dev/psci/smccc.h>

/*
 * On gs101 the PMU registers are protected: reads are plain MMIO, but
 * writes have to go through a secure monitor call -- a direct store faults
 * with an SError.  This is the SMC the EL3 firmware implements (matching the
 * Linux gs101-pmu driver); TENSOR_PMUREG_RMW asks it to read-modify-write a
 * register so no separate read is needed here.
 */
#define	TENSOR_SMC_PMU_SEC_REG	0x82000504
#define	TENSOR_PMUREG_RMW	2

/* Watchdog registers. */
#define	WTCON			0x00
#define	 WTCON_RSTEN		(1u << 0)
#define	 WTCON_INTEN		(1u << 2)
#define	 WTCON_CLKSEL_DIV128	(3u << 3)
#define	 WTCON_ENABLE		(1u << 5)
#define	 WTCON_PRESCALE(x)	(((x) & 0xff) << 8)
#define	 WTCON_DBGACK_MASK	(1u << 16)
#define	WTDAT			0x04
#define	WTCNT			0x08
#define	WTCLRINT		0x0c

#define	WDT_DIVISOR		128
#define	WDT_MAX_CNT		0xffffffffu
#define	WDT_SRC_HZ		24500000u	/* ext_24_5m oscillator */

/*
 * gs101 PMU gating is per CPU cluster: each cluster has its own
 * counter-enable and reset-mask register and bit.  Setting the reset-mask
 * bit unmasks (allows) the reset -- the field is inverted on gs101.
 */
struct exynos_wdt_cluster {
	bus_size_t	cnt_en_reg;
	uint32_t	cnt_en_bit;
	bus_size_t	mask_reset_reg;
	uint32_t	mask_bit;
};

static const struct exynos_wdt_cluster gs101_clusters[] = {
	{	/* cluster 0 */
		.cnt_en_reg = 0x1220,		/* CLUSTER0_NONCPU_OUT */
		.cnt_en_bit = (1u << 8),
		.mask_reset_reg = 0x1244,	/* CLUSTER0_NONCPU_INT_EN */
		.mask_bit = (1u << 2),
	},
	{	/* cluster 1 */
		.cnt_en_reg = 0x1420,		/* CLUSTER1_NONCPU_OUT */
		.cnt_en_bit = (1u << 7),
		.mask_reset_reg = 0x1444,	/* CLUSTER1_NONCPU_INT_EN */
		.mask_bit = (1u << 2),
	},
};

struct exynos_wdt_softc {
	device_t			dev;
	struct resource			*res;
	uint64_t			pmu_phys;
	const struct exynos_wdt_cluster	*cl;
	struct mtx			mtx;
};

#define	RD4(sc, r)	bus_read_4((sc)->res, (r))
#define	WR4(sc, r, v)	bus_write_4((sc)->res, (r), (v))

static struct ofw_compat_data compat_data[] = {
	{ "google,gs101-wdt",	1 },
	{ NULL,			0 }
};

/*
 * Program the PMU gates.  Both bits are simple read-modify-write: setting
 * GS_WDT_RST_UNMASK allows the watchdog to reset the SoC, GS_WDT_CNT_EN
 * starts the counter.
 */
static void
exynos_wdt_pmu_rmw(struct exynos_wdt_softc *sc, bus_size_t reg, uint32_t mask,
    uint32_t val)
{
	struct arm_smccc_res res;

	arm_smccc_smc(TENSOR_SMC_PMU_SEC_REG, sc->pmu_phys + reg,
	    TENSOR_PMUREG_RMW, mask, val, 0, 0, 0, &res);
}

static void
exynos_wdt_pmu(struct exynos_wdt_softc *sc, bool enable)
{
	const struct exynos_wdt_cluster *cl = sc->cl;

	/* Unmask the reset, then enable the counter (or the reverse). */
	exynos_wdt_pmu_rmw(sc, cl->mask_reset_reg, cl->mask_bit,
	    enable ? cl->mask_bit : 0);
	exynos_wdt_pmu_rmw(sc, cl->cnt_en_reg, cl->cnt_en_bit,
	    enable ? cl->cnt_en_bit : 0);
}

static void
exynos_wdt_stop(struct exynos_wdt_softc *sc)
{

	WR4(sc, WTCON, RD4(sc, WTCON) & ~(WTCON_ENABLE | WTCON_RSTEN));
	exynos_wdt_pmu(sc, false);
}

static void
exynos_wdt_start(struct exynos_wdt_softc *sc, uint32_t count)
{

	exynos_wdt_pmu(sc, true);

	WR4(sc, WTDAT, count);
	WR4(sc, WTCNT, count);
	WR4(sc, WTCON, WTCON_ENABLE | WTCON_RSTEN | WTCON_CLKSEL_DIV128 |
	    WTCON_PRESCALE(0) | WTCON_DBGACK_MASK);
}

static void
exynos_wdt_watchdog_fn(void *private, u_int cmd, int *error)
{
	struct exynos_wdt_softc *sc;
	uint64_t timeout_s, count;

	sc = private;
	mtx_lock(&sc->mtx);

	cmd &= WD_INTERVAL;
	if (cmd == 0) {
		/* Disarm. */
		exynos_wdt_stop(sc);
		mtx_unlock(&sc->mtx);
		return;
	}

	/* cmd is the timeout as a power-of-two number of nanoseconds. */
	timeout_s = ((uint64_t)1 << cmd) / 1000000000ULL;
	if (timeout_s == 0)
		timeout_s = 1;

	count = timeout_s * (WDT_SRC_HZ / WDT_DIVISOR);
	if (count == 0 || count > WDT_MAX_CNT) {
		/* Out of range; leave the watchdog disarmed. */
		exynos_wdt_stop(sc);
		mtx_unlock(&sc->mtx);
		return;
	}

	exynos_wdt_start(sc, (uint32_t)count);
	*error = 0;

	mtx_unlock(&sc->mtx);
}

static int
exynos_wdt_probe(device_t dev)
{

	if (!ofw_bus_status_okay(dev))
		return (ENXIO);
	if (ofw_bus_search_compatible(dev, compat_data)->ocd_data == 0)
		return (ENXIO);

	device_set_desc(dev, "Exynos watchdog timer");
	return (BUS_PROBE_DEFAULT);
}

static int
exynos_wdt_attach(device_t dev)
{
	struct exynos_wdt_softc *sc;
	phandle_t node, pmu_node;
	bus_addr_t pa;
	bus_size_t size;
	pcell_t xref;
	uint32_t cluster;
	int rid;

	sc = device_get_softc(dev);
	sc->dev = dev;
	node = ofw_bus_get_node(dev);

	cluster = 0;
	OF_getencprop(node, "samsung,cluster-index", &cluster, sizeof(cluster));
	if (cluster >= nitems(gs101_clusters)) {
		device_printf(dev, "unsupported cluster index %u\n", cluster);
		return (ENXIO);
	}
	sc->cl = &gs101_clusters[cluster];

	/* Resolve the PMU physical base; its writes go through the SMC. */
	if (OF_getencprop(node, "samsung,syscon-phandle", &xref,
	    sizeof(xref)) <= 0 ||
	    (pmu_node = OF_node_from_xref(xref)) == 0 ||
	    ofw_reg_to_paddr(pmu_node, 0, &pa, &size, NULL) != 0) {
		device_printf(dev, "could not resolve PMU base\n");
		return (ENXIO);
	}
	sc->pmu_phys = pa;

	rid = 0;
	sc->res = bus_alloc_resource_any(dev, SYS_RES_MEMORY, &rid, RF_ACTIVE);
	if (sc->res == NULL) {
		device_printf(dev, "could not allocate memory resource\n");
		return (ENXIO);
	}

	mtx_init(&sc->mtx, "exynos_wdt", NULL, MTX_DEF);

	/* Start disarmed; watchdog(9) arms us on demand. */
	exynos_wdt_stop(sc);

	EVENTHANDLER_REGISTER(watchdog_list, exynos_wdt_watchdog_fn, sc, 0);

	return (0);
}

static device_method_t exynos_wdt_methods[] = {
	DEVMETHOD(device_probe,		exynos_wdt_probe),
	DEVMETHOD(device_attach,	exynos_wdt_attach),

	DEVMETHOD_END
};

static driver_t exynos_wdt_driver = {
	"exynos_wdt",
	exynos_wdt_methods,
	sizeof(struct exynos_wdt_softc),
};

DRIVER_MODULE(exynos_wdt, simplebus, exynos_wdt_driver, 0, 0);
