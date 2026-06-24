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
 * Samsung CMU clock-controller framework.
 *
 * This file is the SoC-independent half of the driver: it provides the
 * clkdev(9) register accessors, the table-to-clknode translation helpers and
 * the common attach path.  A per-SoC driver (e.g. exynos_clk_zuma.c)
 * supplies the clock tables via struct samsung_cmu_info and inherits this
 * class for the register interface.
 */

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/bus.h>
#include <sys/kernel.h>
#include <sys/lock.h>
#include <sys/malloc.h>
#include <sys/module.h>
#include <sys/mutex.h>
#include <sys/rman.h>

#include <machine/bus.h>

#include <dev/ofw/ofw_bus.h>
#include <dev/ofw/ofw_bus_subr.h>

#include <dev/clk/clk.h>
#include <dev/clk/clk_div.h>
#include <dev/clk/clk_fixed.h>
#include <dev/clk/clk_gate.h>
#include <dev/clk/clk_mux.h>

#include <arm64/exynos/clk/exynos_clk.h>

#include "clkdev_if.h"

#define	OSCCLK_NAME		"oscclk"
#define	OSCCLK_DEFAULT_FREQ	24576000	/* 24.576 MHz */

/*
 * Register layout used by exynos_cmu_init_clocks(), matching the arm64 Exynos
 * CMU layout shared by Linux's clk-exynos-arm64.c.
 */
#define	GATE_OFF_START		0x2000
#define	GATE_OFF_END		0x2fff
#define	GATE_MANUAL		(1U << 20)
#define	GATE_ENABLE_HWACG	(1U << 28)

#define	CMU_READ4(sc, reg)		bus_read_4((sc)->res, (reg))
#define	CMU_WRITE4(sc, reg, val)	bus_write_4((sc)->res, (reg), (val))

/* ---- clkdev register interface ------------------------------------------ */

static int
exynos_clk_write_4(device_t dev, bus_addr_t addr, uint32_t val)
{
	struct exynos_clk_softc *sc;

	sc = device_get_softc(dev);
	CMU_WRITE4(sc, addr, val);
	return (0);
}

static int
exynos_clk_read_4(device_t dev, bus_addr_t addr, uint32_t *val)
{
	struct exynos_clk_softc *sc;

	sc = device_get_softc(dev);
	*val = CMU_READ4(sc, addr);
	return (0);
}

static int
exynos_clk_modify_4(device_t dev, bus_addr_t addr, uint32_t clr, uint32_t set)
{
	struct exynos_clk_softc *sc;
	uint32_t reg;

	sc = device_get_softc(dev);
	reg = CMU_READ4(sc, addr);
	reg &= ~clr;
	reg |= set;
	CMU_WRITE4(sc, addr, reg);
	return (0);
}

static void
exynos_clk_device_lock(device_t dev)
{
	struct exynos_clk_softc *sc;

	sc = device_get_softc(dev);
	mtx_lock(&sc->mtx);
}

static void
exynos_clk_device_unlock(device_t dev)
{
	struct exynos_clk_softc *sc;

	sc = device_get_softc(dev);
	mtx_unlock(&sc->mtx);
}

/* ---- input clock resolution --------------------------------------------- */

/*
 * Build the map from this CMU's "clock-names" inputs to the global clk(9)
 * names of their providers.  "oscclk" always maps to the framework-registered
 * oscillator; the rest are resolved through the FDT "clocks" phandles (their
 * providers, e.g. CMU_TOP, are already registered by the time the leaf domains
 * attach at the later bus pass).
 */
static void
exynos_clk_build_inputs(struct exynos_clk_softc *sc)
{
	phandle_t node;
	const char **names;
	clk_t clk;
	int i, n;

	node = ofw_bus_get_node(sc->dev);
	n = ofw_bus_string_list_to_array(node, "clock-names", &names);
	if (n <= 0)
		return;

	sc->inputs = malloc(n * sizeof(*sc->inputs), M_DEVBUF,
	    M_WAITOK | M_ZERO);
	for (i = 0; i < n; i++) {
		sc->inputs[i].name = strdup(names[i], M_DEVBUF);
		if (strcmp(names[i], OSCCLK_NAME) == 0)
			sc->inputs[i].global = OSCCLK_NAME;
		else if (clk_get_by_ofw_name(sc->dev, 0, names[i], &clk) == 0) {
			sc->inputs[i].global = strdup(clk_get_name(clk),
			    M_DEVBUF);
			clk_release(clk);
		} else {
			sc->inputs[i].global = NULL;
			device_printf(sc->dev,
			    "input clock \"%s\" is not available\n", names[i]);
		}
	}
	sc->ninputs = n;
	OF_prop_free(names);
}

/*
 * Rewrite a table parent reference: if it names one of this CMU's inputs,
 * return the resolved global provider name; otherwise return it unchanged.
 */
static const char *
exynos_clk_xlate(struct exynos_clk_softc *sc, const char *name)
{
	int i;

	if (name == NULL)
		return (NULL);
	for (i = 0; i < sc->ninputs; i++) {
		if (strcmp(name, sc->inputs[i].name) == 0)
			return (sc->inputs[i].global != NULL ?
			    sc->inputs[i].global : name);
	}
	return (name);
}

/* ---- table -> clknode translation --------------------------------------- */

static void
exynos_clk_register_fixed_rates(struct exynos_clk_softc *sc,
    const struct samsung_fixed_rate_clock *list, unsigned int n)
{
	struct clk_fixed_def def;
	const char *parent;
	unsigned int i;

	for (i = 0; i < n; i++) {
		memset(&def, 0, sizeof(def));
		def.clkdef.id = list[i].id;
		def.clkdef.name = list[i].name;
		if (list[i].parent_name != NULL) {
			parent = exynos_clk_xlate(sc, list[i].parent_name);
			def.clkdef.parent_names = &parent;
			def.clkdef.parent_cnt = 1;
		}
		def.freq = list[i].fixed_rate;
		if (clknode_fixed_register(sc->clkdom, &def) != 0)
			device_printf(sc->dev, "cannot register clock %s\n",
			    list[i].name);
	}
}

static void
exynos_clk_register_fixed_factors(struct exynos_clk_softc *sc,
    const struct samsung_fixed_factor_clock *list, unsigned int n)
{
	struct clk_fixed_def def;
	const char *parent;
	unsigned int i;

	for (i = 0; i < n; i++) {
		memset(&def, 0, sizeof(def));
		def.clkdef.id = list[i].id;
		def.clkdef.name = list[i].name;
		parent = exynos_clk_xlate(sc, list[i].parent_name);
		def.clkdef.parent_names = &parent;
		def.clkdef.parent_cnt = 1;
		def.mult = list[i].mult;
		def.div = list[i].div;
		if (clknode_fixed_register(sc->clkdom, &def) != 0)
			device_printf(sc->dev, "cannot register clock %s\n",
			    list[i].name);
	}
}

static void
exynos_clk_register_muxes(struct exynos_clk_softc *sc,
    const struct samsung_mux_clock *list, unsigned int n)
{
	struct clk_mux_def def;
	const char **parents;
	unsigned int i;
	uint8_t j;

	for (i = 0; i < n; i++) {
		memset(&def, 0, sizeof(def));
		def.clkdef.id = list[i].id;
		def.clkdef.name = list[i].name;
		parents = malloc(list[i].num_parents * sizeof(*parents),
		    M_DEVBUF, M_WAITOK);
		for (j = 0; j < list[i].num_parents; j++)
			parents[j] = exynos_clk_xlate(sc,
			    list[i].parent_names[j]);
		def.clkdef.parent_names = parents;
		def.clkdef.parent_cnt = list[i].num_parents;
		def.offset = list[i].offset;
		def.shift = list[i].shift;
		def.width = list[i].width;
		if (clknode_mux_register(sc->clkdom, &def) != 0)
			device_printf(sc->dev, "cannot register clock %s\n",
			    list[i].name);
		free(parents, M_DEVBUF);
	}
}

static void
exynos_clk_register_divs(struct exynos_clk_softc *sc,
    const struct samsung_div_clock *list, unsigned int n)
{
	struct clk_div_def def;
	const char *parent;
	unsigned int i;

	for (i = 0; i < n; i++) {
		memset(&def, 0, sizeof(def));
		def.clkdef.id = list[i].id;
		def.clkdef.name = list[i].name;
		parent = exynos_clk_xlate(sc, list[i].parent_name);
		def.clkdef.parent_names = &parent;
		def.clkdef.parent_cnt = 1;
		def.offset = list[i].offset;
		def.i_shift = list[i].shift;
		def.i_width = list[i].width;
		if (list[i].table != NULL) {
			def.div_flags = CLK_DIV_WITH_TABLE;
			def.div_table = __DECONST(struct clk_div_table *,
			    list[i].table);
		}
		if (clknode_div_register(sc->clkdom, &def) != 0)
			device_printf(sc->dev, "cannot register clock %s\n",
			    list[i].name);
	}
}

static void
exynos_clk_register_gates(struct exynos_clk_softc *sc,
    const struct samsung_gate_clock *list, unsigned int n)
{
	struct clk_gate_def def;
	const char *parent;
	unsigned int i;

	for (i = 0; i < n; i++) {
		memset(&def, 0, sizeof(def));
		def.clkdef.id = list[i].id;
		def.clkdef.name = list[i].name;
		parent = exynos_clk_xlate(sc, list[i].parent_name);
		def.clkdef.parent_names = &parent;
		def.clkdef.parent_cnt = 1;
		def.offset = list[i].offset;
		def.shift = list[i].bit_idx;
		def.mask = 1;
		def.on_value = 1;
		def.off_value = 0;
		if (clknode_gate_register(sc->clkdom, &def) != 0)
			device_printf(sc->dev, "cannot register clock %s\n",
			    list[i].name);
	}
}

/*
 * Register the external oscillator the CMU tables refer to by the well-known
 * "oscclk" name.  The clk(9) namespace is global, so this is done exactly
 * once, by whichever CMU attaches first (CMU_TOP, at the earlier bus pass).
 * Device probing is single-threaded during boot, so a plain flag suffices and
 * avoids taking the clock topology lock here.  The reference rate is read from
 * the CMU's "oscclk" input when the provider is already up, else defaulted.
 */
static bool exynos_oscclk_registered = false;

static void
exynos_clk_register_oscclk(struct exynos_clk_softc *sc)
{
	struct clk_fixed_def def;
	clk_t parent;
	uint64_t freq;

	if (exynos_oscclk_registered)
		return;

	freq = OSCCLK_DEFAULT_FREQ;
	if (clk_get_by_ofw_name(sc->dev, 0, OSCCLK_NAME, &parent) == 0) {
		if (clk_get_freq(parent, &freq) != 0 || freq == 0)
			freq = OSCCLK_DEFAULT_FREQ;
		clk_release(parent);
	}

	memset(&def, 0, sizeof(def));
	def.clkdef.name = OSCCLK_NAME;
	def.freq = freq;
	if (clknode_fixed_register(sc->clkdom, &def) != 0)
		device_printf(sc->dev, "cannot register %s\n", OSCCLK_NAME);
	else
		exynos_oscclk_registered = true;
}

/*
 * Put the CMU's gate clocks into manual-control mode, matching the
 * configuration Linux's exynos_arm64_init_clocks() applies for these arm64
 * Exynos DTs.  Without the legacy "samsung,...-cmu" auto-gating property the
 * global automatic mode stays off, so we only clear HWACG and assert the
 * (TRM-reserved) manual override on each gate register the SoC lists.
 */
static void
exynos_cmu_init_clocks(struct exynos_clk_softc *sc)
{
	const struct samsung_cmu_info *cmu = sc->cmu;
	unsigned long off;
	uint32_t val;
	unsigned int i;

	for (i = 0; i < cmu->nr_clk_regs; i++) {
		off = cmu->clk_regs[i];
		if (off < GATE_OFF_START || off > GATE_OFF_END)
			continue;
		val = CMU_READ4(sc, off);
		val |= GATE_MANUAL;
		val &= ~GATE_ENABLE_HWACG;
		CMU_WRITE4(sc, off, val);
	}
}

/* ---- attach ------------------------------------------------------------- */

int
exynos_cmu_attach(device_t dev)
{
	struct exynos_clk_softc *sc;
	const struct samsung_cmu_info *cmu;
	phandle_t node;
	int rid;

	sc = device_get_softc(dev);
	sc->dev = dev;
	cmu = sc->cmu;
	node = ofw_bus_get_node(dev);

	rid = 0;
	sc->res = bus_alloc_resource_any(dev, SYS_RES_MEMORY, &rid, RF_ACTIVE);
	if (sc->res == NULL) {
		device_printf(dev, "cannot allocate registers\n");
		return (ENXIO);
	}

	mtx_init(&sc->mtx, device_get_nameunit(dev), NULL, MTX_DEF);

	sc->clkdom = clkdom_create(dev);
	if (sc->clkdom == NULL) {
		device_printf(dev, "cannot create clock domain\n");
		bus_release_resource(dev, SYS_RES_MEMORY, rid, sc->res);
		mtx_destroy(&sc->mtx);
		return (ENXIO);
	}

	exynos_clk_register_oscclk(sc);
	exynos_clk_build_inputs(sc);

	if (cmu->nr_clk_regs != 0)
		exynos_cmu_init_clocks(sc);

	exynos_clk_register_plls(sc, cmu->pll_clks, cmu->nr_pll_clks);
	exynos_clk_register_muxes(sc, cmu->mux_clks, cmu->nr_mux_clks);
	exynos_clk_register_divs(sc, cmu->div_clks, cmu->nr_div_clks);
	exynos_clk_register_gates(sc, cmu->gate_clks, cmu->nr_gate_clks);
	exynos_clk_register_fixed_factors(sc, cmu->fixed_factor_clks,
	    cmu->nr_fixed_factor_clks);
	exynos_clk_register_fixed_rates(sc, cmu->fixed_clks,
	    cmu->nr_fixed_clks);

	if (clkdom_finit(sc->clkdom) != 0)
		device_printf(dev, "cannot finalize clock domain\n");

	if (bootverbose)
		clkdom_dump(sc->clkdom);

	clk_set_assigned(dev, node);

	return (0);
}

static device_method_t exynos_cmu_methods[] = {
	/* clkdev interface */
	DEVMETHOD(clkdev_write_4,	exynos_clk_write_4),
	DEVMETHOD(clkdev_read_4,		exynos_clk_read_4),
	DEVMETHOD(clkdev_modify_4,	exynos_clk_modify_4),
	DEVMETHOD(clkdev_device_lock,	exynos_clk_device_lock),
	DEVMETHOD(clkdev_device_unlock,	exynos_clk_device_unlock),

	DEVMETHOD_END
};

DEFINE_CLASS_0(exynos_cmu, exynos_cmu_driver, exynos_cmu_methods,
    sizeof(struct exynos_clk_softc));
