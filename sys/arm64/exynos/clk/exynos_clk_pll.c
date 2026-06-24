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
 * Samsung PLL clock node for the generic Exynos clock framework.
 *
 * Most modern (arm64) Exynos PLLs share the integer "PMS" layout: the
 * control register holds a pre-divider P, a multiplier M and a post-divide
 * shift S, giving
 *
 *	fout = (fin * M) / (P << S)
 *
 * (the pll_0516x flavour additionally doubles the result).  These are
 * implemented here.  The PLLs are programmed by firmware, so the node is
 * read-only: it reports the running rate and never reprograms the PLL.
 *
 * Fractional-N families (e.g. pll_36xx) are not implemented yet; a node of an
 * unsupported type is still registered so that consumers resolve, but it
 * reports its parent's rate and warns once.
 */

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/bus.h>

#include <dev/clk/clk.h>

#include <arm64/exynos/clk/exynos_clk.h>

#include "clkdev_if.h"

/* Integer PMS layout shared by the pll_35xx and pll_0822x families. */
#define	PLL_PMS_MDIV_MASK	0x3ff
#define	PLL_PMS_PDIV_MASK	0x3f
#define	PLL_PMS_SDIV_MASK	0x7
#define	PLL_PMS_MDIV_SHIFT	16
#define	PLL_PMS_PDIV_SHIFT	8
#define	PLL_PMS_SDIV_SHIFT	0

struct exynos_clk_pll_sc {
	uint32_t			con_offset;
	enum samsung_pll_type		type;
	const struct samsung_pll_rate_table *rate_table;
	bool				warned;
};

#define	RD4(_clk, off, val)	\
	CLKDEV_READ_4(clknode_get_device(_clk), (off), (val))

/*
 * True for PLL flavours whose recalc uses the plain integer PMS layout above.
 * This covers the families Linux routes through its pll35xx/pll0822x ops.
 */
static bool
exynos_clk_pll_is_pms(enum samsung_pll_type type)
{

	switch (type) {
	case pll_35xx:
	case pll_2550:
	case pll_142xx:
	case pll_1017x:
	case pll_1450x:
	case pll_1451x:
	case pll_1452x:
	case pll_1417x:
	case pll_1418x:
	case pll_1051x:
	case pll_1052x:
	case pll_0818x:
	case pll_0822x:
	case pll_0831x:
	case pll_0516x:
	case pll_0517x:
	case pll_0518x:
	case pll_0717x:
	case pll_0718x:
	case pll_0732x:
	case pll_531x:
		return (true);
	default:
		return (false);
	}
}

static int
exynos_clk_pll_init(struct clknode *clk, device_t dev)
{

	/* Single parent (the reference oscillator). */
	clknode_init_parent_idx(clk, 0);
	return (0);
}

static int
exynos_clk_pll_recalc(struct clknode *clk, uint64_t *freq)
{
	struct exynos_clk_pll_sc *sc;
	uint32_t con, mdiv, pdiv, sdiv;
	uint64_t fvco;

	sc = clknode_get_softc(clk);

	if (!exynos_clk_pll_is_pms(sc->type)) {
		if (!sc->warned) {
			printf("%s: unsupported Samsung PLL type %d, "
			    "reporting parent rate\n", clknode_get_name(clk),
			    sc->type);
			sc->warned = true;
		}
		return (0);
	}

	RD4(clk, sc->con_offset, &con);

	mdiv = (con >> PLL_PMS_MDIV_SHIFT) & PLL_PMS_MDIV_MASK;
	pdiv = (con >> PLL_PMS_PDIV_SHIFT) & PLL_PMS_PDIV_MASK;
	sdiv = (con >> PLL_PMS_SDIV_SHIFT) & PLL_PMS_SDIV_MASK;

	if (pdiv == 0) {
		*freq = 0;
		return (0);
	}

	fvco = *freq;
	fvco *= mdiv;
	if (sc->type == pll_0516x)
		fvco *= 2;
	fvco /= ((uint64_t)pdiv << sdiv);
	*freq = fvco;

	return (0);
}

static clknode_method_t exynos_clk_pll_methods[] = {
	CLKNODEMETHOD(clknode_init,		exynos_clk_pll_init),
	CLKNODEMETHOD(clknode_recalc_freq,	exynos_clk_pll_recalc),
	CLKNODEMETHOD_END
};

DEFINE_CLASS_1(exynos_clk_pll, exynos_clk_pll_class, exynos_clk_pll_methods,
    sizeof(struct exynos_clk_pll_sc), clknode_class);

void
exynos_clk_register_plls(struct exynos_clk_softc *cmu_sc,
    const struct samsung_pll_clock *list, unsigned int n)
{
	struct clknode_init_def def;
	struct exynos_clk_pll_sc *sc;
	struct clknode *clk;
	unsigned int i;

	for (i = 0; i < n; i++) {
		memset(&def, 0, sizeof(def));
		def.id = list[i].id;
		def.name = list[i].name;
		def.parent_names =
		    __DECONST(const char **, &list[i].parent_name);
		def.parent_cnt = 1;

		clk = clknode_create(cmu_sc->clkdom, &exynos_clk_pll_class,
		    &def);
		if (clk == NULL) {
			device_printf(cmu_sc->dev,
			    "cannot create clock %s\n", list[i].name);
			continue;
		}

		sc = clknode_get_softc(clk);
		sc->con_offset = list[i].con_offset;
		sc->type = list[i].type;
		sc->rate_table = list[i].rate_table;
		sc->warned = false;

		clknode_register(cmu_sc->clkdom, clk);
	}
}
