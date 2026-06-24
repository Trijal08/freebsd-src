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
 * Common clock framework support for Samsung "CMU" clock controllers, as
 * used by the arm64 Exynos and Google Tensor SoCs.
 *
 * This mirrors the table-driven model of Linux's drivers/clk/samsung: a SoC
 * is described as a set of CMU domains, each an array of PLL, mux, divider,
 * gate and fixed-factor clocks.  The same struct layouts and table macros are
 * used here so the per-SoC tables can be carried over from Linux essentially
 * verbatim; the registration helpers translate each entry into a node of
 * FreeBSD's generic clk(9) framework.
 */

#ifndef _ARM64_EXYNOS_CLK_H_
#define	_ARM64_EXYNOS_CLK_H_

#include <dev/clk/clk.h>

#define	ARRAY_SIZE(x)	nitems(x)

/*
 * Linux common-clock flags referenced by the per-SoC tables.  FreeBSD's
 * clk(9) framework expresses these policies differently, so the values are
 * recorded but not otherwise acted upon during registration.
 */
#define	CLK_SET_RATE_PARENT		0x0001
#define	CLK_IGNORE_UNUSED		0x0002
#define	CLK_IS_CRITICAL			0x0004
#define	CLK_SET_RATE_NO_REPARENT	0x0008
#define	CLK_GET_RATE_NOCACHE		0x0010

/*
 * Samsung PLL flavours.  The full set is enumerated (matching Linux's
 * drivers/clk/samsung/clk-pll.h) so any Exynos SoC's clock tables compile
 * against this framework; exynos_clk_pll.c implements the families it knows
 * and rejects the rest at registration time.
 */
enum samsung_pll_type {
	pll_2126,
	pll_3000,
	pll_35xx,
	pll_36xx,
	pll_2550,
	pll_2650,
	pll_4500,
	pll_4502,
	pll_4508,
	pll_4600,
	pll_4650,
	pll_4650c,
	pll_6552,
	pll_6552_s3c2416,
	pll_6553,
	pll_2550x,
	pll_2550xx,
	pll_2650x,
	pll_2650xx,
	pll_1417x,
	pll_1418x,
	pll_1450x,
	pll_1451x,
	pll_1452x,
	pll_1460x,
	pll_0818x,
	pll_0822x,
	pll_0831x,
	pll_142xx,
	pll_0516x,
	pll_0517x,
	pll_0518x,
	pll_531x,
	pll_1051x,
	pll_1052x,
	pll_0717x,
	pll_0718x,
	pll_0732x,
	pll_4311,
	pll_1017x,
	pll_1031x,
	pll_a9fracm,
	pll_a9fraco,
};

/* One entry of a PLL's PMS rate table (matches Linux layout). */
struct samsung_pll_rate_table {
	unsigned int rate;
	unsigned int pdiv;
	unsigned int mdiv;
	unsigned int sdiv;
	unsigned int kdiv;
	unsigned int afc;
	unsigned int mfr;
	unsigned int mrr;
	unsigned int vsel;
};

/*
 * PLL rate-table entry constructors.  The reference frequency (_fin) is
 * recorded by convention only; the node is read-only so the table merely
 * documents the firmware-programmed operating point.
 */
#define	PLL_35XX_RATE(_fin, _rate, _m, _p, _s)		\
	{						\
		.rate	= (_rate),			\
		.mdiv	= (_m),				\
		.pdiv	= (_p),				\
		.sdiv	= (_s),				\
	}

#define	PLL_36XX_RATE(_fin, _rate, _m, _p, _s, _k)	\
	{						\
		.rate	= (_rate),			\
		.mdiv	= (_m),				\
		.pdiv	= (_p),				\
		.sdiv	= (_s),				\
		.kdiv	= (_k),				\
	}

struct samsung_fixed_rate_clock {
	unsigned int		id;
	const char		*name;
	const char		*parent_name;
	unsigned long		flags;
	unsigned long		fixed_rate;
};

#define	FRATE(_id, cname, pname, f, frate)		\
	{						\
		.id		= _id,			\
		.name		= cname,		\
		.parent_name	= pname,		\
		.flags		= f,			\
		.fixed_rate	= frate,		\
	}

struct samsung_fixed_factor_clock {
	unsigned int		id;
	const char		*name;
	const char		*parent_name;
	unsigned long		mult;
	unsigned long		div;
	unsigned long		flags;
};

#define	FFACTOR(_id, cname, pname, m, d, f)		\
	{						\
		.id		= _id,			\
		.name		= cname,		\
		.parent_name	= pname,		\
		.mult		= m,			\
		.div		= d,			\
		.flags		= f,			\
	}

struct samsung_mux_clock {
	unsigned int		id;
	const char		*name;
	const char *const	*parent_names;
	uint8_t			num_parents;
	unsigned long		flags;
	unsigned long		offset;
	uint8_t			shift;
	uint8_t			width;
	uint8_t			mux_flags;
};

#define	__MUX(_id, cname, pnames, o, s, w, f, mf)	\
	{						\
		.id		= _id,			\
		.name		= cname,		\
		.parent_names	= pnames,		\
		.num_parents	= nitems(pnames),	\
		.flags		= f,			\
		.offset		= o,			\
		.shift		= s,			\
		.width		= w,			\
		.mux_flags	= mf,			\
	}

#define	MUX(_id, cname, pnames, o, s, w)			\
	__MUX(_id, cname, pnames, o, s, w, CLK_SET_RATE_NO_REPARENT, 0)
#define	MUX_F(_id, cname, pnames, o, s, w, f, mf)		\
	__MUX(_id, cname, pnames, o, s, w, (f) | CLK_SET_RATE_NO_REPARENT, mf)
#define	nMUX(_id, cname, pnames, o, s, w)			\
	__MUX(_id, cname, pnames, o, s, w, 0, 0)
#define	nMUX_F(_id, cname, pnames, o, s, w, f, mf)		\
	__MUX(_id, cname, pnames, o, s, w, f, mf)

struct samsung_div_clock {
	unsigned int		id;
	const char		*name;
	const char		*parent_name;
	unsigned long		flags;
	unsigned long		offset;
	uint8_t			shift;
	uint8_t			width;
	uint8_t			div_flags;
	const void		*table;
};

#define	__DIV(_id, cname, pname, o, s, w, f, df, t)	\
	{						\
		.id		= _id,			\
		.name		= cname,		\
		.parent_name	= pname,		\
		.flags		= f,			\
		.offset		= o,			\
		.shift		= s,			\
		.width		= w,			\
		.div_flags	= df,			\
		.table		= t,			\
	}

#define	DIV(_id, cname, pname, o, s, w)				\
	__DIV(_id, cname, pname, o, s, w, 0, 0, NULL)
#define	DIV_F(_id, cname, pname, o, s, w, f, df)			\
	__DIV(_id, cname, pname, o, s, w, f, df, NULL)
#define	DIV_T(_id, cname, pname, o, s, w, t)			\
	__DIV(_id, cname, pname, o, s, w, 0, 0, t)

struct samsung_gate_clock {
	unsigned int		id;
	const char		*name;
	const char		*parent_name;
	unsigned long		flags;
	unsigned long		offset;
	uint8_t			bit_idx;
	uint8_t			gate_flags;
};

#define	__GATE(_id, cname, pname, o, b, f, gf)			\
	{						\
		.id		= _id,			\
		.name		= cname,		\
		.parent_name	= pname,		\
		.flags		= f,			\
		.offset		= o,			\
		.bit_idx	= b,			\
		.gate_flags	= gf,			\
	}

#define	GATE(_id, cname, pname, o, b, f, gf)			\
	__GATE(_id, cname, pname, o, b, f, gf)

#define	PNAME(x)	static const char *const x[]

struct samsung_pll_clock {
	unsigned int		id;
	const char		*name;
	const char		*parent_name;
	unsigned long		flags;
	int			con_offset;
	int			lock_offset;
	enum samsung_pll_type	type;
	const struct samsung_pll_rate_table *rate_table;
};

#define	__PLL(_typ, _id, _name, _pname, _flags, _lock, _con, _rtable)	\
	{								\
		.id		= _id,					\
		.type		= _typ,					\
		.name		= _name,				\
		.parent_name	= _pname,				\
		.flags		= _flags,				\
		.con_offset	= _con,					\
		.lock_offset	= _lock,				\
		.rate_table	= _rtable,				\
	}

#define	PLL(_typ, _id, _name, _pname, _lock, _con, _rtable)		\
	__PLL(_typ, _id, _name, _pname, CLK_GET_RATE_NOCACHE, _lock,	\
	      _con, _rtable)

/*
 * Description of one CMU domain: the clock tables plus the register-init
 * metadata.  A single instance is referenced from the FDT match data of the
 * corresponding clock-controller compatible.
 */
struct samsung_cmu_info {
	const struct samsung_pll_clock *pll_clks;
	unsigned int nr_pll_clks;
	const struct samsung_mux_clock *mux_clks;
	unsigned int nr_mux_clks;
	const struct samsung_div_clock *div_clks;
	unsigned int nr_div_clks;
	const struct samsung_gate_clock *gate_clks;
	unsigned int nr_gate_clks;
	const struct samsung_fixed_rate_clock *fixed_clks;
	unsigned int nr_fixed_clks;
	const struct samsung_fixed_factor_clock *fixed_factor_clks;
	unsigned int nr_fixed_factor_clks;
	unsigned int nr_clk_ids;

	const unsigned long *clk_regs;
	unsigned int nr_clk_regs;

	const char *clk_name;

	const unsigned long *sysreg_clk_regs;
	unsigned int nr_sysreg_clk_regs;

	bool manual_plls;
	bool auto_clock_gate;
	uint32_t gate_dbg_offset;
	uint32_t option_offset;
	uint32_t drcg_offset;
	uint32_t memclk_offset;
};

/*
 * One FDT input clock of a CMU: the local "clock-names" token and the global
 * clk(9) name of the provider it resolves to.  The leaf-CMU tables refer to
 * their inputs by these local tokens ("bus", "dsim", ...); the framework
 * rewrites such parent references to the resolved global name, mirroring how
 * Linux's clock framework resolves a parent name against a node's clock-names.
 */
struct exynos_clk_input {
	char		*name;
	const char	*global;
};

/* Per-CMU device softc, shared by the framework and the SoC driver. */
struct exynos_clk_softc {
	device_t			dev;
	struct resource			*res;
	struct mtx			mtx;
	struct clkdom			*clkdom;
	const struct samsung_cmu_info	*cmu;
	struct exynos_clk_input		*inputs;
	int				ninputs;
};

DECLARE_CLASS(exynos_cmu_driver);

int	exynos_cmu_attach(device_t dev);

/* Implemented in exynos_clk_pll.c. */
void	exynos_clk_register_plls(struct exynos_clk_softc *sc,
	    const struct samsung_pll_clock *list, unsigned int n);

#endif /* _ARM64_EXYNOS_CLK_H_ */
