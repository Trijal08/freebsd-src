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
 * Samsung "Exynos" pin controller, as used by the Google Tensor (gs101 /
 * zuma / zumapro) SoCs.
 *
 * Modelled on Linux's drivers/pinctrl/samsung: a pin controller is described
 * as a set of pin banks, each a contiguous group of pins with CON
 * (function/mux), DAT (GPIO value), PUD (pull) and DRV (drive strength)
 * register fields.  The bank tables are carried over from Linux's
 * pinctrl-exynos-arm64.c essentially verbatim and the driver exposes them as
 * a flat gpio(4) provider and an fdt_pinctrl(9) backend.
 *
 * Implemented: pin mux (CON), pull (PUD), drive strength (DRV) and GPIO
 * input/output/value (DAT).  External interrupts (EINT/wakeup) and power
 * management are not implemented yet.
 */

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/bus.h>
#include <sys/gpio.h>
#include <sys/kernel.h>
#include <sys/lock.h>
#include <sys/malloc.h>
#include <sys/module.h>
#include <sys/mutex.h>
#include <sys/proc.h>
#include <sys/rman.h>

#include <sys/intr.h>

#include <machine/bus.h>
#include <machine/intr.h>
#include <machine/resource.h>

#include <dev/gpio/gpiobusvar.h>
#include <dev/ofw/ofw_bus.h>
#include <dev/ofw/ofw_bus_subr.h>
#include <dev/fdt/fdt_pinctrl.h>

#include <dev/clk/clk.h>

#include "gpio_if.h"
#include "pic_if.h"

/* Pin-configuration register classes (index into the bank type arrays). */
enum exynos_pincfg_type {
	PINCFG_TYPE_FUNC,	/* function / mux select */
	PINCFG_TYPE_DAT,	/* GPIO data value */
	PINCFG_TYPE_PUD,	/* pull up/down */
	PINCFG_TYPE_DRV,	/* drive strength */
	PINCFG_TYPE_CON_PDN,	/* function in power-down mode */
	PINCFG_TYPE_PUD_PDN,	/* pull in power-down mode */
	PINCFG_TYPE_NUM
};

/* Samsung pull and function field values (from the DT binding). */
#define	EXYNOS_PIN_PULL_NONE	0
#define	EXYNOS_PIN_PULL_DOWN	1
#define	EXYNOS_PIN_PULL_UP	3
#define	EXYNOS_PIN_FUNC_INPUT	0
#define	EXYNOS_PIN_FUNC_OUTPUT	1
#define	EXYNOS_PIN_FUNC_EINT	0xf

/*
 * Wakeup (ALIVE) external-interrupt registers.  Per bank the control, mask and
 * pending registers live at these bases plus the bank's eint_offset; within a
 * register a pin occupies a 4-bit trigger field (CON) or a single bit
 * (MASK/PEND).  These are the EXYNOS7/GS101 offsets.
 */
#define	EXYNOS_WKUP_ECON_OFFSET		0x700
#define	EXYNOS_WKUP_EMASK_OFFSET	0x900
#define	EXYNOS_WKUP_EPEND_OFFSET	0xa00
#define	EXYNOS_EINT_CON_LEN		4

/*
 * Per-pin EINT filter control.  gs101 gates the interrupt through this filter,
 * so it must be enabled (FLTCON_EN) for the pin to deliver.  The field is 8
 * bits per pin, four pins to a 32-bit register, based at the bank's
 * eint_fltcon_offset.
 */
#define	EXYNOS_EFLTCON_OFFSET		0x800
#define	EXYNOS_FLTCON_EN		(1u << 7)
#define	EXYNOS_FLTCON_ANALOG		0
#define	EXYNOS_FLTCON_LEN		8

/* EINT trigger encodings written to the CON field. */
#define	EXYNOS_EINT_LEVEL_LOW		0
#define	EXYNOS_EINT_LEVEL_HIGH		1
#define	EXYNOS_EINT_EDGE_FALLING	2
#define	EXYNOS_EINT_EDGE_RISING		3
#define	EXYNOS_EINT_EDGE_BOTH		4

/*
 * Register layout of a class of pin bank: the per-pin field width (bits) and
 * the register offset of each configuration field.  GS101 widens the pull and
 * drive-strength fields to 4 bits per pin (vs. 2 on older Exynos).
 */
struct exynos_pin_bank_type {
	uint8_t	fld_width[PINCFG_TYPE_NUM];
	uint8_t	reg_offset[PINCFG_TYPE_NUM];
};

static const struct exynos_pin_bank_type exynos850_bank_type_off = {
	.fld_width = { 4, 1, 4, 4, 2, 4, },
	.reg_offset = { 0x00, 0x04, 0x08, 0x0c, 0x10, 0x14, },
};

static const struct exynos_pin_bank_type exynos850_bank_type_alive = {
	.fld_width = { 4, 1, 4, 4, },
	.reg_offset = { 0x00, 0x04, 0x08, 0x0c, },
};

/*
 * One pin bank.  The data tables initialise everything except @pin_base and
 * @node, which are filled in at attach (the controller's per-pin base in the
 * flat GPIO space, and the OF node of the bank's gpio-controller subnode used
 * to resolve consumer references).
 */
struct exynos_pin_bank {
	const struct exynos_pin_bank_type *type;
	uint32_t	pctl_offset;
	uint8_t		nr_pins;
	const char	*name;
	bool		wkup;			/* ALIVE wakeup-EINT bank */
	uint32_t	eint_offset;
	uint32_t	eint_fltcon_offset;	/* unused: EINT filter */
	uint32_t	pin_base;		/* runtime */
	phandle_t	node;			/* runtime */
	int		eint_rid_base;		/* runtime: parent IRQ rids */
};

#define	EXYNOS_PIN_BANK(type_, wkup_, pins_, reg_, id_, offs_, fltcon_)	\
	{								\
		.type			= (type_),			\
		.wkup			= (wkup_),			\
		.pctl_offset		= (reg_),			\
		.nr_pins		= (pins_),			\
		.name			= (id_),			\
		.eint_offset		= (offs_),			\
		.eint_fltcon_offset	= (fltcon_),			\
	}

#define	GS101_PIN_BANK_EINTG(pins, reg, id, offs, fltcon)		\
	EXYNOS_PIN_BANK(&exynos850_bank_type_off, false, pins, reg, id,	\
	    offs, fltcon)
#define	GS101_PIN_BANK_EINTW(pins, reg, id, offs, fltcon)		\
	EXYNOS_PIN_BANK(&exynos850_bank_type_alive, true, pins, reg, id,	\
	    offs, fltcon)

/*
 * One pin-controller instance: its register base (used to match the FDT node)
 * and its set of banks.
 */
struct exynos_pin_ctrl {
	bus_addr_t			base;
	const struct exynos_pin_bank	*banks;
	unsigned int			nr_banks;
};

/*
 * One external-interrupt source: a flat-space pin that has been claimed as an
 * interrupt.  On the ALIVE controllers each such pin is delivered by its own
 * dedicated GIC SPI (listed in the bank's "interrupts"), so the source chains
 * to that parent interrupt set up in pic_setup_intr.
 */
struct exynos_eint_isrc {
	struct intr_irqsrc		isrc;
	struct exynos_pinctrl_softc	*sc;
	struct exynos_pin_bank		*bank;
	uint32_t			pin;	/* pin within bank */
	struct resource			*pres;	/* parent GIC interrupt */
	void				*pcookie;
};

struct exynos_pinctrl_softc {
	device_t			dev;
	device_t			busdev;
	struct resource			*res;
	struct mtx			mtx;
	struct exynos_pin_bank		*banks;	/* runtime copy */
	int				nbanks;
	int				npins;
	bool				has_wkup;
	struct mtx			eint_mtx;	/* EINT register access */
	struct exynos_eint_isrc		*isrcs;	/* [npins], wkup pins only */
};

#define	PINCTRL_LOCK(sc)	mtx_lock(&(sc)->mtx)
#define	PINCTRL_UNLOCK(sc)	mtx_unlock(&(sc)->mtx)

/* ---- zuma / zumapro pin banks (verbatim from Linux) --------------------- */

/* pin banks of zuma pin-controller (ALIVE) */
static const struct exynos_pin_bank zuma_pin_alive[] = {
	GS101_PIN_BANK_EINTW(4, 0x0, "gpa0", 0x00, 0x00),
	GS101_PIN_BANK_EINTW(6, 0x20, "gpa1", 0x04, 0x04),
	GS101_PIN_BANK_EINTW(4, 0x40, "gpa2", 0x08, 0x0c),
	GS101_PIN_BANK_EINTW(4, 0x60, "gpa3", 0x0c, 0x10),
	GS101_PIN_BANK_EINTW(2, 0x80, "gpa4", 0x10, 0x14),
	GS101_PIN_BANK_EINTW(6, 0xa0, "gpa6", 0x14, 0x18),
	GS101_PIN_BANK_EINTW(8, 0xc0, "gpa7", 0x18, 0x20),
	GS101_PIN_BANK_EINTW(4, 0xe0, "gpa8", 0x1c, 0x28),
	GS101_PIN_BANK_EINTW(7, 0x100, "gpa9", 0x20, 0x2c),
	GS101_PIN_BANK_EINTW(5, 0x120, "gpa10", 0x24, 0x34),
};

/* pin banks of zuma pin-controller (CUSTOM_ALIVE) */
static const struct exynos_pin_bank zuma_pin_custom[] = {
	GS101_PIN_BANK_EINTW(1, 0x0, "gpn0", 0x00, 0x00),
	GS101_PIN_BANK_EINTW(1, 0x20, "gpn1", 0x04, 0x04),
	GS101_PIN_BANK_EINTW(1, 0x40, "gpn2", 0x08, 0x08),
	GS101_PIN_BANK_EINTW(1, 0x60, "gpn3", 0x0c, 0x0c),
	GS101_PIN_BANK_EINTW(1, 0x80, "gpn4", 0x10, 0x10),
	GS101_PIN_BANK_EINTW(1, 0xa0, "gpn5", 0x14, 0x14),
	GS101_PIN_BANK_EINTW(1, 0xc0, "gpn6", 0x18, 0x18),
	GS101_PIN_BANK_EINTW(1, 0xe0, "gpn7", 0x1c, 0x1c),
	GS101_PIN_BANK_EINTW(1, 0x100, "gpn8", 0x20, 0x20),
	GS101_PIN_BANK_EINTW(1, 0x120, "gpn9", 0x24, 0x24),
};

/* pin banks of zuma pin-controller (FAR_ALIVE) */
static const struct exynos_pin_bank zuma_pin_far[] = {
	GS101_PIN_BANK_EINTW(8, 0x0, "gpa5", 0x00, 0x00),
};

/* pin banks of zuma pin-controller (GSACORE0) */
static const struct exynos_pin_bank zuma_pin_gsacore0[] = {
	GS101_PIN_BANK_EINTG(2, 0x0, "gps0", 0x00, 0x00),
};

/* pin banks of zuma pin-controller (GSACORE1) */
static const struct exynos_pin_bank zuma_pin_gsacore1[] = {
	GS101_PIN_BANK_EINTG(4, 0x0, "gps1", 0x00, 0x00),
};

/* pin banks of zuma pin-controller (GSACORE2) */
static const struct exynos_pin_bank zuma_pin_gsacore2[] = {
	GS101_PIN_BANK_EINTG(4, 0x0, "gps2", 0x00, 0x00),
};

/* pin banks of zuma pin-controller (GSACORE3) */
static const struct exynos_pin_bank zuma_pin_gsacore3[] = {
	GS101_PIN_BANK_EINTG(3, 0x0, "gps3", 0x00, 0x00),
};

/* pin banks of zuma pin-controller (GSACTRL) */
static const struct exynos_pin_bank zuma_pin_gsactrl[] = {
	GS101_PIN_BANK_EINTW(4, 0x0, "gps4", 0x00, 0x00),
};

/* pin banks of zuma pin-controller (HSI1) */
static const struct exynos_pin_bank zuma_pin_hsi1[] = {
	GS101_PIN_BANK_EINTG(4, 0x0, "gph0", 0x00, 0x00),
	GS101_PIN_BANK_EINTG(8, 0x20, "gph1", 0x04, 0x04),
	GS101_PIN_BANK_EINTG(4, 0x40, "gph2", 0x08, 0x0c),
};

/* pin banks of zuma pin-controller (HSI2) */
static const struct exynos_pin_bank zuma_pin_hsi2[] = {
	GS101_PIN_BANK_EINTG(6, 0x0, "gph3", 0x00, 0x00),
	GS101_PIN_BANK_EINTG(7, 0x20, "gph4", 0x04, 0x08),
};

/* pin banks of zuma pin-controller (HSI2UFS) */
static const struct exynos_pin_bank zuma_pin_hsi2ufs[] = {
	GS101_PIN_BANK_EINTG(2, 0x0, "gph5", 0x00, 0x00),
};

/* pin banks of zuma pin-controller (PERIC0) */
static const struct exynos_pin_bank zuma_pin_peric0[] = {
	GS101_PIN_BANK_EINTG(5, 0x0, "gpp0", 0x00, 0x00),
	GS101_PIN_BANK_EINTG(4, 0x20, "gpp1", 0x04, 0x08),
	GS101_PIN_BANK_EINTG(4, 0x40, "gpp2", 0x08, 0x0c),
	GS101_PIN_BANK_EINTG(2, 0x60, "gpp3", 0x0c, 0x10),
	GS101_PIN_BANK_EINTG(4, 0x80, "gpp4", 0x10, 0x14),
	GS101_PIN_BANK_EINTG(2, 0xa0, "gpp5", 0x14, 0x18),
	GS101_PIN_BANK_EINTG(4, 0xc0, "gpp6", 0x18, 0x1c),
	GS101_PIN_BANK_EINTG(2, 0xe0, "gpp7", 0x1c, 0x20),
	GS101_PIN_BANK_EINTG(4, 0x100, "gpp8", 0x20, 0x24),
	GS101_PIN_BANK_EINTG(2, 0x120, "gpp9", 0x24, 0x28),
	GS101_PIN_BANK_EINTG(4, 0x140, "gpp10", 0x28, 0x2c),
	GS101_PIN_BANK_EINTG(2, 0x160, "gpp11", 0x2c, 0x30),
	GS101_PIN_BANK_EINTG(4, 0x180, "gpp12", 0x30, 0x34),
	GS101_PIN_BANK_EINTG(2, 0x1a0, "gpp13", 0x34, 0x38),
	GS101_PIN_BANK_EINTG(2, 0x1c0, "gpp14", 0x38, 0x3c),
	GS101_PIN_BANK_EINTG(2, 0x1e0, "gpp15", 0x3c, 0x40),
	GS101_PIN_BANK_EINTG(2, 0x200, "gpp17", 0x40, 0x44),
	GS101_PIN_BANK_EINTG(4, 0x220, "gpp16", 0x44, 0x48),
};

/* pin banks of zuma pin-controller (PERIC1) */
static const struct exynos_pin_bank zuma_pin_peric1[] = {
	GS101_PIN_BANK_EINTG(8, 0x0, "gpp19", 0x00, 0x00),
	GS101_PIN_BANK_EINTG(4, 0x20, "gpp20", 0x04, 0x08),
	GS101_PIN_BANK_EINTG(8, 0x40, "gpp21", 0x08, 0x0c),
	GS101_PIN_BANK_EINTG(4, 0x60, "gpp24", 0x0c, 0x14),
	GS101_PIN_BANK_EINTG(4, 0x80, "gpp22", 0x10, 0x18),
	GS101_PIN_BANK_EINTG(4, 0xa0, "gpp23", 0x14, 0x1c),
};

#define	BANKS(b)	(b), nitems(b)

static const struct exynos_pin_ctrl zuma_pin_ctrl[] = {
	{ 0x154d0000, BANKS(zuma_pin_alive) },		/* ALIVE */
	{ 0x15060000, BANKS(zuma_pin_custom) },		/* CUSTOM_ALIVE */
	{ 0x154e0000, BANKS(zuma_pin_far) },		/* FAR_ALIVE */
	{ 0x16280000, BANKS(zuma_pin_gsacore0) },	/* GSACORE0 */
	{ 0x16290000, BANKS(zuma_pin_gsacore1) },	/* GSACORE1 */
	{ 0x162a0000, BANKS(zuma_pin_gsacore2) },	/* GSACORE2 */
	{ 0x162b0000, BANKS(zuma_pin_gsacore3) },	/* GSACORE3 */
	{ 0x16140000, BANKS(zuma_pin_gsactrl) },	/* GSACTRL */
	{ 0x12040000, BANKS(zuma_pin_hsi1) },		/* HSI1 */
	{ 0x13040000, BANKS(zuma_pin_hsi2) },		/* HSI2 */
	{ 0x13060000, BANKS(zuma_pin_hsi2ufs) },	/* HSI2UFS */
	{ 0x10840000, BANKS(zuma_pin_peric0) },		/* PERIC0 */
	{ 0x10c40000, BANKS(zuma_pin_peric1) },		/* PERIC1 */
};

static struct ofw_compat_data compat_data[] = {
	{ "google,zuma-pinctrl",	1 },
	{ "google,zumapro-pinctrl",	1 },
	{ NULL,				0 }
};

/* ---- register field access ---------------------------------------------- */

static uint32_t
exynos_pin_getcfg(struct exynos_pinctrl_softc *sc, struct exynos_pin_bank *bank,
    uint32_t pin, enum exynos_pincfg_type cfg)
{
	uint32_t off, width;

	width = bank->type->fld_width[cfg];
	off = bank->pctl_offset + bank->type->reg_offset[cfg];
	return ((bus_read_4(sc->res, off) >> (pin * width)) &
	    ((1u << width) - 1));
}

static void
exynos_pin_setcfg(struct exynos_pinctrl_softc *sc, struct exynos_pin_bank *bank,
    uint32_t pin, enum exynos_pincfg_type cfg, uint32_t val)
{
	uint32_t off, width, shift, mask, reg;

	width = bank->type->fld_width[cfg];
	off = bank->pctl_offset + bank->type->reg_offset[cfg];
	shift = pin * width;
	mask = ((1u << width) - 1) << shift;

	reg = bus_read_4(sc->res, off);
	reg = (reg & ~mask) | ((val << shift) & mask);
	bus_write_4(sc->res, off, reg);
}

static struct exynos_pin_bank *
exynos_pin_to_bank(struct exynos_pinctrl_softc *sc, uint32_t pin,
    uint32_t *local)
{
	struct exynos_pin_bank *bank;
	int i;

	for (i = 0; i < sc->nbanks; i++) {
		bank = &sc->banks[i];
		if (pin >= bank->pin_base &&
		    pin < bank->pin_base + bank->nr_pins) {
			*local = pin - bank->pin_base;
			return (bank);
		}
	}
	return (NULL);
}

static struct exynos_pin_bank *
exynos_bank_by_name(struct exynos_pinctrl_softc *sc, const char *name)
{
	int i;

	for (i = 0; i < sc->nbanks; i++) {
		if (strcmp(sc->banks[i].name, name) == 0)
			return (&sc->banks[i]);
	}
	return (NULL);
}

/* ---- gpio interface ----------------------------------------------------- */

static device_t
exynos_pinctrl_get_bus(device_t dev)
{
	struct exynos_pinctrl_softc *sc;

	sc = device_get_softc(dev);
	return (sc->busdev);
}

static int
exynos_pinctrl_pin_max(device_t dev, int *maxpin)
{
	struct exynos_pinctrl_softc *sc;

	sc = device_get_softc(dev);
	*maxpin = sc->npins - 1;
	return (0);
}

static int
exynos_pinctrl_pin_getname(device_t dev, uint32_t pin, char *name)
{
	struct exynos_pinctrl_softc *sc;
	struct exynos_pin_bank *bank;
	uint32_t local;

	sc = device_get_softc(dev);
	bank = exynos_pin_to_bank(sc, pin, &local);
	if (bank == NULL)
		return (EINVAL);

	snprintf(name, GPIOMAXNAME, "%s-%u", bank->name, local);
	return (0);
}

static int
exynos_pinctrl_pin_getcaps(device_t dev, uint32_t pin, uint32_t *caps)
{

	*caps = GPIO_PIN_INPUT | GPIO_PIN_OUTPUT |
	    GPIO_PIN_PULLUP | GPIO_PIN_PULLDOWN;
	return (0);
}

static int
exynos_pinctrl_pin_getflags(device_t dev, uint32_t pin, uint32_t *flags)
{
	struct exynos_pinctrl_softc *sc;
	struct exynos_pin_bank *bank;
	uint32_t local, func, pud;

	sc = device_get_softc(dev);
	bank = exynos_pin_to_bank(sc, pin, &local);
	if (bank == NULL)
		return (EINVAL);

	*flags = 0;
	PINCTRL_LOCK(sc);
	func = exynos_pin_getcfg(sc, bank, local, PINCFG_TYPE_FUNC);
	pud = exynos_pin_getcfg(sc, bank, local, PINCFG_TYPE_PUD);
	PINCTRL_UNLOCK(sc);

	if (func == EXYNOS_PIN_FUNC_INPUT)
		*flags |= GPIO_PIN_INPUT;
	else if (func == EXYNOS_PIN_FUNC_OUTPUT)
		*flags |= GPIO_PIN_OUTPUT;

	if (pud == EXYNOS_PIN_PULL_UP)
		*flags |= GPIO_PIN_PULLUP;
	else if (pud == EXYNOS_PIN_PULL_DOWN)
		*flags |= GPIO_PIN_PULLDOWN;

	return (0);
}

static int
exynos_pinctrl_pin_setflags(device_t dev, uint32_t pin, uint32_t flags)
{
	struct exynos_pinctrl_softc *sc;
	struct exynos_pin_bank *bank;
	uint32_t local, pud;

	sc = device_get_softc(dev);
	bank = exynos_pin_to_bank(sc, pin, &local);
	if (bank == NULL)
		return (EINVAL);

	PINCTRL_LOCK(sc);
	if ((flags & GPIO_PIN_INPUT) != 0)
		exynos_pin_setcfg(sc, bank, local, PINCFG_TYPE_FUNC,
		    EXYNOS_PIN_FUNC_INPUT);
	else if ((flags & GPIO_PIN_OUTPUT) != 0)
		exynos_pin_setcfg(sc, bank, local, PINCFG_TYPE_FUNC,
		    EXYNOS_PIN_FUNC_OUTPUT);

	if ((flags & (GPIO_PIN_PULLUP | GPIO_PIN_PULLDOWN)) != 0) {
		pud = (flags & GPIO_PIN_PULLUP) != 0 ?
		    EXYNOS_PIN_PULL_UP : EXYNOS_PIN_PULL_DOWN;
		exynos_pin_setcfg(sc, bank, local, PINCFG_TYPE_PUD, pud);
	}
	PINCTRL_UNLOCK(sc);

	return (0);
}

static int
exynos_pinctrl_pin_get(device_t dev, uint32_t pin, unsigned int *val)
{
	struct exynos_pinctrl_softc *sc;
	struct exynos_pin_bank *bank;
	uint32_t local;

	sc = device_get_softc(dev);
	bank = exynos_pin_to_bank(sc, pin, &local);
	if (bank == NULL)
		return (EINVAL);

	PINCTRL_LOCK(sc);
	*val = exynos_pin_getcfg(sc, bank, local, PINCFG_TYPE_DAT);
	PINCTRL_UNLOCK(sc);
	return (0);
}

static int
exynos_pinctrl_pin_set(device_t dev, uint32_t pin, unsigned int value)
{
	struct exynos_pinctrl_softc *sc;
	struct exynos_pin_bank *bank;
	uint32_t local;

	sc = device_get_softc(dev);
	bank = exynos_pin_to_bank(sc, pin, &local);
	if (bank == NULL)
		return (EINVAL);

	PINCTRL_LOCK(sc);
	exynos_pin_setcfg(sc, bank, local, PINCFG_TYPE_DAT, value ? 1 : 0);
	PINCTRL_UNLOCK(sc);
	return (0);
}

static int
exynos_pinctrl_pin_toggle(device_t dev, uint32_t pin)
{
	struct exynos_pinctrl_softc *sc;
	struct exynos_pin_bank *bank;
	uint32_t local, val;

	sc = device_get_softc(dev);
	bank = exynos_pin_to_bank(sc, pin, &local);
	if (bank == NULL)
		return (EINVAL);

	PINCTRL_LOCK(sc);
	val = exynos_pin_getcfg(sc, bank, local, PINCFG_TYPE_DAT);
	exynos_pin_setcfg(sc, bank, local, PINCFG_TYPE_DAT, val ^ 1);
	PINCTRL_UNLOCK(sc);
	return (0);
}

/*
 * Translate a consumer's gpio specifier.  @gparent is the OF node of the
 * referenced gpio-controller -- i.e. the bank subnode -- so the pin index is
 * relative to that bank and is mapped into the flat space here.
 */
static int
exynos_pinctrl_map_gpios(device_t dev, phandle_t pdev, phandle_t gparent,
    int gcells, pcell_t *gpios, uint32_t *pin, uint32_t *flags)
{
	struct exynos_pinctrl_softc *sc;
	struct exynos_pin_bank *bank;
	int i;

	sc = device_get_softc(dev);
	bank = NULL;
	for (i = 0; i < sc->nbanks; i++) {
		if (sc->banks[i].node == gparent) {
			bank = &sc->banks[i];
			break;
		}
	}
	if (bank == NULL || gpios[0] >= bank->nr_pins)
		return (EINVAL);

	*pin = bank->pin_base + gpios[0];
	*flags = gcells > 1 ? gpios[1] : 0;
	return (0);
}

/* ---- fdt_pinctrl interface ---------------------------------------------- */

static void
exynos_pinctrl_config_group(struct exynos_pinctrl_softc *sc, phandle_t node)
{
	struct exynos_pin_bank *bank;
	const char **pins;
	char bankname[16];
	char *dash;
	uint32_t func, pud, drv;
	bool have_func, have_pud, have_drv;
	int npins, i, pinnum;

	npins = ofw_bus_string_list_to_array(node, "samsung,pins", &pins);
	if (npins <= 0)
		return;

	have_func = OF_getencprop(node, "samsung,pin-function", &func,
	    sizeof(func)) > 0;
	have_pud = OF_getencprop(node, "samsung,pin-pud", &pud,
	    sizeof(pud)) > 0;
	have_drv = OF_getencprop(node, "samsung,pin-drv", &drv,
	    sizeof(drv)) > 0;

	for (i = 0; i < npins; i++) {
		/* "<bank>-<pin>", e.g. "gpp1-2". */
		strlcpy(bankname, pins[i], sizeof(bankname));
		dash = strrchr(bankname, '-');
		if (dash == NULL)
			continue;
		*dash = '\0';
		pinnum = (int)strtoul(dash + 1, NULL, 10);

		/*
		 * fdt_pinctrl_configure_tree() routes every pin group in the
		 * tree to each pin controller, so a group naming pins that
		 * belong to a different controller is not an error here -- it
		 * is simply not ours.  Skip it silently.
		 */
		bank = exynos_bank_by_name(sc, bankname);
		if (bank == NULL || pinnum < 0 || pinnum >= bank->nr_pins)
			continue;

		PINCTRL_LOCK(sc);
		if (have_func)
			exynos_pin_setcfg(sc, bank, pinnum, PINCFG_TYPE_FUNC,
			    func);
		if (have_pud)
			exynos_pin_setcfg(sc, bank, pinnum, PINCFG_TYPE_PUD,
			    pud);
		if (have_drv)
			exynos_pin_setcfg(sc, bank, pinnum, PINCFG_TYPE_DRV,
			    drv);
		PINCTRL_UNLOCK(sc);
	}

	OF_prop_free(pins);
}

static int
exynos_pinctrl_configure(device_t dev, phandle_t cfgxref)
{
	struct exynos_pinctrl_softc *sc;
	phandle_t node, child;

	sc = device_get_softc(dev);
	node = OF_node_from_xref(cfgxref);

	if (OF_hasprop(node, "samsung,pins")) {
		exynos_pinctrl_config_group(sc, node);
	} else {
		/* A container of pin groups: configure each child. */
		for (child = OF_child(node); child != 0; child = OF_peer(child))
			exynos_pinctrl_config_group(sc, child);
	}

	return (0);
}

/* ---- external interrupts (ALIVE wakeup EINT) ---------------------------- */

/* Caller holds eint_mtx. */
static void
exynos_eint_mask(struct exynos_pinctrl_softc *sc, struct exynos_pin_bank *bank,
    uint32_t pin)
{
	uint32_t reg, v;

	reg = EXYNOS_WKUP_EMASK_OFFSET + bank->eint_offset;
	v = bus_read_4(sc->res, reg) | (1u << pin);
	bus_write_4(sc->res, reg, v);
}

static void
exynos_eint_unmask(struct exynos_pinctrl_softc *sc, struct exynos_pin_bank *bank,
    uint32_t pin)
{
	uint32_t reg, v;

	reg = EXYNOS_WKUP_EMASK_OFFSET + bank->eint_offset;
	v = bus_read_4(sc->res, reg) & ~(1u << pin);
	bus_write_4(sc->res, reg, v);
}

static void
exynos_eint_ack(struct exynos_pinctrl_softc *sc, struct exynos_pin_bank *bank,
    uint32_t pin)
{
	uint32_t reg;

	reg = EXYNOS_WKUP_EPEND_OFFSET + bank->eint_offset;
	bus_write_4(sc->res, reg, 1u << pin);
}

static void
exynos_eint_set_trigger(struct exynos_pinctrl_softc *sc,
    struct exynos_pin_bank *bank, uint32_t pin, uint32_t trig)
{
	uint32_t reg, shift, v;

	reg = EXYNOS_WKUP_ECON_OFFSET + bank->eint_offset;
	shift = EXYNOS_EINT_CON_LEN * pin;
	v = bus_read_4(sc->res, reg);
	v &= ~(0xfu << shift);
	v |= (trig & 0xfu) << shift;
	bus_write_4(sc->res, reg, v);
}

/* Enable the per-pin EINT filter; gs101 needs this for the pin to deliver. */
static void
exynos_eint_enable_filter(struct exynos_pinctrl_softc *sc,
    struct exynos_pin_bank *bank, uint32_t pin)
{
	uint32_t reg, shift, v;

	reg = EXYNOS_EFLTCON_OFFSET + bank->eint_fltcon_offset + (pin / 4) * 4;
	shift = (pin % 4) * EXYNOS_FLTCON_LEN;
	v = bus_read_4(sc->res, reg);
	v &= ~(0xffu << shift);
	v |= (EXYNOS_FLTCON_EN | EXYNOS_FLTCON_ANALOG) << shift;
	bus_write_4(sc->res, reg, v);
}

/* Parent (GIC) interrupt handler: ack the EINT latch and dispatch upward. */
static int
exynos_eint_filter(void *arg)
{
	struct exynos_eint_isrc *eint = arg;
	struct exynos_pinctrl_softc *sc = eint->sc;

	mtx_lock_spin(&sc->eint_mtx);
	exynos_eint_ack(sc, eint->bank, eint->pin);
	mtx_unlock_spin(&sc->eint_mtx);

	intr_isrc_dispatch(&eint->isrc, curthread->td_intr_frame);
	return (FILTER_HANDLED);
}

/*
 * Resolve the dedicated GIC interrupt that backs @pin of a wakeup bank, from
 * the bank's "interrupts" property (one specifier per pin).
 */
static u_int
exynos_eint_parent_irq(struct exynos_pinctrl_softc *sc,
    struct exynos_pin_bank *bank, uint32_t pin)
{
	struct intr_map_data_fdt *fdt;
	pcell_t *cells = NULL;
	phandle_t iparent;
	u_int irq;
	int ncells, icells;

	iparent = ofw_bus_find_iparent(bank->node);
	if (iparent == 0)
		return (INTR_IRQ_INVALID);
	if (OF_getencprop(OF_node_from_xref(iparent), "#interrupt-cells",
	    &icells, sizeof(icells)) <= 0)
		return (INTR_IRQ_INVALID);

	ncells = OF_getencprop_alloc_multi(bank->node, "interrupts",
	    sizeof(pcell_t), (void **)&cells);
	if (ncells < (int)((pin + 1) * icells))
		goto fail;

	fdt = (struct intr_map_data_fdt *)intr_alloc_map_data(
	    INTR_MAP_DATA_FDT, sizeof(*fdt) + icells * sizeof(pcell_t),
	    M_WAITOK | M_ZERO);
	fdt->iparent = iparent;
	fdt->ncells = icells;
	memcpy(fdt->cells, &cells[pin * icells], icells * sizeof(pcell_t));
	OF_prop_free(cells);

	irq = intr_map_irq(NULL, iparent, (struct intr_map_data *)fdt);
	return (irq);
fail:
	if (cells != NULL)
		OF_prop_free(cells);
	return (INTR_IRQ_INVALID);
}

static int
exynos_pinctrl_pic_map_intr(device_t dev, struct intr_map_data *data,
    struct intr_irqsrc **isrcp)
{
	struct exynos_pinctrl_softc *sc;
	struct intr_map_data_gpio *gd;
	struct exynos_pin_bank *bank;
	uint32_t local;

	sc = device_get_softc(dev);
	if (data->type != INTR_MAP_DATA_GPIO)
		return (ENOTSUP);
	gd = (struct intr_map_data_gpio *)data;
	if (gd->gpio_pin_num >= (u_int)sc->npins)
		return (EINVAL);
	bank = exynos_pin_to_bank(sc, gd->gpio_pin_num, &local);
	if (bank == NULL || !bank->wkup)
		return (ENOTSUP);

	*isrcp = &sc->isrcs[gd->gpio_pin_num].isrc;
	return (0);
}

static int
exynos_pinctrl_pic_setup_intr(device_t dev, struct intr_irqsrc *isrc,
    struct resource *res, struct intr_map_data *data)
{
	struct exynos_pinctrl_softc *sc;
	struct exynos_eint_isrc *eint;
	struct intr_map_data_gpio *gd;
	struct exynos_pin_bank *bank;
	uint32_t mode, trig, local;

	sc = device_get_softc(dev);
	eint = (struct exynos_eint_isrc *)isrc;

	if (data == NULL || data->type != INTR_MAP_DATA_GPIO)
		return (ENOTSUP);
	gd = (struct intr_map_data_gpio *)data;
	mode = gd->gpio_intr_mode;

	if ((mode & GPIO_INTR_EDGE_BOTH) == GPIO_INTR_EDGE_BOTH)
		trig = EXYNOS_EINT_EDGE_BOTH;
	else if (mode & GPIO_INTR_EDGE_RISING)
		trig = EXYNOS_EINT_EDGE_RISING;
	else if (mode & GPIO_INTR_EDGE_FALLING)
		trig = EXYNOS_EINT_EDGE_FALLING;
	else if (mode & GPIO_INTR_LEVEL_HIGH)
		trig = EXYNOS_EINT_LEVEL_HIGH;
	else if (mode & GPIO_INTR_LEVEL_LOW)
		trig = EXYNOS_EINT_LEVEL_LOW;
	else
		return (ENOTSUP);

	/*
	 * The dedicated parent GIC interrupt was wired up at attach.  Here we
	 * only route the pin to EINT mode and program the trigger; the source
	 * stays masked until pic_enable_intr().
	 */
	bank = exynos_pin_to_bank(sc, eint->bank->pin_base + eint->pin, &local);
	PINCTRL_LOCK(sc);
	exynos_pin_setcfg(sc, bank, local, PINCFG_TYPE_FUNC,
	    EXYNOS_PIN_FUNC_EINT);
	PINCTRL_UNLOCK(sc);

	mtx_lock_spin(&sc->eint_mtx);
	exynos_eint_set_trigger(sc, bank, eint->pin, trig);
	exynos_eint_enable_filter(sc, bank, eint->pin);
	mtx_unlock_spin(&sc->eint_mtx);

	return (0);
}

static int
exynos_pinctrl_pic_teardown_intr(device_t dev, struct intr_irqsrc *isrc,
    struct resource *res, struct intr_map_data *data)
{
	struct exynos_pinctrl_softc *sc;
	struct exynos_eint_isrc *eint;

	sc = device_get_softc(dev);
	eint = (struct exynos_eint_isrc *)isrc;

	/*
	 * Just mask the source; the parent GIC interrupt is owned by attach
	 * (released in detach), not per-consumer.
	 */
	mtx_lock_spin(&sc->eint_mtx);
	exynos_eint_mask(sc, eint->bank, eint->pin);
	mtx_unlock_spin(&sc->eint_mtx);

	return (0);
}

static void
exynos_pinctrl_pic_enable_intr(device_t dev, struct intr_irqsrc *isrc)
{
	struct exynos_pinctrl_softc *sc;
	struct exynos_eint_isrc *eint;

	sc = device_get_softc(dev);
	eint = (struct exynos_eint_isrc *)isrc;
	mtx_lock_spin(&sc->eint_mtx);
	exynos_eint_unmask(sc, eint->bank, eint->pin);
	mtx_unlock_spin(&sc->eint_mtx);
}

static void
exynos_pinctrl_pic_disable_intr(device_t dev, struct intr_irqsrc *isrc)
{
	struct exynos_pinctrl_softc *sc;
	struct exynos_eint_isrc *eint;

	sc = device_get_softc(dev);
	eint = (struct exynos_eint_isrc *)isrc;
	mtx_lock_spin(&sc->eint_mtx);
	exynos_eint_mask(sc, eint->bank, eint->pin);
	mtx_unlock_spin(&sc->eint_mtx);
}

static void
exynos_pinctrl_pic_pre_ithread(device_t dev, struct intr_irqsrc *isrc)
{

	exynos_pinctrl_pic_disable_intr(dev, isrc);
}

static void
exynos_pinctrl_pic_post_ithread(device_t dev, struct intr_irqsrc *isrc)
{

	exynos_pinctrl_pic_enable_intr(dev, isrc);
}

static void
exynos_pinctrl_pic_post_filter(device_t dev, struct intr_irqsrc *isrc)
{
}

/* Register this controller's wakeup banks as a GPIO interrupt controller. */
static int
exynos_eint_setup(struct exynos_pinctrl_softc *sc)
{
	struct exynos_eint_isrc *eint;
	struct exynos_pin_bank *bank;
	const char *devname;
	u_int irq;
	int i, rid;
	uint32_t p;

	sc->has_wkup = false;
	for (i = 0; i < sc->nbanks; i++) {
		if (sc->banks[i].wkup) {
			sc->has_wkup = true;
			break;
		}
	}
	if (!sc->has_wkup)
		return (0);

	mtx_init(&sc->eint_mtx, "exynos eint", NULL, MTX_SPIN);
	sc->isrcs = malloc(sc->npins * sizeof(*sc->isrcs), M_DEVBUF,
	    M_WAITOK | M_ZERO);

	if (intr_pic_register(sc->dev,
	    OF_xref_from_node(ofw_bus_get_node(sc->dev))) == NULL) {
		device_printf(sc->dev, "cannot register interrupt controller\n");
		return (ENXIO);
	}

	/*
	 * Wire up each wakeup pin's dedicated parent GIC interrupt now, while
	 * we are in a sleepable context: PIC_SETUP_INTR later runs with the
	 * isrc table lock held and must not allocate.  The EINT is masked
	 * until a consumer enables it, so the always-set-up parent stays
	 * quiet meanwhile.
	 */
	devname = device_get_nameunit(sc->dev);
	for (i = 0; i < sc->nbanks; i++) {
		bank = &sc->banks[i];
		if (!bank->wkup || bank->node == 0)
			continue;
		for (p = 0; p < bank->nr_pins; p++) {
			eint = &sc->isrcs[bank->pin_base + p];
			eint->sc = sc;
			eint->bank = bank;
			eint->pin = p;
			if (intr_isrc_register(&eint->isrc, sc->dev, 0,
			    "%s,%s-%u", devname, bank->name, p) != 0) {
				device_printf(sc->dev,
				    "cannot register isrc for %s-%u\n",
				    bank->name, p);
				continue;
			}

			irq = exynos_eint_parent_irq(sc, bank, p);
			if (irq == INTR_IRQ_INVALID)
				continue;
			rid = (int)(bank->pin_base + p);
			eint->pres = bus_alloc_resource(sc->dev, SYS_RES_IRQ,
			    &rid, irq, irq, 1, RF_ACTIVE | RF_SHAREABLE);
			if (eint->pres == NULL)
				continue;

			mtx_lock_spin(&sc->eint_mtx);
			exynos_eint_mask(sc, bank, p);
			mtx_unlock_spin(&sc->eint_mtx);

			if (bus_setup_intr(sc->dev, eint->pres,
			    INTR_TYPE_MISC | INTR_MPSAFE, exynos_eint_filter,
			    NULL, eint, &eint->pcookie) != 0) {
				bus_release_resource(sc->dev, SYS_RES_IRQ, rid,
				    eint->pres);
				eint->pres = NULL;
			}
		}
	}
	return (0);
}

/* ---- device interface --------------------------------------------------- */

static phandle_t
exynos_pinctrl_bank_node(phandle_t parent, const char *name)
{
	char buf[32], suffixed[40];
	phandle_t child;

	snprintf(suffixed, sizeof(suffixed), "%s-gpio-bank", name);
	for (child = OF_child(parent); child != 0; child = OF_peer(child)) {
		if (OF_getprop(child, "name", buf, sizeof(buf)) <= 0)
			continue;
		if (strcmp(buf, suffixed) == 0 || strcmp(buf, name) == 0)
			return (child);
	}
	return (0);
}

static int
exynos_pinctrl_probe(device_t dev)
{

	if (!ofw_bus_status_okay(dev))
		return (ENXIO);
	if (ofw_bus_search_compatible(dev, compat_data)->ocd_data == 0)
		return (ENXIO);
	device_set_desc(dev, "Google Tensor pin controller");
	return (BUS_PROBE_DEFAULT);
}

static int
exynos_pinctrl_detach(device_t dev)
{
	struct exynos_pinctrl_softc *sc;

	sc = device_get_softc(dev);
	if (sc->busdev != NULL)
		gpiobus_detach_bus(dev);
	if (sc->isrcs != NULL) {
		int i;

		for (i = 0; i < sc->npins; i++) {
			if (sc->isrcs[i].pres == NULL)
				continue;
			bus_teardown_intr(dev, sc->isrcs[i].pres,
			    sc->isrcs[i].pcookie);
			bus_release_resource(dev, SYS_RES_IRQ, i,
			    sc->isrcs[i].pres);
		}
		free(sc->isrcs, M_DEVBUF);
	}
	if (sc->has_wkup && mtx_initialized(&sc->eint_mtx))
		mtx_destroy(&sc->eint_mtx);
	if (sc->banks != NULL)
		free(sc->banks, M_DEVBUF);
	if (sc->res != NULL)
		bus_release_resource(dev, SYS_RES_MEMORY, 0, sc->res);
	if (mtx_initialized(&sc->mtx))
		mtx_destroy(&sc->mtx);
	return (0);
}

static int
exynos_pinctrl_attach(device_t dev)
{
	struct exynos_pinctrl_softc *sc;
	const struct exynos_pin_ctrl *ctrl;
	struct exynos_pin_bank *bank;
	clk_t clk;
	phandle_t node;
	bus_addr_t base;
	uint32_t pin_base;
	pcell_t clkhandle;
	int i, rid;

	sc = device_get_softc(dev);
	sc->dev = dev;
	node = ofw_bus_get_node(dev);

	rid = 0;
	sc->res = bus_alloc_resource_any(dev, SYS_RES_MEMORY, &rid, RF_ACTIVE);
	if (sc->res == NULL) {
		device_printf(dev, "cannot allocate registers\n");
		return (ENXIO);
	}

	/* Select the controller whose register base matches this node. */
	base = rman_get_start(sc->res);
	ctrl = NULL;
	for (i = 0; i < (int)nitems(zuma_pin_ctrl); i++) {
		if (zuma_pin_ctrl[i].base == base) {
			ctrl = &zuma_pin_ctrl[i];
			break;
		}
	}
	if (ctrl == NULL) {
		device_printf(dev, "no pin data for base %#jx\n",
		    (uintmax_t)base);
		bus_release_resource(dev, SYS_RES_MEMORY, rid, sc->res);
		return (ENXIO);
	}

	mtx_init(&sc->mtx, device_get_nameunit(dev), NULL, MTX_DEF);

	/*
	 * The controller register block is clocked by "pclk".  Some
	 * controllers (gsactrl, hsi1) carry a "clocks = <0>" placeholder
	 * upstream because their CMU is not yet supported; skip those so the
	 * clk parser does not warn about the null provider's missing
	 * #clock-cells.  The bootloader leaves the gate enabled either way.
	 */
	if (OF_getencprop(node, "clocks", &clkhandle, sizeof(clkhandle)) > 0 &&
	    clkhandle != 0 &&
	    clk_get_by_ofw_name(dev, 0, "pclk", &clk) == 0)
		clk_enable(clk);

	/* Build the runtime bank array and the flat pin numbering. */
	sc->nbanks = ctrl->nr_banks;
	sc->banks = malloc(sc->nbanks * sizeof(*sc->banks), M_DEVBUF,
	    M_WAITOK | M_ZERO);

	pin_base = 0;
	for (i = 0; i < sc->nbanks; i++) {
		bank = &sc->banks[i];
		*bank = ctrl->banks[i];
		bank->pin_base = pin_base;
		pin_base += bank->nr_pins;
		bank->node = exynos_pinctrl_bank_node(node, bank->name);
		if (bank->node != 0)
			OF_device_register_xref(OF_xref_from_node(bank->node),
			    dev);
	}
	sc->npins = pin_base;

	fdt_pinctrl_register(dev, "samsung,pins");
	fdt_pinctrl_configure_tree(dev);

	/* ALIVE controllers double as a wakeup-EINT interrupt controller. */
	exynos_eint_setup(sc);

	sc->busdev = gpiobus_add_bus(dev);
	if (sc->busdev == NULL) {
		device_printf(dev, "cannot attach gpiobus\n");
		exynos_pinctrl_detach(dev);
		return (ENXIO);
	}

	bus_attach_children(dev);
	return (0);
}

static phandle_t
exynos_pinctrl_get_node(device_t bus, device_t dev)
{

	return (ofw_bus_get_node(bus));
}

static device_method_t exynos_pinctrl_methods[] = {
	/* Device interface */
	DEVMETHOD(device_probe,		exynos_pinctrl_probe),
	DEVMETHOD(device_attach,	exynos_pinctrl_attach),
	DEVMETHOD(device_detach,	exynos_pinctrl_detach),

	/* GPIO interface */
	DEVMETHOD(gpio_get_bus,		exynos_pinctrl_get_bus),
	DEVMETHOD(gpio_pin_max,		exynos_pinctrl_pin_max),
	DEVMETHOD(gpio_pin_getname,	exynos_pinctrl_pin_getname),
	DEVMETHOD(gpio_pin_getcaps,	exynos_pinctrl_pin_getcaps),
	DEVMETHOD(gpio_pin_getflags,	exynos_pinctrl_pin_getflags),
	DEVMETHOD(gpio_pin_setflags,	exynos_pinctrl_pin_setflags),
	DEVMETHOD(gpio_pin_get,		exynos_pinctrl_pin_get),
	DEVMETHOD(gpio_pin_set,		exynos_pinctrl_pin_set),
	DEVMETHOD(gpio_pin_toggle,	exynos_pinctrl_pin_toggle),
	DEVMETHOD(gpio_map_gpios,	exynos_pinctrl_map_gpios),

	/* ofw_bus interface */
	DEVMETHOD(ofw_bus_get_node,	exynos_pinctrl_get_node),

	/* fdt_pinctrl interface */
	DEVMETHOD(fdt_pinctrl_configure, exynos_pinctrl_configure),

	/* Interrupt controller interface (wakeup EINT) */
	DEVMETHOD(pic_map_intr,		exynos_pinctrl_pic_map_intr),
	DEVMETHOD(pic_setup_intr,	exynos_pinctrl_pic_setup_intr),
	DEVMETHOD(pic_teardown_intr,	exynos_pinctrl_pic_teardown_intr),
	DEVMETHOD(pic_enable_intr,	exynos_pinctrl_pic_enable_intr),
	DEVMETHOD(pic_disable_intr,	exynos_pinctrl_pic_disable_intr),
	DEVMETHOD(pic_pre_ithread,	exynos_pinctrl_pic_pre_ithread),
	DEVMETHOD(pic_post_ithread,	exynos_pinctrl_pic_post_ithread),
	DEVMETHOD(pic_post_filter,	exynos_pinctrl_pic_post_filter),

	DEVMETHOD_END
};

/*
 * The driver is named "gpio" (not after the SoC) so that the gpiobus driver,
 * which attaches to parents in the "gpio" devclass, binds to it -- the same
 * convention apple_pinctrl and rk_gpio follow.  Device instances are thus
 * gpio0, gpio1, ...
 */
static driver_t exynos_pinctrl_driver = {
	"gpio",
	exynos_pinctrl_methods,
	sizeof(struct exynos_pinctrl_softc),
};

EARLY_DRIVER_MODULE(exynos_pinctrl, simplebus, exynos_pinctrl_driver,
    0, 0, BUS_PASS_INTERRUPT + BUS_PASS_ORDER_LATE);

MODULE_VERSION(exynos_pinctrl, 1);
