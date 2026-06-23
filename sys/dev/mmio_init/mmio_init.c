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
 * Early one-shot MMIO initializer.
 *
 * Walks the device tree for "linux,mmio-init-helper" nodes and writes the
 * node's "value" to its register, optionally as a masked read-modify-write
 * when a "mask" property is present.  It implements the same device tree
 * binding as the Linux "linux,mmio-init-helper" driver.
 *
 * Unlike a normal driver, mmio_init_early() is called directly from the
 * machine-dependent boot path before cninit(), i.e. before the console and
 * the simple-framebuffer are brought up.  This guarantees display state the
 * bootloader left running -- for example the Exynos/DECON autorefresh bit --
 * is set before anything else touches the framebuffer, with no visible gap.
 *
 *	mmio-init@19470030 {
 *		compatible = "linux,mmio-init-helper";
 *		reg = <0x19470030 0x4>;
 *		value = <0x3061>;
 *	};
 */

#include <sys/param.h>
#include <sys/systm.h>

#include <vm/vm.h>
#include <vm/pmap.h>

#include <machine/bus.h>

#include <dev/ofw/openfirm.h>
#include <dev/ofw/ofw_bus.h>
#include <dev/ofw/ofw_bus_subr.h>
#include <dev/ofw/ofw_subr.h>

#include <dev/mmio_init/mmio_init.h>

#define	MMIO_INIT_COMPAT	"linux,mmio-init-helper"

static void
mmio_init_node(phandle_t node)
{
	bus_addr_t pa;
	bus_size_t size;
	uint32_t mask, reg, value;
	volatile uint32_t *va;

	if (OF_getencprop(node, "value", &value, sizeof(value)) <= 0)
		return;
	if (ofw_reg_to_paddr(node, 0, &pa, &size, NULL) != 0)
		return;

	va = pmap_mapdev(pa, size);
	if (va == NULL)
		return;

	if (OF_getencprop(node, "mask", &mask, sizeof(mask)) > 0) {
		reg = *va;
		reg = (reg & ~mask) | (value & mask);
		*va = reg;
	} else {
		*va = value;
	}

	pmap_unmapdev(__DEVOLATILE(void *, va), size);
}

static void
mmio_init_walk(phandle_t node)
{
	for (; node != 0; node = OF_peer(node)) {
		if (ofw_bus_node_is_compatible(node, MMIO_INIT_COMPAT))
			mmio_init_node(node);
		mmio_init_walk(OF_child(node));
	}
}

void
mmio_init_early(void)
{
	phandle_t root;

	root = OF_finddevice("/");
	if (root == -1)
		return;
	mmio_init_walk(OF_child(root));
}
