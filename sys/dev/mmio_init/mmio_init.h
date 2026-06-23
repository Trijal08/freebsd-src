/*-
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Copyright (c) 2025 Trijal Saha <trijalsaha2012@gmail.com>
 */

#ifndef _DEV_MMIO_INIT_MMIO_INIT_H_
#define	_DEV_MMIO_INIT_MMIO_INIT_H_

/*
 * Apply all device-tree "linux,mmio-init-helper" register writes.  Called
 * from the machine-dependent boot path before cninit().
 */
void	mmio_init_early(void);

#endif /* _DEV_MMIO_INIT_MMIO_INIT_H_ */
