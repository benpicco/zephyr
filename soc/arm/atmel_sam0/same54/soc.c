/*
 * Copyright (c) 2019 ML!PA Consulting GmbH
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief Atmel SAMD MCU series initialization code
 */

#include <arch/cpu.h>
#include <cortex_m/exc.h>
#include <device.h>
#include <init.h>
#include <kernel.h>
#include <soc.h>

static void _gclk_setup(int gclk, uint32_t val)
{
	GCLK->GENCTRL[gclk].reg = val;
	while (GCLK->SYNCBUSY.reg & GCLK_SYNCBUSY_GENCTRL(gclk));
}

static int atmel_samd_init(struct device *arg)
{
	u32_t key;

	ARG_UNUSED(arg);

	key = irq_lock();

	/* Install default handler that simply resets the CPU
	 * if configured in the kernel, NOP otherwise
	 */
	NMI_INIT();

	/* Setup GCLK generators - use 48MHz DFLLL */
	_gclk_setup(0, GCLK_GENCTRL_GENEN | GCLK_GENCTRL_SRC_DFLL);

	irq_unlock(key);

	return 0;
}

SYS_INIT(atmel_samd_init, PRE_KERNEL_1, 0);
