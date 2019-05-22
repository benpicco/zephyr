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

#if CONFIG_SOC_ATMEL_SAMD5X_XOSC32K_AS_MAIN
static void osc32k_init(void)
{
	OSC32KCTRL->XOSC32K.reg = OSC32KCTRL_XOSC32K_ENABLE | OSC32KCTRL_XOSC32K_XTALEN
				| OSC32KCTRL_XOSC32K_EN32K | OSC32KCTRL_XOSC32K_RUNSTDBY
				| OSC32KCTRL_XOSC32K_STARTUP(7);

	while (!OSC32KCTRL->STATUS.bit.XOSC32KRDY) {
	}

	GCLK->GENCTRL[1].reg = GCLK_GENCTRL_SRC(GCLK_SOURCE_XOSC32K)
			     | GCLK_GENCTRL_RUNSTDBY | GCLK_GENCTRL_GENEN;

}
#elif CONFIG_SOC_ATMEL_SAMD5X_OSCULP32K_AS_MAIN
static void osc32k_init(void)
{
	GCLK->GENCTRL[1].reg = GCLK_GENCTRL_SRC(GCLK_SOURCE_OSCULP32K)
			     | GCLK_GENCTRL_RUNSTDBY | GCLK_GENCTRL_GENEN;
}
#else
#error "No Clock Source selected."
#endif

static void dpll0_init(u32_t f_cpu)
{
	const u32_t LDR = ((f_cpu * 16) / SOC_ATMEL_SAM0_OSC32K_FREQ_HZ);

	/* source FDPLL from 32kHz clock */
	GCLK->PCHCTRL[OSCCTRL_GCLK_ID_FDPLL0].reg = GCLK_PCHCTRL_GEN(1)
						  | GCLK_PCHCTRL_CHEN;
	while (!(GCLK->PCHCTRL[OSCCTRL_GCLK_ID_FDPLL0].reg & GCLK_PCHCTRL_CHEN)) {
	}

	GCLK->PCHCTRL[OSCCTRL_GCLK_ID_FDPLL032K].reg = GCLK_PCHCTRL_GEN(1)
						     | GCLK_PCHCTRL_CHEN;
	while (!(GCLK->PCHCTRL[OSCCTRL_GCLK_ID_FDPLL032K].reg & GCLK_PCHCTRL_CHEN)) {
	}

	OSCCTRL->Dpll[0].DPLLRATIO.reg = OSCCTRL_DPLLRATIO_LDRFRAC(LDR % 16)
				       | OSCCTRL_DPLLRATIO_LDR((LDR / 16) - 1);

	OSCCTRL->Dpll[0].DPLLCTRLB.reg = OSCCTRL_DPLLCTRLB_REFCLK_XOSC32
				       | OSCCTRL_DPLLCTRLB_DIV(1)
				       | OSCCTRL_DPLLCTRLB_WUF
				       | OSCCTRL_DPLLCTRLB_LBYPASS;

	OSCCTRL->Dpll[0].DPLLCTRLA.reg = OSCCTRL_DPLLCTRLA_ENABLE;

	while (!OSCCTRL->Dpll[0].DPLLSTATUS.bit.CLKRDY ||
	       !OSCCTRL->Dpll[0].DPLLSTATUS.bit.LOCK) {
	}

	GCLK->GENCTRL[0].reg = GCLK_GENCTRL_SRC(GCLK_SOURCE_DPLL0)
			     | GCLK_GENCTRL_GENEN;
}

static void dfll48_init(void)
{
	GCLK->GENCTRL[2].reg = GCLK_GENCTRL_SRC(GCLK_SOURCE_DFLL48M)
			     | GCLK_GENCTRL_GENEN;
}

static int atmel_samd_init(struct device *arg)
{
	u32_t key;

	ARG_UNUSED(arg);

	key = irq_lock();

	/* enable the Cortex M Cache Controller */
	CMCC->CTRL.bit.CEN = 1;

	osc32k_init();
	dfll48_init();
	dpll0_init(CONFIG_SYS_CLOCK_HW_CYCLES_PER_SEC);

	/* Install default handler that simply resets the CPU
	 * if configured in the kernel, NOP otherwise
	 */
	NMI_INIT();

	irq_unlock(key);

	return 0;
}

SYS_INIT(atmel_samd_init, PRE_KERNEL_1, 0);
