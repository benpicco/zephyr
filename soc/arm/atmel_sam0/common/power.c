/*
 * Copyright (c) 2019 Benjamin Valentin
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include <zephyr.h>
#include <power.h>
#include <soc.h>

#include <logging/log.h>
LOG_MODULE_DECLARE(soc, CONFIG_SOC_LOG_LEVEL);

static inline void sam0_idle(uint8_t mode)
{
	SCB->SCR &= ~SCB_SCR_SLEEPDEEP_Msk;
	PM->SLEEP.reg = mode;
	__DSB();
	__WFI();
}

static inline void sam0_deep_sleep(void)
{
	SCB->SCR |= SCB_SCR_SLEEPDEEP_Msk;
	__DSB();
	__WFI();
}

/* Invoke Low Power/System Off specific Tasks */
void sys_set_power_state(enum power_states state)
{
	switch (state) {

#ifdef CONFIG_SYS_POWER_LOW_POWER_STATES
	case SYS_POWER_STATE_CPU_LPS_1:
		sam0_idle(0);
		break;
	case SYS_POWER_STATE_CPU_LPS_2:
		sam0_idle(1);
		break;
	case SYS_POWER_STATE_CPU_LPS_3:
		sam0_idle(2);
		break;
#endif

#ifdef CONFIG_SYS_POWER_DEEP_SLEEP_STATES
	case SYS_POWER_STATE_DEEP_SLEEP_1:
		sam0_deep_sleep();
		break;
#endif

	default:
		LOG_ERR("Unsupported power state %u", state);
		break;
	}
}

/* Handle SOC specific activity after Low Power Mode Exit */
void sys_power_state_post_ops(enum power_states state)
{
	switch (state) {
#ifdef CONFIG_SYS_POWER_LOW_POWER_STATES
	case SYS_POWER_STATE_CPU_LPS_1:
		/* Nothing to do. */
		break;
	case SYS_POWER_STATE_CPU_LPS_2:
		/* Nothing to do. */
		break;
	case SYS_POWER_STATE_CPU_LPS_3:
		/* Nothing to do. */
		break;
#endif

#ifdef CONFIG_SYS_POWER_DEEP_SLEEP_STATES
	case SYS_POWER_STATE_DEEP_SLEEP_1:
		/* Nothing to do. */
		break;
#endif
	default:
		LOG_ERR("Unsupported power state %u", state);
		break;
	}

	/*
	 * System is now in active mode. Reenable interrupts which were disabled
	 * when OS started idling code.
	 */
	irq_unlock(0);
}
