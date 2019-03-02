/*
 * Copyright (c) 2019 Derek Hageman <hageman@inthat.cloud>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <device.h>
#include <soc.h>
#include "sam0_eic.h"
#include "sam0_eic_priv.h"

struct sam0_eic_line_assignment {
	u8_t pin : 5;
	u8_t port : 2;
	u8_t enabled : 1;
};

struct sam0_eic_port_data {
	sam0_eic_callback_t cb;
	void *data;
};

struct sam0_eic_data {
	struct sam0_eic_port_data ports[PORT_GROUPS];
	struct sam0_eic_line_assignment lines[EIC_EXTINT_NUM];
};

#define DEV_DATA(dev) \
	((struct sam0_eic_data *const)(dev)->driver_data)

DEVICE_DECLARE(sam0_eic);


static void wait_synchronization(void)
{
	while (EIC->STATUS.bit.SYNCBUSY) {
	}
}

#if DT_EIC_SAM0_IRQ(1) > 0
#define SAM0_EIC_PER_LINE_IRQ
#endif

#ifdef SAM0_EIC_PER_LINE_IRQ
static void sam0_eic_isr_line(void *arg, int line_index)
{
	struct device *dev = (struct device *)arg;
	struct sam0_eic_data *const dev_data = DEV_DATA(dev);
	struct sam0_eic_line_assignment *line_assignment =
			&dev_data->lines[line_index];
	struct sam0_eic_port_data *port_data =
			&dev_data->ports[line_assignment->port];
	u32_t bits = BIT(line_index);

	/* Check that the interrupt is actually present */
	if (!(EIC->INTFLAG.reg & bits)) {
		return;
	}

	/* Acknowledge the interrupt */
	EIC->INTFLAG.reg = bits;

	port_data->cb(BIT(line_assignment->pin), port_data->data);
}

#define SAM0_EIC_LINE_IRQ_DECL(n)					\
	static void sam0_eic_isr_line_ ## n(void *arg)			\
	{								\
		sam0_eic_isr_line(arg, n);				\
	}

#if DT_EIC_SAM0_IRQ(1) > 0
SAM0_EIC_LINE_IRQ_DECL(0)
#undef SAM0_EIC_COMPOSITE_IRQ
#define SAM0_EIC_COMPOSITE_IRQ 1
#endif

#if DT_EIC_SAM0_IRQ(2) > 0
SAM0_EIC_LINE_IRQ_DECL(1)
#undef SAM0_EIC_COMPOSITE_IRQ
#define SAM0_EIC_COMPOSITE_IRQ 2
#endif

#if DT_EIC_SAM0_IRQ(3) > 0
SAM0_EIC_LINE_IRQ_DECL(2)
#undef SAM0_EIC_COMPOSITE_IRQ
#define SAM0_EIC_COMPOSITE_IRQ 3
#endif

#if DT_EIC_SAM0_IRQ(4) > 0
SAM0_EIC_LINE_IRQ_DECL(3)
#undef SAM0_EIC_COMPOSITE_IRQ
#define SAM0_EIC_COMPOSITE_IRQ 4
#endif

#if DT_EIC_SAM0_IRQ(5) > 0
SAM0_EIC_LINE_IRQ_DECL(4)
#undef SAM0_EIC_COMPOSITE_IRQ
#define SAM0_EIC_COMPOSITE_IRQ 5
#endif

#if DT_EIC_SAM0_IRQ(6) > 0
SAM0_EIC_LINE_IRQ_DECL(5)
#undef SAM0_EIC_COMPOSITE_IRQ
#define SAM0_EIC_COMPOSITE_IRQ 6
#endif

#if DT_EIC_SAM0_IRQ(7) > 0
SAM0_EIC_LINE_IRQ_DECL(6)
#undef SAM0_EIC_COMPOSITE_IRQ
#define SAM0_EIC_COMPOSITE_IRQ 7
#endif

#if DT_EIC_SAM0_IRQ(8) > 0
SAM0_EIC_LINE_IRQ_DECL(7)
#undef SAM0_EIC_COMPOSITE_IRQ
#define SAM0_EIC_COMPOSITE_IRQ 8
#endif

#if DT_EIC_SAM0_IRQ(9) > 0
SAM0_EIC_LINE_IRQ_DECL(8)
#undef SAM0_EIC_COMPOSITE_IRQ
#define SAM0_EIC_COMPOSITE_IRQ 9
#endif

#if DT_EIC_SAM0_IRQ(10) > 0
SAM0_EIC_LINE_IRQ_DECL(9)
#undef SAM0_EIC_COMPOSITE_IRQ
#define SAM0_EIC_COMPOSITE_IRQ 10
#endif

#if DT_EIC_SAM0_IRQ(11) > 0
SAM0_EIC_LINE_IRQ_DECL(10)
#undef SAM0_EIC_COMPOSITE_IRQ
#define SAM0_EIC_COMPOSITE_IRQ 11
#endif

#if DT_EIC_SAM0_IRQ(12) > 0
SAM0_EIC_LINE_IRQ_DECL(11)
#undef SAM0_EIC_COMPOSITE_IRQ
#define SAM0_EIC_COMPOSITE_IRQ 12
#endif

#if DT_EIC_SAM0_IRQ(13) > 0
SAM0_EIC_LINE_IRQ_DECL(12)
#undef SAM0_EIC_COMPOSITE_IRQ
#define SAM0_EIC_COMPOSITE_IRQ 13
#endif

#if DT_EIC_SAM0_IRQ(14) > 0
SAM0_EIC_LINE_IRQ_DECL(13)
#undef SAM0_EIC_COMPOSITE_IRQ
#define SAM0_EIC_COMPOSITE_IRQ 14
#endif

#if DT_EIC_SAM0_IRQ(15) > 0
SAM0_EIC_LINE_IRQ_DECL(14)
SAM0_EIC_LINE_IRQ_DECL(15)
#undef SAM0_EIC_COMPOSITE_IRQ
#endif

#else
#define SAM0_EIC_COMPOSITE_IRQ 0
#endif

#ifdef SAM0_EIC_COMPOSITE_IRQ
static void sam0_eic_isr_composite(void *arg)
{
	struct device *dev = (struct device *)arg;
	struct sam0_eic_data *const dev_data = DEV_DATA(dev);
	u16_t bits = EIC->INTFLAG.reg;
	u32_t line_index;

	/* Acknowledge all interrupts */
	EIC->INTFLAG.reg = bits;

	/* No clz on M0, so just do a quick test */
#if __CORTEX_M >= 3
	line_index = __CLZ(__RBIT(bits));
	bits >>= line_index;
#else
	if (bits & 0xFF) {
		line_index = 0;
	} else {
		line_index = 8;
		bits >>= 8;
	}
#endif

	/*
	 * Map the EIC lines to the port pin masks based on which port is
	 * selected in the line data.
	 */
	for (; bits; bits >>= 1, line_index++) {
		if (!(bits & 1)) {
			continue;
		}

		/*
		 * These could be aggregated together into one call, but
		 * usually on a single one will be set, so just call them
		 * one by one.
		 */
		struct sam0_eic_line_assignment *line_assignment =
				&dev_data->lines[line_index];
		struct sam0_eic_port_data *port_data =
				&dev_data->ports[line_assignment->port];

		port_data->cb(BIT(line_assignment->pin), port_data->data);
	}
}
#endif

static int sam0_eic_irq_for_line(int line_index)
{
	switch (line_index) {
	default:
#if DT_EIC_SAM0_IRQ(15) > 0
	case 15:
		return DT_EIC_SAM0_IRQ(15);
#endif
#if DT_EIC_SAM0_IRQ(14) > 0
	case 14:
		return DT_EIC_SAM0_IRQ(14);
#endif
#if DT_EIC_SAM0_IRQ(13) > 0
	case 13:
		return DT_EIC_SAM0_IRQ(13);
#endif
#if DT_EIC_SAM0_IRQ(12) > 0
	case 12:
		return DT_EIC_SAM0_IRQ(12);
#endif
#if DT_EIC_SAM0_IRQ(11) > 0
	case 11:
		return DT_EIC_SAM0_IRQ(11);
#endif
#if DT_EIC_SAM0_IRQ(10) > 0
	case 10:
		return DT_EIC_SAM0_IRQ(10);
#endif
#if DT_EIC_SAM0_IRQ(9) > 0
	case 9:
		return DT_EIC_SAM0_IRQ(9);
#endif
#if DT_EIC_SAM0_IRQ(8) > 0
	case 8:
		return DT_EIC_SAM0_IRQ(8);
#endif
#if DT_EIC_SAM0_IRQ(7) > 0
	case 7:
		return DT_EIC_SAM0_IRQ(7);
#endif
#if DT_EIC_SAM0_IRQ(6) > 0
	case 6:
		return DT_EIC_SAM0_IRQ(6);
#endif
#if DT_EIC_SAM0_IRQ(5) > 0
	case 5:
		return DT_EIC_SAM0_IRQ(5);
#endif
#if DT_EIC_SAM0_IRQ(4) > 0
	case 4:
		return DT_EIC_SAM0_IRQ(4);
#endif
#if DT_EIC_SAM0_IRQ(3) > 0
	case 3:
		return DT_EIC_SAM0_IRQ(3);
#endif
#if DT_EIC_SAM0_IRQ(2) > 0
	case 2:
		return DT_EIC_SAM0_IRQ(2);
#endif
#if DT_EIC_SAM0_IRQ(1) > 0
	case 1:
		return DT_EIC_SAM0_IRQ(1);
#endif
#if DT_EIC_SAM0_IRQ(0) > 0
	case 0:
		return DT_EIC_SAM0_IRQ(0);
#endif
	}
}

int sam0_eic_acquire(int port, int pin, enum sam0_eic_trigger trigger,
		     bool filter, sam0_eic_callback_t cb, void *data)
{
	struct device *dev = DEVICE_GET(sam0_eic);
	struct sam0_eic_data *dev_data = dev->driver_data;
	struct sam0_eic_port_data *port_data;
	struct sam0_eic_line_assignment *line_assignment;
	u32_t mask;
	int line_index;
	int config_index;
	int config_shift;
	u32_t config;

	line_index = sam0_eic_map_to_line(port, pin);
	if (line_index < 0) {
		return line_index;
	}
	mask = BIT(line_index);
	config_index = line_index / 8;
	config_shift = (line_index % 8) * 4;

	/* Disable the EIC for reconfiguration */
	EIC->CTRL.bit.ENABLE = 0;
	wait_synchronization();
	irq_disable(sam0_eic_irq_for_line(line_index));

	line_assignment = &dev_data->lines[line_index];

	/* Check that the required line is available */
	if (line_assignment->enabled) {
		if (line_assignment->port != port ||
				line_assignment->pin != pin) {
			goto err_in_use;
		}
	}

	/* Set the EIC configuration data */
	port_data = &dev_data->ports[port];
	port_data->cb = cb;
	port_data->data = data;
	line_assignment->pin = pin;
	line_assignment->port = port;
	line_assignment->enabled = 1;

	config = EIC->CONFIG[config_index].reg;
	config &= ~(0xF << config_shift);
	switch (trigger) {
	case SAM0_EIC_RISING:
		config |= EIC_CONFIG_SENSE0_RISE << config_shift;
		break;
	case SAM0_EIC_FALLING:
		config |= EIC_CONFIG_SENSE0_FALL << config_shift;
		break;
	case SAM0_EIC_BOTH:
		config |= EIC_CONFIG_SENSE0_BOTH << config_shift;
		break;
	case SAM0_EIC_HIGH:
		config |= EIC_CONFIG_SENSE0_HIGH << config_shift;
		break;
	case SAM0_EIC_LOW:
		config |= EIC_CONFIG_SENSE0_LOW << config_shift;
		break;
	}
	if (filter) {
		config |= EIC_CONFIG_FILTEN0 << config_shift;
	}

	/* Apply the config to the EIC itself */
	EIC->CONFIG[config_index].reg = config;

	EIC->CTRL.bit.ENABLE = 1;
	wait_synchronization();
	/*
	 * Errata: The EIC generates a spurious interrupt for the newly
	 * enabled pin after being enabled, so clear it before re-enabling
	 * the IRQ.
	 */
	EIC->INTFLAG.reg = mask;
	irq_enable(sam0_eic_irq_for_line(line_index));
	return 0;

err_in_use:
	EIC->CTRL.bit.ENABLE = 1;
	wait_synchronization();
	irq_enable(sam0_eic_irq_for_line(line_index));
	return -EBUSY;
}

static bool sam0_eic_check_ownership(int port, int pin, int line_index)
{
	struct device *dev = DEVICE_GET(sam0_eic);
	struct sam0_eic_data *dev_data = dev->driver_data;
	struct sam0_eic_line_assignment *line_assignment =
			&dev_data->lines[line_index];

	if (!line_assignment->enabled) {
		return false;
	}
	if (line_assignment->port != port ||
			line_assignment->pin != pin) {
		return false;
	}

	return true;
}

int sam0_eic_release(int port, int pin)
{
	struct device *dev = DEVICE_GET(sam0_eic);
	struct sam0_eic_data *dev_data = dev->driver_data;
	u32_t mask;
	int line_index;
	int config_index;
	int config_shift;

	line_index = sam0_eic_map_to_line(port, pin);
	if (line_index < 0) {
		return line_index;
	}
	mask = BIT(line_index);
	config_index = line_index / 8;
	config_shift = (line_index % 8) * 4;

	/* Disable the EIC */
	EIC->CTRL.bit.ENABLE = 0;
	wait_synchronization();
	irq_disable(sam0_eic_irq_for_line(line_index));

	/*
	 * Check to make sure the requesting actually owns the line and do
	 * nothing if it does not.
	 */
	if (!sam0_eic_check_ownership(port, pin, line_index)) {
		goto done;
	}
	dev_data->lines[line_index].enabled = 0;

	/* Clear the EIC config, including the trigger condition */
	EIC->CONFIG[config_index].reg &= ~(0xF << config_shift);

	/* Clear any pending interrupt for it */
	EIC->INTENCLR.reg = mask;
	EIC->INTFLAG.reg = mask;

done:
	EIC->CTRL.bit.ENABLE = 1;
	wait_synchronization();
	irq_enable(sam0_eic_irq_for_line(line_index));
	return 0;
}

int sam0_eic_enable_interrupt(int port, int pin)
{
	u32_t mask;
	int line_index;

	line_index = sam0_eic_map_to_line(port, pin);
	if (line_index < 0) {
		return line_index;
	}

	if (!sam0_eic_check_ownership(port, pin, line_index)) {
		return -EBUSY;
	}

	mask = BIT(line_index);
	EIC->INTFLAG.reg = mask;
	EIC->INTENSET.reg = mask;

	return 0;
}

int sam0_eic_disable_interrupt(int port, int pin)
{
	u32_t mask;
	int line_index;

	line_index = sam0_eic_map_to_line(port, pin);
	if (line_index < 0) {
		return line_index;
	}

	if (!sam0_eic_check_ownership(port, pin, line_index)) {
		return -EBUSY;
	}

	mask = BIT(line_index);
	EIC->INTENCLR.reg = mask;
	EIC->INTFLAG.reg = mask;

	return 0;
}

u32_t sam0_eic_interrupt_pending(int port)
{
	struct device *dev = DEVICE_GET(sam0_eic);
	struct sam0_eic_data *dev_data = dev->driver_data;
	struct sam0_eic_line_assignment *line_assignment;
	u32_t set = EIC->INTFLAG.reg;
	u32_t mask = 0;

	for (int line_index = 0; line_index < EIC_EXTINT_NUM; line_index++) {
		line_assignment = &dev_data->lines[line_index];
		if (!line_assignment->enabled) {
			continue;
		}
		if (line_assignment->port != port) {
			continue;
		}
		if (!(set & BIT(line_index))) {
			continue;
		}
		mask |= BIT(line_assignment->pin);
	}

	return mask;
}

#define SAM0_EIC_LINE_IRQ_CONNECT(n) do {				\
	IRQ_CONNECT(DT_EIC_SAM0_IRQ(n), DT_EIC_SAM0_IRQ_PRIORITY(n),	\
		    sam0_eic_isr_line_ ## n, DEVICE_GET(sam0_eic), 0);	\
	irq_enable(DT_EIC_SAM0_IRQ(n));					\
} while (0)


static int sam0_eic_init(struct device *dev)
{
	ARG_UNUSED(dev);

	/* Enable the EIC clock in PM */
	PM->APBAMASK.bit.EIC_ = 1;

	/* Enable the GCLK */
	GCLK->CLKCTRL.reg = GCLK_CLKCTRL_ID_EIC | GCLK_CLKCTRL_GEN_GCLK0 |
			    GCLK_CLKCTRL_CLKEN;

#if !defined(SAM0_EIC_COMPOSITE_IRQ) || SAM0_EIC_COMPOSITE_IRQ > 0
	SAM0_EIC_LINE_IRQ_CONNECT(0);
#endif
#if !defined(SAM0_EIC_COMPOSITE_IRQ) || SAM0_EIC_COMPOSITE_IRQ > 1
	SAM0_EIC_LINE_IRQ_CONNECT(1);
#endif
#if !defined(SAM0_EIC_COMPOSITE_IRQ) || SAM0_EIC_COMPOSITE_IRQ > 2
	SAM0_EIC_LINE_IRQ_CONNECT(2);
#endif
#if !defined(SAM0_EIC_COMPOSITE_IRQ) || SAM0_EIC_COMPOSITE_IRQ > 3
	SAM0_EIC_LINE_IRQ_CONNECT(3);
#endif
#if !defined(SAM0_EIC_COMPOSITE_IRQ) || SAM0_EIC_COMPOSITE_IRQ > 4
	SAM0_EIC_LINE_IRQ_CONNECT(4);
#endif
#if !defined(SAM0_EIC_COMPOSITE_IRQ) || SAM0_EIC_COMPOSITE_IRQ > 5
	SAM0_EIC_LINE_IRQ_CONNECT(5);
#endif
#if !defined(SAM0_EIC_COMPOSITE_IRQ) || SAM0_EIC_COMPOSITE_IRQ > 6
	SAM0_EIC_LINE_IRQ_CONNECT(6);
#endif
#if !defined(SAM0_EIC_COMPOSITE_IRQ) || SAM0_EIC_COMPOSITE_IRQ > 7
	SAM0_EIC_LINE_IRQ_CONNECT(7);
#endif
#if !defined(SAM0_EIC_COMPOSITE_IRQ) || SAM0_EIC_COMPOSITE_IRQ > 8
	SAM0_EIC_LINE_IRQ_CONNECT(8);
#endif
#if !defined(SAM0_EIC_COMPOSITE_IRQ) || SAM0_EIC_COMPOSITE_IRQ > 9
	SAM0_EIC_LINE_IRQ_CONNECT(9);
#endif
#if !defined(SAM0_EIC_COMPOSITE_IRQ) || SAM0_EIC_COMPOSITE_IRQ > 10
	SAM0_EIC_LINE_IRQ_CONNECT(10);
#endif
#if !defined(SAM0_EIC_COMPOSITE_IRQ) || SAM0_EIC_COMPOSITE_IRQ > 11
	SAM0_EIC_LINE_IRQ_CONNECT(11);
#endif
#if !defined(SAM0_EIC_COMPOSITE_IRQ) || SAM0_EIC_COMPOSITE_IRQ > 12
	SAM0_EIC_LINE_IRQ_CONNECT(12);
#endif
#if !defined(SAM0_EIC_COMPOSITE_IRQ) || SAM0_EIC_COMPOSITE_IRQ > 13
	SAM0_EIC_LINE_IRQ_CONNECT(13);
#endif
#if !defined(SAM0_EIC_COMPOSITE_IRQ) || SAM0_EIC_COMPOSITE_IRQ > 14
	SAM0_EIC_LINE_IRQ_CONNECT(14);
#endif
#if !defined(SAM0_EIC_COMPOSITE_IRQ) || SAM0_EIC_COMPOSITE_IRQ > 15
	SAM0_EIC_LINE_IRQ_CONNECT(15);
#endif

#ifdef SAM0_EIC_COMPOSITE_IRQ
	IRQ_CONNECT(MACRO_MAP_1(DT_EIC_SAM0_IRQ,
				SAM0_EIC_COMPOSITE_IRQ),
		    MACRO_MAP_1(DT_EIC_SAM0_IRQ_PRIORITY,
				SAM0_EIC_COMPOSITE_IRQ),
		    sam0_eic_isr_composite, DEVICE_GET(sam0_eic), 0);
	irq_enable(MACRO_MAP_1(DT_EIC_SAM0_IRQ, SAM0_EIC_COMPOSITE_IRQ));
#endif

	EIC->CTRL.bit.ENABLE = 1;
	wait_synchronization();

	return 0;
}

static struct sam0_eic_data eic_data;
DEVICE_INIT(sam0_eic, DT_EIC_SAM0_LABEL, sam0_eic_init,
	    &eic_data, NULL,
	    PRE_KERNEL_1, CONFIG_KERNEL_INIT_PRIORITY_DEFAULT);
