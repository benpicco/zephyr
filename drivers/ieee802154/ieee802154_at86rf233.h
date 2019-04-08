/*
 * Copyright (c) 2019 ML!PA Consulting GmbH
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_DRIVERS_IEEE802154_AT86RF233_H_
#define ZEPHYR_DRIVERS_IEEE802154_AT86RF233_H_

#include <linker/sections.h>
#include <atomic.h>
#include <spi.h>

#define AT86_READ	(0x00)
#define AT86_WRITE	(0x40)
#define AT86_SRAM	(0x00)
#define AT86_FIFO	(0x20)
#define AT86_REG	(0x80)

/* Runtime context structure
 ***************************
 */
struct rf233_context {
	struct net_if *iface;
	/**************************/
	struct device *irq_gpio;
	struct device *reset_gpio;
	struct device *slp_tr_gpio;
	struct gpio_callback irq_cb;
	struct device *spi;
	struct spi_config spi_cfg;
#if defined(DT_ATMEL_AT86RF233_0_CS_GPIO_CONTROLLER)
	struct spi_cs_control cs_ctrl;
#endif
	u8_t mac_addr[8];
	struct k_mutex phy_mutex;
	struct k_sem isr_sem;
	/************RX************/
	K_THREAD_STACK_MEMBER(rf233_rx_stack,
			      CONFIG_IEEE802154_AT86RF233_RX_STACK_SIZE);
	struct k_thread rf233_rx_thread;
};

#endif /* ZEPHYR_DRIVERS_IEEE802154_AT86RF233_H_ */
