/*
 * Copyright (c) 2019 ML!PA Consulting GmbH
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_DRIVERS_IEEE802154_MRF24J40_H_
#define ZEPHYR_DRIVERS_IEEE802154_MRF24J40_H_

#include <linker/sections.h>
#include <atomic.h>
#include <spi.h>

/* Runtime context structure
 ***************************
 */
struct mrf24j40_context {
	struct net_if *iface;
	/**************************/
	struct device *irq_gpio;
	struct device *reset_gpio;
	struct device *wake_gpio;
	struct gpio_callback irq_cb;
	struct device *spi;
	struct spi_config spi_cfg;
#if defined(DT_MICROCHIP_MRF24J40_0_CS_GPIO_CONTROLLER)
	struct spi_cs_control cs_ctrl;
#endif
	u8_t mac_addr[8];
	struct k_mutex phy_mutex;
	struct k_sem isr_sem;
	/************RX************/
	K_THREAD_STACK_MEMBER(mrf24j40_rx_stack,
			      CONFIG_MRF24J40_RX_STACK_SIZE);
	struct k_thread mrf24j40_rx_thread;
};

#endif /* ZEPHYR_DRIVERS_IEEE802154_MRF24J40_H_ */
