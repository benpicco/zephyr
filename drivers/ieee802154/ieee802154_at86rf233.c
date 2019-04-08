/* ieee802154_at86rf233.c - Atmel AT86RF233 driver */

/*
 * Copyright (c) 2019 ML!PA Consulting GmbH
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define LOG_MODULE_NAME ieee802154_at86rf233
#define LOG_LEVEL CONFIG_IEEE802154_DRIVER_LOG_LEVEL

#define AT86RF233_PSDU_LENGTH		(125)

#include <logging/log.h>
LOG_MODULE_REGISTER(LOG_MODULE_NAME);

#include <errno.h>

#include <kernel.h>
#include <arch/cpu.h>

#include <device.h>
#include <init.h>
#include <net/net_if.h>
#include <net/net_pkt.h>

#include <misc/byteorder.h>
#include <string.h>
#include <random/rand32.h>

#include <gpio.h>

#include <net/ieee802154_radio.h>

#include "ieee802154_at86rf233.h"
#include "ieee802154_at86rf233_regs.h"

static inline void irq_int_handler(struct device *port,
				   struct gpio_callback *cb, u32_t pins)
{
	struct rf233_context *rf233 = CONTAINER_OF(cb,
						     struct rf233_context,
						     irq_cb);
	k_sem_give(&rf233->isr_sem);
}

static inline void set_reset(struct device *dev, u32_t value)
{
	struct rf233_context *rf233 = dev->driver_data;

	gpio_pin_write(rf233->reset_gpio,
			DT_ATMEL_AT86RF233_0_RESET_GPIOS_PIN, value);
}

static inline void set_slp_tr(struct device *dev, u32_t value)
{
	struct rf233_context *rf233 = dev->driver_data;

	gpio_pin_write(rf233->reset_gpio,
			DT_ATMEL_AT86RF233_0_SLP_TR_GPIOS_PIN, value);
}

static void enable_irq_interrupt(struct rf233_context *rf233,
				 bool enable)
{
	if (enable) {
		gpio_pin_enable_callback(rf233->irq_gpio,
					 DT_ATMEL_AT86RF233_0_IRQ_GPIOS_PIN);
	} else {
		gpio_pin_disable_callback(rf233->irq_gpio,
					  DT_ATMEL_AT86RF233_0_IRQ_GPIOS_PIN);
	}
}

static inline void setup_gpio_callbacks(struct rf233_context *rf233)
{
	gpio_init_callback(&rf233->irq_cb,
			   irq_int_handler,
			   BIT(DT_ATMEL_AT86RF233_0_IRQ_GPIOS_PIN));
	gpio_add_callback(rf233->irq_gpio, &rf233->irq_cb);
}

static bool spi_write_reg(struct rf233_context *rf233, u8_t reg, u8_t data)
{
	u8_t tx_buffer[2];
	tx_buffer[0] = AT86_WRITE | AT86_REG | reg;
	tx_buffer[1] = data;

	const struct spi_buf buf = {
		.buf = tx_buffer,
		.len = sizeof(tx_buffer)
	};

	const struct spi_buf_set tx = {
		.buffers = &buf,
		.count = 1
	};

	return (spi_write(rf233->spi, &rf233->spi_cfg, &tx) == 0);
}

static u8_t spi_read_reg(struct rf233_context *rf233, u8_t reg)
{
	u8_t buffer[2];
	buffer[0] = AT86_READ | AT86_REG | reg;
	buffer[1] = 0;

	const struct spi_buf buf_trx = {
		.buf = buffer,
		.len = sizeof(buffer)
	};

	const struct spi_buf_set tx = {
		.buffers = &buf_trx,
		.count = 1
	};

	const struct spi_buf_set rx = {
		.buffers = &buf_trx,
		.count = 1
	};

	if (spi_transceive(rf233->spi, &rf233->spi_cfg, &tx, &rx) == 0) {
		return buffer[1];
	}

	LOG_ERR("Failed");

	return 0;
}

static bool spi_write_tx_fifo(struct rf233_context *rf233, u8_t *data, u8_t size)
{
	u8_t tx_buffer[2];
	tx_buffer[0] = AT86_WRITE | AT86_FIFO;
	tx_buffer[1] = size;

	const struct spi_buf bufs[2] = {
		{
			.buf = tx_buffer,
			.len = sizeof(tx_buffer)
		},
		{
			.buf = data,
			.len = size
		}
	};

	const struct spi_buf_set tx = {
		.buffers = bufs,
		.count = 2
	};

	return (spi_write(rf233->spi, &rf233->spi_cfg, &tx) == 0);
}

static u8_t _fifo_read_size(struct rf233_context *rf233)
{
	u8_t buffer[2];
	buffer[0] = AT86_READ | AT86_FIFO;
	buffer[1] = 0;

	const struct spi_buf buf_cmd = {
		.buf = buffer,
		.len = sizeof(buffer)
	};

	const struct spi_buf_set rx = {
		.buffers = &buf_cmd,
		.count = 1
	};

	const struct spi_buf_set tx = {
		.buffers = &buf_cmd,
		.count = 1
	};

	rf233->spi_cfg.operation |= SPI_HOLD_ON_CS;
	spi_transceive(rf233->spi, &rf233->spi_cfg, &tx, &rx);
	rf233->spi_cfg.operation &= ~SPI_HOLD_ON_CS;

	return buffer[1];
}

static bool spi_read_rx_fifo(struct rf233_context *rf233, u8_t *dst, u8_t *size, u8_t max_size, u8_t *pLqi)
{
	int ret;
	u8_t crc_lqi[3];

	*size = _fifo_read_size(rf233);

	const struct spi_buf bufs[2] = {
		{
			.buf = dst,
			.len = MIN(*size, max_size)
		},
		{
			.buf = crc_lqi,
			.len = sizeof(crc_lqi)
		}
	};

	const struct spi_buf_set rx = {
		.buffers = bufs,
		.count = 2
	};

	ret = spi_read(rf233->spi, &rf233->spi_cfg, &rx);

	// TODO: check CRC?
	*pLqi = crc_lqi[2];

	return (ret == 0);
}

static void rf233_hardware_reset(struct device *dev)
{
	set_reset(dev, 0);
	k_busy_wait(62U);
	set_reset(dev, 1);
	k_busy_wait(62U);
}

static enum ieee802154_hw_caps rf233_get_capabilities(struct device *dev)
{
	return    IEEE802154_HW_FCS
		| IEEE802154_HW_2_4_GHZ
		| IEEE802154_HW_TX_RX_ACK
		| IEEE802154_HW_FILTER;
}

/* Note: CCA before TX is enabled by default */
static int rf233_cca(struct device *dev)
{
	// TODO
	return -EIO;
}

static int rf233_set_channel(struct device *dev, u16_t channel)
{
	struct rf233_context *rf233 = dev->driver_data;

	k_mutex_lock(&rf233->phy_mutex, K_FOREVER);

	spi_write_reg(rf233, RG_PHY_CC_CCA, channel);

	k_mutex_unlock(&rf233->phy_mutex);

	return 0;
}

static int rf233_set_pan_id(struct device *dev, u16_t pan_id)
{
	struct rf233_context *rf233 = dev->driver_data;

	k_mutex_lock(&rf233->phy_mutex, K_FOREVER);

	spi_write_reg(rf233, RG_PAN_ID_0, pan_id >> 8);
	spi_write_reg(rf233, RG_PAN_ID_1, pan_id & 0xFF);

	k_mutex_unlock(&rf233->phy_mutex);

	return 0;
}

static int rf233_set_short_addr(struct device *dev, u16_t short_addr)
{
	struct rf233_context *rf233 = dev->driver_data;

	k_mutex_lock(&rf233->phy_mutex, K_FOREVER);

	spi_write_reg(rf233, RG_SHORT_ADDR_0, short_addr >> 8);
	spi_write_reg(rf233, RG_SHORT_ADDR_1, short_addr & 0xFF);

	k_mutex_unlock(&rf233->phy_mutex);

	return 0;
}

static int rf233_set_ieee_addr(struct device *dev, const u8_t *ieee_addr)
{
	struct rf233_context *rf233 = dev->driver_data;

	k_mutex_lock(&rf233->phy_mutex, K_FOREVER);

	spi_write_reg(rf233, RG_IEEE_ADDR_0, ieee_addr[0]);
	spi_write_reg(rf233, RG_IEEE_ADDR_1, ieee_addr[1]);
	spi_write_reg(rf233, RG_IEEE_ADDR_2, ieee_addr[2]);
	spi_write_reg(rf233, RG_IEEE_ADDR_3, ieee_addr[3]);
	spi_write_reg(rf233, RG_IEEE_ADDR_4, ieee_addr[4]);
	spi_write_reg(rf233, RG_IEEE_ADDR_5, ieee_addr[5]);
	spi_write_reg(rf233, RG_IEEE_ADDR_6, ieee_addr[6]);
	spi_write_reg(rf233, RG_IEEE_ADDR_7, ieee_addr[7]);

	k_mutex_unlock(&rf233->phy_mutex);

	return 0;
}

static int rf233_filter(struct device *dev,
			 bool set,
			 enum ieee802154_filter_type type,
			 const struct ieee802154_filter *filter)
{
	LOG_DBG("Applying filter %u", type);

	if (!set) {
		return -ENOTSUP;
	}

	if (type == IEEE802154_FILTER_TYPE_IEEE_ADDR) {
		return rf233_set_ieee_addr(dev, filter->ieee_addr);
	} else if (type == IEEE802154_FILTER_TYPE_SHORT_ADDR) {
		return rf233_set_short_addr(dev, filter->short_addr);
	} else if (type == IEEE802154_FILTER_TYPE_PAN_ID) {
		return rf233_set_pan_id(dev, filter->pan_id);
	}

	return -ENOTSUP;
}

static int rf233_set_txpower(struct device *dev, s16_t dbm)
{
	struct rf233_context *rf233 = dev->driver_data;

	k_mutex_lock(&rf233->phy_mutex, K_FOREVER);

	// TODO
//	spi_write_reg(rf233, RG_PHY_TX_PWR, );

	k_mutex_unlock(&rf233->phy_mutex);

	return 0;
}

static int rf233_tx(struct device *dev,
		     struct net_pkt *pkt,
		     struct net_buf *frag)
{
	struct rf233_context *rf233 = dev->driver_data;

	k_mutex_lock(&rf233->phy_mutex, K_FOREVER);

	LOG_DBG("%p (%u)", frag, frag->len);

	// TODO

	k_mutex_unlock(&rf233->phy_mutex);
	return -EIO;
}

// power up
static int rf233_start(struct device *dev)
{
	struct rf233_context *rf233 = dev->driver_data;

	k_mutex_lock(&rf233->phy_mutex, K_FOREVER);

	// TODO

	k_mutex_unlock(&rf233->phy_mutex);
	return 0;
}

// power down
static int rf233_stop(struct device *dev)
{
	// TODO
	return -EIO;
}

static int power_on_and_setup(struct device *dev)
{
	struct rf233_context *rf233 = dev->driver_data;

	k_mutex_lock(&rf233->phy_mutex, K_FOREVER);

	rf233_hardware_reset(dev);

	// wait until the device is ready
	while (spi_read_reg(rf233, RG_PART_NUM) != AT86RF233_PART_NUM) {
	}

	spi_write_reg(rf233, RG_TRX_STATE, CMD_FORCE_TRX_OFF);	// disable the radio
	spi_write_reg(rf233, RG_IRQ_MASK, AT_IRQ_RX_START| AT_IRQ_TRX_END);
	spi_read_reg(rf233, RG_IRQ_STATUS);			// clear interrupt flags
	spi_write_reg(rf233, RG_ANT_DIV, RADIO_CHIP_ANTENNA);	// use chip antenna
	spi_write_reg(rf233, RG_TRX_CTRL_1, 0x20);		// enable CRC calculation on TX

	// wait until the device switched it's state
	while ((spi_read_reg(rf233, RG_TRX_STATUS) & 0x1F) != TRX_OFF) {
	}

	k_mutex_unlock(&rf233->phy_mutex);

	setup_gpio_callbacks(rf233);

	return 0;
}

static inline u8_t *get_mac(struct device *dev)
{
	struct rf233_context *rf233 = dev->driver_data;

	u32_t *ptr = (u32_t *)(rf233->mac_addr);

	UNALIGNED_PUT(sys_rand32_get(), ptr);
	ptr = (u32_t *)(rf233->mac_addr + 4);
	UNALIGNED_PUT(sys_rand32_get(), ptr);

	rf233->mac_addr[0] = (rf233->mac_addr[0] & ~0x01) | 0x02;

	return rf233->mac_addr;
}

static inline int configure_gpios(struct device *dev)
{
	struct rf233_context *rf233 = dev->driver_data;

	/* setup gpio for the modem interrupt */
	rf233->irq_gpio =
		device_get_binding(DT_ATMEL_AT86RF233_0_IRQ_GPIOS_CONTROLLER);
	if (rf233->irq_gpio == NULL) {
		LOG_ERR("Failed to get pointer to %s device",
			DT_ATMEL_AT86RF233_0_IRQ_GPIOS_CONTROLLER);
		return -EINVAL;
	}

	gpio_pin_configure(rf233->irq_gpio,
			   DT_ATMEL_AT86RF233_0_IRQ_GPIOS_PIN,
			   GPIO_DIR_IN | GPIO_INT | GPIO_INT_EDGE |
			   GPIO_PUD_PULL_UP |
			   GPIO_INT_ACTIVE_LOW);

	/* setup gpio for the modems reset */
	rf233->reset_gpio =
		device_get_binding(DT_ATMEL_AT86RF233_0_RESET_GPIOS_CONTROLLER);
	if (rf233->reset_gpio == NULL) {
		LOG_ERR("Failed to get pointer to %s device",
			DT_ATMEL_AT86RF233_0_RESET_GPIOS_CONTROLLER);
		return -EINVAL;
	}

	gpio_pin_configure(rf233->reset_gpio,
			   DT_ATMEL_AT86RF233_0_RESET_GPIOS_PIN,
			   GPIO_DIR_OUT);

	/* setup gpio for SLP_TR */
	rf233->slp_tr_gpio =
		device_get_binding(DT_ATMEL_AT86RF233_0_SLP_TR_GPIOS_CONTROLLER);
	if (rf233->slp_tr_gpio == NULL) {
		LOG_ERR("Failed to get pointer to %s device",
			DT_ATMEL_AT86RF233_0_SLP_TR_GPIOS_CONTROLLER);
		return -EINVAL;
	}

	gpio_pin_configure(rf233->slp_tr_gpio,
			   DT_ATMEL_AT86RF233_0_SLP_TR_GPIOS_PIN,
			   GPIO_DIR_OUT);

	set_slp_tr(dev, 0);
	set_reset(dev, 1);

	return 0;
}

static inline int configure_spi(struct device *dev)
{
	struct rf233_context *rf233 = dev->driver_data;

	rf233->spi = device_get_binding(DT_ATMEL_AT86RF233_0_BUS_NAME);
	if (!rf233->spi) {
		LOG_ERR("Unable to get SPI device");
		return -ENODEV;
	}

#if defined(DT_ATMEL_AT86RF233_0_CS_GPIO_CONTROLLER)
	rf233->cs_ctrl.gpio_dev = device_get_binding(
		DT_ATMEL_AT86RF233_0_CS_GPIO_CONTROLLER);
	if (!rf233->cs_ctrl.gpio_dev) {
		LOG_ERR("Unable to get GPIO SPI CS device");
		return -ENODEV;
	}

	rf233->cs_ctrl.gpio_pin = DT_ATMEL_AT86RF233_0_CS_GPIO_PIN;
	rf233->cs_ctrl.delay = 0U;

	rf233->spi_cfg.cs = &rf233->cs_ctrl;

	LOG_DBG("SPI GPIO CS configured on %s:%u",
		DT_ATMEL_AT86RF233_0_CS_GPIO_CONTROLLER,
		DT_ATMEL_AT86RF233_0_CS_GPIO_PIN);
#endif /* DT_ATMEL_AT86RF233_0_CS_GPIO_CONTROLLER */

	rf233->spi_cfg.frequency = DT_ATMEL_AT86RF233_0_SPI_MAX_FREQUENCY;
	rf233->spi_cfg.operation = SPI_WORD_SET(8);
	rf233->spi_cfg.slave = DT_ATMEL_AT86RF233_0_BASE_ADDRESS;

	LOG_DBG("SPI configured %s, %d",
		DT_ATMEL_AT86RF233_0_BUS_NAME,
		DT_ATMEL_AT86RF233_0_BASE_ADDRESS);

	return 0;
}

static int rf233_init(struct device *dev)
{
	struct rf233_context *rf233 = dev->driver_data;

	k_mutex_init(&rf233->phy_mutex);
	k_sem_init(&rf233->isr_sem, 0, 1);

	LOG_DBG("\nInitialize RF233 Transceiver\n");

	if (configure_gpios(dev) != 0) {
		LOG_ERR("Configuring GPIOS failed");
		return -EIO;
	}

	if (configure_spi(dev) != 0) {
		LOG_ERR("Configuring SPI failed");
		return -EIO;
	}

	LOG_DBG("GPIO and SPI configured");

	if (power_on_and_setup(dev) != 0) {
		LOG_ERR("Configuring RF233 failed");
		return -EIO;
	}

//	k_thread_create(&rf233->rf233_rx_thread, rf233->rf233_rx_stack,
//			CONFIG_IEEE802154_AT86RF233_RX_STACK_SIZE,
//			(k_thread_entry_t)rf233_thread_main,
//			dev, NULL, NULL, K_PRIO_COOP(2), 0, 0);

	return 0;
}

static void rf233_iface_init(struct net_if *iface)
{
	struct device *dev = net_if_get_device(iface);
	struct rf233_context *rf233 = dev->driver_data;
	u8_t *mac = get_mac(dev);

	net_if_set_link_addr(iface, mac, 8, NET_LINK_IEEE802154);
	rf233->iface = iface;
	ieee802154_init(iface);
}

static struct rf233_context rf233_context_data;

static struct ieee802154_radio_api rf233_radio_api = {
	.iface_api.init	= rf233_iface_init,

	.get_capabilities	= rf233_get_capabilities,
	.cca			= rf233_cca,
	.set_channel		= rf233_set_channel,
	.filter			= rf233_filter,
	.set_txpower		= rf233_set_txpower,
	.start			= rf233_start,
	.stop			= rf233_stop,
	.tx			= rf233_tx,
};

#if defined(CONFIG_IEEE802154_RAW_MODE)
DEVICE_AND_API_INIT(rf233, CONFIG_IEEE802154_AT86RF233_DRV_NAME,
		    rf233_init, &rf233_context_data, NULL,
		    POST_KERNEL, CONFIG_IEEE802154_AR86RF233_INIT_PRIO,
		    &rf233_radio_api);
#else
NET_DEVICE_INIT(rf233, CONFIG_IEEE802154_AT86RF233_DRV_NAME,
		rf233_init, &rf233_context_data, NULL,
		CONFIG_IEEE802154_AT86RF233_INIT_PRIO,
		&rf233_radio_api, IEEE802154_L2,
		NET_L2_GET_CTX_TYPE(IEEE802154_L2),
		AT86RF233_PSDU_LENGTH);
#endif
