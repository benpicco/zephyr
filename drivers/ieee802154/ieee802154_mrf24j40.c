/* ieee802154_mrf24j40.c - Microchip MRF24J40 driver */

/*
 * Copyright (c) 2019 ML!PA Consulting GmbH
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define LOG_MODULE_NAME ieee802154_mrf24j40
#define LOG_LEVEL CONFIG_IEEE802154_DRIVER_LOG_LEVEL

#define MRF24J40_PSDU_LENGTH		(125)

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

#include "ieee802154_mrf24j40.h"
#include "ieee802154_mrf24j40_regs.h"

static inline void irq_int_handler(struct device *port,
				   struct gpio_callback *cb, u32_t pins)
{
	struct mrf24j40_context *mrf24j40 = CONTAINER_OF(cb,
						     struct mrf24j40_context,
						     irq_cb);
	k_sem_give(&mrf24j40->isr_sem);
}

static inline void set_reset(struct device *dev, u32_t value)
{
	struct mrf24j40_context *mrf24j40 = dev->driver_data;

	gpio_pin_write(mrf24j40->reset_gpio,
			DT_MICROCHIP_MRF24J40_0_RESET_GPIOS_PIN, value);
}

static inline void set_slp_tr(struct device *dev, u32_t value)
{
	struct mrf24j40_context *mrf24j40 = dev->driver_data;

	gpio_pin_write(mrf24j40->reset_gpio,
			DT_MICROCHIP_MRF24J40_0_WAKE_GPIOS_PIN, value);
}

static void enable_irq_interrupt(struct mrf24j40_context *mrf24j40,
				 bool enable)
{
	if (enable) {
		gpio_pin_enable_callback(mrf24j40->irq_gpio,
					 DT_MICROCHIP_MRF24J40_0_IRQ_GPIOS_PIN);
	} else {
		gpio_pin_disable_callback(mrf24j40->irq_gpio,
					  DT_MICROCHIP_MRF24J40_0_IRQ_GPIOS_PIN);
	}
}

static inline void setup_gpio_callbacks(struct mrf24j40_context *mrf24j40)
{
	gpio_init_callback(&mrf24j40->irq_cb,
			   irq_int_handler,
			   BIT(DT_MICROCHIP_MRF24J40_0_IRQ_GPIOS_PIN));
	gpio_add_callback(mrf24j40->irq_gpio, &mrf24j40->irq_cb);
}

static enum ieee802154_hw_caps mrf24j40_get_capabilities(struct device *dev)
{
	return    IEEE802154_HW_FCS
		| IEEE802154_HW_2_4_GHZ
		| IEEE802154_HW_TX_RX_ACK
		| IEEE802154_HW_FILTER;
}

static void mrf24j40_hardware_reset(struct device *dev)
{
	set_reset(dev, 0);
	k_busy_wait(20000U);
	set_reset(dev, 1);
	k_busy_wait(2000U);
}

static u8_t mrf24j40_read_reg_short(struct mrf24j40_context *dev, u8_t addr)
{
	u8_t cmd_buf[2] = {
		(addr << MRF24J40_ADDR_OFFSET) | MRF24J40_SHORT_ADDR_TRANS | MRF24J40_ACCESS_READ,
		0
	};
	const struct spi_buf buf = {
		.buf = cmd_buf,
		.len = sizeof(cmd_buf)
	};
	const struct spi_buf_set tx = {
		.buffers = &buf,
		.count = 1
	};
	const struct spi_buf_set rx = {
		.buffers = &buf,
		.count = 1
	};

	if (spi_transceive(dev->spi, &dev->spi_cfg, &tx, &rx) == 0) {
		return cmd_buf[1];
	}

	LOG_ERR("Failed");

	return 0;
}

static bool mrf24j40_write_reg_short(struct mrf24j40_context *dev, u8_t addr, u8_t value)
{
	u8_t cmd_buf[2] = {
		(addr << MRF24J40_ADDR_OFFSET) | MRF24J40_SHORT_ADDR_TRANS | MRF24J40_ACCESS_WRITE,
		value
	};
	const struct spi_buf buf = {
		.buf = cmd_buf,
		.len = sizeof(cmd_buf)
	};
	const struct spi_buf_set tx = {
		.buffers = &buf,
		.count = 1
	};

	return spi_write(dev->spi, &dev->spi_cfg, &tx) == 0;
}

static u8_t mrf24j40_read_reg_long(struct mrf24j40_context *dev, u16_t addr)
{
	u8_t cmd_buf[3] = {
		(addr >> 3) | MRF24J40_LONG_ADDR_TRANS,
		(addr << 5) | MRF24J40_ACCESS_READ,
		0
	};
	const struct spi_buf buf = {
		.buf = cmd_buf,
		.len = sizeof(cmd_buf)
	};
	const struct spi_buf_set tx = {
		.buffers = &buf,
		.count = 1
	};
	const struct spi_buf_set rx = {
		.buffers = &buf,
		.count = 1
	};

	if (spi_transceive(dev->spi, &dev->spi_cfg, &tx, &rx) == 0) {
		return cmd_buf[2];
	}

	LOG_ERR("Failed");

	return 0;
}

static bool mrf24j40_write_reg_long(struct mrf24j40_context *dev, u16_t addr, u8_t value)
{
	u8_t cmd_buf[3] = {
		(addr >> 3) | MRF24J40_LONG_ADDR_TRANS,
		(addr << 5) | MRF24J40_ACCESS_WRITE_LNG,
		value
	};
	const struct spi_buf buf = {
		.buf = cmd_buf,
		.len = sizeof(cmd_buf)
	};
	const struct spi_buf_set tx = {
		.buffers = &buf,
		.count = 1
	};

	return spi_write(dev->spi, &dev->spi_cfg, &tx) == 0;
}

static void mrf24j40_state_machine_reset(struct mrf24j40_context *mrf24j40) {
	mrf24j40_write_reg_short(mrf24j40, MRF24J40_REG_RFCTL, MRF24J40_RFCTL_RFRST);
	mrf24j40_write_reg_short(mrf24j40, MRF24J40_REG_RFCTL, 0);

	k_busy_wait(192);
	while ((mrf24j40_read_reg_long(mrf24j40, MRF24J40_REG_RFSTATE) & MRF24J40_RFSTATE_MASK)
	       != MRF24J40_RFSTATE_RX) {}
}

/* Note: CCA before TX is enabled by default */
static int mrf24j40_cca(struct device *dev)
{
	struct mrf24j40_context *mrf24j40 = dev->driver_data;
	uint8_t ccaedth, rssi;

	k_mutex_lock(&mrf24j40->phy_mutex, K_FOREVER);

	mrf24j40_write_reg_short(mrf24j40, MRF24J40_REG_BBREG6, MRF24J40_BBREG6_RSSIMODE1);

	while (!(mrf24j40_read_reg_short(mrf24j40, MRF24J40_REG_BBREG6) & MRF24J40_BBREG2_RSSIRDY)) {}

	ccaedth = mrf24j40_read_reg_short(mrf24j40, MRF24J40_REG_CCAEDTH);
	rssi    = mrf24j40_read_reg_long(mrf24j40, MRF24J40_REG_RSSI);

	k_mutex_unlock(&mrf24j40->phy_mutex);

	return rssi < ccaedth ? 0 : -EBUSY;
}

static int mrf24j40_set_channel(struct device *dev, u16_t channel)
{
	struct mrf24j40_context *mrf24j40 = dev->driver_data;

	if (channel < 11 || channel > 26) {
		return -EINVAL;
	}

	k_mutex_lock(&mrf24j40->phy_mutex, K_FOREVER);

	mrf24j40_write_reg_long(mrf24j40, MRF24J40_REG_RFCON0, 16 * (channel - 11) + 3);

	mrf24j40_state_machine_reset(mrf24j40);

	k_mutex_unlock(&mrf24j40->phy_mutex);

	return 0;
}

static int mrf24j40_set_pan_id(struct device *dev, u16_t pan_id)
{
	struct mrf24j40_context *mrf24j40 = dev->driver_data;

	pan_id = sys_le16_to_cpu(pan_id);
	k_mutex_lock(&mrf24j40->phy_mutex, K_FOREVER);

	mrf24j40_write_reg_short(mrf24j40, MRF24J40_REG_PANIDL, pan_id & 0xFF);
	mrf24j40_write_reg_short(mrf24j40, MRF24J40_REG_PANIDH, pan_id >> 8);

	k_mutex_unlock(&mrf24j40->phy_mutex);

	return 0;
}

static int mrf24j40_set_short_addr(struct device *dev, u16_t short_addr)
{
	struct mrf24j40_context *mrf24j40 = dev->driver_data;

	short_addr = sys_le16_to_cpu(short_addr);
	k_mutex_lock(&mrf24j40->phy_mutex, K_FOREVER);

	mrf24j40_write_reg_short(mrf24j40, MRF24J40_REG_SADRL, short_addr & 0xFF);
	mrf24j40_write_reg_short(mrf24j40, MRF24J40_REG_SADRH, short_addr >> 8);

	k_mutex_unlock(&mrf24j40->phy_mutex);

	return 0;
}

static int mrf24j40_set_ieee_addr(struct device *dev, const u8_t *ieee_addr)
{
	struct mrf24j40_context *mrf24j40 = dev->driver_data;

	k_mutex_lock(&mrf24j40->phy_mutex, K_FOREVER);

	for (int i = 0; i < 8; ++i) {
		mrf24j40_write_reg_short(mrf24j40, MRF24J40_REG_EADR0 + i, ieee_addr[i]);
	}

	k_mutex_unlock(&mrf24j40->phy_mutex);

	return 0;
}

static int mrf24j40_filter(struct device *dev,
			 bool set,
			 enum ieee802154_filter_type type,
			 const struct ieee802154_filter *filter)
{
	LOG_DBG("Applying filter %u", type);

	if (!set) {
		return -ENOTSUP;
	}

	if (type == IEEE802154_FILTER_TYPE_IEEE_ADDR) {
		return mrf24j40_set_ieee_addr(dev, filter->ieee_addr);
	} else if (type == IEEE802154_FILTER_TYPE_SHORT_ADDR) {
		return mrf24j40_set_short_addr(dev, filter->short_addr);
	} else if (type == IEEE802154_FILTER_TYPE_PAN_ID) {
		return mrf24j40_set_pan_id(dev, filter->pan_id);
	}

	return -ENOTSUP;
}

static int mrf24j40_set_txpower(struct device *dev, s16_t dbm)
{
	struct mrf24j40_context *mrf24j40 = dev->driver_data;

	k_mutex_lock(&mrf24j40->phy_mutex, K_FOREVER);

	// TODO

	k_mutex_unlock(&mrf24j40->phy_mutex);

	return 0;
}

static int mrf24j40_tx(struct device *dev,
		     struct net_pkt *pkt,
		     struct net_buf *frag)
{
	struct mrf24j40_context *mrf24j40 = dev->driver_data;

	k_mutex_lock(&mrf24j40->phy_mutex, K_FOREVER);

	LOG_DBG("%p (%u)", frag, frag->len);

	// TODO

	k_mutex_unlock(&mrf24j40->phy_mutex);
	return -EIO;
}

// power up
static int mrf24j40_start(struct device *dev)
{
	struct mrf24j40_context *mrf24j40 = dev->driver_data;

	k_mutex_lock(&mrf24j40->phy_mutex, K_FOREVER);

	mrf24j40_write_reg_short(mrf24j40, MRF24J40_REG_WAKECON,
				 MRF24J40_WAKECON_IMMWAKE | MRF24J40_WAKECON_REGWAKE);

	k_busy_wait(50);
	mrf24j40_state_machine_reset(mrf24j40);

	/* clear interrupts */
	mrf24j40_read_reg_short(mrf24j40, MRF24J40_REG_INTSTAT);

	k_mutex_unlock(&mrf24j40->phy_mutex);
	return 0;
}

// power down
static int mrf24j40_stop(struct device *dev)
{
	struct mrf24j40_context *mrf24j40 = dev->driver_data;

	k_mutex_lock(&mrf24j40->phy_mutex, K_FOREVER);

	mrf24j40_write_reg_short(mrf24j40, MRF24J40_REG_SOFTRST, MRF24J40_SOFTRST_RSTPWR);
	mrf24j40_write_reg_short(mrf24j40, MRF24J40_REG_SLPACK, MRF24J40_SLPACK_SLPACK);

	k_mutex_unlock(&mrf24j40->phy_mutex);

	return 0;
}

static int power_on_and_setup(struct device *dev)
{
	struct mrf24j40_context *mrf24j40 = dev->driver_data;

	mrf24j40_hardware_reset(dev);

	/* do a soft reset */
	mrf24j40_write_reg_short(mrf24j40, MRF24J40_REG_SOFTRST, MRF24J40_SOFTRST_RSTPWR |
								 MRF24J40_SOFTRST_RSTBB  |
								 MRF24J40_SOFTRST_RSTMAC);
	/* flush RX FIFO */
	mrf24j40_write_reg_short(mrf24j40, MRF24J40_REG_RXFLUSH, MRF24J40_RXFLUSH_RXFLUSH);

	/* Here starts init-process as described on MRF24J40 Manual Chap. 3.2 */
	mrf24j40_write_reg_short(mrf24j40, MRF24J40_REG_PACON2, MRF24J40_PACON2_TXONTS2 |
								MRF24J40_PACON2_TXONTS1 |
								MRF24J40_PACON2_FIFOEN);
	mrf24j40_write_reg_short(mrf24j40, MRF24J40_REG_TXSTBL, MRF24J40_TXSTBL_RFSTBL3 |
								MRF24J40_TXSTBL_RFSTBL0 |
								MRF24J40_TXSTBL_MSIFS2  |
								MRF24J40_TXSTBL_MSIFS0);
	mrf24j40_write_reg_long(mrf24j40, MRF24J40_REG_RFCON1, MRF24J40_RFCON1_VCOOPT1);
	mrf24j40_write_reg_long(mrf24j40, MRF24J40_REG_RFCON2, MRF24J40_RFCON2_PLLEN);
	mrf24j40_write_reg_long(mrf24j40, MRF24J40_REG_RFCON6, MRF24J40_RFCON6_TXFIL |
							       MRF24J40_RFCON6_20MRECVR);
	mrf24j40_write_reg_long(mrf24j40, MRF24J40_REG_RFCON7, MRF24J40_RFCON7_SLPCLKSEL1);
	mrf24j40_write_reg_long(mrf24j40, MRF24J40_REG_RFCON8, MRF24J40_RFCON8_RFVCO);
	mrf24j40_write_reg_long(mrf24j40, MRF24J40_REG_SLPCON1, MRF24J40_SLPCON1_CLKOUTEN |
								MRF24J40_SLPCON1_SLPCLKDIV0);
	mrf24j40_write_reg_short(mrf24j40, MRF24J40_REG_BBREG2, MRF25J40_BBREG2_CCAMODE1);
	mrf24j40_write_reg_short(mrf24j40, MRF24J40_REG_CCAEDTH, 0x60);
	mrf24j40_write_reg_short(mrf24j40, MRF24J40_REG_BBREG6, MRF24J40_BBREG6_RSSIMODE2);

	/* Enable immediate sleep mode */
	mrf24j40_write_reg_short(mrf24j40, MRF24J40_REG_WAKECON, MRF24J40_WAKECON_IMMWAKE);

	/* set interrupt pin polarity, rising edge */
	mrf24j40_write_reg_long(mrf24j40, MRF24J40_REG_SLPCON0, MRF24J40_SLPCON0_INTEDGE);

	mrf24j40_state_machine_reset(mrf24j40);

	/* clear interrupts */
	mrf24j40_read_reg_short(mrf24j40, MRF24J40_REG_INTSTAT);

	setup_gpio_callbacks(mrf24j40);

	/* set interrupt sources */
	mrf24j40_write_reg_short(mrf24j40, MRF24J40_REG_INTCON,
				~(MRF24J40_INTCON_RXIE | MRF24J40_INTCON_TXNIE));

	return 0;
}

static inline u8_t *get_mac(struct device *dev)
{
	struct mrf24j40_context *mrf24j40 = dev->driver_data;

	u32_t *ptr = (u32_t *)(mrf24j40->mac_addr);

	UNALIGNED_PUT(sys_rand32_get(), ptr);
	ptr = (u32_t *)(mrf24j40->mac_addr + 4);
	UNALIGNED_PUT(sys_rand32_get(), ptr);

	mrf24j40->mac_addr[0] = (mrf24j40->mac_addr[0] & ~0x01) | 0x02;

	return mrf24j40->mac_addr;
}

static inline int configure_gpios(struct device *dev)
{
	struct mrf24j40_context *mrf24j40 = dev->driver_data;

	/* setup gpio for the modem interrupt */
	mrf24j40->irq_gpio =
		device_get_binding(DT_MICROCHIP_MRF24J40_0_IRQ_GPIOS_CONTROLLER);
	if (mrf24j40->irq_gpio == NULL) {
		LOG_ERR("Failed to get pointer to %s device",
			DT_MICROCHIP_MRF24J40_0_IRQ_GPIOS_CONTROLLER);
		return -EINVAL;
	}

	gpio_pin_configure(mrf24j40->irq_gpio,
			   DT_MICROCHIP_MRF24J40_0_IRQ_GPIOS_PIN,
			   GPIO_DIR_IN | GPIO_INT | GPIO_INT_EDGE |
			   GPIO_PUD_PULL_UP |
			   GPIO_INT_ACTIVE_LOW);

	/* setup gpio for the modems reset */
	mrf24j40->reset_gpio =
		device_get_binding(DT_MICROCHIP_MRF24J40_0_RESET_GPIOS_CONTROLLER);
	if (mrf24j40->reset_gpio == NULL) {
		LOG_ERR("Failed to get pointer to %s device",
			DT_MICROCHIP_MRF24J40_0_RESET_GPIOS_CONTROLLER);
		return -EINVAL;
	}

	gpio_pin_configure(mrf24j40->reset_gpio,
			   DT_MICROCHIP_MRF24J40_0_RESET_GPIOS_PIN,
			   GPIO_DIR_OUT);
	set_reset(dev, 0);

	/* setup gpio for WAKE */
	mrf24j40->wake_gpio =
		device_get_binding(DT_MICROCHIP_MRF24J40_0_WAKE_GPIOS_CONTROLLER);
	if (mrf24j40->wake_gpio == NULL) {
		LOG_ERR("Failed to get pointer to %s device",
			DT_MICROCHIP_MRF24J40_0_WAKE_GPIOS_CONTROLLER);
		return -EINVAL;
	}

	gpio_pin_configure(mrf24j40->wake_gpio,
			   DT_MICROCHIP_MRF24J40_0_WAKE_GPIOS_PIN,
			   GPIO_DIR_OUT);
	return 0;
}

static inline int configure_spi(struct device *dev)
{
	struct mrf24j40_context *mrf24j40 = dev->driver_data;

	mrf24j40->spi = device_get_binding(DT_MICROCHIP_MRF24J40_0_BUS_NAME);
	if (!mrf24j40->spi) {
		LOG_ERR("Unable to get SPI device");
		return -ENODEV;
	}

#if defined(DT_MICROCHIP_MRF24J40_0_CS_GPIO_CONTROLLER)
	mrf24j40->cs_ctrl.gpio_dev = device_get_binding(
		DT_MICROCHIP_MRF24J40_0_CS_GPIO_CONTROLLER);
	if (!mrf24j40->cs_ctrl.gpio_dev) {
		LOG_ERR("Unable to get GPIO SPI CS device");
		return -ENODEV;
	}

	mrf24j40->cs_ctrl.gpio_pin = DT_MICROCHIP_MRF24J40_0_CS_GPIO_PIN;
	mrf24j40->cs_ctrl.delay = 0U;

	mrf24j40->spi_cfg.cs = &mrf24j40->cs_ctrl;

	LOG_DBG("SPI GPIO CS configured on %s:%u",
		DT_MICROCHIP_MRF24J40_0_CS_GPIO_CONTROLLER,
		DT_MICROCHIP_MRF24J40_0_CS_GPIO_PIN);
#endif /* DT_MICROCHIP_MRF24J40_0_CS_GPIO_CONTROLLER */

	mrf24j40->spi_cfg.frequency = DT_MICROCHIP_MRF24J40_0_SPI_MAX_FREQUENCY;
	mrf24j40->spi_cfg.operation = SPI_WORD_SET(8);
	mrf24j40->spi_cfg.slave = DT_MICROCHIP_MRF24J40_0_BASE_ADDRESS;

	LOG_DBG("SPI configured %s, %d",
		DT_MICROCHIP_MRF24J40_0_BUS_NAME,
		DT_MICROCHIP_MRF24J40_0_BASE_ADDRESS);

	return 0;
}

static int mrf24j40_init(struct device *dev)
{
	struct mrf24j40_context *mrf24j40 = dev->driver_data;

	k_mutex_init(&mrf24j40->phy_mutex);
	k_sem_init(&mrf24j40->isr_sem, 0, 1);

	LOG_DBG("\nInitialize MRF24J40 Transceiver\n");

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
		LOG_ERR("Configuring MRF24J40 failed");
		return -EIO;
	}

//	k_thread_create(&mrf24j40->mrf24j40_rx_thread, mrf24j40->mrf24j40_rx_stack,
//			CONFIG_IEEE802154_MRF24J40_RX_STACK_SIZE,
//			(k_thread_entry_t)mrf24j40_thread_main,
//			dev, NULL, NULL, K_PRIO_COOP(2), 0, 0);

	return 0;
}

static void mrf24j40_iface_init(struct net_if *iface)
{
	struct device *dev = net_if_get_device(iface);
	struct mrf24j40_context *mrf24j40 = dev->driver_data;
	u8_t *mac = get_mac(dev);

	net_if_set_link_addr(iface, mac, 8, NET_LINK_IEEE802154);
	mrf24j40->iface = iface;
	ieee802154_init(iface);
}

static struct mrf24j40_context mrf24j40_context_data;

static struct ieee802154_radio_api mrf24j40_radio_api = {
	.iface_api.init	= mrf24j40_iface_init,

	.get_capabilities	= mrf24j40_get_capabilities,
	.cca			= mrf24j40_cca,
	.set_channel		= mrf24j40_set_channel,
	.filter			= mrf24j40_filter,
	.set_txpower		= mrf24j40_set_txpower,
	.start			= mrf24j40_start,
	.stop			= mrf24j40_stop,
	.tx			= mrf24j40_tx,
};

#if defined(CONFIG_IEEE802154_RAW_MODE)
DEVICE_AND_API_INIT(mrf24j40, CONFIG_IEEE802154_MRF24J40_DRV_NAME,
		    mrf24j40_init, &mrf24j40_context_data, NULL,
		    POST_KERNEL, CONFIG_IEEE802154_MRF24J40_INIT_PRIO,
		    &mrf24j40_radio_api);
#else
NET_DEVICE_INIT(mrf24j40, CONFIG_IEEE802154_MRF24J40_DRV_NAME,
		mrf24j40_init, &mrf24j40_context_data, NULL,
		CONFIG_IEEE802154_MRF24J40_INIT_PRIO,
		&mrf24j40_radio_api, IEEE802154_L2,
		NET_L2_GET_CTX_TYPE(IEEE802154_L2),
		MRF24J40_PSDU_LENGTH);
#endif
