/* IEEE 802.15.4 driver for the Microchip MRF24J40 */

#define DT_DRV_COMPAT microchip_mrf24j40

/*
 * Copyright (c) 2026
 * SPDX-License-Identifier: Apache-2.0
 */

#define LOG_MODULE_NAME ieee802154_mrf24j40
#define LOG_LEVEL CONFIG_IEEE802154_DRIVER_LOG_LEVEL

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(LOG_MODULE_NAME);

#include <errno.h>
#include <string.h>

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/init.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/net/ieee802154_radio.h>
#include <zephyr/net/net_if.h>
#include <zephyr/net/net_pkt.h>
#include <zephyr/net/ieee802154_pkt.h>
#include <zephyr/random/random.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/util.h>

#if defined(CONFIG_NET_L2_OPENTHREAD)
#include <net/openthread.h>
#endif

#define MRF24J40_TX_FIFO		0x000
#define MRF24J40_RX_FIFO		0x300
#define MRF24J40_MAX_PSDU		127
#define MRF24J40_FCS_LEN		2
#define MRF24J40_CHANNEL_MIN		11
#define MRF24J40_CHANNEL_MAX		26

#define MRF24J40_RXMCR			0x00
#define MRF24J40_PANIDL			0x01
#define MRF24J40_SADRL			0x03
#define MRF24J40_EADR0			0x05
#define MRF24J40_RXFLUSH		0x0d
#define MRF24J40_TXMCR			0x11
#define MRF24J40_TXNCON			0x1b
#define MRF24J40_TXSTAT			0x24
#define MRF24J40_SOFTRST		0x2a
#define MRF24J40_INTSTAT		0x31
#define MRF24J40_INTCON			0x32
#define MRF24J40_BBREG2			0x3a
#define MRF24J40_BBREG6			0x3e
#define MRF24J40_CCAEDTH		0x3f

#define MRF24J40_RFCON0			0x200
#define MRF24J40_RFCON1			0x201
#define MRF24J40_RFCON2			0x202
#define MRF24J40_RFCON3			0x203
#define MRF24J40_RFCON6			0x206
#define MRF24J40_RFCON7			0x207
#define MRF24J40_RFCON8			0x208
#define MRF24J40_RSSI			0x210
#define MRF24J40_SLPCON0		0x211
#define MRF24J40_SLPCON1		0x220
#define MRF24J40_TESTMODE		0x22f
#define MRF24J40_RFCTL			0x36
#define MRF24J40_WAKECON		0x22
#define MRF24J40_SLPACK			0x35

#define MRF24J40_INT_RX			BIT(3)
#define MRF24J40_INT_TX			BIT(0)
#define MRF24J40_TXNCON_ACKREQ		BIT(2)
#define MRF24J40_TXNCON_TRIGGER		BIT(0)
#define MRF24J40_TXSTAT_FAILED		BIT(0)
#define MRF24J40_TXSTAT_CCAFAIL		BIT(5)
#define MRF24J40_RXMCR_PROMISC		BIT(0)
#define MRF24J40_RXMCR_NOACK		BIT(5)
#define MRF24J40_TXMCR_NO_CSMA		BIT(7)
#define MRF24J40_BBREG6_RSSI_MODE1	BIT(7)
#define MRF24J40_BBREG6_RSSI_MODE2	BIT(6)
#define MRF24J40_BBREG2_RSSI_READY	BIT(0)

IEEE802154_DEFINE_PHY_SUPPORTED_CHANNELS(mrf24j40_channels, 11, 26);

struct mrf24j40_config {
	struct spi_dt_spec spi;
	struct gpio_dt_spec irq;
	struct gpio_dt_spec reset;
	bool has_mac;
	uint8_t mac[8];
};

struct mrf24j40_data {
	struct net_if *iface;
	struct gpio_callback irq_cb;
	struct k_mutex lock;
	struct k_sem irq_sem;
	struct k_sem tx_sem;
	K_KERNEL_STACK_MEMBER(rx_stack,
			     CONFIG_IEEE802154_MRF24J40_RX_STACK_SIZE);
	struct k_thread rx_thread;
	uint8_t mac[8];
	int tx_result;
	bool tx_pending;
	bool tx_ack_requested;
	bool started;
};

static int mrf24j40_read(const struct device *dev, bool long_addr,
			 uint16_t addr, uint8_t *value)
{
	const struct mrf24j40_config *cfg = dev->config;
	uint8_t cmd[3];
	uint8_t rx[3] = { 0 };
	size_t len;
	struct spi_buf tx_buf = { .buf = cmd };
	struct spi_buf rx_buf = { .buf = rx };
	struct spi_buf_set tx = { .buffers = &tx_buf, .count = 1 };
	struct spi_buf_set rxs = { .buffers = &rx_buf, .count = 1 };

	if (long_addr) {
		cmd[0] = 0x80 | (addr >> 3);
		cmd[1] = addr << 5;
		len = 3;
	} else {
		cmd[0] = addr << 1;
		len = 2;
	}

	tx_buf.len = len;
	rx_buf.len = len;
	if (spi_transceive_dt(&cfg->spi, &tx, &rxs) != 0) {
		return -EIO;
	}

	*value = rx[len - 1];
	return 0;
}

static int mrf24j40_write(const struct device *dev, bool long_addr,
			  uint16_t addr, uint8_t value)
{
	const struct mrf24j40_config *cfg = dev->config;
	uint8_t cmd[3];
	struct spi_buf tx_buf = { .buf = cmd };
	struct spi_buf_set tx = { .buffers = &tx_buf, .count = 1 };

	if (long_addr) {
		cmd[0] = 0x80 | (addr >> 3);
		cmd[1] = (addr << 5) | 0x10;
		cmd[2] = value;
		tx_buf.len = 3;
	} else {
		cmd[0] = (addr << 1) | 0x01;
		cmd[1] = value;
		tx_buf.len = 2;
	}

	return spi_write_dt(&cfg->spi, &tx) == 0 ? 0 : -EIO;
}

static int mrf24j40_write_long_burst(const struct device *dev, uint16_t addr,
				     const uint8_t *data, size_t len)
{
	const struct mrf24j40_config *cfg = dev->config;
	uint8_t cmd[2] = { 0x80 | (addr >> 3), (addr << 5) | 0x10 };
	struct spi_buf bufs[] = {
		{ .buf = cmd, .len = sizeof(cmd) },
		{ .buf = (void *)data, .len = len },
	};
	struct spi_buf_set tx = { .buffers = bufs, .count = ARRAY_SIZE(bufs) };

	return spi_write_dt(&cfg->spi, &tx) == 0 ? 0 : -EIO;
}

static int mrf24j40_read_long_burst(const struct device *dev, uint16_t addr,
				    uint8_t *data, size_t len)
{
	const struct mrf24j40_config *cfg = dev->config;
	uint8_t cmd[2] = { 0x80 | (addr >> 3), addr << 5 };
	struct spi_buf bufs[] = {
		{ .buf = cmd, .len = sizeof(cmd) },
		{ .buf = data, .len = len },
	};
	struct spi_buf_set tx = { .buffers = bufs, .count = ARRAY_SIZE(bufs) };
	struct spi_buf_set rx = { .buffers = bufs, .count = ARRAY_SIZE(bufs) };

	return spi_transceive_dt(&cfg->spi, &tx, &rx) == 0 ? 0 : -EIO;
}

static int mrf24j40_write_checked(const struct device *dev, bool long_addr,
				  uint16_t addr, uint8_t value)
{
	int ret = mrf24j40_write(dev, long_addr, addr, value);

	if (ret != 0) {
		LOG_ERR("SPI register write failed (0x%x)", addr);
	}
	return ret;
}

static int mrf24j40_set_channel_locked(const struct device *dev, uint16_t channel)
{
	uint8_t rfcon0;

	if (channel < MRF24J40_CHANNEL_MIN || channel > MRF24J40_CHANNEL_MAX) {
		return -EINVAL;
	}

	rfcon0 = ((channel - MRF24J40_CHANNEL_MIN) << 4) | 0x03;
	if (mrf24j40_write_checked(dev, true, MRF24J40_RFCON0, rfcon0) != 0 ||
	    mrf24j40_write_checked(dev, false, MRF24J40_RFCTL, BIT(2)) != 0 ||
	    mrf24j40_write_checked(dev, false, MRF24J40_RFCTL, 0) != 0) {
		return -EIO;
	}
	k_busy_wait(200);
	return 0;
}

static int mrf24j40_set_channel(const struct device *dev, uint16_t channel)
{
	struct mrf24j40_data *data = dev->data;
	int ret;

	k_mutex_lock(&data->lock, K_FOREVER);
	ret = mrf24j40_set_channel_locked(dev, channel);
	k_mutex_unlock(&data->lock);
	return ret;
}

static int mrf24j40_cca_locked(const struct device *dev)
{
	uint8_t reg;
	uint8_t rssi;
	int ret;
	int timeout = 100;

	ret = mrf24j40_read(dev, false, MRF24J40_BBREG6, &reg);
	if (ret != 0) {
		return ret;
	}
	ret = mrf24j40_write(dev, false, MRF24J40_BBREG6,
			     (reg & ~MRF24J40_BBREG6_RSSI_MODE2) |
			     MRF24J40_BBREG6_RSSI_MODE1);
	if (ret != 0) {
		return ret;
	}

	do {
		ret = mrf24j40_read(dev, false, MRF24J40_BBREG2, &reg);
		if (ret != 0) {
			goto restore;
		}
		if (reg & MRF24J40_BBREG2_RSSI_READY) {
			break;
		}
		k_busy_wait(10);
	} while (--timeout > 0);

	if (timeout == 0) {
		ret = -ETIMEDOUT;
		goto restore;
	}

	ret = mrf24j40_read(dev, true, MRF24J40_RSSI, &rssi);
	if (ret == 0) {
		ret = mrf24j40_read(dev, false, MRF24J40_CCAEDTH, &reg);
	}
	if (ret != 0) {
		goto restore;
	}
	ret = rssi < reg ? 0 : -EBUSY;

restore:
	if (mrf24j40_write(dev, false, MRF24J40_BBREG6,
			   MRF24J40_BBREG6_RSSI_MODE2) != 0) {
		return -EIO;
	}
	return ret;
}

static int mrf24j40_cca(const struct device *dev)
{
	struct mrf24j40_data *data = dev->data;
	int ret;

	k_mutex_lock(&data->lock, K_FOREVER);
	ret = data->started ? mrf24j40_cca_locked(dev) : -ENETDOWN;
	k_mutex_unlock(&data->lock);
	return ret;
}

static int mrf24j40_set_txpower(const struct device *dev, int16_t dbm)
{
	static const uint8_t power_reg[] = {
		0x00, 0x10, 0x18, 0x20, 0x28, 0x30, 0x38, 0x38, 0x38,
		0x40, 0x40, 0x50, 0x58, 0x60, 0x68, 0x70, 0x78, 0x78,
		0x78, 0x80, 0x80, 0x90, 0x98, 0xa0, 0xa8, 0xb0, 0xb8,
		0xb8, 0xb8, 0xc0, 0xc0, 0xd0, 0xd8, 0xe0, 0xe8, 0xf0,
		0xf8,
	};
	struct mrf24j40_data *data = dev->data;
	int ret;

	if (dbm > 0 || dbm < -36) {
		return -EINVAL;
	}

	k_mutex_lock(&data->lock, K_FOREVER);
	ret = mrf24j40_write(dev, true, MRF24J40_RFCON3, power_reg[-dbm]);
	k_mutex_unlock(&data->lock);
	return ret;
}

static int mrf24j40_filter(const struct device *dev, bool set,
			   enum ieee802154_filter_type type,
			   const struct ieee802154_filter *filter)
{
	struct mrf24j40_data *data = dev->data;
	uint8_t bytes[8];
	uint16_t value;
	uint16_t address;
	int ret = 0;

	k_mutex_lock(&data->lock, K_FOREVER);
	switch (type) {
	case IEEE802154_FILTER_TYPE_IEEE_ADDR:
		for (size_t i = 0; i < 8; i++) {
			ret = mrf24j40_write(dev, false, MRF24J40_EADR0 + i,
					     set ? filter->ieee_addr[i] : 0);
			if (ret != 0) {
				break;
			}
		}
		break;
	case IEEE802154_FILTER_TYPE_SHORT_ADDR:
		address = MRF24J40_SADRL;
		value = set ? filter->short_addr : 0xffff;
		sys_put_le16(value, bytes);
		ret = mrf24j40_write(dev, false, address, bytes[0]);
		if (ret == 0) {
			ret = mrf24j40_write(dev, false, address + 1, bytes[1]);
		}
		break;
	case IEEE802154_FILTER_TYPE_PAN_ID:
		address = MRF24J40_PANIDL;
		value = set ? filter->pan_id : 0xffff;
		sys_put_le16(value, bytes);
		ret = mrf24j40_write(dev, false, address, bytes[0]);
		if (ret == 0) {
			ret = mrf24j40_write(dev, false, address + 1, bytes[1]);
		}
		break;
	default:
		ret = -ENOTSUP;
		break;
	}
	k_mutex_unlock(&data->lock);
	return ret;
}

static int mrf24j40_configure(const struct device *dev,
			      enum ieee802154_config_type type,
			      const struct ieee802154_config *config)
{
	struct mrf24j40_data *data = dev->data;
	uint8_t reg;
	int ret = 0;

	k_mutex_lock(&data->lock, K_FOREVER);
	switch (type) {
	case IEEE802154_CONFIG_PROMISCUOUS:
		ret = mrf24j40_read(dev, false, MRF24J40_RXMCR, &reg);
		if (ret == 0) {
			reg = config->promiscuous ?
				(reg | MRF24J40_RXMCR_PROMISC | MRF24J40_RXMCR_NOACK) :
				(reg & ~(MRF24J40_RXMCR_PROMISC | MRF24J40_RXMCR_NOACK));
			ret = mrf24j40_write(dev, false, MRF24J40_RXMCR, reg);
		}
		break;
	case IEEE802154_CONFIG_PAN_COORDINATOR:
		ret = mrf24j40_read(dev, false, MRF24J40_RXMCR, &reg);
		if (ret == 0) {
			reg = config->pan_coordinator ? reg | BIT(3) : reg & ~BIT(3);
			ret = mrf24j40_write(dev, false, MRF24J40_RXMCR, reg);
		}
		break;
	default:
		ret = -ENOTSUP;
		break;
	}
	k_mutex_unlock(&data->lock);
	return ret;
}

static void mrf24j40_rx(const struct device *dev)
{
	struct mrf24j40_data *data = dev->data;
	struct net_pkt *pkt;
	uint8_t frame[MRF24J40_MAX_PSDU];
	uint8_t phr;
	uint8_t lqi;
	uint8_t rssi;
	size_t len;
	int ret;

	if (mrf24j40_read(dev, true, MRF24J40_RX_FIFO, &phr) != 0 ||
	    phr < MRF24J40_FCS_LEN + 3 || phr > MRF24J40_MAX_PSDU) {
		LOG_ERR("Invalid RX frame length");
		(void)mrf24j40_write(dev, false, MRF24J40_RXFLUSH, BIT(0));
		return;
	}

	len = phr;
	if (!IS_ENABLED(CONFIG_IEEE802154_L2_PKT_INCL_FCS)) {
		len -= MRF24J40_FCS_LEN;
	}
	pkt = net_pkt_rx_alloc_with_buffer(data->iface, len, NET_AF_UNSPEC, 0,
					   K_NO_WAIT);
	if (pkt == NULL) {
		LOG_ERR("No packet buffer available");
		(void)mrf24j40_write(dev, false, MRF24J40_RXFLUSH, BIT(0));
		return;
	}

	ret = mrf24j40_read_long_burst(dev, MRF24J40_RX_FIFO + 1,
				       frame, len);
	if (ret == 0) {
		ret = mrf24j40_read(dev, true, MRF24J40_RX_FIFO + phr + 1, &lqi);
	}
	if (ret == 0) {
		ret = mrf24j40_read(dev, true, MRF24J40_RX_FIFO + phr + 2, &rssi);
	}
	if (ret != 0) {
		LOG_ERR("Failed to read RX FIFO");
		net_pkt_unref(pkt);
		(void)mrf24j40_write(dev, false, MRF24J40_RXFLUSH, BIT(0));
		return;
	}

	if (net_pkt_write(pkt, frame, len) != 0) {
		LOG_ERR("Failed to copy RX frame to packet");
		net_pkt_unref(pkt);
		return;
	}
	net_pkt_set_ieee802154_lqi(pkt, lqi);
	net_pkt_set_ieee802154_rssi_dbm(pkt, -90 + (rssi * 55) / 255);

	if (ieee802154_handle_ack(data->iface, pkt) == NET_OK) {
		net_pkt_unref(pkt);
		return;
	}
	if (net_recv_data(data->iface, pkt) < 0) {
		net_pkt_unref(pkt);
	}
}

static void mrf24j40_irq_thread(void *arg, void *unused1, void *unused2)
{
	const struct device *dev = arg;
	struct mrf24j40_data *data = dev->data;
	uint8_t status;
	uint8_t tx_status;

	ARG_UNUSED(unused1);
	ARG_UNUSED(unused2);

	while (true) {
		k_sem_take(&data->irq_sem, K_FOREVER);
		k_mutex_lock(&data->lock, K_FOREVER);
		if (mrf24j40_read(dev, false, MRF24J40_INTSTAT, &status) != 0) {
			LOG_ERR("Failed to read interrupt status");
			k_mutex_unlock(&data->lock);
			continue;
		}
		if (status & MRF24J40_INT_RX) {
			mrf24j40_rx(dev);
		}
		if ((status & MRF24J40_INT_TX) && data->tx_pending) {
			if (mrf24j40_read(dev, false, MRF24J40_TXSTAT, &tx_status) != 0) {
				data->tx_result = -EIO;
			} else if (tx_status & MRF24J40_TXSTAT_FAILED) {
				if (tx_status & MRF24J40_TXSTAT_CCAFAIL) {
					data->tx_result = -EBUSY;
				} else {
					data->tx_result = data->tx_ack_requested ?
						-ENOMSG : -EIO;
				}
			} else {
				data->tx_result = 0;
			}
			data->tx_pending = false;
			k_sem_give(&data->tx_sem);
		}
		k_mutex_unlock(&data->lock);
	}
}

static void mrf24j40_gpio_callback(const struct device *port,
				   struct gpio_callback *cb, uint32_t pins)
{
	struct mrf24j40_data *data =
		CONTAINER_OF(cb, struct mrf24j40_data, irq_cb);

	ARG_UNUSED(port);
	ARG_UNUSED(pins);
	k_sem_give(&data->irq_sem);
}

static int mrf24j40_tx(const struct device *dev, enum ieee802154_tx_mode mode,
		       struct net_pkt *pkt, struct net_buf *frag)
{
	struct mrf24j40_data *data = dev->data;
	uint8_t len;
	uint8_t control;
	int ret;

	ARG_UNUSED(pkt);

	if (frag->len + MRF24J40_FCS_LEN > MRF24J40_MAX_PSDU) {
		return -EMSGSIZE;
	}
	if (mode != IEEE802154_TX_MODE_DIRECT &&
	    mode != IEEE802154_TX_MODE_CCA &&
	    mode != IEEE802154_TX_MODE_CSMA_CA) {
		return -ENOTSUP;
	}
	if ((mode == IEEE802154_TX_MODE_CCA) &&
	    (ret = mrf24j40_cca(dev)) != 0) {
		return ret;
	}

	k_mutex_lock(&data->lock, K_FOREVER);
	if (!data->started || data->tx_pending) {
		k_mutex_unlock(&data->lock);
		return data->started ? -EBUSY : -ENETDOWN;
	}
	k_sem_reset(&data->tx_sem);
	ret = mrf24j40_read(dev, false, MRF24J40_INTSTAT, &control);
	if (ret != 0) {
		k_mutex_unlock(&data->lock);
		return ret;
	}
	len = frag->len + MRF24J40_FCS_LEN;
	ret = mrf24j40_write_long_burst(dev, MRF24J40_TX_FIFO + 1, &len, 1);
	if (ret == 0) {
		ret = mrf24j40_write_long_burst(dev, MRF24J40_TX_FIFO + 2,
						frag->data, frag->len);
	}
	if (ret == 0) {
		ret = mrf24j40_write(dev, false, MRF24J40_TXMCR,
				     mode == IEEE802154_TX_MODE_CSMA_CA ?
				     0x04 : (0x04 | MRF24J40_TXMCR_NO_CSMA));
	}
	if (ret == 0) {
		control = MRF24J40_TXNCON_TRIGGER;
		if (ieee802154_is_ar_flag_set(frag)) {
			control |= MRF24J40_TXNCON_ACKREQ;
		}
		data->tx_ack_requested = (control & MRF24J40_TXNCON_ACKREQ) != 0;
		data->tx_pending = true;
		ret = mrf24j40_write(dev, false, MRF24J40_TXNCON, control);
	}
	if (ret != 0) {
		data->tx_pending = false;
	}
	k_mutex_unlock(&data->lock);
	if (ret != 0) {
		return ret;
	}

	ret = k_sem_take(&data->tx_sem, K_MSEC(100));
	if (ret != 0) {
		k_mutex_lock(&data->lock, K_FOREVER);
		data->tx_pending = false;
		k_mutex_unlock(&data->lock);
		LOG_ERR("Transmit timed out");
		return -ETIMEDOUT;
	}

	return data->tx_result;
}

static int mrf24j40_start(const struct device *dev)
{
	struct mrf24j40_data *data = dev->data;
	const struct mrf24j40_config *cfg = dev->config;
	uint8_t value;
	int ret;

	k_mutex_lock(&data->lock, K_FOREVER);
	if (data->started) {
		k_mutex_unlock(&data->lock);
		return -EALREADY;
	}
	ret = mrf24j40_write(dev, false, MRF24J40_WAKECON, 0xc0);
	if (ret == 0) {
		k_busy_wait(50);
		ret = mrf24j40_write(dev, false, MRF24J40_WAKECON, 0x80);
	}
	if (ret == 0) {
		k_busy_wait(2000);
		ret = mrf24j40_write(dev, false, MRF24J40_RFCTL, BIT(2));
	}
	if (ret == 0) {
		ret = mrf24j40_write(dev, false, MRF24J40_RFCTL, 0);
	}
	if (ret == 0) {
		ret = mrf24j40_read(dev, false, MRF24J40_INTSTAT, &value);
	}
	if (ret == 0) {
		ret = mrf24j40_write(dev, false, MRF24J40_INTCON,
				     (uint8_t)~(MRF24J40_INT_RX | MRF24J40_INT_TX));
	}
	if (ret == 0) {
		ret = gpio_pin_interrupt_configure_dt(&cfg->irq,
						      GPIO_INT_EDGE_TO_ACTIVE);
	}
	if (ret == 0) {
		data->started = true;
	}
	k_mutex_unlock(&data->lock);
	return ret;
}

static int mrf24j40_stop(const struct device *dev)
{
	struct mrf24j40_data *data = dev->data;
	const struct mrf24j40_config *cfg = dev->config;
	int ret;

	k_mutex_lock(&data->lock, K_FOREVER);
	if (!data->started) {
		k_mutex_unlock(&data->lock);
		return -EALREADY;
	}
	ret = gpio_pin_interrupt_configure_dt(&cfg->irq, GPIO_INT_DISABLE);
	if (ret == 0) {
		ret = mrf24j40_write(dev, false, MRF24J40_INTCON, 0xff);
	}
	if (ret == 0) {
		ret = mrf24j40_write(dev, false, MRF24J40_SOFTRST, BIT(2));
	}
	if (ret == 0) {
		ret = mrf24j40_write(dev, false, MRF24J40_SLPACK, BIT(7));
	}
	if (ret == 0) {
		data->started = false;
	}
	k_mutex_unlock(&data->lock);
	return ret;
}

static enum ieee802154_hw_caps mrf24j40_get_capabilities(const struct device *dev)
{
	ARG_UNUSED(dev);

	return IEEE802154_HW_FCS | IEEE802154_HW_PROMISC | IEEE802154_HW_FILTER |
	       IEEE802154_HW_CSMA |
	       IEEE802154_HW_TX_RX_ACK | IEEE802154_HW_RX_TX_ACK |
	       IEEE802154_HW_RETRANSMISSION;
}

static int mrf24j40_attr_get(const struct device *dev, enum ieee802154_attr attr,
			     struct ieee802154_attr_value *value)
{
	ARG_UNUSED(dev);

	switch (attr) {
	case IEEE802154_ATTR_PHY_SUPPORTED_CHANNEL_PAGES:
		value->phy_supported_channel_pages =
			IEEE802154_ATTR_PHY_CHANNEL_PAGE_ZERO_OQPSK_2450_BPSK_868_915;
		return 0;
	case IEEE802154_ATTR_PHY_SUPPORTED_CHANNEL_RANGES:
		value->phy_supported_channels = &mrf24j40_channels.phy_supported_channels;
		return 0;
	default:
		return -ENOENT;
	}
}

static void mrf24j40_iface_init(struct net_if *iface)
{
	const struct device *dev = net_if_get_device(iface);
	struct mrf24j40_data *data = dev->data;

	data->iface = iface;
	net_if_set_link_addr(iface, data->mac, sizeof(data->mac),
			     NET_LINK_IEEE802154);
	ieee802154_init(iface);
}

static struct ieee802154_radio_api mrf24j40_api = {
	.iface_api.init = mrf24j40_iface_init,
	.get_capabilities = mrf24j40_get_capabilities,
	.cca = mrf24j40_cca,
	.set_channel = mrf24j40_set_channel,
	.filter = mrf24j40_filter,
	.set_txpower = mrf24j40_set_txpower,
	.tx = mrf24j40_tx,
	.start = mrf24j40_start,
	.stop = mrf24j40_stop,
	.configure = mrf24j40_configure,
	.attr_get = mrf24j40_attr_get,
};

static int mrf24j40_init(const struct device *dev)
{
	const struct mrf24j40_config *cfg = dev->config;
	struct mrf24j40_data *data = dev->data;
	struct gpio_callback *cb = &data->irq_cb;
	uint8_t value;
	int ret;

	if (!spi_is_ready_dt(&cfg->spi) || !gpio_is_ready_dt(&cfg->irq) ||
	    !gpio_is_ready_dt(&cfg->reset)) {
		LOG_ERR("SPI or GPIO device is not ready");
		return -ENODEV;
	}

	k_mutex_init(&data->lock);
	k_sem_init(&data->irq_sem, 0, 1);
	k_sem_init(&data->tx_sem, 0, 1);

	ret = gpio_pin_configure_dt(&cfg->reset, GPIO_OUTPUT_INACTIVE);
	if (ret != 0) {
		return ret;
	}
	ret = gpio_pin_set_dt(&cfg->reset, 1);
	if (ret != 0) {
		return ret;
	}
	k_busy_wait(20000);
	ret = gpio_pin_set_dt(&cfg->reset, 0);
	if (ret != 0) {
		return ret;
	}
	k_busy_wait(2000);

	ret = gpio_pin_configure_dt(&cfg->irq, GPIO_INPUT);
	if (ret != 0) {
		return ret;
	}
	gpio_init_callback(cb, mrf24j40_gpio_callback, BIT(cfg->irq.pin));
	ret = gpio_add_callback(cfg->irq.port, cb);
	if (ret != 0) {
		return ret;
	}
	ret = gpio_pin_interrupt_configure_dt(&cfg->irq, GPIO_INT_DISABLE);
	if (ret != 0) {
		return ret;
	}

	/* Reset MAC, baseband and power blocks, then apply the datasheet setup. */
	if (mrf24j40_write(dev, false, MRF24J40_SOFTRST, 0x07) != 0) {
		return -EIO;
	}
	k_busy_wait(2000);
	if (mrf24j40_write(dev, false, MRF24J40_RXFLUSH, BIT(0)) != 0 ||
	    mrf24j40_write(dev, false, 0x18, 0x98) != 0 ||
	    mrf24j40_write(dev, false, 0x2e, 0x95) != 0 ||
	    mrf24j40_write(dev, true, MRF24J40_RFCON1, 0x02) != 0 ||
	    mrf24j40_write(dev, true, MRF24J40_RFCON2, 0x80) != 0 ||
	    mrf24j40_write(dev, true, MRF24J40_RFCON6, 0x90) != 0 ||
	    mrf24j40_write(dev, true, MRF24J40_RFCON7, 0x80) != 0 ||
	    mrf24j40_write(dev, true, MRF24J40_RFCON8, 0x10) != 0 ||
	    mrf24j40_write(dev, true, MRF24J40_SLPCON1, 0x21) != 0 ||
	    mrf24j40_write(dev, false, MRF24J40_BBREG2, 0x80) != 0 ||
	    mrf24j40_write(dev, false, MRF24J40_CCAEDTH, 0x60) != 0 ||
	    mrf24j40_write(dev, false, MRF24J40_BBREG6,
			   MRF24J40_BBREG6_RSSI_MODE2) != 0 ||
	    mrf24j40_write(dev, false, MRF24J40_WAKECON, 0x80) != 0 ||
	    mrf24j40_write(dev, true, MRF24J40_SLPCON0, BIT(1)) != 0 ||
	    mrf24j40_write(dev, false, MRF24J40_RFCTL, BIT(2)) != 0 ||
	    mrf24j40_write(dev, false, MRF24J40_RFCTL, 0) != 0 ||
	    mrf24j40_read(dev, false, MRF24J40_INTSTAT, &value) != 0 ||
	    mrf24j40_write(dev, false, MRF24J40_INTCON, 0xff) != 0) {
		LOG_ERR("Radio initialization failed");
		return -EIO;
	}

	ret = mrf24j40_set_channel(dev, MRF24J40_CHANNEL_MIN);
	if (ret != 0) {
		return ret;
	}
	if (cfg->has_mac) {
		memcpy(data->mac, cfg->mac, sizeof(data->mac));
	} else {
		sys_put_le32(sys_rand32_get(), data->mac);
		sys_put_le32(sys_rand32_get(), data->mac + 4);
		data->mac[0] = (data->mac[0] & ~BIT(0)) | BIT(1);
	}

	k_thread_create(&data->rx_thread, data->rx_stack,
			K_KERNEL_STACK_SIZEOF(data->rx_stack),
			mrf24j40_irq_thread, (void *)dev, NULL, NULL,
			K_PRIO_COOP(2), 0, K_NO_WAIT);
	return 0;
}

#if !defined(CONFIG_IEEE802154_RAW_MODE)
#if defined(CONFIG_NET_L2_IEEE802154)
#define MRF24J40_L2 IEEE802154_L2
#define MRF24J40_L2_CTX NET_L2_GET_CTX_TYPE(IEEE802154_L2)
#elif defined(CONFIG_NET_L2_OPENTHREAD)
#define MRF24J40_L2 OPENTHREAD_L2
#define MRF24J40_L2_CTX NET_L2_GET_CTX_TYPE(OPENTHREAD_L2)
#endif
#endif

#define MRF24J40_LOCAL_MAC(inst)						\
	COND_CODE_1(DT_INST_NODE_HAS_PROP(inst, local_mac_address),		\
		    (DT_INST_PROP(inst, local_mac_address)), (0))

#define MRF24J40_DEFINE(inst)						 \
	static const struct mrf24j40_config mrf24j40_config_##inst = {	 \
		.spi = SPI_DT_SPEC_INST_GET(inst, SPI_WORD_SET(8) |	 \
					    SPI_TRANSFER_MSB),		 \
		.irq = GPIO_DT_SPEC_INST_GET(inst, irq_gpios),		 \
		.reset = GPIO_DT_SPEC_INST_GET(inst, reset_gpios),	 \
		.has_mac = DT_INST_NODE_HAS_PROP(inst, local_mac_address), \
		.mac = { MRF24J40_LOCAL_MAC(inst) },			 \
	};								 \
	static struct mrf24j40_data mrf24j40_data_##inst;		 \
	COND_CODE_1(CONFIG_IEEE802154_RAW_MODE, (			 \
		DEVICE_DT_INST_DEFINE(inst, mrf24j40_init,		 \
			NULL, &mrf24j40_data_##inst,			 \
			&mrf24j40_config_##inst, POST_KERNEL,		 \
			CONFIG_IEEE802154_MRF24J40_INIT_PRIO,		 \
			&mrf24j40_api);), (				 \
		NET_DEVICE_DT_INST_DEFINE(inst, mrf24j40_init,		 \
			NULL, &mrf24j40_data_##inst,			 \
			&mrf24j40_config_##inst,			 \
			CONFIG_IEEE802154_MRF24J40_INIT_PRIO,		 \
			&mrf24j40_api, MRF24J40_L2, MRF24J40_L2_CTX,	 \
			MRF24J40_MAX_PSDU);))

DT_INST_FOREACH_STATUS_OKAY(MRF24J40_DEFINE)
