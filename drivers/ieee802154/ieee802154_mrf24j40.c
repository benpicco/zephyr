/*
 * Copyright (c) 2026 The Zephyr Project Contributors
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/* IEEE 802.15.4 driver for the Microchip MRF24J40 */

#define DT_DRV_COMPAT microchip_mrf24j40

#define LOG_MODULE_NAME ieee802154_mrf24j40
#define LOG_LEVEL CONFIG_IEEE802154_DRIVER_LOG_LEVEL

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(LOG_MODULE_NAME);

#include <errno.h>
#include <string.h>

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/net/ieee802154_frame.h>
#include <zephyr/net/ieee802154_radio.h>
#include <zephyr/net/net_if.h>
#include <zephyr/net/net_pkt.h>
#include <zephyr/net/ieee802154_pkt.h>
#include <zephyr/random/random.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/util.h>

#if defined(CONFIG_NET_L2_OPENTHREAD)
#include <zephyr/net/openthread.h>
#endif

#define MRF24J40_TX_FIFO		0x000
#define MRF24J40_RX_FIFO		0x300
#define MRF24J40_MAX_PSDU		127
#define MRF24J40_FCS_LEN		2
#define MRF24J40_CHANNEL_MIN		11
#define MRF24J40_CHANNEL_MAX		26
#define MRF24J40_TX_TIMEOUT		K_MSEC(100)

/* Short registers (0x00 - 0x3f) */
#define MRF24J40_RXMCR			0x00
#define MRF24J40_PANIDL			0x01
#define MRF24J40_SADRL			0x03
#define MRF24J40_EADR0			0x05
#define MRF24J40_RXFLUSH		0x0d
#define MRF24J40_TXMCR			0x11
#define MRF24J40_PACON2			0x18
#define MRF24J40_TXNCON			0x1b
#define MRF24J40_WAKECON		0x22
#define MRF24J40_TXSTAT			0x24
#define MRF24J40_SOFTRST		0x2a
#define MRF24J40_TXSTBL			0x2e
#define MRF24J40_INTSTAT		0x31
#define MRF24J40_INTCON			0x32
#define MRF24J40_SLPACK			0x35
#define MRF24J40_RFCTL			0x36
#define MRF24J40_BBREG1			0x39
#define MRF24J40_BBREG2			0x3a
#define MRF24J40_BBREG6			0x3e
#define MRF24J40_CCAEDTH		0x3f

/* Long registers (0x200 - 0x38f) */
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

#define MRF24J40_RXMCR_PROMI		BIT(0)
#define MRF24J40_RXMCR_PANCOORD		BIT(3)
#define MRF24J40_RXMCR_NOACKRSP		BIT(5)
#define MRF24J40_RXFLUSH_RXFLUSH	BIT(0)
#define MRF24J40_TXMCR_CSMABF		GENMASK(2, 0)
#define MRF24J40_TXMCR_MACMINBE		GENMASK(4, 3)
#define MRF24J40_TXMCR_NOCSMA		BIT(7)
#define MRF24J40_PACON2_TXONTS		GENMASK(5, 2)
#define MRF24J40_PACON2_FIFOEN		BIT(7)
#define MRF24J40_TXNCON_TXNTRIG		BIT(0)
#define MRF24J40_TXNCON_TXNACKREQ	BIT(2)
#define MRF24J40_WAKECON_REGWAKE	BIT(6)
#define MRF24J40_WAKECON_IMMWAKE	BIT(7)
#define MRF24J40_TXSTAT_TXNSTAT		BIT(0)
#define MRF24J40_TXSTAT_CCAFAIL		BIT(5)
#define MRF24J40_SOFTRST_RSTMAC		BIT(0)
#define MRF24J40_SOFTRST_RSTBB		BIT(1)
#define MRF24J40_SOFTRST_RSTPWR		BIT(2)
#define MRF24J40_TXSTBL_MSIFS		GENMASK(3, 0)
#define MRF24J40_TXSTBL_RFSTBL		GENMASK(7, 4)
#define MRF24J40_INT_TXN		BIT(0)
#define MRF24J40_INT_RX			BIT(3)
/* INTCON bits are active-low interrupt enables */
#define MRF24J40_INTCON_DISABLE_ALL	0xff
#define MRF24J40_SLPACK_SLPACK		BIT(7)
#define MRF24J40_RFCTL_RFRST		BIT(2)
#define MRF24J40_BBREG1_RXDECINV	BIT(2)
#define MRF24J40_BBREG2_CCAMODE		GENMASK(7, 6)
#define MRF24J40_BBREG2_CCAMODE_ED	2
#define MRF24J40_BBREG6_RSSIRDY		BIT(0)
#define MRF24J40_BBREG6_RSSIMODE2	BIT(6)
#define MRF24J40_BBREG6_RSSIMODE1	BIT(7)
#define MRF24J40_RFCON0_RFOPT		GENMASK(3, 0)
#define MRF24J40_RFCON0_CHANNEL		GENMASK(7, 4)
#define MRF24J40_RFCON2_PLLEN		BIT(7)
#define MRF24J40_RFCON6_20MRECVR	BIT(4)
#define MRF24J40_RFCON6_TXFIL		BIT(7)
#define MRF24J40_RFCON7_SLPCLKSEL	GENMASK(7, 6)
#define MRF24J40_RFCON7_SLPCLKSEL_100K	2
#define MRF24J40_RFCON8_RFVCO		BIT(4)
#define MRF24J40_SLPCON0_INTEDGE	BIT(1)
#define MRF24J40_SLPCON1_SLPCLKDIV	GENMASK(4, 0)
#define MRF24J40_SLPCON1_CLKOUTEN	BIT(5)

/* Recommended values from the datasheet, see section 3.2 */
#define MRF24J40_RFOPT_DEFAULT		0x03
#define MRF24J40_VCOOPT_DEFAULT		0x02
#define MRF24J40_TXONTS_DEFAULT		0x06
#define MRF24J40_RFSTBL_DEFAULT		0x09
#define MRF24J40_MSIFS_DEFAULT		0x05
/* Energy detection threshold for CCA, approximately -69 dBm */
#define MRF24J40_CCAEDTH_DEFAULT	0x60

/* Reset value: macMinBE = 3, macMaxCSMABackoffs = 4 */
#define MRF24J40_TXMCR_DEFAULT		(FIELD_PREP(MRF24J40_TXMCR_MACMINBE, 3) | \
					 FIELD_PREP(MRF24J40_TXMCR_CSMABF, 4))

IEEE802154_DEFINE_PHY_SUPPORTED_CHANNELS(mrf24j40_channels, MRF24J40_CHANNEL_MIN,
					 MRF24J40_CHANNEL_MAX);

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
	K_KERNEL_STACK_MEMBER(rx_stack, CONFIG_IEEE802154_MRF24J40_RX_STACK_SIZE);
	struct k_thread rx_thread;
	uint8_t mac[8];
	int tx_result;
	bool tx_pending;
	bool tx_ack_requested;
	bool started;
};

/*
 * Short registers live below 0x40 and long registers / FIFOs at or above it,
 * so the access type can be derived from the address.
 */
static inline bool mrf24j40_is_long(uint16_t addr)
{
	return addr >= 0x40;
}

static size_t mrf24j40_cmd(uint8_t *cmd, uint16_t addr, bool write)
{
	if (mrf24j40_is_long(addr)) {
		cmd[0] = BIT(7) | (addr >> 3);
		cmd[1] = (addr << 5) | (write ? BIT(4) : 0);
		return 2;
	}

	cmd[0] = (addr << 1) | (write ? BIT(0) : 0);
	return 1;
}

static int mrf24j40_read_burst(const struct device *dev, uint16_t addr,
			       uint8_t *data, size_t len)
{
	const struct mrf24j40_config *cfg = dev->config;
	uint8_t cmd[2];
	size_t cmd_len = mrf24j40_cmd(cmd, addr, false);
	const struct spi_buf tx_buf = { .buf = cmd, .len = cmd_len };
	const struct spi_buf rx_bufs[] = {
		{ .buf = NULL, .len = cmd_len },
		{ .buf = data, .len = len },
	};
	const struct spi_buf_set tx = { .buffers = &tx_buf, .count = 1 };
	const struct spi_buf_set rx = { .buffers = rx_bufs, .count = ARRAY_SIZE(rx_bufs) };

	return spi_transceive_dt(&cfg->spi, &tx, &rx);
}

static int mrf24j40_write_burst(const struct device *dev, uint16_t addr,
				const uint8_t *data, size_t len)
{
	const struct mrf24j40_config *cfg = dev->config;
	uint8_t cmd[2];
	const struct spi_buf bufs[] = {
		{ .buf = cmd, .len = mrf24j40_cmd(cmd, addr, true) },
		{ .buf = (void *)data, .len = len },
	};
	const struct spi_buf_set tx = { .buffers = bufs, .count = ARRAY_SIZE(bufs) };

	return spi_write_dt(&cfg->spi, &tx);
}

static inline int mrf24j40_read(const struct device *dev, uint16_t addr, uint8_t *value)
{
	return mrf24j40_read_burst(dev, addr, value, 1);
}

static inline int mrf24j40_write(const struct device *dev, uint16_t addr, uint8_t value)
{
	return mrf24j40_write_burst(dev, addr, &value, 1);
}

static int mrf24j40_update(const struct device *dev, uint16_t addr, uint8_t mask, bool set)
{
	uint8_t value;
	int ret;

	ret = mrf24j40_read(dev, addr, &value);
	if (ret != 0) {
		return ret;
	}

	return mrf24j40_write(dev, addr, set ? (value | mask) : (value & ~mask));
}

/* RF state machine reset, required after waking up and changing channel */
static int mrf24j40_rf_reset(const struct device *dev)
{
	int ret;

	ret = mrf24j40_write(dev, MRF24J40_RFCTL, MRF24J40_RFCTL_RFRST);
	if (ret == 0) {
		ret = mrf24j40_write(dev, MRF24J40_RFCTL, 0);
	}
	k_busy_wait(200);

	return ret;
}

static int mrf24j40_set_channel(const struct device *dev, uint16_t channel)
{
	struct mrf24j40_data *data = dev->data;
	int ret;

	if (channel < MRF24J40_CHANNEL_MIN || channel > MRF24J40_CHANNEL_MAX) {
		return -EINVAL;
	}

	k_mutex_lock(&data->lock, K_FOREVER);
	ret = mrf24j40_write(dev, MRF24J40_RFCON0,
			     FIELD_PREP(MRF24J40_RFCON0_CHANNEL, channel - MRF24J40_CHANNEL_MIN) |
			     FIELD_PREP(MRF24J40_RFCON0_RFOPT, MRF24J40_RFOPT_DEFAULT));
	if (ret == 0) {
		ret = mrf24j40_rf_reset(dev);
	}
	k_mutex_unlock(&data->lock);

	return ret;
}

static int mrf24j40_cca_locked(const struct device *dev)
{
	uint8_t reg;
	uint8_t rssi;
	int ret;

	/* Trigger a single RSSI measurement */
	ret = mrf24j40_write(dev, MRF24J40_BBREG6, MRF24J40_BBREG6_RSSIMODE1);
	if (ret != 0) {
		return ret;
	}

	if (!WAIT_FOR(mrf24j40_read(dev, MRF24J40_BBREG6, &reg) == 0 &&
		      (reg & MRF24J40_BBREG6_RSSIRDY), 1000, k_busy_wait(10))) {
		ret = -ETIMEDOUT;
		goto restore;
	}

	ret = mrf24j40_read(dev, MRF24J40_RSSI, &rssi);
	if (ret == 0) {
		ret = mrf24j40_read(dev, MRF24J40_CCAEDTH, &reg);
	}
	if (ret == 0) {
		ret = rssi < reg ? 0 : -EBUSY;
	}

restore:
	/* Go back to appending RSSI to every received frame */
	if (mrf24j40_write(dev, MRF24J40_BBREG6, MRF24J40_BBREG6_RSSIMODE2) != 0) {
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
	/* RFCON3 values for 0 dBm down to -36 dBm in 1 dB steps */
	static const uint8_t power_reg[] = {
		0x00, 0x10, 0x18, 0x20, 0x28, 0x30, 0x38, 0x38, 0x38,
		0x40, 0x40, 0x50, 0x58, 0x60, 0x68, 0x70, 0x78, 0x78,
		0x78, 0x80, 0x80, 0x90, 0x98, 0xa0, 0xa8, 0xb0, 0xb8,
		0xb8, 0xb8, 0xc0, 0xc0, 0xd0, 0xd8, 0xe0, 0xe8, 0xf0,
		0xf8,
	};
	struct mrf24j40_data *data = dev->data;
	int ret;

	if (dbm > 0 || -dbm >= ARRAY_SIZE(power_reg)) {
		return -EINVAL;
	}

	k_mutex_lock(&data->lock, K_FOREVER);
	ret = mrf24j40_write(dev, MRF24J40_RFCON3, power_reg[-dbm]);
	k_mutex_unlock(&data->lock);

	return ret;
}

static int mrf24j40_filter(const struct device *dev, bool set,
			   enum ieee802154_filter_type type,
			   const struct ieee802154_filter *filter)
{
	struct mrf24j40_data *data = dev->data;
	uint8_t buf[8] = { 0 };
	uint16_t addr;
	size_t len;
	int ret;

	switch (type) {
	case IEEE802154_FILTER_TYPE_IEEE_ADDR:
		/* ieee_addr is little endian, matching EADR0..EADR7 */
		if (set) {
			memcpy(buf, filter->ieee_addr, sizeof(buf));
		}
		addr = MRF24J40_EADR0;
		len = 8;
		break;
	case IEEE802154_FILTER_TYPE_SHORT_ADDR:
		sys_put_le16(set ? filter->short_addr : IEEE802154_BROADCAST_ADDRESS, buf);
		addr = MRF24J40_SADRL;
		len = 2;
		break;
	case IEEE802154_FILTER_TYPE_PAN_ID:
		sys_put_le16(set ? filter->pan_id : IEEE802154_BROADCAST_PAN_ID, buf);
		addr = MRF24J40_PANIDL;
		len = 2;
		break;
	default:
		return -ENOTSUP;
	}

	/* Short register addresses don't auto-increment within one transfer */
	k_mutex_lock(&data->lock, K_FOREVER);
	ret = 0;
	for (size_t i = 0; i < len && ret == 0; i++) {
		ret = mrf24j40_write(dev, addr + i, buf[i]);
	}
	k_mutex_unlock(&data->lock);

	return ret;
}

static int mrf24j40_configure(const struct device *dev,
			      enum ieee802154_config_type type,
			      const struct ieee802154_config *config)
{
	struct mrf24j40_data *data = dev->data;
	int ret;

	k_mutex_lock(&data->lock, K_FOREVER);
	switch (type) {
	case IEEE802154_CONFIG_PROMISCUOUS:
		ret = mrf24j40_update(dev, MRF24J40_RXMCR,
				      MRF24J40_RXMCR_PROMI | MRF24J40_RXMCR_NOACKRSP,
				      config->promiscuous);
		break;
	case IEEE802154_CONFIG_PAN_COORDINATOR:
		ret = mrf24j40_update(dev, MRF24J40_RXMCR, MRF24J40_RXMCR_PANCOORD,
				      config->pan_coordinator);
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
	/* PSDU (including FCS) followed by LQI and RSSI */
	uint8_t buf[MRF24J40_MAX_PSDU + 2];
	struct net_pkt *pkt;
	uint8_t phr;
	size_t len;
	int ret;

	/* Stop reception while the RX FIFO is read out, see datasheet example 3-2 */
	ret = mrf24j40_write(dev, MRF24J40_BBREG1, MRF24J40_BBREG1_RXDECINV);
	if (ret == 0) {
		ret = mrf24j40_read(dev, MRF24J40_RX_FIFO, &phr);
	}
	if (ret == 0 && (phr < IEEE802154_MIN_LENGTH || phr > MRF24J40_MAX_PSDU)) {
		ret = -EBADMSG;
	}
	if (ret == 0) {
		ret = mrf24j40_read_burst(dev, MRF24J40_RX_FIFO + 1, buf, phr + 2);
	}
	if (ret != 0) {
		(void)mrf24j40_write(dev, MRF24J40_RXFLUSH, MRF24J40_RXFLUSH_RXFLUSH);
	}
	(void)mrf24j40_write(dev, MRF24J40_BBREG1, 0);

	if (ret != 0) {
		LOG_ERR("Failed to read RX frame (%d)", ret);
		return;
	}

	len = IS_ENABLED(CONFIG_IEEE802154_L2_PKT_INCL_FCS) ? phr : phr - MRF24J40_FCS_LEN;
	pkt = net_pkt_rx_alloc_with_buffer(data->iface, len, NET_AF_UNSPEC, 0, K_NO_WAIT);
	if (pkt == NULL) {
		LOG_ERR("No packet buffer available");
		return;
	}

	if (net_pkt_write(pkt, buf, len) != 0) {
		LOG_ERR("Failed to copy RX frame to packet");
		net_pkt_unref(pkt);
		return;
	}

	net_pkt_set_ieee802154_lqi(pkt, buf[phr]);
	net_pkt_set_ieee802154_rssi_dbm(pkt, -90 + (buf[phr + 1] * 55) / 255);

	if (net_recv_data(data->iface, pkt) < 0) {
		net_pkt_unref(pkt);
	}
}

static void mrf24j40_tx_done(const struct device *dev)
{
	struct mrf24j40_data *data = dev->data;
	uint8_t status;

	if (mrf24j40_read(dev, MRF24J40_TXSTAT, &status) != 0) {
		data->tx_result = -EIO;
	} else if (!(status & MRF24J40_TXSTAT_TXNSTAT)) {
		data->tx_result = 0;
	} else if (status & MRF24J40_TXSTAT_CCAFAIL) {
		data->tx_result = -EBUSY;
	} else {
		data->tx_result = data->tx_ack_requested ? -ENOMSG : -EIO;
	}

	data->tx_pending = false;
	k_sem_give(&data->tx_sem);
}

static void mrf24j40_irq_thread(void *arg1, void *arg2, void *arg3)
{
	const struct device *dev = arg1;
	struct mrf24j40_data *data = dev->data;
	uint8_t status;

	ARG_UNUSED(arg2);
	ARG_UNUSED(arg3);

	while (true) {
		k_sem_take(&data->irq_sem, K_FOREVER);
		k_mutex_lock(&data->lock, K_FOREVER);

		/* INTSTAT is cleared on read, which also releases the INT pin */
		if (mrf24j40_read(dev, MRF24J40_INTSTAT, &status) != 0) {
			LOG_ERR("Failed to read interrupt status");
			status = 0;
		}
		if (status & MRF24J40_INT_RX) {
			mrf24j40_rx(dev);
		}
		if ((status & MRF24J40_INT_TXN) && data->tx_pending) {
			mrf24j40_tx_done(dev);
		}

		k_mutex_unlock(&data->lock);
	}
}

static void mrf24j40_gpio_callback(const struct device *port,
				   struct gpio_callback *cb, uint32_t pins)
{
	struct mrf24j40_data *data = CONTAINER_OF(cb, struct mrf24j40_data, irq_cb);

	ARG_UNUSED(port);
	ARG_UNUSED(pins);

	k_sem_give(&data->irq_sem);
}

static int mrf24j40_write_tx_fifo(const struct device *dev, struct net_buf *frag)
{
	const struct mrf24j40_config *cfg = dev->config;
	uint8_t cmd[2];
	/* Header length (only used for security) and frame length (incl. FCS) */
	uint8_t hdr[2] = { 0, frag->len + MRF24J40_FCS_LEN };
	const struct spi_buf bufs[] = {
		{ .buf = cmd, .len = mrf24j40_cmd(cmd, MRF24J40_TX_FIFO, true) },
		{ .buf = hdr, .len = sizeof(hdr) },
		{ .buf = frag->data, .len = frag->len },
	};
	const struct spi_buf_set tx = { .buffers = bufs, .count = ARRAY_SIZE(bufs) };

	return spi_write_dt(&cfg->spi, &tx);
}

static int mrf24j40_tx(const struct device *dev, enum ieee802154_tx_mode mode,
		       struct net_pkt *pkt, struct net_buf *frag)
{
	struct mrf24j40_data *data = dev->data;
	uint8_t txmcr = MRF24J40_TXMCR_DEFAULT;
	int ret;

	ARG_UNUSED(pkt);

	if (frag->len + MRF24J40_FCS_LEN > MRF24J40_MAX_PSDU) {
		return -EMSGSIZE;
	}

	switch (mode) {
	case IEEE802154_TX_MODE_DIRECT:
	case IEEE802154_TX_MODE_CCA:
		txmcr |= MRF24J40_TXMCR_NOCSMA;
		break;
	case IEEE802154_TX_MODE_CSMA_CA:
		break;
	default:
		return -ENOTSUP;
	}

	k_mutex_lock(&data->lock, K_FOREVER);
	if (!data->started) {
		ret = -ENETDOWN;
		goto out;
	}

	if (mode == IEEE802154_TX_MODE_CCA) {
		ret = mrf24j40_cca_locked(dev);
		if (ret != 0) {
			goto out;
		}
	}

	ret = mrf24j40_write_tx_fifo(dev, frag);
	if (ret == 0) {
		ret = mrf24j40_write(dev, MRF24J40_TXMCR, txmcr);
	}
	if (ret != 0) {
		goto out;
	}

	data->tx_ack_requested = ieee802154_is_ar_flag_set(frag);
	ret = mrf24j40_write(dev, MRF24J40_TXNCON,
			     MRF24J40_TXNCON_TXNTRIG |
			     (data->tx_ack_requested ? MRF24J40_TXNCON_TXNACKREQ : 0));
	if (ret == 0) {
		k_sem_reset(&data->tx_sem);
		data->tx_pending = true;
	}

out:
	k_mutex_unlock(&data->lock);
	if (ret != 0) {
		return ret;
	}

	if (k_sem_take(&data->tx_sem, MRF24J40_TX_TIMEOUT) != 0) {
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
	uint8_t status;
	int ret;

	k_mutex_lock(&data->lock, K_FOREVER);
	if (data->started) {
		ret = 0;
		goto out;
	}

	/* Wake up from immediate sleep, then let the main oscillator settle */
	ret = mrf24j40_write(dev, MRF24J40_WAKECON,
			     MRF24J40_WAKECON_IMMWAKE | MRF24J40_WAKECON_REGWAKE);
	if (ret == 0) {
		ret = mrf24j40_write(dev, MRF24J40_WAKECON, MRF24J40_WAKECON_IMMWAKE);
	}
	if (ret == 0) {
		k_msleep(2);
		ret = mrf24j40_rf_reset(dev);
	}
	/* Clear stale interrupts so that the INT pin is deasserted */
	if (ret == 0) {
		ret = mrf24j40_read(dev, MRF24J40_INTSTAT, &status);
	}
	if (ret == 0) {
		ret = mrf24j40_write(dev, MRF24J40_INTCON,
				     (uint8_t)~(MRF24J40_INT_RX | MRF24J40_INT_TXN));
	}
	if (ret == 0) {
		ret = gpio_pin_interrupt_configure_dt(&cfg->irq, GPIO_INT_EDGE_TO_ACTIVE);
	}
	if (ret == 0) {
		data->started = true;
	}

out:
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
		ret = 0;
		goto out;
	}

	/* Enter immediate sleep; register contents are retained */
	ret = gpio_pin_interrupt_configure_dt(&cfg->irq, GPIO_INT_DISABLE);
	if (ret == 0) {
		ret = mrf24j40_write(dev, MRF24J40_INTCON, MRF24J40_INTCON_DISABLE_ALL);
	}
	if (ret == 0) {
		ret = mrf24j40_write(dev, MRF24J40_SOFTRST, MRF24J40_SOFTRST_RSTPWR);
	}
	if (ret == 0) {
		ret = mrf24j40_write(dev, MRF24J40_SLPACK, MRF24J40_SLPACK_SLPACK);
	}
	if (ret == 0) {
		data->started = false;
	}

out:
	k_mutex_unlock(&data->lock);

	return ret;
}

static enum ieee802154_hw_caps mrf24j40_get_capabilities(const struct device *dev)
{
	ARG_UNUSED(dev);

	return IEEE802154_HW_FCS | IEEE802154_HW_PROMISC | IEEE802154_HW_FILTER |
	       IEEE802154_HW_CSMA | IEEE802154_HW_TX_RX_ACK |
	       IEEE802154_HW_RX_TX_ACK | IEEE802154_HW_RETRANSMISSION;
}

static int mrf24j40_attr_get(const struct device *dev, enum ieee802154_attr attr,
			     struct ieee802154_attr_value *value)
{
	ARG_UNUSED(dev);

	return ieee802154_attr_get_channel_page_and_range(
		attr, IEEE802154_ATTR_PHY_CHANNEL_PAGE_ZERO_OQPSK_2450_BPSK_868_915,
		&mrf24j40_channels.phy_supported_channels, value);
}

static void mrf24j40_iface_init(struct net_if *iface)
{
	const struct device *dev = net_if_get_device(iface);
	struct mrf24j40_data *data = dev->data;

	data->iface = iface;
	net_if_set_link_addr(iface, data->mac, sizeof(data->mac), NET_LINK_IEEE802154);
	ieee802154_init(iface);
}

static const struct ieee802154_radio_api mrf24j40_api = {
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

/* Recommended register settings, see datasheet section 3.2 */
static const struct {
	uint16_t addr;
	uint8_t value;
} mrf24j40_init_regs[] = {
	{ MRF24J40_PACON2, MRF24J40_PACON2_FIFOEN |
			   FIELD_PREP(MRF24J40_PACON2_TXONTS, MRF24J40_TXONTS_DEFAULT) },
	{ MRF24J40_TXSTBL, FIELD_PREP(MRF24J40_TXSTBL_RFSTBL, MRF24J40_RFSTBL_DEFAULT) |
			   FIELD_PREP(MRF24J40_TXSTBL_MSIFS, MRF24J40_MSIFS_DEFAULT) },
	{ MRF24J40_RFCON1, MRF24J40_VCOOPT_DEFAULT },
	{ MRF24J40_RFCON2, MRF24J40_RFCON2_PLLEN },
	{ MRF24J40_RFCON6, MRF24J40_RFCON6_TXFIL | MRF24J40_RFCON6_20MRECVR },
	{ MRF24J40_RFCON7, FIELD_PREP(MRF24J40_RFCON7_SLPCLKSEL, MRF24J40_RFCON7_SLPCLKSEL_100K) },
	{ MRF24J40_RFCON8, MRF24J40_RFCON8_RFVCO },
	{ MRF24J40_SLPCON1, MRF24J40_SLPCON1_CLKOUTEN | FIELD_PREP(MRF24J40_SLPCON1_SLPCLKDIV, 1) },
	{ MRF24J40_BBREG2, FIELD_PREP(MRF24J40_BBREG2_CCAMODE, MRF24J40_BBREG2_CCAMODE_ED) },
	{ MRF24J40_CCAEDTH, MRF24J40_CCAEDTH_DEFAULT },
	{ MRF24J40_BBREG6, MRF24J40_BBREG6_RSSIMODE2 },
	{ MRF24J40_WAKECON, MRF24J40_WAKECON_IMMWAKE },
	{ MRF24J40_SLPCON0, MRF24J40_SLPCON0_INTEDGE },
	{ MRF24J40_INTCON, MRF24J40_INTCON_DISABLE_ALL },
};

static int mrf24j40_hw_init(const struct device *dev)
{
	const struct mrf24j40_config *cfg = dev->config;
	uint8_t status;
	int ret;

	ret = gpio_pin_configure_dt(&cfg->reset, GPIO_OUTPUT_ACTIVE);
	if (ret != 0) {
		return ret;
	}
	k_msleep(20);
	ret = gpio_pin_set_dt(&cfg->reset, 0);
	if (ret != 0) {
		return ret;
	}
	k_msleep(2);

	ret = mrf24j40_write(dev, MRF24J40_SOFTRST, MRF24J40_SOFTRST_RSTPWR |
			     MRF24J40_SOFTRST_RSTBB | MRF24J40_SOFTRST_RSTMAC);
	if (ret != 0) {
		return ret;
	}
	k_msleep(2);

	ret = mrf24j40_write(dev, MRF24J40_RXFLUSH, MRF24J40_RXFLUSH_RXFLUSH);
	for (size_t i = 0; i < ARRAY_SIZE(mrf24j40_init_regs) && ret == 0; i++) {
		ret = mrf24j40_write(dev, mrf24j40_init_regs[i].addr,
				     mrf24j40_init_regs[i].value);
	}
	if (ret == 0) {
		ret = mrf24j40_read(dev, MRF24J40_INTSTAT, &status);
	}
	if (ret != 0) {
		return ret;
	}

	/* Also performs the RF state machine reset */
	return mrf24j40_set_channel(dev, MRF24J40_CHANNEL_MIN);
}

static int mrf24j40_init(const struct device *dev)
{
	const struct mrf24j40_config *cfg = dev->config;
	struct mrf24j40_data *data = dev->data;
	int ret;

	if (!spi_is_ready_dt(&cfg->spi) || !gpio_is_ready_dt(&cfg->irq) ||
	    !gpio_is_ready_dt(&cfg->reset)) {
		LOG_ERR("SPI or GPIO device is not ready");
		return -ENODEV;
	}

	k_mutex_init(&data->lock);
	k_sem_init(&data->irq_sem, 0, 1);
	k_sem_init(&data->tx_sem, 0, 1);

	ret = gpio_pin_configure_dt(&cfg->irq, GPIO_INPUT);
	if (ret != 0) {
		return ret;
	}
	gpio_init_callback(&data->irq_cb, mrf24j40_gpio_callback, BIT(cfg->irq.pin));
	ret = gpio_add_callback(cfg->irq.port, &data->irq_cb);
	if (ret != 0) {
		return ret;
	}

	ret = mrf24j40_hw_init(dev);
	if (ret != 0) {
		LOG_ERR("Radio initialization failed (%d)", ret);
		return ret;
	}

	if (cfg->has_mac) {
		memcpy(data->mac, cfg->mac, sizeof(data->mac));
	} else {
		sys_rand_get(data->mac, sizeof(data->mac));
		/* Locally administered, unicast */
		data->mac[0] = (data->mac[0] & ~BIT(0)) | BIT(1);
	}

	k_thread_create(&data->rx_thread, data->rx_stack,
			K_KERNEL_STACK_SIZEOF(data->rx_stack),
			mrf24j40_irq_thread, (void *)dev, NULL, NULL,
			K_PRIO_COOP(2), 0, K_NO_WAIT);
	k_thread_name_set(&data->rx_thread, "mrf24j40_rx");

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

#define MRF24J40_DEFINE(inst)							\
	static const struct mrf24j40_config mrf24j40_config_##inst = {		\
		.spi = SPI_DT_SPEC_INST_GET(inst, SPI_WORD_SET(8) | SPI_TRANSFER_MSB), \
		.irq = GPIO_DT_SPEC_INST_GET(inst, irq_gpios),			\
		.reset = GPIO_DT_SPEC_INST_GET(inst, reset_gpios),		\
		.has_mac = DT_INST_NODE_HAS_PROP(inst, local_mac_address),	\
		.mac = DT_INST_PROP_OR(inst, local_mac_address, {0}),		\
	};									\
	static struct mrf24j40_data mrf24j40_data_##inst;			\
	COND_CODE_1(CONFIG_IEEE802154_RAW_MODE, (				\
		DEVICE_DT_INST_DEFINE(inst, mrf24j40_init, NULL,		\
				      &mrf24j40_data_##inst,			\
				      &mrf24j40_config_##inst, POST_KERNEL,	\
				      CONFIG_IEEE802154_MRF24J40_INIT_PRIO,	\
				      &mrf24j40_api);), (			\
		NET_DEVICE_DT_INST_DEFINE(inst, mrf24j40_init, NULL,		\
					  &mrf24j40_data_##inst,		\
					  &mrf24j40_config_##inst,		\
					  CONFIG_IEEE802154_MRF24J40_INIT_PRIO,	\
					  &mrf24j40_api, MRF24J40_L2,		\
					  MRF24J40_L2_CTX, MRF24J40_MAX_PSDU);))

DT_INST_FOREACH_STATUS_OKAY(MRF24J40_DEFINE)
