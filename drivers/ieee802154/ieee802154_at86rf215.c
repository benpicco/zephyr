/*
 * Copyright (c) 2026 ML!PA Consulting GmbH
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Driver for the Microchip (Atmel) AT86RF215 dual-band IEEE 802.15.4
 * transceiver. Each radio uses one of the following PHYs:
 *  - legacy O-QPSK (250 kb/s):
 *    - 2.4 GHz radio: channel page 0, channels 11-26 (2000 kchip/s)
 *    - sub-GHz radio: channel page 2, channels 1-10 (915 MHz band, 1000 kchip/s)
 *  - SUN O-QPSK (MR-O-QPSK): channel page 9, band, chip rate and rate mode
 *    selected in devicetree
 */

#define DT_DRV_COMPAT atmel_at86rf215

#define LOG_MODULE_NAME ieee802154_at86rf215
#define LOG_LEVEL       CONFIG_IEEE802154_DRIVER_LOG_LEVEL

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
#include <zephyr/random/random.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/util.h>

#include "ieee802154_at86rf215.h"
#include "ieee802154_at86rf215_regs.h"

/* Legacy O-QPSK PHY, see IEEE 802.15.4-2024 clause 13 */
#define AT86RF215_LEGACY_SYMBOL_US          16U
/* Preamble (8 symbols) and SFD (2 symbols) */
#define AT86RF215_LEGACY_SHR_SYMBOLS        10U
#define AT86RF215_LEGACY_SYMBOLS_PER_OCTET  2U
/* aTurnaroundTime, equal to macSifsPeriod */
#define AT86RF215_LEGACY_TURNAROUND_SYMBOLS 12U
#define AT86RF215_LEGACY_CCA_SYMBOLS        8U

/*
 * SUN O-QPSK PHY, see IEEE 802.15.4-2024 clause 22. The symbol period is
 * the bit period of the SHR, (32,1)-DSSS at 100 kchip/s, (64,1)-DSSS at
 * 1000 kchip/s and (128,1)-DSSS at 2000 kchip/s. The proprietary
 * 200 kchip/s mode of the AT86RF215 is the 100 kchip/s PHY at twice the
 * chip rate.
 */
#define AT86RF215_MR_SYMBOL_US_100   320U
#define AT86RF215_MR_SYMBOL_US       64U
/* phyCcaDuration (table 22-24) */
#define AT86RF215_MR_CCA_SYMBOLS_100 4U
#define AT86RF215_MR_CCA_SYMBOLS     8U
#define AT86RF215_MR_SFD_SYMBOLS     16U
/* 60 interleaved PHR code bits, (N,1)-DSSS with N = SHR spreading factor / 4 */
#define AT86RF215_MR_PHR_SYMBOLS     15U
/* Number of PSDU information bits per interleaver block (N_INTRLV / 2) */
#define AT86RF215_MR_BITS_PER_BLOCK  63U
/* Termination bits appended to the PSDU before encoding */
#define AT86RF215_MR_TAIL_BITS       6U
/* AIFS of the SUN PHYs, aTurnaroundTime is 1 ms rounded up to full symbols */
#define AT86RF215_MR_AIFS_US         1000U
#define AT86RF215_MR_TURNAROUND_US   1000U

/* Upper bound for the transmission of an automatic ACK frame after its start */
#define AT86RF215_AACK_MARGIN_US       1500U
/* Margin for the frame transmission watchdog */
#define AT86RF215_TX_TIMEOUT_MARGIN_MS 500U
#define AT86RF215_CCA_TIMEOUT          K_MSEC(10)

/* t_RST: minimum reset pulse width */
#define AT86RF215_RESET_PULSE_US    16U
#define AT86RF215_WAKEUP_TIMEOUT_US 10000U
#define AT86RF215_STATE_TIMEOUT_US  1000U

/* Rough mapping of PAC.TXPWR to output power with PAC.PACUR = 3 (TXPWR=31: 14 dBm) */
#define AT86RF215_TXPWR_OFFSET_DBM 17
#define AT86RF215_PACUR_MAX        3

/* Channel configuration of the 2450 MHz band O-QPSK PHY (page 0, channels 11-26) */
#define AT86RF215_RF24_CCF0_OFFSET_KHZ 1500000U
#define AT86RF215_2_4GHZ_SPACING_KHZ   5000U
#define AT86RF215_2_4GHZ_CENTER0_KHZ   2350000U
#define AT86RF215_2_4GHZ_CHANNEL_MIN   11U
#define AT86RF215_2_4GHZ_CHANNEL_MAX   26U
#define AT86RF215_2_4GHZ_CHANNEL_DEF   26U

/* Channel configuration of the 915 MHz band O-QPSK PHY (page 2, channels 1-10) */
#define AT86RF215_SUBGHZ_SPACING_KHZ 2000U
#define AT86RF215_SUBGHZ_CENTER0_KHZ 904000U
#define AT86RF215_SUBGHZ_CHANNEL_MIN 1U
#define AT86RF215_SUBGHZ_CHANNEL_MAX 10U
#define AT86RF215_SUBGHZ_CHANNEL_DEF 1U

/* Channel plan of the SUN O-QPSK PHY, see IEEE 802.15.4-2024 table 11-14 */
struct at86rf215_sun_channels {
	/* band designation (MHz) */
	uint16_t band;
	/* kchip/s */
	uint16_t chip_rate;
	uint32_t center0_khz;
	/* 0: center frequencies of table 11-16 */
	uint16_t spacing_khz;
	uint16_t num_channels;
};

static const struct at86rf215_sun_channels at86rf215_sun_channels[] = {
	{470, 100, 470200, 200, 199}, {780, 100, 779200, 200, 39},     {780, 1000, 780000, 2000, 4},
	{866, 100, 865100, 200, 15},  {868, 100, 868300, 0, 3},        {870, 100, 870200, 200, 29},
	{915, 100, 902200, 200, 129}, {915, 1000, 904000, 2000, 12},   {917, 100, 917100, 200, 32},
	{917, 1000, 918100, 2000, 3}, {2450, 2000, 2405000, 5000, 16},
};

/* Center frequencies of the SUN O-QPSK PHY in the 868-870 MHz band (table 11-16) */
static const uint32_t at86rf215_sun_868_khz[] = {868300, 868950, 869525};

/* Front end configuration for O-QPSK, see datasheet tables 6-103, 6-105 and 6-106 */
struct at86rf215_oqpsk_fe {
	uint8_t paramp;
	uint8_t lpfcut;
	uint8_t tx_rcut;
	uint8_t rx_bw;
	uint8_t rx_rcut;
	uint8_t sr;
	uint8_t avgs;
	/* EDD.DF with EDD.DTB = 128 us, equals phyCcaDuration of the SUN O-QPSK PHY */
	uint8_t edd_df;
};

/* Indexed by OQPSKC0.FCHIP: 100, 200, 1000 and 2000 kchip/s */
static const struct at86rf215_oqpsk_fe at86rf215_oqpsk_fe[] = {
	{.paramp = 3,
	 .lpfcut = 7,
	 .tx_rcut = 3,
	 .rx_bw = 0x0,
	 .rx_rcut = 1,
	 .sr = 0xA,
	 .avgs = 2,
	 .edd_df = 10},
	{.paramp = 2,
	 .lpfcut = 7,
	 .tx_rcut = 3,
	 .rx_bw = 0x2,
	 .rx_rcut = 1,
	 .sr = 0x5,
	 .avgs = 2,
	 .edd_df = 5},
	{.paramp = 0,
	 .lpfcut = 0xB,
	 .tx_rcut = 3,
	 .rx_bw = 0x8,
	 .rx_rcut = 0,
	 .sr = 0x1,
	 .avgs = 0,
	 .edd_df = 4},
	{.paramp = 0,
	 .lpfcut = 0xB,
	 .tx_rcut = 4,
	 .rx_bw = 0xB,
	 .rx_rcut = 2,
	 .sr = 0x1,
	 .avgs = 0,
	 .edd_df = 4},
};

#define AT86RF215_FREQ_RES_KHZ 25U

/* Frame filter: accept beacon, data and MAC command frames */
#define AT86RF215_AFFTM_DEFAULT                                                                    \
	(BIT(IEEE802154_FRAME_TYPE_BEACON) | BIT(IEEE802154_FRAME_TYPE_DATA) |                     \
	 BIT(IEEE802154_FRAME_TYPE_MAC_COMMAND))
#define AT86RF215_AFFTM_ACK BIT(IEEE802154_FRAME_TYPE_ACK)

/* MAC frame control field */
#define AT86RF215_FCF_TYPE_MASK      0x07
#define AT86RF215_FCF_AR             BIT(5)
#define AT86RF215_FCF_DST_MODE(fcf1) (((fcf1) >> 2) & 0x03)
#define AT86RF215_FCF_VERSION(fcf1)  (((fcf1) >> 4) & 0x03)
#define AT86RF215_ADDR_MODE_SHORT    0x02

/* Minimal frame: ACK frame (FCF + sequence number) */
#define AT86RF215_MIN_PSDU_LEN (IEEE802154_ACK_PKT_LENGTH + IEEE802154_FCS_LENGTH)

#ifdef CONFIG_NET_L2_IEEE802154_RADIO_CSMA_CA_MAX_BO
#define AT86RF215_CSMA_MAX_BO CONFIG_NET_L2_IEEE802154_RADIO_CSMA_CA_MAX_BO
#define AT86RF215_CSMA_MIN_BE CONFIG_NET_L2_IEEE802154_RADIO_CSMA_CA_MIN_BE
#define AT86RF215_CSMA_MAX_BE CONFIG_NET_L2_IEEE802154_RADIO_CSMA_CA_MAX_BE
#else
/* default values of macMaxCSMABackoffs, macMinBE and macMaxBE */
#define AT86RF215_CSMA_MAX_BO 4
#define AT86RF215_CSMA_MIN_BE 3
#define AT86RF215_CSMA_MAX_BE 5
#endif

/* Bits of at86rf215_radio.timeout */
#define AT86RF215_TIMEOUT_STATE   0
#define AT86RF215_TIMEOUT_BACKOFF 1

#ifdef CONFIG_IEEE802154_L2_PKT_INCL_FCS
#define AT86RF215_L2_FCS_LEN IEEE802154_FCS_LENGTH
#else
#define AT86RF215_L2_FCS_LEN 0
#endif

/* SPI access */

static int at86rf215_access(struct at86rf215_chip *chip, bool write, uint16_t addr, uint8_t *data,
			    size_t len)
{
	uint8_t cmd[2];
	struct spi_buf tx_bufs[2] = {
		{.buf = cmd, .len = sizeof(cmd)},
		{.buf = data, .len = len},
	};
	const struct spi_buf_set tx = {.buffers = tx_bufs, .count = write ? 2 : 1};
	int ret;

	sys_put_be16((addr & AT86RF215_SPI_ADDR_MASK) | (write ? AT86RF215_SPI_WRITE : 0), cmd);

	if (write) {
		ret = spi_write_dt(&chip->cfg->spi, &tx);
	} else {
		struct spi_buf rx_bufs[2] = {
			{.buf = NULL, .len = sizeof(cmd)},
			{.buf = data, .len = len},
		};
		const struct spi_buf_set rx = {.buffers = rx_bufs, .count = 2};

		ret = spi_transceive_dt(&chip->cfg->spi, &tx, &rx);
	}

	if (ret < 0) {
		LOG_ERR("SPI %s of 0x%04x failed (%d)", write ? "write" : "read", addr, ret);
	}

	return ret;
}

static inline int at86rf215_read(struct at86rf215_chip *chip, uint16_t addr, uint8_t *data,
				 size_t len)
{
	return at86rf215_access(chip, false, addr, data, len);
}

static inline int at86rf215_write(struct at86rf215_chip *chip, uint16_t addr, const uint8_t *data,
				  size_t len)
{
	return at86rf215_access(chip, true, addr, (uint8_t *)data, len);
}

static uint8_t at86rf215_reg_read(struct at86rf215_chip *chip, uint16_t addr)
{
	uint8_t val = 0;

	at86rf215_read(chip, addr, &val, 1);

	return val;
}

static void at86rf215_reg_write(struct at86rf215_chip *chip, uint16_t addr, uint8_t val)
{
	at86rf215_write(chip, addr, &val, 1);
}

static inline uint8_t rf_read(struct at86rf215_radio *r, uint8_t reg)
{
	return at86rf215_reg_read(r->chip, r->rf_base + reg);
}

static inline void rf_write(struct at86rf215_radio *r, uint8_t reg, uint8_t val)
{
	at86rf215_reg_write(r->chip, r->rf_base + reg, val);
}

static inline uint8_t bbc_read(struct at86rf215_radio *r, uint8_t reg)
{
	return at86rf215_reg_read(r->chip, r->bbc_base + reg);
}

static inline void bbc_write(struct at86rf215_radio *r, uint8_t reg, uint8_t val)
{
	at86rf215_reg_write(r->chip, r->bbc_base + reg, val);
}

static inline void rf_cmd(struct at86rf215_radio *r, uint8_t cmd)
{
	rf_write(r, AT86RF215_RF_CMD, cmd);
}

static inline uint8_t rf_state(struct at86rf215_radio *r)
{
	return rf_read(r, AT86RF215_RF_STATE) & AT86RF215_STATE_MASK;
}

static int rf_await_state(struct at86rf215_radio *r, uint8_t state)
{
	for (uint32_t t = 0; t < AT86RF215_STATE_TIMEOUT_US; t += 10) {
		if (rf_state(r) == state) {
			return 0;
		}
		k_busy_wait(10);
	}

	return -ETIMEDOUT;
}

/* Wait until a transmission (e.g. of an automatic ACK) has finished */
static void rf_await_tx_end(struct at86rf215_radio *r)
{
	for (uint32_t t = 0; t < AT86RF215_STATE_TIMEOUT_US; t += 10) {
		uint8_t state = rf_state(r);

		if (state != AT86RF215_STATE_TX && state != AT86RF215_STATE_TRANSITION) {
			return;
		}
		k_busy_wait(10);
	}
}

static bool at86rf215_irq_pending(struct at86rf215_chip *chip)
{
	return gpio_pin_get_dt(&chip->cfg->irq_gpio) > 0;
}

/* Radio state machine, all functions are called with the chip lock held */

static void at86rf215_start_timer(struct at86rf215_radio *r, uint32_t usec)
{
	atomic_clear_bit(&r->timeout, AT86RF215_TIMEOUT_STATE);
	k_timer_start(&r->timer, K_USEC(usec), K_NO_WAIT);
}

static void at86rf215_stop_timer(struct at86rf215_radio *r)
{
	k_timer_stop(&r->timer);
	atomic_clear_bit(&r->timeout, AT86RF215_TIMEOUT_STATE);
}

/* Radio is in TXPREP: start the frame transmission */
static void at86rf215_start_tx(struct at86rf215_radio *r)
{
	r->txprep_issued = false;
	r->tx_pending = false;

	/* switch to RX after TX, AACK must not be used together with TX2RX */
	bbc_write(r, AT86RF215_BBC_AMCS, (r->amcs & ~AT86RF215_AMCS_AACK) | AT86RF215_AMCS_TX2RX);

	/* only receive ACK frames while waiting for the ACK */
	if (r->ack_requested) {
		bbc_write(r, AT86RF215_BBC_AFFTM, AT86RF215_AFFTM_ACK);
	}

	r->state = AT86RF215_TRX_TX;
	rf_cmd(r, AT86RF215_CMD_TX);
}

static void at86rf215_txprep(struct at86rf215_radio *r)
{
	r->txprep_issued = true;

	/* no TRXRDY interrupt will occur if the radio is already in TXPREP */
	if (rf_state(r) == AT86RF215_STATE_TXPREP) {
		at86rf215_start_tx(r);
	} else {
		rf_cmd(r, AT86RF215_CMD_TXPREP);
	}
}

/* Restore the receive configuration after a transmission attempt */
static void at86rf215_restore_rx(struct at86rf215_radio *r)
{
	bbc_write(r, AT86RF215_BBC_AMCS, r->amcs);

	if (r->ack_requested) {
		bbc_write(r, AT86RF215_BBC_AFFTM, AT86RF215_AFFTM_DEFAULT);
	}
}

static void at86rf215_tx_done(struct at86rf215_radio *r)
{
	at86rf215_restore_rx(r);
	r->ack_requested = false;
}

static void at86rf215_tx_end(struct at86rf215_radio *r, int result)
{
	at86rf215_tx_done(r);
	r->state = AT86RF215_TRX_IDLE;
	r->tx_result = result;
	k_sem_give(&r->tx_sem);
}

static void at86rf215_set_idle(struct at86rf215_radio *r)
{
	r->state = AT86RF215_TRX_IDLE;

	/* a CSMA-CA transmission is started by the backoff timer */
	if (r->tx_pending && !r->csma && !r->agc_hold) {
		at86rf215_txprep(r);
	} else {
		rf_cmd(r, AT86RF215_CMD_RX);
	}
}

static void at86rf215_csma_cca(struct at86rf215_radio *r);

static void at86rf215_csma_backoff(struct at86rf215_radio *r)
{
	uint32_t periods = sys_rand32_get() & BIT_MASK(r->be);

	if (periods == 0) {
		at86rf215_csma_cca(r);
		return;
	}

	atomic_clear_bit(&r->timeout, AT86RF215_TIMEOUT_BACKOFF);
	k_timer_start(&r->backoff_timer, K_USEC(periods * r->unit_backoff_us), K_NO_WAIT);
}

/* Unslotted CSMA-CA, see IEEE 802.15.4-2020 section 6.2.5.1 */
static void at86rf215_csma_start(struct at86rf215_radio *r)
{
	r->nb = 0;
	r->be = AT86RF215_CSMA_MIN_BE;
	at86rf215_csma_backoff(r);
}

static void at86rf215_tx_end(struct at86rf215_radio *r, int result);

static void at86rf215_csma_busy(struct at86rf215_radio *r)
{
	if (++r->nb > AT86RF215_CSMA_MAX_BO) {
		LOG_DBG("Channel access failure");
		r->tx_pending = false;
		at86rf215_tx_end(r, -EBUSY);
		return;
	}

	r->be = MIN(r->be + 1, AT86RF215_CSMA_MAX_BE);
	at86rf215_csma_backoff(r);
}

/* Backoff period expired: perform CCA, the frame is sent automatically if the channel is clear */
static void at86rf215_csma_cca(struct at86rf215_radio *r)
{
	/* an ongoing reception or ACK transmission occupies the channel */
	if (r->state != AT86RF215_TRX_IDLE || r->agc_hold || r->cca_pending ||
	    rf_await_state(r, AT86RF215_STATE_RX) < 0) {
		at86rf215_csma_busy(r);
		return;
	}

	r->state = AT86RF215_TRX_CCATX;

	/* AACK must not be active during the transmission, CCATX excludes TX2RX */
	bbc_write(r, AT86RF215_BBC_AMCS, (r->amcs & ~AT86RF215_AMCS_AACK) | AT86RF215_AMCS_CCATX);
	/* do not receive frames during the energy measurement */
	bbc_write(r, AT86RF215_BBC_PC, r->pc & ~AT86RF215_PC_BBEN);
	rf_write(r, AT86RF215_RF_EDC, AT86RF215_EDC_EDM_SINGLE);
}

static void at86rf215_abort(struct at86rf215_radio *r)
{
	at86rf215_stop_timer(r);
	k_timer_stop(&r->backoff_timer);
	atomic_clear(&r->timeout);

	if (r->cca_pending || r->state == AT86RF215_TRX_CCATX) {
		r->cca_pending = false;
		rf_write(r, AT86RF215_RF_EDC, AT86RF215_EDC_EDM_AUTO);
		bbc_write(r, AT86RF215_BBC_PC, r->pc);
	}

	if (r->tx_pending || r->state == AT86RF215_TRX_CCATX || r->state == AT86RF215_TRX_TX ||
	    r->state == AT86RF215_TRX_TX_WAIT_ACK) {
		r->tx_pending = false;
		r->txprep_issued = false;
		at86rf215_tx_done(r);
	}

	r->state = AT86RF215_TRX_IDLE;
}

static void at86rf215_handle_cca(struct at86rf215_radio *r)
{
	int8_t ed = (int8_t)rf_read(r, AT86RF215_RF_EDV);

	r->cca_pending = false;
	rf_write(r, AT86RF215_RF_EDC, AT86RF215_EDC_EDM_AUTO);
	bbc_write(r, AT86RF215_BBC_PC, r->pc);

	r->cca_result = ed > r->cca_threshold ? -EBUSY : 0;
	LOG_DBG("CCA: %d dBm (%s)", ed, r->cca_result ? "busy" : "clear");

	k_sem_give(&r->cca_sem);
}

static bool at86rf215_ack_received(struct at86rf215_radio *r)
{
	uint8_t len[2];
	uint8_t ack[IEEE802154_ACK_PKT_LENGTH];

	at86rf215_read(r->chip, r->bbc_base + AT86RF215_BBC_RXFLL, len, sizeof(len));
	if ((sys_get_le16(len) & AT86RF215_FL_MASK) != AT86RF215_MIN_PSDU_LEN) {
		return false;
	}

	at86rf215_read(r->chip, r->fb_rx, ack, sizeof(ack));

	return (ack[0] & AT86RF215_FCF_TYPE_MASK) == IEEE802154_FRAME_TYPE_ACK &&
	       ack[2] == r->tx_seq;
}

/* Determine if the AACK procedure will acknowledge the received frame */
static bool at86rf215_aack_pending(struct at86rf215_radio *r, const uint8_t *psdu, size_t len)
{
	uint8_t type = psdu[0] & AT86RF215_FCF_TYPE_MASK;

	if (!(r->amcs & AT86RF215_AMCS_AACK) || !(psdu[0] & AT86RF215_FCF_AR)) {
		return false;
	}

	if (type != IEEE802154_FRAME_TYPE_DATA && type != IEEE802154_FRAME_TYPE_MAC_COMMAND) {
		return false;
	}

	/* broadcast frames are not acknowledged */
	if (AT86RF215_FCF_DST_MODE(psdu[1]) == AT86RF215_ADDR_MODE_SHORT && len >= 7 &&
	    sys_get_le16(&psdu[5]) == IEEE802154_BROADCAST_ADDRESS) {
		return false;
	}

	return true;
}

/* Read a received frame and pass it to the network stack */
static bool at86rf215_rx(struct at86rf215_radio *r)
{
	struct net_pkt *pkt;
	uint8_t buf[2];
	uint16_t psdu_len;
	size_t pkt_len;
	int8_t rssi;
	bool aack;

	at86rf215_read(r->chip, r->bbc_base + AT86RF215_BBC_RXFLL, buf, sizeof(buf));
	psdu_len = sys_get_le16(buf) & AT86RF215_FL_MASK;

	if (psdu_len < AT86RF215_MIN_PSDU_LEN || psdu_len > IEEE802154_MAX_PHY_PACKET_SIZE) {
		LOG_DBG("Invalid frame length %u", psdu_len);
		return false;
	}

	at86rf215_read(r->chip, r->fb_rx, r->rx_buf, psdu_len);

	/* energy detection is performed automatically during frame reception */
	rssi = (int8_t)rf_read(r, AT86RF215_RF_EDV);

	aack = at86rf215_aack_pending(r, r->rx_buf, psdu_len);

	if (r->iface == NULL) {
		return aack;
	}

	pkt_len = psdu_len - IEEE802154_FCS_LENGTH + AT86RF215_L2_FCS_LEN;

	pkt = net_pkt_rx_alloc_with_buffer(r->iface, pkt_len, NET_AF_UNSPEC, 0, K_NO_WAIT);
	if (pkt == NULL) {
		LOG_ERR("No RX buffer available");
		return aack;
	}

	if (net_pkt_write(pkt, r->rx_buf, pkt_len) < 0) {
		net_pkt_unref(pkt);
		return aack;
	}

	net_pkt_set_ieee802154_rssi_dbm(pkt, rssi);
	/* the transceiver does not provide a link quality, derive it from the RSSI */
	net_pkt_set_ieee802154_lqi(pkt, CLAMP((rssi + 100) * 4, 0, UINT8_MAX));

	LOG_DBG("RX %u bytes, RSSI %d dBm", psdu_len, rssi);

	if (net_recv_data(r->iface, pkt) < 0) {
		LOG_DBG("Packet dropped by NET stack");
		net_pkt_unref(pkt);
	}

	return aack;
}

static void at86rf215_isr(struct at86rf215_radio *r, uint8_t rf_irq, uint8_t bb_irq)
{
	atomic_val_t timeouts = atomic_clear(&r->timeout);
	bool timeout = timeouts & BIT(AT86RF215_TIMEOUT_STATE);

	if (!r->started) {
		return;
	}

	if (bb_irq & AT86RF215_BB_IRQ_AGCH) {
		r->agc_hold = true;
	}

	if (bb_irq & AT86RF215_BB_IRQ_AGCR) {
		r->agc_hold = false;
	}

	if ((rf_irq & AT86RF215_RF_IRQ_EDC) && r->cca_pending) {
		at86rf215_handle_cca(r);
	}

	if ((rf_irq & AT86RF215_RF_IRQ_EDC) && r->state == AT86RF215_TRX_CCATX) {
		if (bbc_read(r, AT86RF215_BBC_AMCS) & AT86RF215_AMCS_CCAED) {
			/* channel busy: TXFE is issued as well, radio remains in RX */
			bb_irq &= ~AT86RF215_BB_IRQ_TXFE;
			rf_write(r, AT86RF215_RF_EDC, AT86RF215_EDC_EDM_AUTO);
			bbc_write(r, AT86RF215_BBC_AMCS, r->amcs);
			bbc_write(r, AT86RF215_BBC_PC, r->pc);
			r->state = AT86RF215_TRX_IDLE;
			at86rf215_csma_busy(r);
		} else {
			/* channel clear: transceiver transmits the frame */
			r->tx_pending = false;
			r->state = AT86RF215_TRX_TX;
		}
	}

	if ((rf_irq & AT86RF215_RF_IRQ_TRXRDY) && r->txprep_issued) {
		at86rf215_start_tx(r);
	}

	/* frame transmitted - must be handled before a following ACK frame */
	if ((bb_irq & AT86RF215_BB_IRQ_TXFE) && r->state == AT86RF215_TRX_TX) {
		bb_irq &= ~AT86RF215_BB_IRQ_TXFE;

		if (r->csma) {
			/* CCATX ends in TXPREP, switch to RX quickly to receive the ACK */
			rf_cmd(r, AT86RF215_CMD_RX);
			rf_write(r, AT86RF215_RF_EDC, AT86RF215_EDC_EDM_AUTO);
			bbc_write(r, AT86RF215_BBC_AMCS, r->amcs & ~AT86RF215_AMCS_AACK);
		}

		if (r->ack_requested) {
			bbc_write(r, AT86RF215_BBC_AFFTM, AT86RF215_AFFTM_ACK);
			r->state = AT86RF215_TRX_TX_WAIT_ACK;
			at86rf215_start_timer(r, r->ack_timeout_us);
		} else {
			at86rf215_tx_end(r, 0);
		}
	}

	if (bb_irq & AT86RF215_BB_IRQ_RXFE) {
		switch (r->state) {
		case AT86RF215_TRX_IDLE:
			if (at86rf215_rx(r)) {
				r->state = AT86RF215_TRX_RX_SEND_ACK;
				at86rf215_start_timer(r, r->aack_timeout_us);
			} else {
				at86rf215_set_idle(r);
			}
			break;
		case AT86RF215_TRX_TX_WAIT_ACK:
			if (at86rf215_ack_received(r)) {
				at86rf215_stop_timer(r);
				timeout = false;
				at86rf215_tx_end(r, 0);
			}
			rf_cmd(r, AT86RF215_CMD_RX);
			break;
		default:
			break;
		}
	}

	/* automatic ACK transmitted */
	if ((bb_irq & AT86RF215_BB_IRQ_TXFE) && r->state == AT86RF215_TRX_RX_SEND_ACK) {
		at86rf215_stop_timer(r);
		timeout = false;
		at86rf215_set_idle(r);
	}

	if (timeout) {
		switch (r->state) {
		case AT86RF215_TRX_RX_SEND_ACK:
			LOG_DBG("No AACK transmission");
			at86rf215_set_idle(r);
			break;
		case AT86RF215_TRX_TX_WAIT_ACK:
			if (r->agc_hold) {
				/* a frame (likely the ACK) is still being received */
				at86rf215_start_timer(r, r->ack_timeout_us);
			} else if (r->retries > 0) {
				r->retries--;
				r->tx_pending = true;
				if (r->csma) {
					/* receive any frame during the backoff periods */
					at86rf215_restore_rx(r);
					r->state = AT86RF215_TRX_IDLE;
					at86rf215_csma_start(r);
				} else {
					at86rf215_txprep(r);
				}
			} else {
				LOG_DBG("No ACK received");
				at86rf215_tx_end(r, -ENOMSG);
			}
			break;
		default:
			break;
		}
	}

	if ((timeouts & BIT(AT86RF215_TIMEOUT_BACKOFF)) && r->tx_pending && r->csma) {
		at86rf215_csma_cca(r);
	}

	/* resume a transmission that had to be deferred */
	if (r->state == AT86RF215_TRX_IDLE && r->tx_pending && !r->csma && !r->txprep_issued &&
	    !r->agc_hold && !r->cca_pending) {
		at86rf215_txprep(r);
	}
}

/* Handle pending interrupts of both radios, called with the chip lock held */
static void at86rf215_process_irqs(struct at86rf215_chip *chip)
{
	uint8_t irqs[4];
	int loops = 0;

	do {
		/* RF09_IRQS, RF24_IRQS, BBC0_IRQS, BBC1_IRQS - cleared on read */
		at86rf215_read(chip, AT86RF215_REG_RF09_IRQS, irqs, sizeof(irqs));

		for (int i = 0; i < AT86RF215_NUM_RADIOS; i++) {
			if (chip->radio[i] != NULL) {
				at86rf215_isr(chip->radio[i], irqs[i], irqs[2 + i]);
			}
		}
	} while (at86rf215_irq_pending(chip) && ++loops < 4);

	if (at86rf215_irq_pending(chip)) {
		k_sem_give(&chip->isr_sem);
	}
}

/*
 * The IRQ thread can be starved by a cooperative caller (e.g. during the
 * busy-waiting CSMA-CA backoff of L2), so service pending interrupts
 * before the radio state is evaluated.
 */
static void at86rf215_sync_irqs(struct at86rf215_chip *chip)
{
	if (at86rf215_irq_pending(chip)) {
		at86rf215_process_irqs(chip);
	}
}

static void at86rf215_thread_main(void *p1, void *p2, void *p3)
{
	struct at86rf215_chip *chip = p1;

	ARG_UNUSED(p2);
	ARG_UNUSED(p3);

	while (true) {
		k_sem_take(&chip->isr_sem, K_FOREVER);
		k_mutex_lock(&chip->lock, K_FOREVER);
		at86rf215_process_irqs(chip);
		k_mutex_unlock(&chip->lock);
	}
}

static void at86rf215_irq_handler(const struct device *port, struct gpio_callback *cb,
				  uint32_t pins)
{
	struct at86rf215_chip *chip = CONTAINER_OF(cb, struct at86rf215_chip, irq_cb);

	ARG_UNUSED(port);
	ARG_UNUSED(pins);

	k_sem_give(&chip->isr_sem);
}

static void at86rf215_timer_handler(struct k_timer *timer)
{
	struct at86rf215_radio *r = k_timer_user_data_get(timer);

	atomic_set_bit(&r->timeout, AT86RF215_TIMEOUT_STATE);
	k_sem_give(&r->chip->isr_sem);
}

static void at86rf215_backoff_handler(struct k_timer *timer)
{
	struct at86rf215_radio *r = k_timer_user_data_get(timer);

	atomic_set_bit(&r->timeout, AT86RF215_TIMEOUT_BACKOFF);
	k_sem_give(&r->chip->isr_sem);
}

/* Radio configuration, called with the chip lock held */

static void at86rf215_write_channel(struct at86rf215_radio *r, uint16_t channel)
{
	uint8_t regs[5];
	uint32_t center0 = r->center0_khz;
	uint16_t cn = channel;

	if (r->spacing_khz == 0) {
		center0 = at86rf215_sun_868_khz[channel];
		cn = 0;
	}

	if (r->idx == AT86RF215_RADIO_2_4GHZ) {
		center0 -= AT86RF215_RF24_CCF0_OFFSET_KHZ;
	}

	/* CS, CCF0L, CCF0H, CNL, CNM - writing CNM applies the new channel */
	regs[0] = r->spacing_khz / AT86RF215_FREQ_RES_KHZ;
	sys_put_le16(center0 / AT86RF215_FREQ_RES_KHZ, &regs[1]);
	regs[3] = cn & 0xff;
	regs[4] = (cn >> 8) & AT86RF215_CNM_CNH;

	at86rf215_write(r->chip, r->rf_base + AT86RF215_RF_CS, regs, sizeof(regs));
	r->channel = channel;
}

/* Configure the legacy O-QPSK or the MR-O-QPSK PHY */
static void at86rf215_configure_oqpsk(struct at86rf215_radio *r)
{
	const struct at86rf215_oqpsk_fe *fe = &at86rf215_oqpsk_fe[r->fchip];
	bool mr = r->cfg->sun_band != 0;
	/* direct modulation shall be used for 100 kchip/s on v.3 devices */
	bool dm = r->fchip == AT86RF215_OQPSKC0_FCHIP_100 && r->chip->vn == 3;
	uint8_t edd;

	/* baseband must be disabled while it is reconfigured */
	bbc_write(r, AT86RF215_BBC_PC, 0);

	/* transmitter frontend, see datasheet table 6-103 */
	rf_write(r, AT86RF215_RF_TXCUTC,
		 FIELD_PREP(AT86RF215_TXCUTC_PARAMP_MASK, fe->paramp) |
			 FIELD_PREP(AT86RF215_TXCUTC_LPFCUT_MASK, fe->lpfcut));
	rf_write(r, AT86RF215_RF_TXDFE,
		 FIELD_PREP(AT86RF215_TXDFE_RCUT_MASK, fe->tx_rcut) |
			 FIELD_PREP(AT86RF215_TXDFE_SR_MASK, fe->sr) |
			 (dm ? AT86RF215_TXDFE_DM : 0));

	/* receiver frontend, see datasheet tables 6-105 and 6-106 */
	rf_write(r, AT86RF215_RF_RXBWC, FIELD_PREP(AT86RF215_RXBWC_BW_MASK, fe->rx_bw));
	rf_write(r, AT86RF215_RF_RXDFE,
		 FIELD_PREP(AT86RF215_RXDFE_RCUT_MASK, fe->rx_rcut) |
			 FIELD_PREP(AT86RF215_RXDFE_SR_MASK, fe->sr));
	rf_write(r, AT86RF215_RF_AGCC,
		 AT86RF215_AGCC_EN | FIELD_PREP(AT86RF215_AGCC_AVGS_MASK, fe->avgs));
	rf_write(r, AT86RF215_RF_AGCS, FIELD_PREP(AT86RF215_AGCS_TGT_MASK, 3));

	/* energy detection over phyCcaDuration */
	if (mr) {
		edd = FIELD_PREP(AT86RF215_EDD_DF_MASK, fe->edd_df) |
		      FIELD_PREP(AT86RF215_EDD_DTB_MASK, AT86RF215_EDD_DTB_128US);
	} else {
		/* 8 symbols (128 us) */
		edd = FIELD_PREP(AT86RF215_EDD_DF_MASK, 4) |
		      FIELD_PREP(AT86RF215_EDD_DTB_MASK, AT86RF215_EDD_DTB_32US);
	}
	rf_write(r, AT86RF215_RF_EDD, edd);
	rf_write(r, AT86RF215_RF_EDC, AT86RF215_EDC_EDM_AUTO);

	bbc_write(r, AT86RF215_BBC_OQPSKC0,
		  FIELD_PREP(AT86RF215_OQPSKC0_FCHIP_MASK, r->fchip) |
			  (dm ? AT86RF215_OQPSKC0_DM : 0));
	/* only receive the configured PHY, proprietary rate modes disabled */
	bbc_write(r, AT86RF215_BBC_OQPSKC2,
		  FIELD_PREP(AT86RF215_OQPSKC2_RXM_MASK,
			     mr ? AT86RF215_OQPSKC2_RXM_MR : AT86RF215_OQPSKC2_RXM_LEGACY) |
			  AT86RF215_OQPSKC2_FCSTLEG);
	/* PPDU type 2 is not used: only search for SFD 0 */
	bbc_write(r, AT86RF215_BBC_OQPSKC3, 0);
	bbc_write(r, AT86RF215_BBC_OQPSKPHRTX,
		  mr ? FIELD_PREP(AT86RF215_OQPSKPHR_MOD_MASK, r->cfg->rate_mode)
		     : AT86RF215_OQPSKPHR_LEG);

	/* enable baseband with 16 bit FCS, automatic FCS generation and filter */
	r->pc = FIELD_PREP(AT86RF215_PC_PT_MASK, AT86RF215_PC_PT_MROQPSK) | AT86RF215_PC_BBEN |
		AT86RF215_PC_FCST | AT86RF215_PC_TXAFCS | AT86RF215_PC_FCSFE;
	bbc_write(r, AT86RF215_BBC_PC, r->pc);
}

static int at86rf215_radio_setup(struct at86rf215_radio *r)
{
	uint8_t aifs[2];
	uint16_t channel;
	int ret;

	rf_cmd(r, AT86RF215_CMD_TRXOFF);
	ret = rf_await_state(r, AT86RF215_STATE_TRXOFF);
	if (ret < 0) {
		LOG_ERR("Radio %u did not enter TRXOFF", r->idx);
		return ret;
	}

	rf_write(r, AT86RF215_RF_IRQM, AT86RF215_RF_IRQ_TRXRDY | AT86RF215_RF_IRQ_EDC);
	bbc_write(r, AT86RF215_BBC_IRQM,
		  AT86RF215_BB_IRQ_RXFE | AT86RF215_BB_IRQ_TXFE | AT86RF215_BB_IRQ_AGCH |
			  AT86RF215_BB_IRQ_AGCR);

	at86rf215_configure_oqpsk(r);

	/* maximum output power */
	rf_write(r, AT86RF215_RF_PAC,
		 FIELD_PREP(AT86RF215_PAC_PACUR_MASK, AT86RF215_PACUR_MAX) |
			 AT86RF215_PAC_TXPWR_MASK);

	/* address filter, automatic ACK with FCS type and data rate of the received frame */
	bbc_write(r, AT86RF215_BBC_AFC0, AT86RF215_AFC0_AFEN0);
	bbc_write(r, AT86RF215_BBC_AFC1, 0);
	bbc_write(r, AT86RF215_BBC_AFFTM, AT86RF215_AFFTM_DEFAULT);
	bbc_write(r, AT86RF215_BBC_AMAACKPD, 0);
	sys_put_le16(r->aifs_us, aifs);
	at86rf215_write(r->chip, r->bbc_base + AT86RF215_BBC_AMAACKTL, aifs, sizeof(aifs));
	bbc_write(r, AT86RF215_BBC_AMEDT, (uint8_t)r->cca_threshold);
	r->amcs = AT86RF215_AMCS_AACK | AT86RF215_AMCS_AACKFA | AT86RF215_AMCS_AACKDR;
	bbc_write(r, AT86RF215_BBC_AMCS, r->amcs);

	if (r->cfg->sun_band == 0 && r->idx == AT86RF215_RADIO_2_4GHZ) {
		channel = AT86RF215_2_4GHZ_CHANNEL_DEF;
	} else if (r->cfg->sun_band == 0) {
		channel = AT86RF215_SUBGHZ_CHANNEL_DEF;
	} else {
		channel = r->channel_range.from_channel;
	}
	at86rf215_write_channel(r, channel);

	return 0;
}

/* Duration of a PPDU with a PSDU of len octets */
static uint32_t at86rf215_ppdu_us(const struct at86rf215_radio *r, uint32_t symbol_us, uint16_t len)
{
	const struct at86rf215_radio_config *cfg = r->cfg;
	uint32_t shr, bit_us, bits;

	if (cfg->sun_band == 0) {
		return (AT86RF215_LEGACY_SHR_SYMBOLS +
			AT86RF215_LEGACY_SYMBOLS_PER_OCTET * (1U + len)) *
		       symbol_us;
	}

	/* preamble length, see IEEE 802.15.4-2024 section 22.2.2.2 */
	if (cfg->sun_band == 780 || cfg->sun_band == 915 || cfg->sun_band == 917 ||
	    cfg->sun_band == 2450) {
		shr = 56U + AT86RF215_MR_SFD_SYMBOLS;
	} else {
		shr = 32U + AT86RF215_MR_SFD_SYMBOLS;
	}

	/* PSDU bit period including rate 1/2 FEC (table 22-4) */
	if (cfg->chip_rate <= 200) {
		bit_us = (160U * 100U / cfg->chip_rate) >> cfg->rate_mode;
	} else {
		bit_us = cfg->rate_mode == 0 ? 32U : (16U >> cfg->rate_mode);
	}

	/* PSDU and termination bits padded to full interleaver blocks */
	bits = ROUND_UP(8U * len + AT86RF215_MR_TAIL_BITS, AT86RF215_MR_BITS_PER_BLOCK);

	/* pilots add 1/16 to the PSDU chips (table 22-20) */
	return (shr + AT86RF215_MR_PHR_SYMBOLS) * symbol_us +
	       DIV_ROUND_UP(bits * bit_us * 17U, 16U);
}

/* Select the channel plan and derive the timing of the PHY */
static int at86rf215_phy_init(struct at86rf215_radio *r)
{
	const struct at86rf215_radio_config *cfg = r->cfg;
	uint32_t symbol_us, turnaround_us, cca_us, ack_us, csma_us = 0;
	uint8_t be = AT86RF215_CSMA_MIN_BE;

	if (cfg->sun_band == 0) {
		bool is_2_4ghz = r->idx == AT86RF215_RADIO_2_4GHZ;

		r->channel_page =
			is_2_4ghz ? IEEE802154_ATTR_PHY_CHANNEL_PAGE_ZERO_OQPSK_2450_BPSK_868_915
				  : IEEE802154_ATTR_PHY_CHANNEL_PAGE_TWO_OQPSK_868_915;
		r->channel_range.from_channel =
			is_2_4ghz ? AT86RF215_2_4GHZ_CHANNEL_MIN : AT86RF215_SUBGHZ_CHANNEL_MIN;
		r->channel_range.to_channel =
			is_2_4ghz ? AT86RF215_2_4GHZ_CHANNEL_MAX : AT86RF215_SUBGHZ_CHANNEL_MAX;
		r->center0_khz =
			is_2_4ghz ? AT86RF215_2_4GHZ_CENTER0_KHZ : AT86RF215_SUBGHZ_CENTER0_KHZ;
		r->spacing_khz =
			is_2_4ghz ? AT86RF215_2_4GHZ_SPACING_KHZ : AT86RF215_SUBGHZ_SPACING_KHZ;
		r->fchip = is_2_4ghz ? AT86RF215_OQPSKC0_FCHIP_2000 : AT86RF215_OQPSKC0_FCHIP_1000;
		r->cca_threshold = CONFIG_IEEE802154_AT86RF215_CCA_THRESHOLD;

		symbol_us = AT86RF215_LEGACY_SYMBOL_US;
		turnaround_us = AT86RF215_LEGACY_TURNAROUND_SYMBOLS * symbol_us;
		cca_us = AT86RF215_LEGACY_CCA_SYMBOLS * symbol_us;
		r->aifs_us = turnaround_us;
	} else {
		const struct at86rf215_sun_channels *ch = NULL;

		/*
		 * Chip rates not defined for the band use the channel plan of the
		 * band's first chip rate (non-standard, but supported by the chip).
		 */
		ARRAY_FOR_EACH_PTR(at86rf215_sun_channels, p) {
			if (p->band != cfg->sun_band) {
				continue;
			}
			if (ch == NULL || p->chip_rate == cfg->chip_rate) {
				ch = p;
			}
		}

		if (ch == NULL) {
			LOG_ERR("Unsupported SUN band %u MHz", cfg->sun_band);
			return -EINVAL;
		}

		if (ch->chip_rate != cfg->chip_rate) {
			LOG_WRN("%u kchip/s is not standard in the %u MHz band", cfg->chip_rate,
				cfg->sun_band);
		}

		r->channel_page = IEEE802154_ATTR_PHY_CHANNEL_PAGE_NINE_SUN_PREDEFINED;
		r->channel_range.from_channel = 0;
		r->channel_range.to_channel = ch->num_channels - 1;
		r->center0_khz = ch->center0_khz;
		r->spacing_khz = ch->spacing_khz;
		r->cca_threshold = CONFIG_IEEE802154_AT86RF215_SUN_CCA_THRESHOLD;

		if (cfg->chip_rate <= 200) {
			r->fchip = cfg->chip_rate == 100 ? AT86RF215_OQPSKC0_FCHIP_100
							 : AT86RF215_OQPSKC0_FCHIP_200;
			symbol_us = AT86RF215_MR_SYMBOL_US_100 * 100U / cfg->chip_rate;
			cca_us = AT86RF215_MR_CCA_SYMBOLS_100 * symbol_us;
		} else {
			r->fchip = cfg->chip_rate == 1000 ? AT86RF215_OQPSKC0_FCHIP_1000
							  : AT86RF215_OQPSKC0_FCHIP_2000;
			symbol_us = AT86RF215_MR_SYMBOL_US;
			cca_us = AT86RF215_MR_CCA_SYMBOLS * symbol_us;
		}

		turnaround_us = ROUND_UP(AT86RF215_MR_TURNAROUND_US, symbol_us);
		r->aifs_us = AT86RF215_MR_AIFS_US;
	}

	/* macUnitBackoffPeriod: aTurnaroundTime + phyCcaDuration */
	r->unit_backoff_us = turnaround_us + cca_us;

	/* macAckWaitDuration: macUnitBackoffPeriod + aTurnaroundTime + duration of the ACK */
	ack_us = at86rf215_ppdu_us(r, symbol_us, AT86RF215_MIN_PSDU_LEN);
	r->ack_timeout_us = r->unit_backoff_us + turnaround_us + ack_us;
	r->aack_timeout_us = r->aifs_us + ack_us + AT86RF215_AACK_MARGIN_US;

	/* worst case of CSMA-CA, frame transmission and ACK wait for all attempts */
	for (int nb = 0; nb <= AT86RF215_CSMA_MAX_BO; nb++) {
		csma_us += BIT_MASK(be) * r->unit_backoff_us + cca_us;
		be = MIN(be + 1, AT86RF215_CSMA_MAX_BE);
	}
	r->tx_timeout_ms = DIV_ROUND_UP((CONFIG_IEEE802154_AT86RF215_TX_RETRIES + 1) *
						(csma_us + r->ack_timeout_us +
						 at86rf215_ppdu_us(r, symbol_us,
								   IEEE802154_MAX_PHY_PACKET_SIZE)),
					USEC_PER_MSEC) +
			   AT86RF215_TX_TIMEOUT_MARGIN_MS;

	LOG_DBG("Radio %u: backoff %u us, ACK wait %u us, TX timeout %u ms", r->idx,
		r->unit_backoff_us, r->ack_timeout_us, r->tx_timeout_ms);

	return 0;
}

static int at86rf215_chip_init(struct at86rf215_chip *chip)
{
	const struct at86rf215_chip_config *cfg = chip->cfg;
	uint8_t irqs[4];
	uint32_t t;
	int ret;

	if (chip->initialized) {
		return 0;
	}

	k_mutex_init(&chip->lock);
	k_sem_init(&chip->isr_sem, 0, 1);

	if (!spi_is_ready_dt(&cfg->spi)) {
		LOG_ERR("SPI bus %s is not ready", cfg->spi.bus->name);
		return -ENODEV;
	}

	if (!gpio_is_ready_dt(&cfg->irq_gpio) || !gpio_is_ready_dt(&cfg->reset_gpio)) {
		LOG_ERR("GPIO controller is not ready");
		return -ENODEV;
	}

	ret = gpio_pin_configure_dt(&cfg->irq_gpio, GPIO_INPUT);
	if (ret < 0) {
		return ret;
	}

	ret = gpio_pin_configure_dt(&cfg->reset_gpio, GPIO_OUTPUT_ACTIVE);
	if (ret < 0) {
		return ret;
	}

	k_busy_wait(AT86RF215_RESET_PULSE_US);
	gpio_pin_set_dt(&cfg->reset_gpio, 0);

	/* registers read 0xff until the transceiver has woken up */
	for (t = 0; t < AT86RF215_WAKEUP_TIMEOUT_US; t += 100) {
		chip->pn = at86rf215_reg_read(chip, AT86RF215_REG_RF_PN);
		if (chip->pn != 0xff && chip->pn != 0x00) {
			break;
		}
		k_busy_wait(100);
	}

	chip->vn = at86rf215_reg_read(chip, AT86RF215_REG_RF_VN);

	switch (chip->pn) {
	case AT86RF215_PN_AT86RF215:
		LOG_INF("AT86RF215 rev. %u", chip->vn);
		break;
	case AT86RF215_PN_AT86RF215M:
		LOG_INF("AT86RF215M rev. %u", chip->vn);
		break;
	case AT86RF215_PN_AT86RF215IQ:
		LOG_ERR("AT86RF215IQ has no baseband core");
		return -ENODEV;
	default:
		LOG_ERR("Unknown part number 0x%02x", chip->pn);
		return -ENODEV;
	}

	/* disable clock output */
	at86rf215_reg_write(chip, AT86RF215_REG_RF_CLKO, 0);

	if (cfg->xtal_trim >= 0) {
		at86rf215_reg_write(chip, AT86RF215_REG_RF_XOC,
				    FIELD_PREP(AT86RF215_XOC_TRIM_MASK, cfg->xtal_trim) |
					    AT86RF215_XOC_FS);
	}

	/* disable the interrupts of both radios and put unused radios to sleep */
	for (int i = 0; i < AT86RF215_NUM_RADIOS; i++) {
		uint16_t rf_base =
			i == AT86RF215_RADIO_2_4GHZ ? AT86RF215_RF24_BASE : AT86RF215_RF09_BASE;
		uint16_t bbc_base =
			i == AT86RF215_RADIO_2_4GHZ ? AT86RF215_BBC1_BASE : AT86RF215_BBC0_BASE;

		at86rf215_reg_write(chip, rf_base + AT86RF215_RF_IRQM, 0);
		at86rf215_reg_write(chip, bbc_base + AT86RF215_BBC_IRQM, 0);

		if (!(cfg->radio_mask & BIT(i))) {
			at86rf215_reg_write(chip, rf_base + AT86RF215_RF_CMD, AT86RF215_CMD_SLEEP);
		}
	}

	/* clear pending interrupts */
	at86rf215_read(chip, AT86RF215_REG_RF09_IRQS, irqs, sizeof(irqs));

	gpio_init_callback(&chip->irq_cb, at86rf215_irq_handler, BIT(cfg->irq_gpio.pin));
	ret = gpio_add_callback(cfg->irq_gpio.port, &chip->irq_cb);
	if (ret < 0) {
		return ret;
	}

	ret = gpio_pin_interrupt_configure_dt(&cfg->irq_gpio, GPIO_INT_EDGE_TO_ACTIVE);
	if (ret < 0) {
		return ret;
	}

	k_thread_create(&chip->thread, cfg->stack, cfg->stack_size, at86rf215_thread_main, chip,
			NULL, NULL, K_PRIO_COOP(CONFIG_IEEE802154_AT86RF215_RX_THREAD_PRIO), 0,
			K_NO_WAIT);
	k_thread_name_set(&chip->thread, "at86rf215");

	chip->initialized = true;

	return 0;
}

/* IEEE 802.15.4 radio API */

static enum ieee802154_hw_caps at86rf215_get_capabilities(const struct device *dev)
{
	ARG_UNUSED(dev);

	return IEEE802154_HW_FCS | IEEE802154_HW_FILTER | IEEE802154_HW_PROMISC |
	       IEEE802154_HW_CSMA | IEEE802154_HW_TX_RX_ACK | IEEE802154_HW_RETRANSMISSION |
	       IEEE802154_HW_RX_TX_ACK;
}

static int at86rf215_cca(const struct device *dev)
{
	struct at86rf215_radio *r = dev->data;
	int ret;

	k_mutex_lock(&r->api_lock, K_FOREVER);
	k_mutex_lock(&r->chip->lock, K_FOREVER);

	if (!r->started) {
		ret = -ENETDOWN;
		goto unlock;
	}

	at86rf215_sync_irqs(r->chip);

	/* a frame is being received or transmitted */
	if (r->state != AT86RF215_TRX_IDLE || r->agc_hold || r->tx_pending) {
		ret = -EBUSY;
		goto unlock;
	}

	k_sem_reset(&r->cca_sem);
	r->cca_pending = true;

	/* disable the baseband during energy detection, radio stays in RX */
	bbc_write(r, AT86RF215_BBC_PC, r->pc & ~AT86RF215_PC_BBEN);
	rf_write(r, AT86RF215_RF_EDC, AT86RF215_EDC_EDM_SINGLE);

	k_mutex_unlock(&r->chip->lock);

	if (k_sem_take(&r->cca_sem, AT86RF215_CCA_TIMEOUT) == 0) {
		ret = r->cca_result;
		goto out;
	}

	LOG_ERR("CCA timeout");
	k_mutex_lock(&r->chip->lock, K_FOREVER);
	if (r->cca_pending) {
		r->cca_pending = false;
		rf_write(r, AT86RF215_RF_EDC, AT86RF215_EDC_EDM_AUTO);
		bbc_write(r, AT86RF215_BBC_PC, r->pc);
	}
	ret = -EIO;

unlock:
	k_mutex_unlock(&r->chip->lock);
out:
	k_mutex_unlock(&r->api_lock);

	return ret;
}

static int at86rf215_set_channel(const struct device *dev, uint16_t channel)
{
	struct at86rf215_radio *r = dev->data;
	uint8_t state;

	if (channel < r->channel_range.from_channel || channel > r->channel_range.to_channel) {
		return -EINVAL;
	}

	k_mutex_lock(&r->api_lock, K_FOREVER);
	k_mutex_lock(&r->chip->lock, K_FOREVER);

	rf_await_tx_end(r);

	/* the channel can only be changed in TRXOFF or TXPREP */
	state = rf_state(r);
	if (state == AT86RF215_STATE_RX) {
		rf_cmd(r, AT86RF215_CMD_TXPREP);
	}

	at86rf215_write_channel(r, channel);

	if (r->started) {
		at86rf215_stop_timer(r);
		r->state = AT86RF215_TRX_IDLE;
		rf_cmd(r, AT86RF215_CMD_RX);
	}

	k_mutex_unlock(&r->chip->lock);
	k_mutex_unlock(&r->api_lock);

	LOG_DBG("Channel %u", channel);

	return 0;
}

static int at86rf215_filter(const struct device *dev, bool set, enum ieee802154_filter_type type,
			    const struct ieee802154_filter *filter)
{
	struct at86rf215_radio *r = dev->data;
	uint8_t buf[2];
	int ret = 0;

	if (!set) {
		return -ENOTSUP;
	}

	k_mutex_lock(&r->chip->lock, K_FOREVER);

	switch (type) {
	case IEEE802154_FILTER_TYPE_IEEE_ADDR:
		/* little endian, as is MACEA0..7 */
		at86rf215_write(r->chip, r->bbc_base + AT86RF215_BBC_MACEA0, filter->ieee_addr, 8);
		break;
	case IEEE802154_FILTER_TYPE_SHORT_ADDR:
		sys_put_le16(filter->short_addr, buf);
		at86rf215_write(r->chip, r->bbc_base + AT86RF215_BBC_MACSHA0F0, buf, sizeof(buf));
		break;
	case IEEE802154_FILTER_TYPE_PAN_ID:
		sys_put_le16(filter->pan_id, buf);
		at86rf215_write(r->chip, r->bbc_base + AT86RF215_BBC_MACPID0F0, buf, sizeof(buf));
		break;
	default:
		ret = -ENOTSUP;
		break;
	}

	k_mutex_unlock(&r->chip->lock);

	return ret;
}

static int at86rf215_set_txpower(const struct device *dev, int16_t dbm)
{
	struct at86rf215_radio *r = dev->data;
	int txpwr = CLAMP(dbm + AT86RF215_TXPWR_OFFSET_DBM, 0,
			  FIELD_GET(AT86RF215_PAC_TXPWR_MASK, AT86RF215_PAC_TXPWR_MASK));

	k_mutex_lock(&r->chip->lock, K_FOREVER);
	rf_write(r, AT86RF215_RF_PAC,
		 FIELD_PREP(AT86RF215_PAC_PACUR_MASK, AT86RF215_PACUR_MAX) |
			 FIELD_PREP(AT86RF215_PAC_TXPWR_MASK, txpwr));
	k_mutex_unlock(&r->chip->lock);

	LOG_DBG("TX power %d dBm (TXPWR %d)", dbm, txpwr);

	return 0;
}

static int at86rf215_tx(const struct device *dev, enum ieee802154_tx_mode mode, struct net_pkt *pkt,
			struct net_buf *frag)
{
	struct at86rf215_radio *r = dev->data;
	uint16_t psdu_len = frag->len - AT86RF215_L2_FCS_LEN + IEEE802154_FCS_LENGTH;
	uint8_t buf[2];
	int ret;

	ARG_UNUSED(pkt);

	if (psdu_len > IEEE802154_MAX_PHY_PACKET_SIZE || frag->len < IEEE802154_ACK_PKT_LENGTH) {
		return -EINVAL;
	}

	k_mutex_lock(&r->api_lock, K_FOREVER);

	switch (mode) {
	case IEEE802154_TX_MODE_DIRECT:
	case IEEE802154_TX_MODE_CSMA_CA:
		break;
	case IEEE802154_TX_MODE_CCA:
		ret = at86rf215_cca(dev);
		if (ret < 0) {
			goto out;
		}
		break;
	default:
		LOG_ERR("TX mode %d not supported", mode);
		ret = -ENOTSUP;
		goto out;
	}

	k_mutex_lock(&r->chip->lock, K_FOREVER);

	if (!r->started) {
		k_mutex_unlock(&r->chip->lock);
		ret = -ENETDOWN;
		goto out;
	}

	/* the FCS is appended by the transceiver */
	at86rf215_write(r->chip, r->fb_tx, frag->data, psdu_len - IEEE802154_FCS_LENGTH);
	sys_put_le16(psdu_len, buf);
	at86rf215_write(r->chip, r->bbc_base + AT86RF215_BBC_TXFLL, buf, sizeof(buf));

	r->ack_requested = frag->data[0] & AT86RF215_FCF_AR;
	r->tx_seq = frag->data[2];
	r->retries = CONFIG_IEEE802154_AT86RF215_TX_RETRIES;
	r->csma = mode == IEEE802154_TX_MODE_CSMA_CA;
	r->tx_result = -EIO;
	k_sem_reset(&r->tx_sem);
	r->tx_pending = true;

	at86rf215_sync_irqs(r->chip);

	if (r->csma) {
		at86rf215_csma_start(r);
	} else if (r->state == AT86RF215_TRX_IDLE && r->tx_pending && !r->txprep_issued &&
		   !r->agc_hold) {
		/*
		 * TXPREP must not be issued during an ongoing reception, the
		 * transmission is then started once the radio is idle again.
		 */
		at86rf215_txprep(r);
	}

	k_mutex_unlock(&r->chip->lock);

	ret = k_sem_take(&r->tx_sem, K_MSEC(r->tx_timeout_ms));

	k_mutex_lock(&r->chip->lock, K_FOREVER);
	if (ret < 0) {
		LOG_ERR("TX timeout");
		at86rf215_abort(r);
		rf_cmd(r, AT86RF215_CMD_RX);
		ret = -EIO;
	} else {
		ret = r->tx_result;
	}
	k_mutex_unlock(&r->chip->lock);

out:
	k_mutex_unlock(&r->api_lock);

	return ret;
}

static int at86rf215_start(const struct device *dev)
{
	struct at86rf215_radio *r = dev->data;

	k_mutex_lock(&r->chip->lock, K_FOREVER);

	if (!r->started) {
		r->state = AT86RF215_TRX_IDLE;
		r->agc_hold = false;
		r->started = true;
		rf_cmd(r, AT86RF215_CMD_RX);
	}

	k_mutex_unlock(&r->chip->lock);

	return 0;
}

static int at86rf215_stop(const struct device *dev)
{
	struct at86rf215_radio *r = dev->data;

	k_mutex_lock(&r->chip->lock, K_FOREVER);

	if (r->started) {
		r->started = false;
		at86rf215_abort(r);
		rf_cmd(r, AT86RF215_CMD_TRXOFF);
		k_sem_give(&r->tx_sem);
		k_sem_give(&r->cca_sem);
	}

	k_mutex_unlock(&r->chip->lock);

	return 0;
}

static int at86rf215_configure(const struct device *dev, enum ieee802154_config_type type,
			       const struct ieee802154_config *config)
{
	struct at86rf215_radio *r = dev->data;
	uint8_t afc;
	int ret = 0;

	k_mutex_lock(&r->api_lock, K_FOREVER);
	k_mutex_lock(&r->chip->lock, K_FOREVER);

	switch (type) {
	case IEEE802154_CONFIG_PROMISCUOUS:
		r->promiscuous = config->promiscuous;

		afc = bbc_read(r, AT86RF215_BBC_AFC0);
		afc = r->promiscuous ? (afc | AT86RF215_AFC0_PM) : (afc & ~AT86RF215_AFC0_PM);
		bbc_write(r, AT86RF215_BBC_AFC0, afc);

		/* do not acknowledge frames in promiscuous mode */
		if (r->promiscuous) {
			r->amcs &= ~AT86RF215_AMCS_AACK;
		} else {
			r->amcs |= AT86RF215_AMCS_AACK;
		}
		bbc_write(r, AT86RF215_BBC_AMCS, r->amcs);
		break;
	case IEEE802154_CONFIG_PAN_COORDINATOR:
		afc = bbc_read(r, AT86RF215_BBC_AFC1);
		afc = config->pan_coordinator ? (afc | AT86RF215_AFC1_PANC0)
					      : (afc & ~AT86RF215_AFC1_PANC0);
		bbc_write(r, AT86RF215_BBC_AFC1, afc);
		break;
	default:
		ret = -ENOTSUP;
		break;
	}

	k_mutex_unlock(&r->chip->lock);
	k_mutex_unlock(&r->api_lock);

	return ret;
}

static int at86rf215_attr_get(const struct device *dev, enum ieee802154_attr attr,
			      struct ieee802154_attr_value *value)
{
	struct at86rf215_radio *r = dev->data;

	return ieee802154_attr_get_channel_page_and_range(attr, r->channel_page, &r->channels,
							  value);
}

static void at86rf215_iface_init(struct net_if *iface)
{
	const struct device *dev = net_if_get_device(iface);
	const struct at86rf215_radio_config *cfg = dev->config;
	struct at86rf215_radio *r = dev->data;
	uint8_t mac_addr[8];

	if (cfg->has_mac) {
		memcpy(mac_addr, cfg->mac_addr, sizeof(mac_addr));
	} else {
		sys_rand_get(mac_addr, sizeof(mac_addr));
		/* unicast, locally administered address */
		mac_addr[0] = (mac_addr[0] & ~0x01) | 0x02;
	}

	/* the link address is copied into the interface */
	net_if_set_link_addr(iface, mac_addr, sizeof(mac_addr), NET_LINK_IEEE802154);

	r->iface = iface;

	ieee802154_init(iface);
}

static int at86rf215_radio_init(const struct device *dev)
{
	const struct at86rf215_radio_config *cfg = dev->config;
	struct at86rf215_radio *r = dev->data;
	struct at86rf215_chip *chip = cfg->chip;
	bool is_2_4ghz = cfg->idx == AT86RF215_RADIO_2_4GHZ;
	int ret;

	r->chip = chip;
	r->cfg = cfg;
	r->idx = cfg->idx;
	r->rf_base = is_2_4ghz ? AT86RF215_RF24_BASE : AT86RF215_RF09_BASE;
	r->bbc_base = is_2_4ghz ? AT86RF215_BBC1_BASE : AT86RF215_BBC0_BASE;
	r->fb_rx = is_2_4ghz ? AT86RF215_BBC1_FBRXS : AT86RF215_BBC0_FBRXS;
	r->fb_tx = is_2_4ghz ? AT86RF215_BBC1_FBTXS : AT86RF215_BBC0_FBTXS;

	k_mutex_init(&r->api_lock);
	k_sem_init(&r->tx_sem, 0, 1);
	k_sem_init(&r->cca_sem, 0, 1);
	k_timer_init(&r->timer, at86rf215_timer_handler, NULL);
	k_timer_user_data_set(&r->timer, r);
	k_timer_init(&r->backoff_timer, at86rf215_backoff_handler, NULL);
	k_timer_user_data_set(&r->backoff_timer, r);

	ret = at86rf215_phy_init(r);
	if (ret < 0) {
		return ret;
	}

	ret = at86rf215_chip_init(chip);
	if (ret < 0) {
		return ret;
	}

	if (is_2_4ghz && chip->pn == AT86RF215_PN_AT86RF215M) {
		LOG_ERR("AT86RF215M has no 2.4 GHz radio");
		return -ENODEV;
	}

	k_mutex_lock(&chip->lock, K_FOREVER);
	ret = at86rf215_radio_setup(r);
	if (ret == 0) {
		chip->radio[r->idx] = r;
	}
	k_mutex_unlock(&chip->lock);

	return ret;
}

static const struct ieee802154_radio_api at86rf215_radio_api = {
	.iface_api.init = at86rf215_iface_init,

	.get_capabilities = at86rf215_get_capabilities,
	.cca = at86rf215_cca,
	.set_channel = at86rf215_set_channel,
	.filter = at86rf215_filter,
	.set_txpower = at86rf215_set_txpower,
	.tx = at86rf215_tx,
	.start = at86rf215_start,
	.stop = at86rf215_stop,
	.configure = at86rf215_configure,
	.attr_get = at86rf215_attr_get,
};

#if !defined(CONFIG_IEEE802154_RAW_MODE)
#if defined(CONFIG_NET_L2_IEEE802154)
#define L2          IEEE802154_L2
#define L2_CTX_TYPE NET_L2_GET_CTX_TYPE(IEEE802154_L2)
#define MTU         IEEE802154_MTU
#elif defined(CONFIG_NET_L2_OPENTHREAD)
#define L2          OPENTHREAD_L2
#define L2_CTX_TYPE NET_L2_GET_CTX_TYPE(OPENTHREAD_L2)
#define MTU         1280
#endif
#endif /* CONFIG_IEEE802154_RAW_MODE */

#define AT86RF215_RADIO_IS_2_4GHZ(node_id) (DT_REG_ADDR(node_id) == AT86RF215_RADIO_2_4GHZ)

#define AT86RF215_RADIO_SUN_BAND(node_id)                                                          \
	(DT_ENUM_HAS_VALUE(node_id, phy, mr_oqpsk) ? DT_PROP_OR(node_id, sun_band, 0) : 0)

#define AT86RF215_RADIO_CHIP_RATE(node_id)                                                         \
	DT_PROP_OR(node_id, chip_rate, (AT86RF215_RADIO_SUN_BAND(node_id) == 2450 ? 2000 : 100))

/*
 * The 2450 MHz band uses 2000 kchip/s (IEEE 802.15.4-2024 table 22-2), the
 * sub-GHz bands allow any chip rate, including ones not defined for the band.
 */
#define AT86RF215_SUN_CHIP_RATE_VALID(band, rate) ((band) != 2450 || (rate) == 2000)

#define AT86RF215_RADIO_NAME(prefix, node_id) _CONCAT(prefix, DT_DEP_ORD(node_id))

#define AT86RF215_RADIO_DEVICE(node_id)                                                            \
	COND_CODE_1(CONFIG_IEEE802154_RAW_MODE,                                                \
		    (DEVICE_DT_DEFINE(node_id, at86rf215_radio_init, NULL,                     \
				      &AT86RF215_RADIO_NAME(at86rf215_radio_data_, node_id),   \
				      &AT86RF215_RADIO_NAME(at86rf215_radio_cfg_, node_id),    \
				      POST_KERNEL, CONFIG_IEEE802154_AT86RF215_INIT_PRIO,      \
				      &at86rf215_radio_api)),                                  \
		    (NET_DEVICE_DT_DEFINE(node_id, at86rf215_radio_init, NULL,                 \
					  &AT86RF215_RADIO_NAME(at86rf215_radio_data_, node_id), \
					  &AT86RF215_RADIO_NAME(at86rf215_radio_cfg_, node_id), \
					  CONFIG_IEEE802154_AT86RF215_INIT_PRIO,               \
					  &at86rf215_radio_api, L2, L2_CTX_TYPE, MTU)))

#define AT86RF215_RADIO_DEFINE(node_id, inst)                                                      \
	BUILD_ASSERT(DT_REG_ADDR(node_id) < AT86RF215_NUM_RADIOS,                                  \
		     "at86rf215: radio reg must be 0 (sub-GHz) or 1 (2.4 GHz)");                   \
	BUILD_ASSERT(!DT_NODE_HAS_PROP(node_id, local_mac_address) ||                              \
			     DT_PROP_LEN_OR(node_id, local_mac_address, 0) == 8,                   \
		     "at86rf215: local-mac-address must be 8 bytes");                              \
	BUILD_ASSERT(!DT_ENUM_HAS_VALUE(node_id, phy, mr_oqpsk) ||                                 \
			     AT86RF215_RADIO_SUN_BAND(node_id) != 0,                               \
		     "at86rf215: sun-band is required for the mr-oqpsk PHY");                      \
	BUILD_ASSERT(AT86RF215_RADIO_SUN_BAND(node_id) == 0 ||                                     \
			     (AT86RF215_RADIO_SUN_BAND(node_id) == 2450) ==                        \
				     AT86RF215_RADIO_IS_2_4GHZ(node_id),                           \
		     "at86rf215: sun-band 2450 requires radio@1, other bands radio@0");            \
	BUILD_ASSERT(AT86RF215_RADIO_SUN_BAND(node_id) == 0 ||                                     \
			     AT86RF215_SUN_CHIP_RATE_VALID(AT86RF215_RADIO_SUN_BAND(node_id),      \
							   AT86RF215_RADIO_CHIP_RATE(node_id)),    \
		     "at86rf215: chip-rate not supported in sun-band");                            \
	static struct at86rf215_radio AT86RF215_RADIO_NAME(at86rf215_radio_data_, node_id) = {     \
		.channels =                                                                        \
			{                                                                          \
				.ranges = &AT86RF215_RADIO_NAME(at86rf215_radio_data_, node_id)    \
						   .channel_range,                                 \
				.num_ranges = 1U,                                                  \
			},                                                                         \
	};                                                                                         \
	static const struct at86rf215_radio_config AT86RF215_RADIO_NAME(at86rf215_radio_cfg_,      \
									node_id) = {               \
		.chip = &at86rf215_chip_##inst,                                                    \
		.mac_addr = DT_PROP_OR(node_id, local_mac_address, {0}),                           \
		.sun_band = AT86RF215_RADIO_SUN_BAND(node_id),                                     \
		.chip_rate = AT86RF215_RADIO_CHIP_RATE(node_id),                                   \
		.idx = DT_REG_ADDR(node_id),                                                       \
		.rate_mode = DT_PROP(node_id, rate_mode),                                          \
		.has_mac = DT_NODE_HAS_PROP(node_id, local_mac_address),                           \
	};                                                                                         \
	AT86RF215_RADIO_DEVICE(node_id);

#define AT86RF215_RADIO_BIT(node_id) | BIT(DT_REG_ADDR(node_id))

#define AT86RF215_INIT(inst)                                                                       \
	BUILD_ASSERT(!DT_INST_NODE_HAS_PROP(inst, xtal_trim) ||                                    \
			     DT_INST_PROP_OR(inst, xtal_trim, 0) <= 15,                            \
		     "at86rf215: xtal-trim must be in the range 0-15");                            \
	K_KERNEL_STACK_DEFINE(at86rf215_stack_##inst, CONFIG_IEEE802154_AT86RF215_RX_STACK_SIZE);  \
	static const struct at86rf215_chip_config at86rf215_chip_cfg_##inst = {                    \
		.spi = SPI_DT_SPEC_INST_GET(inst, SPI_WORD_SET(8) | SPI_TRANSFER_MSB),             \
		.irq_gpio = GPIO_DT_SPEC_INST_GET(inst, irq_gpios),                                \
		.reset_gpio = GPIO_DT_SPEC_INST_GET(inst, reset_gpios),                            \
		.xtal_trim = DT_INST_PROP_OR(inst, xtal_trim, -1),                                 \
		.radio_mask = 0 DT_INST_FOREACH_CHILD_STATUS_OKAY(inst, AT86RF215_RADIO_BIT),      \
		.stack = at86rf215_stack_##inst,                                                   \
		.stack_size = K_KERNEL_STACK_SIZEOF(at86rf215_stack_##inst),                       \
	};                                                                                         \
	static struct at86rf215_chip at86rf215_chip_##inst = {                                     \
		.cfg = &at86rf215_chip_cfg_##inst,                                                 \
	};                                                                                         \
	DT_INST_FOREACH_CHILD_STATUS_OKAY_VARGS(inst, AT86RF215_RADIO_DEFINE, inst)

DT_INST_FOREACH_STATUS_OKAY(AT86RF215_INIT)
