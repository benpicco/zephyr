/*
 * Copyright (c) 2026 ML!PA Consulting GmbH
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_DRIVERS_IEEE802154_IEEE802154_AT86RF215_H_
#define ZEPHYR_DRIVERS_IEEE802154_IEEE802154_AT86RF215_H_

#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/net/ieee802154_radio.h>

#define AT86RF215_RADIO_SUBGHZ 0
#define AT86RF215_RADIO_2_4GHZ 1
#define AT86RF215_NUM_RADIOS   2

/* Software state of a radio */
enum at86rf215_trx_state {
	/* Receiver on, waiting for frames */
	AT86RF215_TRX_IDLE,
	/* CCA with automatic transmission (CCATX) in progress */
	AT86RF215_TRX_CCATX,
	/* Frame with ACK request received, waiting for AACK to finish */
	AT86RF215_TRX_RX_SEND_ACK,
	/* Frame transmission in progress */
	AT86RF215_TRX_TX,
	/* Frame transmitted, waiting for the ACK frame */
	AT86RF215_TRX_TX_WAIT_ACK,
};

struct at86rf215_radio;

struct at86rf215_chip_config {
	struct spi_dt_spec spi;
	struct gpio_dt_spec irq_gpio;
	struct gpio_dt_spec reset_gpio;
	int16_t xtal_trim;
	uint8_t radio_mask;
	k_thread_stack_t *stack;
	size_t stack_size;
};

/* State shared by both radios of a transceiver */
struct at86rf215_chip {
	const struct at86rf215_chip_config *cfg;
	struct at86rf215_radio *radio[AT86RF215_NUM_RADIOS];
	/* Protects SPI access and the radio state machines */
	struct k_mutex lock;
	struct k_sem isr_sem;
	struct gpio_callback irq_cb;
	struct k_thread thread;
	uint8_t pn;
	uint8_t vn;
	bool initialized;
};

struct at86rf215_radio_config {
	struct at86rf215_chip *chip;
	uint8_t mac_addr[8];
	/* SUN band designation (MHz) of the MR-O-QPSK PHY, 0 selects legacy O-QPSK */
	uint16_t sun_band;
	/* MR-O-QPSK chip rate in kchip/s */
	uint16_t chip_rate;
	uint8_t idx;
	/* MR-O-QPSK rate mode (0-3) */
	uint8_t rate_mode;
	bool has_mac;
};

struct at86rf215_radio {
	/* ACK timeout and AACK watchdog */
	struct k_timer timer;
	/* CSMA-CA backoff period */
	struct k_timer backoff_timer;

	struct net_if *iface;
	struct at86rf215_chip *chip;
	const struct at86rf215_radio_config *cfg;

	/* Serializes API calls that use multiple chip transactions */
	struct k_mutex api_lock;
	struct k_sem tx_sem;
	struct k_sem cca_sem;
	atomic_t timeout;

	int tx_result;
	int cca_result;
	/* PHY dependent timing, see at86rf215_phy_init() */
	uint32_t ack_timeout_us;
	uint32_t aack_timeout_us;
	uint32_t tx_timeout_ms;
	/* Channel center frequency: center0 + channel * spacing */
	uint32_t center0_khz;
	struct ieee802154_phy_channel_range channel_range;
	struct ieee802154_phy_supported_channels channels;
	enum ieee802154_phy_channel_page channel_page;

	uint16_t rf_base;
	uint16_t bbc_base;
	uint16_t fb_rx;
	uint16_t fb_tx;
	uint16_t channel;
	uint16_t spacing_khz;
	uint16_t unit_backoff_us;
	/* Time between the end of a received frame and the start of its ACK */
	uint16_t aifs_us;
	enum at86rf215_trx_state state;
	uint8_t idx;
	/* OQPSKC0.FCHIP */
	uint8_t fchip;
	uint8_t pc;
	uint8_t amcs;
	uint8_t tx_seq;
	uint8_t retries;
	/* CSMA-CA number of backoffs (NB) and backoff exponent (BE) */
	uint8_t nb;
	uint8_t be;
	int8_t cca_threshold;
	/* Bitfields share a word: only access with chip->lock held */
	bool started: 1;
	bool promiscuous: 1;
	bool tx_pending: 1;
	bool csma: 1;
	bool txprep_issued: 1;
	bool ack_requested: 1;
	bool cca_pending: 1;
	bool agc_hold: 1;

	uint8_t rx_buf[IEEE802154_MAX_PHY_PACKET_SIZE];
};

#endif /* ZEPHYR_DRIVERS_IEEE802154_IEEE802154_AT86RF215_H_ */
