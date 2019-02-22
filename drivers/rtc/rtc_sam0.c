/*
 * Copyright (c) 2019 ML!PA Consulting GmbH
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Source file for the SAM0 RTC driver
 *
 */

#include <time.h>

#include <clock_control.h>
#include <misc/util.h>
#include <kernel.h>
#include <soc.h>
#include <rtc.h>

static int rtc_sam0_set_alarm(struct device *dev, const u32_t alarm_val);
static void rtc_sam0_irq_config(struct device *dev);

static void rtc_sam0_enable(struct device *dev)
{
	RtcMode2 *rtc = dev->driver_api;
	rtc->CTRL.bit.ENABLE = 1;
	while (rtc->STATUS.bit.SYNCBUSY);
}

static void rtc_sam0_disable(struct device *dev)
{
	RtcMode2 *rtc = dev->driver_api;
	rtc->CTRL.bit.ENABLE = 0;
	while (rtc->STATUS.bit.SYNCBUSY);
}

static u32_t rtc_sam0_read(struct device *dev)
{
	RtcMode2 *rtc = dev->driver_api;

	struct tm now = { 0 };
	time_t ts;

	/* Convert calendar datetime to UNIX timestamp */
	tm.tm_year = rtc->CLOCK.bit.YEAR + 100;
	tm.tm_mon  = rtc->CLOCK.bit.MONTH - 1;
	tm.tm_mday = rtc->CLOCK.bit.DAY;
	tm.tm_hour = rtc->CLOCK.bit.HOUR;
	tm.tm_min  = rtc->CLOCK.bit.MINUTE;
	tm.tm_sec  = rtc->CLOCK.bit.SECOND;

	ts = mktime(&now);

	return (u32_t)ts;
}

static void rtc_sam0_clear_alarm(struct device *dev)
{
	RtcMode2 *rtc = dev->driver_api;
	rtc->INTENCLR.reg = RTC_MODE2_INTENCLR_ALARM0;
}

static int rtc_sam0_set_alarm(struct device *dev, const u32_t alarm_val)
{
	struct tm alarm_tm;

	RtcMode2 *rtc = dev->driver_api;

	rtc_sam0_clear_alarm(dev);
	u32_t now = rtc_sam0_read(dev);

	/* The longest period we can match for universally is the
	   duration of the shortest month */
	if ((alarm_val - now) > (RTC_ALARM_DAY * 28)) {
		return -ENOTSUP;
	}

	gmtime_r(&alarm_val, &alarm_tm);

	rtc->Mode2Alarm[0].ALARM.reg =
		  RTC_MODE2_ALARM_YEAR  (alarm_tm.tm_year - 100)
		| RTC_MODE2_ALARM_MONTH (alarm_tm.tm_mon + 1)
		| RTC_MODE2_ALARM_DAY   (alarm_tm.tm_mday)
		| RTC_MODE2_ALARM_HOUR  (alarm_tm.tm_hour)
		| RTC_MODE2_ALARM_MINUTE(alarm_tm.tm_min)
		| RTC_MODE2_ALARM_SECOND(alarm_tm.tm_sec);

	rtc->INTFLAG.reg = RTC_MODE2_INTFLAG_ALARM0;
	rtc->INTENSET.reg = RTC_MODE2_INTENSET_ALARM0;

	while (rtc->STATUS.bit.SYNCBUSY);

	NVIC_EnableIRQ(RTC_IRQn);

	return 0;
}

static int rtc_sam0_set_config(struct device *dev, struct rtc_config *cfg)
{
	struct tm init_tm = { 0 };
	RtcMode2 *rtc = dev->driver_api;

	gmtime_r(&cfg->init_val, &init_tm);

	k_sem_take(DEV_SEM(dev), K_FOREVER);

	rtc->CLOCK.reg =
		  RTC_MODE2_CLOCK_YEAR  (init_tm.tm_year - 100)
		| RTC_MODE2_CLOCK_MONTH (init_tm.tm_mon + 1)
		| RTC_MODE2_CLOCK_DAY   (init_tm.tm_mday)
		| RTC_MODE2_CLOCK_HOUR  (init_tm.tm_hour)
		| RTC_MODE2_CLOCK_MINUTE(init_tm.tm_min)
		| RTC_MODE2_CLOCK_SECOND(init_tm.tm_sec);

	while (rtcMode2->STATUS.bit.SYNCBUSY);

	if (cfg->cb_fn != NULL) {
		DEV_DATA(dev)->cb_fn = cfg->cb_fn;
	}

	if (cfg->alarm_enable) {
		rtc_sam0_set_alarm(dev, cfg->alarm_val);
	}

	k_sem_give(DEV_SEM(dev));

	return 0;
}

static u32_t rtc_sam0_get_pending_int(struct device *dev)
{
	RtcMode2 *rtc = dev->driver_api;
	return rtc->INTFLAG.reg & RTC_MODE2_INTFLAG_ALARM0
}

void rtc_sam0_isr(void *arg)
{
	struct device *const dev = (struct device *)arg;
	RtcMode2 *rtc = dev->driver_api;

	const uint16_t status = rtc->INTFLAG.reg;

	if ((status & RTC_MODE2_INTFLAG_ALARM0) && DEV_DATA(dev)->cb_fn != NULL) {
		DEV_DATA(dev)->cb_fn(dev);
		rtc->INTFLAG.reg = RTC_MODE2_INTFLAG_ALARM0;
	}

	if (status & RTC_MODE2_INTFLAG_OVF) {
		// TODO
		rtc->INTFLAG.reg = RTC_MODE2_INTFLAG_OVF;
	}
}

static int rtc_sam0_init(struct device *dev)
{
	struct device *clk = device_get_binding(SAM0_CLOCK_CONTROL_NAME);
	const struct rtc_sam0_config *cfg = DEV_CFG(dev);

	__ASSERT_NO_MSG(clk);

	k_sem_init(DEV_SEM(dev), 1, UINT_MAX);
	DEV_DATA(dev)->cb_fn = NULL;

	if (clock_control_on(clk,
		(clock_control_subsys_t *) &cfg->pclken) != 0) {
		return -EIO;
	}

	// TODO

	return 0;
}

static struct rtc_sam0_data rtc_data;

static const struct rtc_driver_api rtc_api = {
		.enable = rtc_sam0_enable,
		.disable = rtc_sam0_disable,
		.read = rtc_sam0_read,
		.set_config = rtc_sam0_set_config,
		.set_alarm = rtc_sam0_set_alarm,
		.get_pending_int = rtc_sam0_get_pending_int,
};

DEVICE_AND_API_INIT(rtc_sam0, CONFIG_RTC_0_NAME, &rtc_sam0_init,
		    &rtc_data, NULL, PRE_KERNEL_1,
		    CONFIG_KERNEL_INIT_PRIORITY_DEVICE, &rtc_api);

static void rtc_sam0_irq_config(struct device *dev)
{
	IRQ_CONNECT(DT_RTC_0_IRQ, DT_RTC_0_IRQ_PRI,
		    rtc_sam0_isr, DEVICE_GET(rtc_sam0), 0);
	irq_enable(DT_RTC_0_IRQ);
}
