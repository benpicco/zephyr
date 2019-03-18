/*
 * Copyright (c) 2019 Benjamin Valentin
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <init.h>
#include <pinmux.h>

static int board_pinmux_init(struct device *dev)
{
	struct device *muxa = device_get_binding(DT_PINMUX_SAM0_A_LABEL);
	struct device *muxb = device_get_binding(DT_PINMUX_SAM0_B_LABEL);
	struct device *muxd = device_get_binding(DT_PINMUX_SAM0_D_LABEL);

	ARG_UNUSED(dev);

#if DT_UART_SAM0_SERCOM0_BASE_ADDRESS
#error Pin mapping is not configured
#endif
#if DT_UART_SAM0_SERCOM1_BASE_ADDRESS
#error Pin mapping is not configured
#endif
#if DT_UART_SAM0_SERCOM2_BASE_ADDRESS
	/* SERCOM2 ON RX=PB24, TX=PB25 */
	pinmux_pin_set(muxb, 24, PINMUX_FUNC_D);
	pinmux_pin_set(muxb, 25, PINMUX_FUNC_D);
#endif
#if DT_UART_SAM0_SERCOM3_BASE_ADDRESS
#error Pin mapping is not configured
#endif
#if DT_UART_SAM0_SERCOM4_BASE_ADDRESS
#error Pin mapping is not configured
#endif
#if DT_UART_SAM0_SERCOM5_BASE_ADDRESS
#error Pin mapping is not configured
#endif
#if DT_UART_SAM0_SERCOM6_BASE_ADDRESS
#error Pin mapping is not configured
#endif
#if DT_UART_SAM0_SERCOM7_BASE_ADDRESS
#error Pin mapping is not configured
#endif

#if DT_SPI_SAM0_SERCOM0_BASE_ADDRESS
#error Pin mapping is not configured
#endif
#if DT_SPI_SAM0_SERCOM1_BASE_ADDRESS
#error Pin mapping is not configured
#endif
#if DT_SPI_SAM0_SERCOM2_BASE_ADDRESS
#error Pin mapping is not configured
#endif
#if DT_SPI_SAM0_SERCOM3_BASE_ADDRESS
#error Pin mapping is not configured
#endif
#if DT_SPI_SAM0_SERCOM4_BASE_ADDRESS
#error Pin mapping is not configured
#endif
#if DT_SPI_SAM0_SERCOM5_BASE_ADDRESS
#error Pin mapping is not configured
#endif
#if DT_SPI_SAM0_SERCOM6_BASE_ADDRESS
#error Pin mapping is not configured
#endif
#if DT_SPI_SAM0_SERCOM7_BASE_ADDRESS
#error Pin mapping is not configured
#endif

#if DT_I2C_SAM0_SERCOM0_BASE_ADDRESS
#error Pin mapping is not configured
#endif
#if DT_I2C_SAM0_SERCOM1_BASE_ADDRESS
#error Pin mapping is not configured
#endif
#if DT_I2C_SAM0_SERCOM2_BASE_ADDRESS
#error Pin mapping is not configured
#endif
#if DT_I2C_SAM0_SERCOM3_BASE_ADDRESS
#error Pin mapping is not configured
#endif
#if DT_I2C_SAM0_SERCOM4_BASE_ADDRESS
#error Pin mapping is not configured
#endif
#if DT_I2C_SAM0_SERCOM5_BASE_ADDRESS
#error Pin mapping is not configured
#endif
#if DT_I2C_SAM0_SERCOM6_BASE_ADDRESS
#error Pin mapping is not configured
#endif
#if DT_I2C_SAM0_SERCOM7_BASE_ADDRESS
	pinmux_pin_set(muxd, 8, PINMUX_FUNC_C);
	pinmux_pin_set(muxd, 9, PINMUX_FUNC_C);
#endif

#ifdef CONFIG_USB_DC_SAM0
	/* USB DP on PA25, USB DM on PA24 */
	pinmux_pin_set(muxa, 25, PINMUX_FUNC_H);
	pinmux_pin_set(muxa, 24, PINMUX_FUNC_H);
#endif
	return 0;
}

SYS_INIT(board_pinmux_init, PRE_KERNEL_1, CONFIG_PINMUX_INIT_PRIORITY);
