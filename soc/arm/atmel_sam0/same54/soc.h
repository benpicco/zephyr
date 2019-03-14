/*
 * Copyright (c) 2019 ML!PA Consulting GmbH
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef _ATMEL_SAME_SOC_H_
#define _ATMEL_SAME_SOC_H_

#ifndef _ASMLANGUAGE

#define DONT_USE_CMSIS_INIT

#include <zephyr/types.h>

#if defined(CONFIG_SOC_PART_NUMBER_SAME54N19A)
#include <same54n19a.h>
#elif defined(CONFIG_SOC_PART_NUMBER_SAME54N20A)
#include <same54n20a.h>
#elif defined(CONFIG_SOC_PART_NUMBER_SAME54P19A)
#include <same54p19a.h>
#elif defined(CONFIG_SOC_PART_NUMBER_SAME54P20A)
#include <same54p20a.h>
#else
#error Library does not support the specified device.
#endif

#endif /* _ASMLANGUAGE */

/** Processor Clock (HCLK) Frequency */
#define SOC_ATMEL_SAM0_HCLK_FREQ_HZ CONFIG_SYS_CLOCK_HW_CYCLES_PER_SEC
/** Master Clock (MCK) Frequency */
#define SOC_ATMEL_SAM0_MCK_FREQ_HZ SOC_ATMEL_SAM0_HCLK_FREQ_HZ
#define SOC_ATMEL_SAM0_XOSC32K_FREQ_HZ 32768
#define SOC_ATMEL_SAM0_OSC8M_FREQ_HZ 8000000
#define SOC_ATMEL_SAM0_GCLK0_FREQ_HZ SOC_ATMEL_SAM0_MCK_FREQ_HZ

#endif /* _ATMEL_SAME_SOC_H_ */
