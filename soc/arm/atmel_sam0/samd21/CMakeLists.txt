# Makefile - Atmel SAMD21 MCU series.
#
# Copyright (c) 2017 Google LLC.
# SPDX-License-Identifier: Apache-2.0

zephyr_include_directories(${ZEPHYR_BASE}/drivers)
zephyr_sources(soc.c)
