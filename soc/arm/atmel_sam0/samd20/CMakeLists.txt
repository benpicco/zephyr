# Makefile - Atmel SAMD20 MCU series.
#
# Copyright (c) 2018 Sean Nyekjaer
# SPDX-License-Identifier: Apache-2.0

zephyr_include_directories(${ZEPHYR_BASE}/drivers)
zephyr_sources(soc.c)
