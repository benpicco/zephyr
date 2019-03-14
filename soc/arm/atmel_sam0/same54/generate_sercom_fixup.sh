#!/bin/bash
#
# Copyright (c) 2019 ML!PA Consulting GmbH
#
# SPDX-License-Identifier: Apache-2.0

for j in {0..7}; do
	for i in {A..D}; do
		echo "#ifdef MCLK_APB${i}MASK_SERCOM${j}"
		echo "#define MCLK_SERCOM${j} (&MCLK->APB${i}MASK.reg)"
		echo "#define MCLK_SERCOM${j}_MASK (MCLK_APB${i}MASK_SERCOM${j})"
		echo "#endif"
	done;
	echo
done
