/*
 * Copyright (c) 2025 Microchip Technology Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/*
 * PIC32CX-BZ6's ETH peripheral is register-compatible with the GMAC IP block
 * used by other Microchip SoC families, but its SoC header
 * (component/eth.h) names the register struct/type "eth_registers_t" and
 * prefixes all fields with "ETH_" instead of "GMAC_". This header provides
 * the aliases needed so that the shared eth_mchp_gmac_g1.c driver, which is
 * written against the "GMAC_"-prefixed naming, builds unmodified for BZ6.
 */

#ifndef ZEPHYR_DRIVERS_ETHERNET_ETH_MCHP_GMAC_G1_BZ6_H_
#define ZEPHYR_DRIVERS_ETHERNET_ETH_MCHP_GMAC_G1_BZ6_H_

typedef eth_registers_t gmac_registers_t;

#define GMAC_CTRLA           ETH_CTRLA
#define GMAC_CTRLA_ENABLE_Msk ETH_CTRLA_ENABLE_Msk
#define GMAC_SYNCB           ETH_SYNCB
#define GMAC_CTRLB           ETH_CTRLB

#define GMAC_NCR      ETH_NCR
#define GMAC_NCFGR    ETH_NCFGR
#define GMAC_UR       ETH_UR
#define GMAC_DCFGR    ETH_DCFGR
#define GMAC_RBQB     ETH_RBQB
#define GMAC_TBQB     ETH_TBQB
#define GMAC_RSR      ETH_RSR
#define GMAC_TSR      ETH_TSR
#define GMAC_ISR      ETH_ISR
#define GMAC_IER      ETH_IER
#define GMAC_IDR      ETH_IDR
#define GMAC_HRB      ETH_HRB
#define GMAC_HRT      ETH_HRT
#define GMAC_SAB      ETH_SAB
#define GMAC_SAT      ETH_SAT
#define GMAC_TBQBAPQ  ETH_TBPQB

#define GMAC_MFT  ETH_MFT
#define GMAC_MCF  ETH_MCF
#define GMAC_EC   ETH_EC
#define GMAC_TUR  ETH_TUR
#define GMAC_SCF  ETH_SCF
#define GMAC_MFR  ETH_MFR
#define GMAC_UFR  ETH_UFR
#define GMAC_OFR  ETH_OFR
#define GMAC_JR   ETH_JR
#define GMAC_FCSE ETH_FCSE
#define GMAC_LFFE ETH_LFFE
#define GMAC_RSE  ETH_RSE
#define GMAC_AE   ETH_AE
#define GMAC_RRE  ETH_RRE
#define GMAC_ROE  ETH_ROE
#define GMAC_IHCE ETH_IHCE
#define GMAC_TCE  ETH_TCE
#define GMAC_UCE  ETH_UCE
#define GMAC_CSE  ETH_CSE

#define GMAC_NCR_TXEN_Msk    ETH_NCR_TXEN_Msk
#define GMAC_NCR_RXEN_Msk    ETH_NCR_RXEN_Msk
#define GMAC_NCR_MPE_Msk     ETH_NCR_MPE_Msk
#define GMAC_NCR_CLRSTAT_Msk ETH_NCR_CLRSTAT_Msk
#define GMAC_NCR_TSTART_Msk  ETH_NCR_TSTART_Msk

#define GMAC_NCFGR_SPD_Msk    ETH_NCFGR_SPD_Msk
#define GMAC_NCFGR_FD_Msk     ETH_NCFGR_FD_Msk
#define GMAC_NCFGR_MAXFS_Msk  ETH_NCFGR_MAXFS_Msk
#define GMAC_NCFGR_RFCS_Msk   ETH_NCFGR_RFCS_Msk
#define GMAC_NCFGR_RXCOEN_Msk ETH_NCFGR_RXCOEN_Msk
#define GMAC_NCFGR_LFERD_Msk  ETH_NCFGR_LFERD_Msk
#define GMAC_NCFGR_MTIHEN_Msk ETH_NCFGR_MTIHEN_Msk
#define GMAC_DCFGR_TXCOEN_Msk ETH_DCFGR_TXCOEN_Msk

/*
 * BZ6's eth.h only provides the generic ETH_NCFGR_CLK(value) field-value
 * macro, unlike the SG family's gmac.h which additionally defines named
 * MCK-divisor constants. Derive equivalent constants here.
 */
#define GMAC_NCFGR_CLK_MCK8  ETH_NCFGR_CLK(0)
#define GMAC_NCFGR_CLK_MCK16 ETH_NCFGR_CLK(1)
#define GMAC_NCFGR_CLK_MCK32 ETH_NCFGR_CLK(2)
#define GMAC_NCFGR_CLK_MCK48 ETH_NCFGR_CLK(3)
#define GMAC_NCFGR_CLK_MCK64 ETH_NCFGR_CLK(4)
#define GMAC_NCFGR_CLK_MCK96 ETH_NCFGR_CLK(5)

#define GMAC_DCFGR_DRBS(value)  ETH_DCFGR_DRBS(value)
#define GMAC_DCFGR_RXBMS(value) ETH_DCFGR_RXBMS(value)
/* BZ6's eth.h has no named FBLDO_INCR4 value; derive it from FBLDO(value). */
#define GMAC_DCFGR_FBLDO_INCR4  ETH_DCFGR_FBLDO(4)

#define GMAC_RBQB_ADDR_Msk ETH_RBQB_ADDR_Msk
#define GMAC_TBQB_ADDR_Msk ETH_TBQB_ADDR_Msk

#define GMAC_SAB_ADDR(value) ETH_SAB_ADDR(value)
#define GMAC_SAT_ADDR(value) ETH_SAT_ADDR(value)

#define GMAC_RSR_BNA_Msk ETH_RSR_BNA_Msk
#define GMAC_RSR_RESETVALUE ETH_RSR_RESETVALUE

#define GMAC_IER_RCOMP_Msk ETH_IER_RCOMP_Msk
#define GMAC_IER_RXUBR_Msk ETH_IER_RXUBR_Msk
#define GMAC_IER_ROVR_Msk  ETH_IER_ROVR_Msk
#define GMAC_IER_TCOMP_Msk ETH_IER_TCOMP_Msk
#define GMAC_IER_TUR_Msk   ETH_IER_TUR_Msk
#define GMAC_IER_RLEX_Msk  ETH_IER_RLEX_Msk
#define GMAC_IER_TFC_Msk   ETH_IER_TFC_Msk
#define GMAC_IER_HRESP_Msk ETH_IER_HRESP_Msk

#define GMAC_ISR_RCOMP_Msk ETH_ISR_RCOMP_Msk
#define GMAC_ISR_TCOMP_Msk ETH_ISR_TCOMP_Msk
#define GMAC_TSR_TXCOMP_Msk ETH_TSR_TXCOMP_Msk

#define GMAC_MAN      ETH_MAN
#define GMAC_NSR      ETH_NSR
#define GMAC_MAN_CLTTO_Msk ETH_MAN_CLTTO_Msk
#define GMAC_MAN_OP(value)   ETH_MAN_OP(value)
#define GMAC_MAN_WTN(value)  ETH_MAN_WTN(value)
#define GMAC_MAN_PHYA(value) ETH_MAN_PHYA(value)
#define GMAC_MAN_REGA(value) ETH_MAN_REGA(value)
#define GMAC_MAN_DATA(value) ETH_MAN_DATA(value)
#define GMAC_MAN_DATA_Msk    ETH_MAN_DATA_Msk
#define GMAC_NSR_IDLE_Msk    ETH_NSR_IDLE_Msk

#endif /* ZEPHYR_DRIVERS_ETHERNET_ETH_MCHP_GMAC_G1_BZ6_H_ */
