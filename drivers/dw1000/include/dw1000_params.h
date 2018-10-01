/*
 * Copyright (C) 2016 Freie Universität Berlin
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @ingroup     drivers_dw1000
 *
 * @{
 * @file
 * @brief       Default configuration for the DW1000 driver
 *
 * @author      Hessam Mohammadmoradi <hmoradi@cs.uh.edu>
 */

#ifndef DW1000_PARAMS_H
#define DW1000_PARAMS_H

#include "board.h"
#include "dw1000.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief   Set default configuration parameters for the DW1000 driver
 * @{
 */
#ifndef DW1000_PARAM_SPI
#define DW1000_PARAM_SPI        (SPI_DEV(1))
#endif
#ifndef DW1000_PARAM_SPI_CLK
#define DW1000_PARAM_SPI_CLK    (SPI_CLK_1MHZ)
#endif
#ifndef DW1000_PARAM_CS
#define DW1000_PARAM_CS         (GPIO_PIN(PORT_A, 8))
#endif
//#ifndef CC2420_PARAM_FIFO
//#define CC2420_PARAM_FIFO       (GPIO_PIN(0, 1))
//#endif
//#ifndef CC2420_PARAM_FIFOP
//#define CC2420_PARAM_FIFOP      (GPIO_PIN(0, 2))
//#endif
//#ifndef CC2420_PARAM_CCA
//#define CC2420_PARAM_CCA        (GPIO_PIN(0, 3))
//#endif
//#ifndef CC2420_PARAM_SFD
//#define CC2420_PARAM_SFD        (GPIO_PIN(0, 3))
//#endif
//#ifndef CC2420_PARAM_VREFEN
//#define CC2420_PARAM_VREFEN     (GPIO_PIN(0, 3))
//#endif
#ifndef DW1000_PARAM_RESET
#define DW1000_PARAM_RESET      (GPIO_PIN(PORT_A, 5))
#endif
#ifndef DW1000_PARAM_IRQ
#define DW1000_PARAM_IRQ      (GPIO_PIN(PORT_B, 11))
#endif

#define DW1000_PARAMS_DEFAULT   {.spi        = DW1000_PARAM_SPI, \
                                 .spi_clk    = DW1000_PARAM_SPI_CLK, \
                                 .pin_cs     = DW1000_PARAM_CS, \
                                  .pin_reset  = DW1000_PARAM_RESET,\
                                  .pin_irq = DW1000_PARAM_IRQ}

/**@}*/

/**
 * @brief   DW1000 configuration
 */
static const dw1000_params_t dw1000_params[] =
{
#ifdef DW1000_PARAMS_BOARD
    DW1000_PARAMS_BOARD,
#else
    DW1000_PARAMS_DEFAULT,
#endif
};

#define NUM_BR 3
#define NUM_PRF 2
#define NUM_PACS 4
#define NUM_BW 2 //2 bandwidths are supported
#define NUM_SFD 2	//supported number of SFDs - standard = 0, non-standard = 1
#define NUM_CH 6 //supported channels are 1, 2, 3, 4, 5, 7
#define NUM_CH_SUPPORTED 8 //supported channels are '0', 1, 2, 3, 4, 5, '6', 7
#define PCODES 25 //supported preamble codes
typedef struct {
    uint32 lo32;
    uint16 target[NUM_PRF];
} agc_cfg_struct ;

extern const agc_cfg_struct agc_config ;

//SFD threshold settings for 110k, 850k, 6.8Mb standard and non-standard
extern const uint16 sftsh[NUM_BR][NUM_SFD];

extern const uint16 dtune1[NUM_PRF];

#define XMLPARAMS_VERSION	(1.17f)

extern const uint8 pll2_config[NUM_CH][5];
extern const uint8 pll2calcfg;
extern const uint8 rx_config[NUM_BW];
extern const uint32 tx_config[NUM_CH];
extern const uint8 dwnsSFDlen[NUM_BR];				//length of SFD for each of the bitrates
extern const uint32 digital_bb_config[NUM_PRF][NUM_PACS];
extern const uint8 chan_idx[NUM_CH_SUPPORTED];

#define PEAK_MULTPLIER (0x60)	//3 -> (0x3 * 32) & 0x00E0
#define N_STD_FACTOR (13)
#define LDE_PARAM1		(PEAK_MULTPLIER | N_STD_FACTOR)

#define LDE_PARAM3_16 (0x1607)
#define LDE_PARAM3_64 (0x0607)

extern const uint16 lde_replicaCoeff[PCODES];
#ifdef __cplusplus
}
#endif

#endif /* DW1000_PARAMS_H */
/** @} */
