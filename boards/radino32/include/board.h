/*
 * Copyright (C) 2014 Freie Universität Berlin
 *
 * This file is subject to the terms and conditions of the GNU Lesser General
 * Public License v2.1. See the file LICENSE in the top level directory for more
 * details.
 */

/**
 * @defgroup    radino32
 * @ingroup     radino32
 * @brief       Board specific files for the radino32 board.
 * @{
 *
 * @file
 * @brief       Board specific definitions for the radino32 board.
 *
 * @author      Hessam Mohammadmoradi <hmoradi@cs.uh.edu>
 */

#ifndef BOARD_H
#define BOARD_H


#include "cpu.h"
#include "periph_conf.h"
#include "arduino_pinmap.h"
#ifdef __cplusplus
extern "C" {
#endif
/**
 * @name Macros for controlling the on-board LED (LD3).
 * @{
 */
#define LED0_PIN            GPIO_PIN(PORT_B, 1)
#define LED0_MASK           (1 << 3)
#define LED0_ON             (GPIOB->BSRR = LED0_MASK)
#define LED0_OFF            (GPIOB->BSRR = (LED0_MASK << 16))
#define LED0_TOGGLE         (GPIOB->ODR  ^= LED0_MASK)
/** @} */
/**
 * @brief   Initialize board specific hardware, including clock, LEDs and std-IO
 */
void board_init(void);
/**
 * @name xtimer configuration
 * @{
 */
#define XTIMER_DEV          TIMER_DEV(0)
#define XTIMER_CHAN         (0)
#define XTIMER_OVERHEAD     (6)
#define XTIMER_BACKOFF      (3)
/** @} */


/**
 * @name    Set the default baudrate to 500K for this board
 * @{
 */
#ifndef UART_STDIO_BAUDRATE
#   define UART_STDIO_BAUDRATE (1000000U)
#endif
/** @} */


#ifdef __cplusplus
}
#endif

#endif /* BOARD_H */
/** @} */
