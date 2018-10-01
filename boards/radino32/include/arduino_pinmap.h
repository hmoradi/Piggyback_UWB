/*
 * Copyright (C)  2017 Inria
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @ingroup     stm32l155cc
 * @{
 *
 * @file
 * @brief       Mapping from MCU pins to Arduino pins
 *
 * You can use the defines in this file for simplified interaction with the
 * Arduino specific pin numbers.
 *
 * @author      hessam mohammadmoradi <hmoradi@cs.uh.edu>
 */

#ifndef ARDUINO_PINMAP_H
#define ARDUINO_PINMAP_H

#include "periph/gpio.h"

#ifdef __cplusplus
extern "C" {
#endif


/**
 * @brief   Mapping of MCU pins to Arduino pins
 * @{
 */
#define ARDUINO_PIN_33          GPIO_PIN(PORT_B, 11)
#define ARDUINO_PIN_32          GPIO_PIN(PORT_B, 10)
#define ARDUINO_PIN_28          GPIO_PIN(PORT_A, 5)
#define ARDUINO_PIN_29          GPIO_PIN(PORT_A, 8)
#define ARDUINO_PIN_34          GPIO_PIN(PORT_B, 13)
#define ARDUINO_PIN_36          GPIO_PIN(PORT_B, 15)
#define ARDUINO_PIN_35          GPIO_PIN(PORT_B, 14)
#define ARDUINO_PIN_31          GPIO_PIN(PORT_A, 14)
#define ARDUINO_PIN_30          GPIO_PIN(PORT_A, 13)
#define ARDUINO_PIN_17          GPIO_PIN(PORT_A, 15)
#define ARDUINO_PIN_18          GPIO_PIN(PORT_B, 8)
#define ARDUINO_PIN_12          GPIO_PIN(PORT_A, 6)
#define ARDUINO_PIN_3          GPIO_PIN(PORT_B, 6)
#define ARDUINO_PIN_2          GPIO_PIN(PORT_B, 7)
#define ARDUINO_PIN_24          GPIO_PIN(PORT_B, 12)
#define ARDUINO_PIN_25          GPIO_PIN(PORT_C, 13)
#define ARDUINO_PIN_27          GPIO_PIN(PORT_A, 12)
#define ARDUINO_PIN_26          GPIO_PIN(PORT_A, 11)
#define ARDUINO_PIN_1          GPIO_PIN(PORT_A, 9)
#define ARDUINO_PIN_0          GPIO_PIN(PORT_A, 10)
#define ARDUINO_PIN_5          GPIO_PIN(PORT_B, 9)
#define ARDUINO_PIN_13          GPIO_PIN(PORT_B, 1)
#define ARDUINO_PIN_11          GPIO_PIN(PORT_B, 0)
#define ARDUINO_PIN_10          GPIO_PIN(PORT_A, 2)
#define ARDUINO_PIN_6          GPIO_PIN(PORT_A, 3)
#define ARDUINO_PIN_15          GPIO_PIN(PORT_B, 3)
#define ARDUINO_PIN_14          GPIO_PIN(PORT_B, 4)
#define ARDUINO_PIN_16          GPIO_PIN(PORT_B, 5)
#define ARDUINO_PIN_20          GPIO_PIN(PORT_A, 1)
#define ARDUINO_PIN_19          GPIO_PIN(PORT_A, 0)
#define ARDUINO_PIN_21          GPIO_PIN(PORT_A, 4)
#define ARDUINO_PIN_22          GPIO_PIN(PORT_A, 7)	
#define ARDUINO_PIN_A0          GPIO_PIN(PORT_A, 0)
#define ARDUINO_PIN_A1          GPIO_PIN(PORT_A, 1)
#define ARDUINO_PIN_A2          GPIO_PIN(PORT_A, 4)
#define ARDUINO_PIN_A3          GPIO_PIN(PORT_A, 7)

/** @ */

#ifdef __cplusplus
}
#endif

#endif /* ARDUINO_PINMAP_H */
/** @} */
