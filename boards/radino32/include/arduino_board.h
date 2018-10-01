/*
 * Copyright (C)  2017 Inria
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @ingroup     boards_radino32
 * @{
 *
 * @file
 * @brief       Board specific configuration for the Arduino API
 *
 * @author      Hessam Mohammadmoradi  <hmoradi@cs.uh.edu>
 */

#ifndef ARDUINO_BOARD_H
#define ARDUINO_BOARD_H

#include "arduino_pinmap.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief   Look-up table for the Arduino's digital pins
 */
static const gpio_t arduino_pinmap[] = {
    ARDUINO_PIN_0,
    ARDUINO_PIN_1,
    ARDUINO_PIN_2,
    ARDUINO_PIN_3,
    ARDUINO_PIN_5,
    ARDUINO_PIN_6,
    ARDUINO_PIN_10,
    ARDUINO_PIN_11,
    ARDUINO_PIN_12,
    ARDUINO_PIN_13,
    ARDUINO_PIN_14,
    ARDUINO_PIN_15,
    ARDUINO_PIN_16,
    ARDUINO_PIN_17,
    ARDUINO_PIN_18,
    ARDUINO_PIN_19,
    ARDUINO_PIN_20,
    ARDUINO_PIN_21,
    ARDUINO_PIN_22,
    ARDUINO_PIN_25,
    ARDUINO_PIN_26,
    ARDUINO_PIN_27,
    ARDUINO_PIN_28,
    ARDUINO_PIN_29,
    ARDUINO_PIN_30,
    ARDUINO_PIN_31,
    ARDUINO_PIN_32,
    ARDUINO_PIN_33,
    ARDUINO_PIN_34,
    ARDUINO_PIN_35,
    ARDUINO_PIN_36,
    ARDUINO_PIN_A0,
    ARDUINO_PIN_A1,
    ARDUINO_PIN_A2,
    ARDUINO_PIN_A3,
};

#ifdef __cplusplus
}
#endif

#endif /* ARDUINO_BOARD_H */
/** @} */
