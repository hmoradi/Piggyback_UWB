/*
 * Copyright (C) 2016 Inria
 *               2016 Freie Universität Berlin
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @ingroup    drivers_dw1000
 * @{
 *
 * @file
 * @brief      Netdev interface for the DW1000
 *
 * @author     Hessam Mohammadmoradi <hmoradi@cs.uh.edu>
 *
 */

#ifndef DW1000_NETDEV_H
#define DW1000_NETDEV_H

#include "net/netdev.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief   Reference to the netdev device driver struct
 */
extern const netdev_driver_t dw1000_driver;

#ifdef __cplusplus
}
#endif

#endif /* DW1000_NETDEV_H */
/** @} */
