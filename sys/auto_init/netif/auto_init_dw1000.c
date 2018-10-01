/*
 * Copyright (C) 2016 Freie Universität Berlin
 *               2016 Inria
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 *
 */

/*
 * @ingroup     auto_init_gnrc_netif
 * @{
 *
 * @file
 * @brief       Auto initialization for DW1000 network devices
 *
 * @author      Hessam Mohammadmoradi <hmoradi@cs.uh.edu>
 */

#ifdef MODULE_DW1000

#include "log.h"
#include "board.h"
#include "net/gnrc/netdev.h"
#include "net/gnrc/netdev/ieee802154.h"
#include "net/gnrc.h"
#define ENABLE_DEBUG (1)
#include "debug.h"
#include "dw1000.h"
#include "dw1000_params.h"

/**
 * @brief   MAC layer stack parameters
 * @{
 */
#define DW1000_MAC_STACKSIZE           (THREAD_STACKSIZE_MAIN)
#ifndef DW1000_MAC_PRIO
#define DW1000_MAC_PRIO                (GNRC_NETDEV_MAC_PRIO)
#endif
/** @} */

/**
 * @brief   Get the number of configured DW1000 devices
 */
#define DW1000_NUMOF        (sizeof(dw1000_params) / sizeof(dw1000_params[0]))

/**
 * @brief   Allocate memory for dev descriptors, stacks, and 802.15.4 adaption
 * @{
 */
static dw1000_t dw1000_devs[DW1000_NUMOF];
static gnrc_netdev_t gnrc_adpt[DW1000_NUMOF];
static char _dw1000_stacks[DW1000_NUMOF][DW1000_MAC_STACKSIZE];
/** @} */
///

///
void auto_init_dw1000(void)
{

    for (unsigned i = 0; i < DW1000_NUMOF; i++) {
        LOG_DEBUG("[auto_init_netif] initializing dw1000 #%u\n", i);
        DEBUG("[auto_init_netif] initializing dw1000 #%u\r\n", i);

        dw1000_setup(&dw1000_devs[i], &dw1000_params[i]);
        DEBUG("auto init dw1000 set up is done \r\n");
        int res = gnrc_netdev_ieee802154_init(&gnrc_adpt[i],
                                              (netdev_ieee802154_t *)&dw1000_devs[i]);

        if (res < 0) {
            LOG_ERROR("[auto_init_netif] error initializing DW1000 #%u\n", i);
           DEBUG("[auto_init_netif] error initializing DW1000 #%u\r\n", i);
        }
        else {
            DEBUG("auto init dw1000 gnrc netdev init \r\n");
            gnrc_netdev_init(_dw1000_stacks[i],
                             DW1000_MAC_STACKSIZE,
                             DW1000_MAC_PRIO,
                             "dw1000", &gnrc_adpt[i]);
        DEBUG("aut init dw10000 %" PRIkernel_pid ": dw1000 init done ))))))))))))))) \r\n",sched_active_thread->pid);
        }
    }
}

#else
typedef int dont_be_pedantic;
#endif /* MODULE_DW1000 */

/** @} */
