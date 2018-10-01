/*
 * Copyright (C) 2015 Freie Universität Berlin
 *               2015 Kaspar Schleiser <kaspar@schleiser.de>
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @{
 * @ingroup     net
 * @file
 * @brief       Glue for netdev devices to netapi
 *
 * @author      Hauke Petersen <hauke.petersen@fu-berlin.de>
 * @author      Kaspar Schleiser <kaspar@schleiser.de>
 * @}
 */

#include <errno.h>

#include "msg.h"
#include "thread.h"

#include "net/gnrc.h"
#include "net/gnrc/nettype.h"
#include "net/netdev.h"

#include "net/gnrc/netdev.h"
#include "net/ethernet/hdr.h"

#define ENABLE_DEBUG    (0)
#include "debug.h"
#include "fmt.h"

#if defined(MODULE_OD) && ENABLE_DEBUG
#include "od.h"
#endif

#define NETDEV_NETAPI_MSG_QUEUE_SIZE 8

static void _pass_on_packet(gnrc_pktsnip_t *pkt);

/**
 * @brief   Function called by the device driver on device events
 *
 * @param[in] event     type of event
 */
static void _event_cb(netdev_t *dev, netdev_event_t event)
{
    DEBUG("gnrc_netdev : event callback is called \r\n");

    gnrc_netdev_t *gnrc_netdev = (gnrc_netdev_t*) dev->context;
    if (event == NETDEV_EVENT_ISR) {
        DEBUG("gnrc_netdev : event callback NETDEV_EVENT_ISR \r\n");
        msg_t msg;

        msg.type = NETDEV_MSG_TYPE_EVENT;
        msg.content.ptr = gnrc_netdev;

        if (msg_send(&msg, gnrc_netdev->pid) <= 0) {
            puts("gnrc_netdev: possibly lost interrupt.\r\n");
        }
    }
    else {
        DEBUG("gnrc_netdev: event triggered -> %i\r\n", event);
        switch(event) {
            case NETDEV_EVENT_RX_COMPLETE:
                {
                    DEBUG("gnrc_netdev: rx event \r\n");

                    gnrc_pktsnip_t *pkt = gnrc_netdev->recv(gnrc_netdev);

                    if (pkt) {
                        _pass_on_packet(pkt);
                    }

                    break;
                }
#ifdef MODULE_NETSTATS_L2
            case NETDEV_EVENT_TX_MEDIUM_BUSY:
                dev->stats.tx_failed++;
                break;
            case NETDEV_EVENT_TX_COMPLETE:
                dev->stats.tx_success++;
                break;
#endif
            default:
                DEBUG("gnrc_netdev: warning: unhandled event %u.\r\n", event);
        }
    }
}

static void _pass_on_packet(gnrc_pktsnip_t *pkt)
{
    /* throw away packet if no one is interested */
    if (!gnrc_netapi_dispatch_receive(pkt->type, GNRC_NETREG_DEMUX_CTX_ALL, pkt)) {
        DEBUG("gnrc_netdev: unable to forward packet of type %i\r\n", pkt->type);
        gnrc_pktbuf_release(pkt);
        return;
    }
}

/**
 * @brief   Startup code and event loop of the gnrc_netdev layer
 *
 * @param[in] args  expects a pointer to the underlying netdev device
 *
 * @return          never returns
 */
static void *_gnrc_netdev_thread(void *args)
{
    //DEBUG("gnrc_netdev: starting thread\r\n");

    gnrc_netdev_t *gnrc_netdev = (gnrc_netdev_t*) args;
    netdev_t *dev = gnrc_netdev->dev;

    gnrc_netdev->pid = thread_getpid();

    gnrc_netapi_opt_t *opt;
    int res;
    msg_t msg, reply, msg_queue[NETDEV_NETAPI_MSG_QUEUE_SIZE];

    //DEBUG("set up mac msg queue \r\n");
    /* setup the MAC layers message queue */
    msg_init_queue(msg_queue, NETDEV_NETAPI_MSG_QUEUE_SIZE);

    //DEBUG("register event callback \r\n");
    /* register the event callback with the device driver */
    dev->event_callback = _event_cb;
    dev->context = (void*) gnrc_netdev;

    DEBUG("reg device to network stack with pid %d \r\n",thread_getpid());
    /* register the device to the network stack*/
    gnrc_netif_add(thread_getpid());

    //DEBUG("init low level driver \r\n");
    /* initialize low-level driver */
    dev->driver->init(dev);

    DEBUG("starting main loop in mac \r\n");
    /* start the event loop */
    while (1) {
        //DEBUG("gnrc_netdev: waiting for incoming messages\r\n");
        msg_receive(&msg);
        /* dispatch NETDEV and NETAPI messages */
        switch (msg.type) {
            case NETDEV_MSG_TYPE_EVENT:
                DEBUG("gnrc_netdev: GNRC_NETDEV_MSG_TYPE_EVENT received\r\n");
                dev->driver->isr(dev);
                break;
            case GNRC_NETAPI_MSG_TYPE_SND:
                DEBUG("gnrc_netdev: GNRC_NETAPI_MSG_TYPE_SND received\r\n");
                gnrc_pktsnip_t *pkt = msg.content.ptr;

                gnrc_netdev->send(gnrc_netdev, pkt);
                break;
            case GNRC_NETAPI_MSG_TYPE_SET:
                /* read incoming options */
                opt = msg.content.ptr;
                DEBUG("gnrc_netdev: GNRC_NETAPI_MSG_TYPE_SET received. opt=%s\r\n",
                        netopt2str(opt->opt));
                /* set option for device driver */
                res = dev->driver->set(dev, opt->opt, opt->data, opt->data_len);
                DEBUG("gnrc_netdev: response of netdev->set: %i\r\n", res);
                /* send reply to calling thread */
                reply.type = GNRC_NETAPI_MSG_TYPE_ACK;
                reply.content.value = (uint32_t)res;
                msg_reply(&msg, &reply);
                break;
            case GNRC_NETAPI_MSG_TYPE_GET:
                /* read incoming options */
                opt = msg.content.ptr;
                DEBUG("gnrc_netdev: GNRC_NETAPI_MSG_TYPE_GET received. opt=%s\r\n",
                        netopt2str(opt->opt));
                /* get option from device driver */
                res = dev->driver->get(dev, opt->opt, opt->data, opt->data_len);
                DEBUG("gnrc_netdev: response of netdev->get: %i\r\n", res);
                /* send reply to calling thread */
                reply.type = GNRC_NETAPI_MSG_TYPE_ACK;
                reply.content.value = (uint32_t)res;
                msg_reply(&msg, &reply);
                break;

            default:
                DEBUG("gnrc_netdev: Unknown command %" PRIu16 "\r\n", msg.type);
                break;
        }
    }
    /* never reached */
    return NULL;
}

kernel_pid_t gnrc_netdev_init(char *stack, int stacksize, char priority,
                        const char *name, gnrc_netdev_t *gnrc_netdev)
{
    kernel_pid_t res;


    /* check if given netdev device is defined and the driver is set */
    if (gnrc_netdev == NULL || gnrc_netdev->dev == NULL) {
        DEBUG("driver is not set yet \r\n");
        return -ENODEV;
    }

    /* create new gnrc_netdev thread */
    res = thread_create(stack, stacksize, priority, THREAD_CREATE_STACKTEST,
                        _gnrc_netdev_thread, (void *)gnrc_netdev, name);
    if (res <= 0) {
        DEBUG("driver thread creation failed \r\n");
        return -EINVAL;
    }

    return res;
}
