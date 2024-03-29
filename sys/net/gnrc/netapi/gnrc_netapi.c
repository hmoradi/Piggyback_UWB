/*
 * Copyright (C) 2015 Freie Universität Berlin
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @{
 * @ingroup     net_gnrc_netapi
 * @file
 * @brief       This file contains a number of helper functions that provide
 *              some shortcuts for some always repeating code snippets when
 *              servicing the netapi interface
 *
 * @author      Hauke Petersen <hauke.petersen@fu-berlin.de>
 * @}
 */

#include "mbox.h"
#include "msg.h"
#include "net/gnrc/netreg.h"
#include "net/gnrc/pktbuf.h"
#include "net/gnrc/netapi.h"

#define ENABLE_DEBUG    (0)
#include "debug.h"

/**
 * @brief   Unified function for getting and setting netapi options
 *
 * @param[in] pid       PID of the targeted thread
 * @param[in] type      specify if option is to be set or get
 * @param[in] opt       option to set or get
 * @param[in] data      data to set or pointer to buffer for reading options
 * @param[in] data_len  size of the given buffer
 *
 * @return              the value from the received ACK message
 */
static inline int _get_set(kernel_pid_t pid, uint16_t type,
                           netopt_t opt, uint16_t context,
                           void *data, size_t data_len)
{
    msg_t cmd;
    msg_t ack;
    gnrc_netapi_opt_t o;
    /* set ńetapi's option struct */
    o.opt = opt;
    o.context = context;
    o.data = data;
    o.data_len = data_len;
    /* set outgoing message's fields */
    cmd.type = type;
    cmd.content.ptr = (void *)&o;
    /* trigger the netapi */
    msg_send_receive(&cmd, &ack, pid);
    assert(ack.type == GNRC_NETAPI_MSG_TYPE_ACK);
    /* return the ACK message's value */
    return (int)ack.content.value;
}

static inline int _snd_rcv(kernel_pid_t pid, uint16_t type, gnrc_pktsnip_t *pkt)
{
    msg_t msg;
    /* set the outgoing message's fields */
    //DEBUG("set the outgoing msg fields\r\n");
    msg.type = type;
    msg.content.ptr = (void *)pkt;
    /* send message */
    //DEBUG("try to send\r\n");
    int ret = msg_try_send(&msg, pid);
    if (ret < 1) {
        DEBUG("gnrc_netapi: dropped message to %" PRIkernel_pid " (%s)\n", pid,
              (ret == 0) ? "receiver queue is full" : "invalid receiver");
    }
    return ret;
}

#ifdef MODULE_GNRC_NETAPI_MBOX
static inline int _snd_rcv_mbox(mbox_t *mbox, uint16_t type, gnrc_pktsnip_t *pkt)
{
    msg_t msg;
    /* set the outgoing message's fields */
    msg.type = type;
    msg.content.ptr = (void *)pkt;
    /* send message */
    int ret = mbox_try_put(mbox, &msg);
    if (ret < 1) {
        DEBUG("gnrc_netapi: dropped message to %p (was full)\n", mbox);
    }
    return ret;
}
#endif

int gnrc_netapi_dispatch(gnrc_nettype_t type, uint32_t demux_ctx,
                         uint16_t cmd, gnrc_pktsnip_t *pkt)
{
    DEBUG("gnrc netapi dispatch : is called \r\n");
    int numof = gnrc_netreg_num(type, demux_ctx);

    if (numof != 0) {
        DEBUG("gnrc netapi dispatch : netreg lookup \r\n");
        gnrc_netreg_entry_t *sendto = gnrc_netreg_lookup(type, demux_ctx);
        DEBUG("gnrc netapi dispatch : pkt buff hold \r\n");
        gnrc_pktbuf_hold(pkt, numof - 1);

        while (sendto) {
            DEBUG("gnrc netapi dispatch : sending to \r\n");
#if defined(MODULE_GNRC_NETAPI_MBOX) || defined(MODULE_GNRC_NETAPI_CALLBACKS)
            int release = 0;
            switch (sendto->type) {
                case GNRC_NETREG_TYPE_DEFAULT:
                DEBUG("gnrc netapi dispatch : GNRC_NETREG_TYPE_DEFAULT \r\n");
                    if (_snd_rcv(sendto->target.pid, cmd, pkt) < 1) {
                        /* unable to dispatch packet */
                        release = 1;
                    }
                    break;
#ifdef MODULE_GNRC_NETAPI_MBOX
                case GNRC_NETREG_TYPE_MBOX:
                    DEBUG("gnrc netapi dispatch : GNRC_NETREG_TYPE_MBOX \r\n");
                    if (_snd_rcv_mbox(sendto->target.mbox, cmd, pkt) < 1) {
                        /* unable to dispatch packet */
                        release = 1;
                    }
                    break;
#endif
#ifdef MODULE_GNRC_NETAPI_CALLBACKS
                case GNRC_NETREG_TYPE_CB:
                DEBUG("gnrc netapi dispatch : GNRC_NETREG_TYPE_CB \r\n");
                    sendto->target.cbd->cb(cmd, pkt, sendto->target.cbd->ctx);
                    break;
#endif
                default:
                    /* unknown dispatch type */
                    DEBUG("gnrc netapi dispatch : unknown dispatch type \r\n");
                    release = 1;
                    break;
            }
            if (release) {
            DEBUG("gnrc netapi dispatch : release \r\n");
                gnrc_pktbuf_release(pkt);
            }
#else
            if (_snd_rcv(sendto->target.pid, cmd, pkt) < 1) {
                /* unable to dispatch packet */
                DEBUG("gnrc net api unable to dispatch , releasing the packet \r\n");
                gnrc_pktbuf_release(pkt);
            }
#endif
            DEBUG("gnrc netapi dispatch : net reg get next \r\n");
            sendto = gnrc_netreg_getnext(sendto);
        }
    }

    return numof;
}

int gnrc_netapi_send(kernel_pid_t pid, gnrc_pktsnip_t *pkt)
{
    return _snd_rcv(pid, GNRC_NETAPI_MSG_TYPE_SND, pkt);
}

int gnrc_netapi_receive(kernel_pid_t pid, gnrc_pktsnip_t *pkt)
{
    return _snd_rcv(pid, GNRC_NETAPI_MSG_TYPE_RCV, pkt);
}

int gnrc_netapi_get(kernel_pid_t pid, netopt_t opt, uint16_t context,
                    void *data, size_t data_len)
{
    return _get_set(pid, GNRC_NETAPI_MSG_TYPE_GET, opt, context,
                    data, data_len);
}

int gnrc_netapi_set(kernel_pid_t pid, netopt_t opt, uint16_t context,
                    void *data, size_t data_len)
{
    return _get_set(pid, GNRC_NETAPI_MSG_TYPE_SET, opt, context,
                    data, data_len);
}
