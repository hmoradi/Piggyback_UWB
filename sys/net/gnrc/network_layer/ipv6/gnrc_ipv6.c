/*
 * Copyright (C) 2015 Martine Lenders <mlenders@inf.fu-berlin.de>
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @{
 *
 * @file
 */
#include <assert.h>
#include <errno.h>
#include <inttypes.h>
#include <stdbool.h>

#include "byteorder.h"
#include "cpu_conf.h"
#include "kernel_types.h"
#include "net/gnrc.h"
#include "net/gnrc/icmpv6.h"
#include "net/gnrc/ndp.h"
#include "net/gnrc/sixlowpan/ctx.h"
#include "net/gnrc/sixlowpan/nd.h"
#include "net/gnrc/sixlowpan/nd/router.h"
#include "net/protnum.h"
#include "thread.h"
#include "utlist.h"

#include "net/gnrc/ipv6/nc.h"
#include "net/gnrc/ipv6/netif.h"
#include "net/gnrc/ipv6/whitelist.h"
#include "net/gnrc/ipv6/blacklist.h"

#include "net/gnrc/ipv6.h"

#define ENABLE_DEBUG    (0)
#include "debug.h"

#define _MAX_L2_ADDR_LEN    (8U)

#if ENABLE_DEBUG
static char _stack[GNRC_IPV6_STACK_SIZE + THREAD_EXTRA_STACKSIZE_PRINTF];
#else
static char _stack[GNRC_IPV6_STACK_SIZE];
#endif

#ifdef MODULE_FIB
#include "net/fib.h"
#include "net/fib/table.h"
/**
 * @brief buffer to store the entries in the IPv6 forwarding table
 */
static fib_entry_t _fib_entries[GNRC_IPV6_FIB_TABLE_SIZE];

/**
 * @brief the IPv6 forwarding table
 */
fib_table_t gnrc_ipv6_fib_table;
#endif

#if ENABLE_DEBUG
static char addr_str[IPV6_ADDR_MAX_STR_LEN];
#endif

kernel_pid_t gnrc_ipv6_pid = KERNEL_PID_UNDEF;

/* handles GNRC_NETAPI_MSG_TYPE_RCV commands */
static void _receive(gnrc_pktsnip_t *pkt);
/* Sends packet over the appropriate interface(s).
 * prep_hdr: prepare header for sending (call to _fill_ipv6_hdr()), otherwise
 * assume it is already prepared */
static void _send(gnrc_pktsnip_t *pkt, bool prep_hdr);
/* Main event loop for IPv6 */
static void *_event_loop(void *args);

/* Handles encapsulated IPv6 packets: http://tools.ietf.org/html/rfc2473 */
static void _decapsulate(gnrc_pktsnip_t *pkt);

kernel_pid_t gnrc_ipv6_init(void)
{
    if (gnrc_ipv6_pid == KERNEL_PID_UNDEF) {
        gnrc_ipv6_pid = thread_create(_stack, sizeof(_stack), GNRC_IPV6_PRIO,
                                      THREAD_CREATE_STACKTEST,
                                      _event_loop, NULL, "ipv6");
    }

#ifdef MODULE_FIB
    gnrc_ipv6_fib_table.data.entries = _fib_entries;
    gnrc_ipv6_fib_table.table_type = FIB_TABLE_TYPE_SH;
    gnrc_ipv6_fib_table.size = GNRC_IPV6_FIB_TABLE_SIZE;
    fib_init(&gnrc_ipv6_fib_table);
#endif

    return gnrc_ipv6_pid;
}

static void _dispatch_next_header(gnrc_pktsnip_t *current, gnrc_pktsnip_t *pkt,
                                  uint8_t nh, bool interested);

/*
 *         current                 pkt
 *         |                       |
 *         v                       v
 * IPv6 <- IPv6_EXT <- IPv6_EXT <- UNDEF
 */
void gnrc_ipv6_demux(kernel_pid_t iface, gnrc_pktsnip_t *current, gnrc_pktsnip_t *pkt, uint8_t nh)
{
    bool interested = false;

    current->type = gnrc_nettype_from_protnum(nh);

    switch (nh) {
#ifdef MODULE_GNRC_ICMPV6
        case PROTNUM_ICMPV6:
            assert(current == pkt);
            interested = true;
            break;
#endif
#ifdef MODULE_GNRC_IPV6_EXT
        case PROTNUM_IPV6_EXT_HOPOPT:
        case PROTNUM_IPV6_EXT_DST:
        case PROTNUM_IPV6_EXT_RH:
        case PROTNUM_IPV6_EXT_FRAG:
        case PROTNUM_IPV6_EXT_AH:
        case PROTNUM_IPV6_EXT_ESP:
        case PROTNUM_IPV6_EXT_MOB:
            interested = true;

            break;
#endif
        case PROTNUM_IPV6:
            assert(current == pkt);
            interested = true;

            break;
        default:
            (void)iface;
#ifdef MODULE_GNRC_SIXLOWPAN_IPHC_NHC
            /* second statement is true for small 6LoWPAN NHC decompressed frames
             * since in this case it looks like
             *
             * * GNRC_NETTYPE_UNDEF <- pkt
             * v
             * * GNRC_NETTYPE_UDP <- current
             * v
             * * GNRC_NETTYPE_EXT
             * v
             * * GNRC_NETTYPE_IPV6
             */
            assert((current == pkt) || (current == pkt->next));
#else
            assert(current == pkt);
#endif
            break;
    }

    _dispatch_next_header(current, pkt, nh, interested);

    if (!interested) {
        return;
    }

    switch (nh) {
#ifdef MODULE_GNRC_ICMPV6
        case PROTNUM_ICMPV6:
            DEBUG("ipv6: handle ICMPv6 packet (nh = %u)\r\n", nh);
            gnrc_icmpv6_demux(iface, pkt);
            return;
#endif
#ifdef MODULE_GNRC_IPV6_EXT
        case PROTNUM_IPV6_EXT_HOPOPT:
        case PROTNUM_IPV6_EXT_DST:
        case PROTNUM_IPV6_EXT_RH:
        case PROTNUM_IPV6_EXT_FRAG:
        case PROTNUM_IPV6_EXT_AH:
        case PROTNUM_IPV6_EXT_ESP:
        case PROTNUM_IPV6_EXT_MOB:
            DEBUG("ipv6: handle extension header (nh = %u)\r\n", nh);

            gnrc_ipv6_ext_demux(iface, current, pkt, nh);

            return;
#endif
        case PROTNUM_IPV6:
            DEBUG("ipv6: handle encapsulated IPv6 packet (nh = %u)\r\n", nh);
            _decapsulate(pkt);
            return;
        default:
            assert(false);
            break;
    }

    assert(false);
}

ipv6_hdr_t *gnrc_ipv6_get_header(gnrc_pktsnip_t *pkt)
{
    ipv6_hdr_t *hdr = NULL;
    gnrc_pktsnip_t *tmp = gnrc_pktsnip_search_type(pkt, GNRC_NETTYPE_IPV6);
    if ((tmp) && ipv6_hdr_is(tmp->data)) {
        hdr = ((ipv6_hdr_t*) tmp->data);
    }

    return hdr;
}

/* internal functions */
static void _dispatch_next_header(gnrc_pktsnip_t *current, gnrc_pktsnip_t *pkt,
                                  uint8_t nh, bool interested)
{
#ifdef MODULE_GNRC_IPV6_EXT
    const bool should_dispatch_current_type = ((current->type != GNRC_NETTYPE_IPV6_EXT) ||
                                               (current->next->type == GNRC_NETTYPE_IPV6));
#else
    const bool should_dispatch_current_type = (current->next->type == GNRC_NETTYPE_IPV6);
#endif

    DEBUG("ipv6: forward nh = %u to other threads\r\n", nh);

    /* dispatch IPv6 extension header only once */
    if (should_dispatch_current_type) {
        bool should_release = (gnrc_netreg_num(GNRC_NETTYPE_IPV6, nh) == 0) &&
                              (!interested);

        if (!should_release) {
            gnrc_pktbuf_hold(pkt, 1);   /* don't remove from packet buffer in
                                         * next dispatch */
        }
        if (gnrc_netapi_dispatch_receive(current->type,
                                         GNRC_NETREG_DEMUX_CTX_ALL,
                                         pkt) == 0) {
            gnrc_pktbuf_release(pkt);
        }

        if (should_release) {
            return;
        }
    }
    if (interested) {
        gnrc_pktbuf_hold(pkt, 1);   /* don't remove from packet buffer in
                                     * next dispatch */
    }
    if (gnrc_netapi_dispatch_receive(GNRC_NETTYPE_IPV6, nh, pkt) == 0) {
        gnrc_pktbuf_release(pkt);
    }
}

static void *_event_loop(void *args)
{
    msg_t msg, reply, msg_q[GNRC_IPV6_MSG_QUEUE_SIZE];
    gnrc_netreg_entry_t me_reg = GNRC_NETREG_ENTRY_INIT_PID(GNRC_NETREG_DEMUX_CTX_ALL,
                                                            sched_active_pid);

    (void)args;
    msg_init_queue(msg_q, GNRC_IPV6_MSG_QUEUE_SIZE);

    /* register interest in all IPv6 packets */
    gnrc_netreg_register(GNRC_NETTYPE_IPV6, &me_reg);

    /* preinitialize ACK */
    reply.type = GNRC_NETAPI_MSG_TYPE_ACK;

    /* start event loop */
    while (1) {
        DEBUG("ipv6: waiting for incoming message.\r\n");
        msg_receive(&msg);

        switch (msg.type) {
            case GNRC_NETAPI_MSG_TYPE_RCV:
                DEBUG("ipv6: GNRC_NETAPI_MSG_TYPE_RCV received\r\n");
                _receive(msg.content.ptr);
                break;

            case GNRC_NETAPI_MSG_TYPE_SND:
                DEBUG("ipv6: GNRC_NETAPI_MSG_TYPE_SND received\r\n");
                _send(msg.content.ptr, true);
                break;

            case GNRC_NETAPI_MSG_TYPE_GET:
            case GNRC_NETAPI_MSG_TYPE_SET:
                DEBUG("ipv6: reply to unsupported get/set\r\n");
                reply.content.value = -ENOTSUP;
                msg_reply(&msg, &reply);
                break;

#ifdef MODULE_GNRC_NDP
            case GNRC_NDP_MSG_RTR_TIMEOUT:
                DEBUG("ipv6: Router timeout received\r\n");
                ((gnrc_ipv6_nc_t *)msg.content.ptr)->flags &= ~GNRC_IPV6_NC_IS_ROUTER;
                break;

            /* XXX reactivate when https://github.com/RIOT-OS/RIOT/issues/5122 is
             * solved properly */
            /* case GNRC_NDP_MSG_ADDR_TIMEOUT: */
            /*     DEBUG("ipv6: Router advertisement timer event received\r\n"); */
            /*     gnrc_ipv6_netif_remove_addr(KERNEL_PID_UNDEF, */
            /*                                 msg.content.ptr); */
            /*     break; */

            case GNRC_NDP_MSG_NBR_SOL_RETRANS:
                DEBUG("ipv6: Neigbor solicitation retransmission timer event received\r\n");
                gnrc_ndp_retrans_nbr_sol(msg.content.ptr);
                break;

            case GNRC_NDP_MSG_NC_STATE_TIMEOUT:
                DEBUG("ipv6: Neigbor cache state timeout received\r\n");
                gnrc_ndp_state_timeout(msg.content.ptr);
                break;
#endif
#ifdef MODULE_GNRC_NDP_ROUTER
            case GNRC_NDP_MSG_RTR_ADV_RETRANS:
                DEBUG("ipv6: Router advertisement retransmission event received\r\n");
                gnrc_ndp_router_retrans_rtr_adv(msg.content.ptr);
                break;
            case GNRC_NDP_MSG_RTR_ADV_DELAY:
                DEBUG("ipv6: Delayed router advertisement event received\r\n");
                gnrc_ndp_router_send_rtr_adv(msg.content.ptr);
                break;
#endif
#ifdef MODULE_GNRC_NDP_HOST
            case GNRC_NDP_MSG_RTR_SOL_RETRANS:
                DEBUG("ipv6: Router solicitation retransmission event received\r\n");
                gnrc_ndp_host_retrans_rtr_sol(msg.content.ptr);
                break;
#endif
#ifdef MODULE_GNRC_SIXLOWPAN_ND
            case GNRC_SIXLOWPAN_ND_MSG_MC_RTR_SOL:
                DEBUG("ipv6: Multicast router solicitation event received\r\n");
                gnrc_sixlowpan_nd_mc_rtr_sol(msg.content.ptr);
                break;
            case GNRC_SIXLOWPAN_ND_MSG_UC_RTR_SOL:
                DEBUG("ipv6: Unicast router solicitation event received\r\n");
                gnrc_sixlowpan_nd_uc_rtr_sol(msg.content.ptr);
                break;
#   ifdef MODULE_GNRC_SIXLOWPAN_CTX
            case GNRC_SIXLOWPAN_ND_MSG_DELETE_CTX:
                DEBUG("ipv6: Delete 6LoWPAN context event received\r\n");
                gnrc_sixlowpan_ctx_remove(((((gnrc_sixlowpan_ctx_t *)msg.content.ptr)->flags_id) &
                                           GNRC_SIXLOWPAN_CTX_FLAGS_CID_MASK));
                break;
#   endif
#endif
#ifdef MODULE_GNRC_SIXLOWPAN_ND_ROUTER
            case GNRC_SIXLOWPAN_ND_MSG_ABR_TIMEOUT:
                DEBUG("ipv6: border router timeout event received\r\n");
                gnrc_sixlowpan_nd_router_abr_remove(msg.content.ptr);
                break;
            /* XXX reactivate when https://github.com/RIOT-OS/RIOT/issues/5122 is
             * solved properly */
            /* case GNRC_SIXLOWPAN_ND_MSG_AR_TIMEOUT: */
            /*     DEBUG("ipv6: address registration timeout received\r\n"); */
            /*     gnrc_sixlowpan_nd_router_gc_nc(msg.content.ptr); */
            /*     break; */
            case GNRC_NDP_MSG_RTR_ADV_SIXLOWPAN_DELAY:
                DEBUG("ipv6: Delayed router advertisement event received\r\n");
                gnrc_ipv6_nc_t *nc_entry = msg.content.ptr;
                gnrc_ndp_internal_send_rtr_adv(nc_entry->iface, NULL,
                                               &(nc_entry->ipv6_addr), false);
                break;
#endif
            default:
                break;
        }
    }

    return NULL;
}

static void _send_to_iface(kernel_pid_t iface, gnrc_pktsnip_t *pkt)
{
    ((gnrc_netif_hdr_t *)pkt->data)->if_pid = iface;
    gnrc_ipv6_netif_t *if_entry = gnrc_ipv6_netif_get(iface);

    assert(if_entry != NULL);
    if (gnrc_pkt_len(pkt->next) > if_entry->mtu) {
        DEBUG("ipv6: packet too big\r\n");
        gnrc_pktbuf_release(pkt);
        return;
    }
#ifdef MODULE_NETSTATS_IPV6
    if_entry->stats.tx_success++;
    if_entry->stats.tx_bytes += gnrc_pkt_len(pkt->next);
#endif

#ifdef MODULE_GNRC_SIXLOWPAN
    if (if_entry->flags & GNRC_IPV6_NETIF_FLAGS_SIXLOWPAN) {
        DEBUG("ipv6: send to 6LoWPAN instead\r\n");
        if (!gnrc_netapi_dispatch_send(GNRC_NETTYPE_SIXLOWPAN, GNRC_NETREG_DEMUX_CTX_ALL, pkt)) {
            DEBUG("ipv6: no 6LoWPAN thread found\r\n");
            gnrc_pktbuf_release(pkt);
        }
        return;
    }
#endif
    if (gnrc_netapi_send(iface, pkt) < 1) {
        DEBUG("ipv6: unable to send packet\r\n");
        gnrc_pktbuf_release(pkt);
    }
}

static gnrc_pktsnip_t *_create_netif_hdr(uint8_t *dst_l2addr,
                                         uint16_t dst_l2addr_len,
                                         gnrc_pktsnip_t *pkt)
{
    gnrc_pktsnip_t *netif = gnrc_netif_hdr_build(NULL, 0, dst_l2addr, dst_l2addr_len);

    if (netif == NULL) {
        DEBUG("ipv6: error on interface header allocation, dropping packet\r\n");
        gnrc_pktbuf_release(pkt);
        return NULL;
    }

    if (pkt->type == GNRC_NETTYPE_NETIF) {
        /* remove old netif header, since checking it for correctness would
         * cause to much overhead.
         * netif header might have been allocated by some higher layer either
         * to set a sending interface or some flags. Interface was already
         * copied using iface parameter, so we only need to copy the flags
         * (minus the broadcast/multicast flags) */
        DEBUG("ipv6: copy old interface header flags\r\n");
        gnrc_netif_hdr_t *netif_new = netif->data, *netif_old = pkt->data;
        netif_new->flags = netif_old->flags & \
                           ~(GNRC_NETIF_HDR_FLAGS_BROADCAST | GNRC_NETIF_HDR_FLAGS_MULTICAST);
        DEBUG("ipv6: removed old interface header\r\n");
        pkt = gnrc_pktbuf_remove_snip(pkt, pkt);
    }

    /* add netif to front of the pkt list */
    LL_PREPEND(pkt, netif);

    return pkt;
}

/* functions for sending */
static void _send_unicast(kernel_pid_t iface, uint8_t *dst_l2addr,
                          uint16_t dst_l2addr_len, gnrc_pktsnip_t *pkt)
{
    DEBUG("ipv6: add interface header to packet\r\n");
    if ((pkt = _create_netif_hdr(dst_l2addr, dst_l2addr_len, pkt)) == NULL) {
        return;
    }
    DEBUG("ipv6: send unicast over interface %" PRIkernel_pid "\r\n", iface);
    /* and send to interface */
#ifdef MODULE_NETSTATS_IPV6
    gnrc_ipv6_netif_get_stats(iface)->tx_unicast_count++;
#endif
    _send_to_iface(iface, pkt);
}

static int _fill_ipv6_hdr(kernel_pid_t iface, gnrc_pktsnip_t *ipv6,
                          gnrc_pktsnip_t *payload)
{
    int res;
    ipv6_hdr_t *hdr = ipv6->data;

    hdr->len = byteorder_htons(gnrc_pkt_len(payload));
    DEBUG("ipv6: set payload length to %u (network byteorder %04" PRIx16 ")\r\n",
          (unsigned) gnrc_pkt_len(payload), hdr->len.u16);

    /* check if e.g. extension header was not already marked */
    if (hdr->nh == PROTNUM_RESERVED) {
        hdr->nh = gnrc_nettype_to_protnum(payload->type);

        /* if still reserved: mark no next header */
        if (hdr->nh == PROTNUM_RESERVED) {
            hdr->nh = PROTNUM_IPV6_NONXT;
        }
    }

    DEBUG("ipv6: set next header to %u\r\n", hdr->nh);

    if (hdr->hl == 0) {
        if (iface == KERNEL_PID_UNDEF) {
            hdr->hl = GNRC_IPV6_NETIF_DEFAULT_HL;
        }
        else {
            hdr->hl = gnrc_ipv6_netif_get(iface)->cur_hl;
        }
    }

    if (ipv6_addr_is_unspecified(&hdr->src)) {
        if (ipv6_addr_is_loopback(&hdr->dst)) {
            ipv6_addr_set_loopback(&hdr->src);
        }
        else {
            ipv6_addr_t *src = gnrc_ipv6_netif_find_best_src_addr(iface, &hdr->dst, false);

            if (src != NULL) {
                DEBUG("ipv6: set packet source to %s\r\n",
                      ipv6_addr_to_str(addr_str, src, sizeof(addr_str)));
                memcpy(&hdr->src, src, sizeof(ipv6_addr_t));
            }
            /* Otherwise leave unspecified */
        }
    }

    DEBUG("ipv6: calculate checksum for upper header.\r\n");

    if ((res = gnrc_netreg_calc_csum(payload, ipv6)) < 0) {
        if (res != -ENOENT) {   /* if there is no checksum we are okay */
            DEBUG("ipv6: checksum calculation failed.\r\n");
            return res;
        }
    }

    return 0;
}

static inline void _send_multicast_over_iface(kernel_pid_t iface, gnrc_pktsnip_t *pkt)
{
    DEBUG("ipv6: send multicast over interface %" PRIkernel_pid "\r\n", iface);
    /* mark as multicast */
    ((gnrc_netif_hdr_t *)pkt->data)->flags |= GNRC_NETIF_HDR_FLAGS_MULTICAST;
#ifdef MODULE_NETSTATS_IPV6
    gnrc_ipv6_netif_get_stats(iface)->tx_mcast_count++;
#endif
    /* and send to interface */
    _send_to_iface(iface, pkt);
}

static void _send_multicast(kernel_pid_t iface, gnrc_pktsnip_t *pkt,
                            gnrc_pktsnip_t *ipv6, gnrc_pktsnip_t *payload,
                            bool prep_hdr)
{
    kernel_pid_t ifs[GNRC_NETIF_NUMOF];
    size_t ifnum = 0;

    if (iface == KERNEL_PID_UNDEF) {
        /* get list of interfaces */
        ifnum = gnrc_netif_get(ifs);

        /* throw away packet if no one is interested */
        if (ifnum == 0) {
            DEBUG("ipv6: no interfaces registered, dropping packet\r\n");
            gnrc_pktbuf_release(pkt);
            return;
        }
    }


#if GNRC_NETIF_NUMOF > 1
    /* interface not given: send over all interfaces */
    if (iface == KERNEL_PID_UNDEF) {
        /* send packet to link layer */
        gnrc_pktbuf_hold(pkt, ifnum - 1);

        for (size_t i = 0; i < ifnum; i++) {
            if (prep_hdr) {
                /* need to get second write access (duplication) to fill IPv6
                 * header interface-local */
                gnrc_pktsnip_t *tmp = gnrc_pktbuf_start_write(pkt);
                gnrc_pktsnip_t *ptr = tmp->next;
                ipv6 = tmp;

                if (ipv6 == NULL) {
                    DEBUG("ipv6: unable to get write access to IPv6 header, "
                          "for interface %" PRIkernel_pid "\r\n", ifs[i]);
                    gnrc_pktbuf_release(pkt);
                    return;
                }

                /* multiple interfaces => possibly different source addresses
                 * => different checksums => duplication of payload needed */
                while (ptr != payload->next) {
                    /* duplicate everything including payload */
                    tmp->next = gnrc_pktbuf_start_write(ptr);
                    if (tmp->next == NULL) {
                        DEBUG("ipv6: unable to get write access to payload, drop it\r\n");
                        gnrc_pktbuf_release(ipv6);
                        return;
                    }
                    tmp = tmp->next;
                    ptr = ptr->next;
                }

                if (_fill_ipv6_hdr(ifs[i], ipv6, tmp) < 0) {
                    /* error on filling up header */
                    gnrc_pktbuf_release(ipv6);
                    return;
                }
            }

            if ((ipv6 = _create_netif_hdr(NULL, 0, ipv6)) == NULL) {
                return;
            }

            _send_multicast_over_iface(ifs[i], ipv6);
        }
    }
    else {
        if (prep_hdr) {
            if (_fill_ipv6_hdr(iface, ipv6, payload) < 0) {
                /* error on filling up header */
                gnrc_pktbuf_release(pkt);
                return;
            }
        }

        _send_multicast_over_iface(iface, pkt);
    }
#else   /* GNRC_NETIF_NUMOF */
    (void)ifnum; /* not used in this build branch */
    if (iface == KERNEL_PID_UNDEF) {
        iface = ifs[0];

        /* allocate interface header */
        if ((pkt = _create_netif_hdr(NULL, 0, pkt)) == NULL) {
            return;
        }
    }

    if (prep_hdr) {
        if (_fill_ipv6_hdr(iface, ipv6, payload) < 0) {
            /* error on filling up header */
            gnrc_pktbuf_release(pkt);
            return;
        }
    }

    _send_multicast_over_iface(iface, pkt);
#endif  /* GNRC_NETIF_NUMOF */
}

static inline kernel_pid_t _next_hop_l2addr(uint8_t *l2addr, uint8_t *l2addr_len,
                                            kernel_pid_t iface, ipv6_addr_t *dst,
                                            gnrc_pktsnip_t *pkt)
{
    kernel_pid_t found_iface;
#if defined(MODULE_GNRC_SIXLOWPAN_ND)
    DEBUG("ipv6 next hop addr 1\r\n");
    (void)pkt;
    found_iface = gnrc_sixlowpan_nd_next_hop_l2addr(l2addr, l2addr_len, iface, dst);
    if (found_iface > KERNEL_PID_UNDEF) {
        return found_iface;
    }
#endif
#if defined(MODULE_GNRC_NDP_NODE)
    DEBUG("ipv6 next hop addr 2\r\n");
    found_iface = gnrc_ndp_node_next_hop_l2addr(l2addr, l2addr_len, iface, dst, pkt);
#elif !defined(MODULE_GNRC_SIXLOWPAN_ND) && defined(MODULE_GNRC_IPV6_NC)
    DEBUG("ipv6 next hop addr 3\r\n");
    (void)pkt;
    gnrc_ipv6_nc_t *nc = gnrc_ipv6_nc_get(iface, dst);
    found_iface = gnrc_ipv6_nc_get_l2_addr(l2addr, l2addr_len, nc);
#elif !defined(MODULE_GNRC_SIXLOWPAN_ND)
    DEBUG("ipv6 next hop addr 4\r\n");
    found_iface = KERNEL_PID_UNDEF;
    (void)l2addr;
    (void)l2addr_len;
    (void)iface;
    (void)dst;
    (void)pkt;
    *l2addr_len = 0;
#endif
    return found_iface;
}

static void _send(gnrc_pktsnip_t *pkt, bool prep_hdr)
{
    kernel_pid_t iface = KERNEL_PID_UNDEF;
    gnrc_pktsnip_t *ipv6, *payload;
    ipv6_addr_t *tmp;
    ipv6_hdr_t *hdr;
    /* get IPv6 snip and (if present) generic interface header */
    if (pkt->type == GNRC_NETTYPE_NETIF) {
        /* If there is already a netif header (routing protocols and
         * neighbor discovery might add them to preset sending interface) */
        iface = ((gnrc_netif_hdr_t *)pkt->data)->if_pid;
        /* seize payload as temporary variable */
        ipv6 = gnrc_pktbuf_start_write(pkt); /* write protect for later removal
                                              * in _send_unicast() */
        if (ipv6 == NULL) {
            DEBUG("ipv6: unable to get write access to netif header, dropping packet\r\n");
            gnrc_pktbuf_release(pkt);
            return;
        }
        pkt = ipv6;  /* Reset pkt from temporary variable */

        ipv6 = pkt->next;
    }
    else {
        ipv6 = pkt;
    }
    /* seize payload as temporary variable */
    payload = gnrc_pktbuf_start_write(ipv6);
    if (payload == NULL) {
        DEBUG("ipv6: unable to get write access to IPv6 header, dropping packet\r\n");
        gnrc_pktbuf_release(pkt);
        return;
    }
    if (ipv6 != pkt) {      /* in case packet has netif header */
        pkt->next = payload;/* pkt is already write-protected so we can do that */
    }
    else {
        pkt = payload;      /* pkt is the IPv6 header so we just write-protected it */
    }
    ipv6 = payload;  /* Reset ipv6 from temporary variable */

    hdr = ipv6->data;
    payload = ipv6->next;

    if (ipv6_addr_is_multicast(&hdr->dst)) {
        _send_multicast(iface, pkt, ipv6, payload, prep_hdr);
    }
    else if ((ipv6_addr_is_loopback(&hdr->dst)) ||      /* dst is loopback address */
             ((iface == KERNEL_PID_UNDEF) && /* or dst registered to any local interface */
              ((iface = gnrc_ipv6_netif_find_by_addr(&tmp, &hdr->dst)) != KERNEL_PID_UNDEF)) ||
             ((iface != KERNEL_PID_UNDEF) && /* or dst registered to given interface */
              (gnrc_ipv6_netif_find_addr(iface, &hdr->dst) != NULL))) {
        uint8_t *rcv_data;
        gnrc_pktsnip_t *ptr = ipv6, *rcv_pkt;

        if (prep_hdr) {
            if (_fill_ipv6_hdr(iface, ipv6, payload) < 0) {
                /* error on filling up header */
                gnrc_pktbuf_release(pkt);
                return;
            }
        }

        rcv_pkt = gnrc_pktbuf_add(NULL, NULL, gnrc_pkt_len(ipv6), GNRC_NETTYPE_IPV6);

        if (rcv_pkt == NULL) {
            DEBUG("ipv6: error on generating loopback packet\r\n");
            gnrc_pktbuf_release(pkt);
            return;
        }

        rcv_data = rcv_pkt->data;

        /* "reverse" packet (by making it one snip as if received from NIC) */
        while (ptr != NULL) {
            memcpy(rcv_data, ptr->data, ptr->size);
            rcv_data += ptr->size;
            ptr = ptr->next;
        }

        gnrc_pktbuf_release(pkt);

        DEBUG("ipv6: packet is addressed to myself => loopback\r\n");

        if (gnrc_netapi_receive(gnrc_ipv6_pid, rcv_pkt) < 1) {
            DEBUG("ipv6: unable to deliver packet\r\n");
            gnrc_pktbuf_release(rcv_pkt);
        }
    }
    else {
        uint8_t l2addr_len = GNRC_IPV6_NC_L2_ADDR_MAX;
        uint8_t l2addr[l2addr_len];

        iface = _next_hop_l2addr(l2addr, &l2addr_len, iface, &hdr->dst, pkt);

        if (iface == KERNEL_PID_UNDEF) {
            DEBUG("ipv6: error determining next hop's link layer address\r\n");
            gnrc_pktbuf_release(pkt);
            return;
        }

        if (prep_hdr) {
            if (_fill_ipv6_hdr(iface, ipv6, payload) < 0) {
                /* error on filling up header */
                gnrc_pktbuf_release(pkt);
                return;
            }
        }

        _send_unicast(iface, l2addr, l2addr_len, pkt);
    }
}

/* functions for receiving */
static inline bool _pkt_not_for_me(kernel_pid_t *iface, ipv6_hdr_t *hdr)
{
    if (ipv6_addr_is_loopback(&hdr->dst)) {
        return false;
    }
    else if ((!ipv6_addr_is_link_local(&hdr->dst)) ||
             (*iface == KERNEL_PID_UNDEF)) {
        kernel_pid_t if_pid = gnrc_ipv6_netif_find_by_addr(NULL, &hdr->dst);
        if (*iface == KERNEL_PID_UNDEF) {
            *iface = if_pid;    /* Use original interface for reply if
                                 * existent */
        }
        return (if_pid == KERNEL_PID_UNDEF);
    }
    else {
        return (gnrc_ipv6_netif_find_addr(*iface, &hdr->dst) == NULL);
    }
}

static void _receive(gnrc_pktsnip_t *pkt)
{
    kernel_pid_t iface = KERNEL_PID_UNDEF;
    gnrc_pktsnip_t *ipv6, *netif, *first_ext;
    ipv6_hdr_t *hdr;

    assert(pkt != NULL);

    netif = gnrc_pktsnip_search_type(pkt, GNRC_NETTYPE_NETIF);

    if (netif != NULL) {
        iface = ((gnrc_netif_hdr_t *)netif->data)->if_pid;

#ifdef MODULE_NETSTATS_IPV6
        assert(iface);
        netstats_t *stats = gnrc_ipv6_netif_get_stats(iface);
        stats->rx_count++;
        stats->rx_bytes += (gnrc_pkt_len(pkt) - netif->size);
#endif
    }

    first_ext = pkt;

    for (ipv6 = pkt; ipv6 != NULL; ipv6 = ipv6->next) { /* find IPv6 header if already marked */
        if ((ipv6->type == GNRC_NETTYPE_IPV6) && (ipv6->size == sizeof(ipv6_hdr_t)) &&
            (ipv6_hdr_is(ipv6->data))) {
            break;
        }

        first_ext = ipv6;
    }

    if (ipv6 == NULL) {
        if (!ipv6_hdr_is(pkt->data)) {
            DEBUG("ipv6: Received packet was not IPv6, dropping packet\r\n");
            gnrc_pktbuf_release(pkt);
            return;
        }
#ifdef MODULE_GNRC_IPV6_WHITELIST
        if (!gnrc_ipv6_whitelisted(&((ipv6_hdr_t *)(pkt->data))->src)) {
            DEBUG("ipv6: Source address not whitelisted, dropping packet\r\n");
            gnrc_pktbuf_release(pkt);
            return;
        }
#endif
#ifdef MODULE_GNRC_IPV6_BLACKLIST
        if (gnrc_ipv6_blacklisted(&((ipv6_hdr_t *)(pkt->data))->src)) {
            DEBUG("ipv6: Source address blacklisted, dropping packet\r\n");
            gnrc_pktbuf_release(pkt);
            return;
        }
#endif
        /* seize ipv6 as a temporary variable */
        ipv6 = gnrc_pktbuf_start_write(pkt);

        if (ipv6 == NULL) {
            DEBUG("ipv6: unable to get write access to packet, drop it\r\n");
            gnrc_pktbuf_release(pkt);
            return;
        }

        pkt = ipv6;     /* reset pkt from temporary variable */

        ipv6 = gnrc_pktbuf_mark(pkt, sizeof(ipv6_hdr_t), GNRC_NETTYPE_IPV6);

        first_ext = pkt;
        pkt->type = GNRC_NETTYPE_UNDEF; /* snip is no longer IPv6 */

        if (ipv6 == NULL) {
            DEBUG("ipv6: error marking IPv6 header, dropping packet\r\n");
            gnrc_pktbuf_release(pkt);
            return;
        }
    }
#ifdef MODULE_GNRC_IPV6_WHITELIST
    else if (!gnrc_ipv6_whitelisted(&((ipv6_hdr_t *)(ipv6->data))->src)) {
        /* if ipv6 header already marked*/
        DEBUG("ipv6: Source address not whitelisted, dropping packet\r\n");
        gnrc_pktbuf_release(pkt);
        return;
    }
#endif
#ifdef MODULE_GNRC_IPV6_BLACKLIST
    else if (gnrc_ipv6_blacklisted(&((ipv6_hdr_t *)(ipv6->data))->src)) {
        /* if ipv6 header already marked*/
        DEBUG("ipv6: Source address blacklisted, dropping packet\r\n");
        gnrc_pktbuf_release(pkt);
        return;
    }
#endif

    /* extract header */
    hdr = (ipv6_hdr_t *)ipv6->data;

    /* if available, remove any padding that was added by lower layers
     * to fulfill their minimum size requirements (e.g. ethernet) */
    if (byteorder_ntohs(hdr->len) < pkt->size) {
        gnrc_pktbuf_realloc_data(pkt, byteorder_ntohs(hdr->len));
    }
    else if (byteorder_ntohs(hdr->len) >
             (gnrc_pkt_len_upto(pkt, GNRC_NETTYPE_IPV6) - sizeof(ipv6_hdr_t))) {
        DEBUG("ipv6: invalid payload length: %d, actual: %d, dropping packet\r\n",
              (int) byteorder_ntohs(hdr->len),
              (int) (gnrc_pkt_len_upto(pkt, GNRC_NETTYPE_IPV6) - sizeof(ipv6_hdr_t)));
        gnrc_pktbuf_release(pkt);
        return;
    }

    DEBUG("ipv6: Received (src = %s, ",
          ipv6_addr_to_str(addr_str, &(hdr->src), sizeof(addr_str)));
    DEBUG("dst = %s, next header = %u, length = %" PRIu16 ")\r\n",
          ipv6_addr_to_str(addr_str, &(hdr->dst), sizeof(addr_str)),
          hdr->nh, byteorder_ntohs(hdr->len));

    if (_pkt_not_for_me(&iface, hdr)) { /* if packet is not for me */
        DEBUG("ipv6: packet destination not this host\r\n");

#ifdef MODULE_GNRC_IPV6_ROUTER    /* only routers redirect */
        /* redirect to next hop */
        DEBUG("ipv6: decrement hop limit to %u\r\n", (uint8_t) (hdr->hl - 1));

        /* RFC 4291, section 2.5.6 states: "Routers must not forward any
         * packets with Link-Local source or destination addresses to other
         * links."
         */
        if ((ipv6_addr_is_link_local(&(hdr->src))) || (ipv6_addr_is_link_local(&(hdr->dst)))) {
            DEBUG("ipv6: do not forward packets with link-local source or"
                  " destination address\r\n");
            gnrc_pktbuf_release(pkt);
            return;
        }
        /* TODO: check if receiving interface is router */
        else if (--(hdr->hl) > 0) {  /* drop packets that *reach* Hop Limit 0 */
            gnrc_pktsnip_t *reversed_pkt = NULL, *ptr = pkt;

            DEBUG("ipv6: forward packet to next hop\r\n");

            /* pkt might not be writable yet, if header was given above */
            ipv6 = gnrc_pktbuf_start_write(ipv6);
            if (ipv6 == NULL) {
                DEBUG("ipv6: unable to get write access to packet: dropping it\r\n");
                gnrc_pktbuf_release(pkt);
                return;
            }

            /* remove L2 headers around IPV6 */
            netif = gnrc_pktsnip_search_type(pkt, GNRC_NETTYPE_NETIF);
            if (netif != NULL) {
                gnrc_pktbuf_remove_snip(pkt, netif);
            }

            /* reverse packet snip list order */
            while (ptr != NULL) {
                gnrc_pktsnip_t *next;
                ptr = gnrc_pktbuf_start_write(ptr);     /* duplicate if not already done */
                if (ptr == NULL) {
                    DEBUG("ipv6: unable to get write access to packet: dropping it\r\n");
                    gnrc_pktbuf_release(reversed_pkt);
                    gnrc_pktbuf_release(pkt);
                    return;
                }
                next = ptr->next;
                ptr->next = reversed_pkt;
                reversed_pkt = ptr;
                ptr = next;
            }
            _send(reversed_pkt, false);
            return;
        }
        else {
            DEBUG("ipv6: hop limit reached 0: drop packet\r\n");
            gnrc_pktbuf_release(pkt);
            return;
        }

#else  /* MODULE_GNRC_IPV6_ROUTER */
        DEBUG("ipv6: dropping packet\r\n");
        /* non rounting hosts just drop the packet */
        gnrc_pktbuf_release(pkt);
        return;
#endif /* MODULE_GNRC_IPV6_ROUTER */
    }

    /* IPv6 internal demuxing (ICMPv6, Extension headers etc.) */
    gnrc_ipv6_demux(iface, first_ext, pkt, hdr->nh);
}

static void _decapsulate(gnrc_pktsnip_t *pkt)
{
    gnrc_pktsnip_t *ptr = pkt;

    pkt->type = GNRC_NETTYPE_UNDEF; /* prevent payload (the encapsulated packet)
                                     * from being removed */

    /* Remove encapsulating IPv6 header */
    while ((ptr->next != NULL) && (ptr->next->type == GNRC_NETTYPE_IPV6)) {
        gnrc_pktbuf_remove_snip(pkt, pkt->next);
    }

    pkt->type = GNRC_NETTYPE_IPV6;

    _receive(pkt);
}

/** @} */
