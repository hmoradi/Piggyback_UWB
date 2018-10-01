/*
 * Copyright (C) 2015 Kaspar Schleiser <kaspar@schleiser.de>
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @{
 * @ingroup     net
 * @file
 * @brief       gnrc netdev ethernet glue code
 *
 * @author      Kaspar Schleiser <kaspar@schleiser.de>
 * @}
 */

#include "net/gnrc.h"
#include "net/gnrc/netdev.h"
#include "net/ethernet/hdr.h"

#ifdef MODULE_GNRC_IPV6
#include "net/ipv6/hdr.h"
#endif

#include "od.h"

#define ENABLE_DEBUG (1)
#include "debug.h"

static gnrc_pktsnip_t *_recv(gnrc_netdev_t *gnrc_netdev)
{
    //DEBUG("netdev eth recv \r\n");
    netdev_t *dev = gnrc_netdev->dev;
    if(dev == NULL){
        DEBUG("dev is not set in netdev eth \r\n");
    }
   // DEBUG("netdev etch recv driver is set \r\n");
    int bytes_expected = dev->driver->recv(dev, NULL, 0, NULL);
    //int bytes_expected = 63;
    //DEBUG("netdev etch bytes expected %d \r\n",bytes_expected);
    gnrc_pktsnip_t *pkt = NULL;

    if (bytes_expected > 0) {
       // DEBUG("netdev eth recv : generate packet \r\n");
        pkt = gnrc_pktbuf_add(NULL, NULL,
                bytes_expected,
                GNRC_NETTYPE_UNDEF);
        //DEBUG("netdev eth recv : generate packet is done \r\n");
        if(!pkt) {
            DEBUG("_recv_ethernet_packet: cannot allocate pktsnip.\r\n");

            /* drop the packet */
            dev->driver->recv(dev, NULL, bytes_expected, NULL);

            goto out;
        }
        //DEBUG("netdev eth recv : read from buffer to fill the packet \r\n");
        int nread = dev->driver->recv(dev, pkt->data, bytes_expected, NULL);
        if(nread <= 0) {
            DEBUG("_recv_ethernet_packet: read error.\r\n");
            goto safe_out;
        }
        //DEBUG("netdev eth recv : packet filled  \r\n");
        if (nread < bytes_expected) {
            /* we've got less then the expected packet size,
             * so free the unused space.*/

            //DEBUG("_recv_ethernet_packet: reallocating.\r\n");
            gnrc_pktbuf_realloc_data(pkt, nread);
        }
        //DEBUG("gnrc etch rec : mark ethernet heard \r\n");
        /* mark ethernet header */
        gnrc_pktsnip_t *eth_hdr = gnrc_pktbuf_mark(pkt, sizeof(ethernet_hdr_t), GNRC_NETTYPE_UNDEF);
        if (!eth_hdr) {
            DEBUG("gnrc_netdev_eth: no space left in packet buffer\r\n");
            goto safe_out;
        }
        //DEBUG("gnrc etch rec : get the eth hearder \r\n");
        ethernet_hdr_t *hdr = (ethernet_hdr_t *)eth_hdr->data;

#ifdef MODULE_L2FILTER
        if (!l2filter_pass(dev->filter, hdr->src, ETHERNET_ADDR_LEN)) {
            DEBUG("gnrc_netdev_eth: incoming packet filtered by l2filter\n");
            goto safe_out;
        }
#endif
        //DEBUG("gnrc etch rec : set payload type from ethertyp \r\n");
        /* set payload type from ethertype */
        pkt->type = gnrc_nettype_from_ethertype(byteorder_ntohs(hdr->type));
        //DEBUG("gnrc etch rec : create netif header \r\n");
        /* create netif header */
        gnrc_pktsnip_t *netif_hdr;
        netif_hdr = gnrc_pktbuf_add(NULL, NULL,
                sizeof(gnrc_netif_hdr_t) + (2 * ETHERNET_ADDR_LEN),
                GNRC_NETTYPE_NETIF);

        if (netif_hdr == NULL) {
            DEBUG("gnrc_netdev_eth: no space left in packet buffer\r\n");
            pkt = eth_hdr;
            goto safe_out;
        }
        gnrc_netif_hdr_init(netif_hdr->data, ETHERNET_ADDR_LEN, ETHERNET_ADDR_LEN);
        gnrc_netif_hdr_set_src_addr(netif_hdr->data, hdr->src, ETHERNET_ADDR_LEN);
        gnrc_netif_hdr_set_dst_addr(netif_hdr->data, hdr->dst, ETHERNET_ADDR_LEN);
        ((gnrc_netif_hdr_t *)netif_hdr->data)->if_pid = thread_getpid();

        DEBUG("gnrc_netdev_eth: received packet from %02x:%02x:%02x:%02x:%02x:%02x "
                "of length %d\r\n",
                hdr->src[0], hdr->src[1], hdr->src[2], hdr->src[3], hdr->src[4],
                hdr->src[5], nread);
#if defined(MODULE_OD) && ENABLE_DEBUG
        od_hex_dump(hdr, nread, OD_WIDTH_DEFAULT);
#endif

        gnrc_pktbuf_remove_snip(pkt, eth_hdr);
        LL_APPEND(pkt, netif_hdr);
    }

out:
    //DEBUG("gnrc etch rec : returning packet \r\n");
    return pkt;

safe_out:
    gnrc_pktbuf_release(pkt);
    return NULL;
}

static inline void _addr_set_broadcast(uint8_t *dst)
{
    memset(dst, 0xff, ETHERNET_ADDR_LEN);
}

static inline void _addr_set_multicast(uint8_t *dst, gnrc_pktsnip_t *payload)
{
    switch (payload->type) {
#ifdef MODULE_GNRC_IPV6
        case GNRC_NETTYPE_IPV6:
            /* https://tools.ietf.org/html/rfc2464#section-7 */
            dst[0] = 0x33;
            dst[1] = 0x33;
            ipv6_hdr_t *ipv6 = payload->data;
            memcpy(dst + 2, ipv6->dst.u8 + 12, 4);
            break;
#endif
        default:
            _addr_set_broadcast(dst);
            break;
    }
}

static int _send(gnrc_netdev_t *gnrc_netdev, gnrc_pktsnip_t *pkt)
{
    //DEBUG("netdev eth send \r\n");
    ethernet_hdr_t hdr;
    gnrc_netif_hdr_t *netif_hdr;
    gnrc_pktsnip_t *payload;
    int res;

    netdev_t *dev = gnrc_netdev->dev;

    if (pkt == NULL) {
        DEBUG("gnrc_netdev_eth: pkt was NULL\r\n");
        return -EINVAL;
    }

    payload = pkt->next;

    if (pkt->type != GNRC_NETTYPE_NETIF) {
        //DEBUG("gnrc_netdev_eth: First header was not generic netif header\r\n");
        return -EBADMSG;
    }

    if (payload) {
        //DEBUG("gnrc_netdev_eth: set type to nettype ethertype \r\n");
        hdr.type = byteorder_htons(gnrc_nettype_to_ethertype(payload->type));
    }
    else {
        //DEBUG("gnrc_netdev_eth: unknown ether type\r\n");
        hdr.type = byteorder_htons(ETHERTYPE_UNKNOWN);
    }

    netif_hdr = pkt->data;

    /* set ethernet header */
    if (netif_hdr->src_l2addr_len == ETHERNET_ADDR_LEN) {
        //DEBUG("gnrc_netdev_eth: set header from src\r\n");
        memcpy(hdr.dst, gnrc_netif_hdr_get_src_addr(netif_hdr),
               netif_hdr->src_l2addr_len);
    }
    else {
        //DEBUG("gnrc_\netdev_eth: get from driver\r\n");
        dev->driver->get(dev, NETOPT_ADDRESS, hdr.src, ETHERNET_ADDR_LEN);
    }

    if (netif_hdr->flags & GNRC_NETIF_HDR_FLAGS_BROADCAST) {
       // DEBUG("gnrc_netdev_eth: set to broad cast\r\n");
        _addr_set_broadcast(hdr.dst);
    }
    else if (netif_hdr->flags & GNRC_NETIF_HDR_FLAGS_MULTICAST) {
        if (payload == NULL) {
            DEBUG("gnrc_netdev_eth: empty multicast packets over Ethernet "\
                  "are not yet supported\r\n");
            return -ENOTSUP;
        }
        _addr_set_multicast(hdr.dst, payload);
    }
    else if (netif_hdr->dst_l2addr_len == ETHERNET_ADDR_LEN) {
        memcpy(hdr.dst, gnrc_netif_hdr_get_dst_addr(netif_hdr),
               ETHERNET_ADDR_LEN);
    }
    else {
        DEBUG("gnrc_netdev_eth: destination address had unexpected format\r\n");
        return -EBADMSG;
    }

    DEBUG("gnrc_netdev_eth: send to %02x:%02x:%02x:%02x:%02x:%02x\r\n",
          hdr.dst[0], hdr.dst[1], hdr.dst[2],
          hdr.dst[3], hdr.dst[4], hdr.dst[5]);

    size_t n;
    payload = gnrc_pktbuf_get_iovec(pkt, &n);   /* use payload as temporary
                                                 * variable */
    res = -ENOBUFS;
    if (payload != NULL) {
        pkt = payload;      /* reassign for later release; vec_snip is prepended to pkt */
        struct iovec *vector = (struct iovec *)pkt->data;
        vector[0].iov_base = (char*)&hdr;
        vector[0].iov_len = sizeof(ethernet_hdr_t);
#ifdef MODULE_NETSTATS_L2
        if ((netif_hdr->flags & GNRC_NETIF_HDR_FLAGS_BROADCAST) ||
            (netif_hdr->flags & GNRC_NETIF_HDR_FLAGS_MULTICAST)) {
            gnrc_netdev->dev->stats.tx_mcast_count++;
        }
        else {
            gnrc_netdev->dev->stats.tx_unicast_count++;
        }
#endif
        res = dev->driver->send(dev, vector, n);
    }

    gnrc_pktbuf_release(pkt);

    return res;
}

int gnrc_netdev_eth_init(gnrc_netdev_t *gnrc_netdev, netdev_t *dev)
{
    gnrc_netdev->send = _send;
    gnrc_netdev->recv = _recv;
    gnrc_netdev->dev = dev;

    return 0;
}
