/*
 * Copyright (C) 2016 Freie Universität Berlin
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @{
 *
 * @file
 * @brief       Show case application for netdev_test_dw1000
 *
 * @author      Hessam Mohammadmoradi <hmoradi@cs.uh.edu>
 *
 * @}
 */

#include <stdio.h>

#include "msg.h"
#include "net/ethernet.h"
#include "net/gnrc.h"
#include "net/gnrc/netdev/eth.h"
#include "net/netdev_test.h"
#include "od.h"
#include "thread.h"
#include "utlist.h"
#include "net/gnrc/netdev/ieee802154.h"
//#include "../../../drivers/dw1000/include/dw1000_netdev.h"
#include "../../../drivers/dw1000/include/dw1000_params.h"
#include "dw1000.h"
#define ENABLE_DEBUG (1)
#include "debug.h"
#include "xtimer.h"
#include "fmt.h"
#define _EXP_LENGTH     (64)
#define LED GPIO_PIN(PORT_B, 1)
#define _MAC_STACKSIZE  (THREAD_STACKSIZE_DEFAULT + THREAD_EXTRA_STACKSIZE_PRINTF)
#define _MAC_PRIO       (THREAD_PRIORITY_MAIN - 4)

#define _MAIN_MSG_QUEUE_SIZE (2)

#define _TEST_PAYLOAD1  "gO3Xt,fP)6* MR161Auk?W^mTb\"LmY^Qc5w1h:C<+n(*/@4k("
#define _TEST_PAYLOAD2  "*b/'XKkraEBexaU\\O-X&<Bl'n%35Ll+nDy,jQ+[Oe4:9( 4cI"
#define _TEST_PAYLOAD3  "Hello World !"
#define EXECUTE(test) \
    if (!test()) { \
        puts(" + failed.\r\n"); \
        return 1; \
    } \
    else { \
        puts(" + succeeded.\r\n"); \
    }

static const uint8_t _test_dst[] = { 0xf5, 0x19, 0x9a, 0x1d, 0xd8, 0x8f,0x01,0x02 };
static const uint8_t _test_src[] = { 0xf5, 0x19, 0x9f, 0x56, 0x36, 0x46,0x03,0x04 };

static char _mac_stack[_MAC_STACKSIZE];
static gnrc_netdev_t _gnrc_dev;
static dw1000_t _devDW1000;
static msg_t _main_msg_queue[_MAIN_MSG_QUEUE_SIZE];
//static uint8_t _tmp[_EXP_LENGTH];
static kernel_pid_t _mac_pid;

/* tests sending */
static int test_send(void)
{
    DEBUG("test send \r\n");
    gnrc_pktsnip_t *pkt, *hdr;
    msg_t msg;

    /* prepare packet for sending */
    DEBUG("prepare packet for sending \r\n");
    pkt = gnrc_pktbuf_add(NULL, _TEST_PAYLOAD3, sizeof(_TEST_PAYLOAD3) - 1,
                          GNRC_NETTYPE_UNDEF);
    if (pkt == NULL) {
        puts("Could not allocate send payload \r\n");
        return 0;
    }
    puts("\r\n");
    puts("sent msg is after pkt generation \r\n");
    print_str(pkt->data);
    puts("\r\n");
    DEBUG("preparing the header \r\n");
    hdr = gnrc_netif_hdr_build((uint8_t *)_test_src, sizeof(_test_src), (uint8_t *)_test_dst, sizeof(_test_dst));
    if (hdr == NULL) {
        gnrc_pktbuf_release(pkt);
        puts("Could not allocate send header \r\n");
        return 0;
    }
    DEBUG("prepending pkt and hdr \r\n");
    LL_PREPEND(pkt, hdr);
    puts("\r\n");
    puts("sent msg is after appending to hdr\r\n");
    print_str(pkt->next->data);
    puts("\r\n");
    DEBUG("register for returned packet status \r\n");
    /* register for returned packet status */
    if (gnrc_neterr_reg(pkt) != 0) {
        puts("Can not register for error reporting \r\n");
        return 0;
    }
    DEBUG("send pkt to MAC layer \r\n");
    /* send packet to MAC layer */

    gnrc_netapi_send(_mac_pid, pkt);
    /* wait for packet status and check */
    DEBUG("wait for pkt status \r\n");
    msg_receive(&msg);
    if ((msg.type != GNRC_NETERR_MSG_TYPE) ||
        (msg.content.value != GNRC_NETERR_SUCCESS)) {
        puts("Error sending packet \r\n");
        return 0;
    }
    gpio_write(LED, 1);

    DEBUG("test send is done ! \r\n");
    return 1;
}

/* tests receiving */
static int test_receive(void)
{
    DEBUG("test recieve \r\n");
    netdev_t *netdev = (netdev_t *)(&_devDW1000);
    DEBUG("set dw1000 to receive mode \r\n");
    netdev->driver->set(netdev, NETOPT_RX_START_IRQ, 0, 0);
    gnrc_pktsnip_t *pkt, *hdr;
    gnrc_netreg_entry_t me = GNRC_NETREG_ENTRY_INIT_PID(GNRC_NETREG_DEMUX_CTX_ALL,
                                                        sched_active_pid);
    msg_t msg;

    /* no gnrc_ipv6 in compile unit => ETHERTYPE_IPV6 translates to
     * GNRC_NETTYPE_UNDEF */

    DEBUG("reg the thread to stack \r\n");
    /* register for GNRC_NETTYPE_UNDEF */
    gnrc_netreg_register(GNRC_NETTYPE_UNDEF, &me);
    /* fire ISR event */
    //_dev.netdev.event_callback((netdev_t *)&_dev.netdev, NETDEV_EVENT_ISR);
    DEBUG("wait for msg from mac layer \r\n");
    /* wait for packet from MAC layer*/
    msg_receive(&msg);
    gpio_write(LED, 1);
    /* check message */
    DEBUG("cudo msg received !!!!!!! \r\n");
    if (msg.sender_pid != _mac_pid) {
        puts("Unexpected sender of netapi receive message\r\n");
        return 0;
    }
    if (msg.type != GNRC_NETAPI_MSG_TYPE_RCV) {
        puts("Expected netapi receive message\r\n");
        return 0;
    }
    pkt = msg.content.ptr;
    /* check payload */
    DEBUG("rec pkt size is %d \r\n",pkt->size);

    if ((pkt->type != GNRC_NETTYPE_UNDEF) ||
        (memcmp(pkt->data, _TEST_PAYLOAD3, pkt->size) != 0)) {
        puts("Unexpected payload\r\n");
        puts("===========================================================\r\n");
        puts("expected\r\n");
        puts("===========================================================\r\n");
        od_hex_dump(_TEST_PAYLOAD3, pkt->size, OD_WIDTH_DEFAULT);
        puts("===========================================================\r\n");
        puts("received  data\r\n");
        puts("===========================================================\r\n");
        od_hex_dump(pkt->data, pkt->size, OD_WIDTH_DEFAULT);
        return 0;
    }
    hdr = pkt->next;
    if (memcmp(gnrc_netif_hdr_get_src_addr(hdr->data), _test_src,
               8) != 0) {
        char addr_str[8 * 3];
        puts("Unexpected source received\r\n");
        puts("=================\r\n");
        puts("expected\r\n");
        puts("=================\r\n");
        puts(gnrc_netif_addr_to_str(addr_str, sizeof(addr_str),
                                    _test_src,
                                    8));
        puts("=================\r\n");
        puts("received source\r\n");
        puts("=================\r\n");
        puts(gnrc_netif_addr_to_str(addr_str, sizeof(addr_str),
                                    gnrc_netif_hdr_get_src_addr(hdr->data),
                                    8));
        return 0;
    }
    if (memcmp(gnrc_netif_hdr_get_dst_addr(hdr->data), _test_dst,
               8) != 0) {
        char addr_str[8 * 3];
        puts("Unexpected destination received\r\n");
        puts("=================\r\n");
        puts("expected\r\n");
        puts("=================\r\n");
        puts(gnrc_netif_addr_to_str(addr_str, sizeof(addr_str),
                                    _test_dst,
                                    8));
        puts("====================\r\n");
        puts("received destination\r\n");
        puts("====================\r\n");
        puts(gnrc_netif_addr_to_str(addr_str, sizeof(addr_str),
                                    gnrc_netif_hdr_get_dst_addr(hdr->data),
                                    8));
        return 0;
    }
    char addr_str[8 * 3];
    puts("=================\r\n");
    puts("received source\r\n");
    puts("=================\r\n");
    puts(gnrc_netif_addr_to_str(addr_str, sizeof(addr_str),
                                gnrc_netif_hdr_get_src_addr(hdr->data),
                                8));
    puts("\r\n received destination\r\n");
    puts("====================\r\n");
    puts(gnrc_netif_addr_to_str(addr_str, sizeof(addr_str),
                                gnrc_netif_hdr_get_dst_addr(hdr->data),
                                8));
    puts("\r\n");
    puts("received msg is \r\n");
    print_str(pkt->data);
    puts("\r\n");
    gnrc_pktbuf_release(pkt);
    gnrc_netreg_unregister(GNRC_NETTYPE_UNDEF, &me);
    return 1;
}


int main(void)
{
    /* initialization */
    DEBUG("DW1000 net driver test \r\n");
    int role = 0; //set to 1 for reciever
    DEBUG("the role is %d \r\n",role);
    xtimer_init();
    if (gpio_init(LED, GPIO_OUT) < 0) {
        DEBUG("[ERROR] SDI GPIO initialization failed.");
    }
    DEBUG("xtimer init is done  ! \r\n");

    gnrc_pktbuf_init();
    msg_init_queue(_main_msg_queue, _MAIN_MSG_QUEUE_SIZE);
    DEBUG("main: setting up dw1000 \r\n");
    dw1000_setup(&_devDW1000, &dw1000_params[0]);
    DEBUG("main: puting to send/rec mode \r\n");
    netdev_t * dev = (netdev_t *)&_devDW1000;
    if(role==1) {
        //start as reciever
        dev->driver->set(dev, NETOPT_RX_START_IRQ, 0, 0);
    }else{
        dev->driver->set(dev, NETOPT_RX_END_IRQ, 0, 0);
    }
    DEBUG("main: init gnrc  \r\n");
    netdev_ieee802154_t *dev802 = (netdev_ieee802154_t *)(&_devDW1000);
    dev802->pan = 12;
    dev802->seq = 0;
    dev802->flags = 0;
    gnrc_netdev_ieee802154_init(&_gnrc_dev,dev802);
    DEBUG("main: reg thread to mac \r\n");
    _mac_pid = gnrc_netdev_init(_mac_stack, _MAC_STACKSIZE, _MAC_PRIO,
                                "gnrc_netdev_eth_test", &_gnrc_dev);
    if (_mac_pid <= KERNEL_PID_UNDEF) {
        puts("Could not start MAC thread\r\n");
        return 1;
    }
    DEBUG("main: init is done \r\n");
    if(role == 0){// sender
        EXECUTE(test_send);
    }
    else{
        EXECUTE(test_receive);
    }
    puts("ALL TESTS SUCCESSFUL");

    return 0;
}


