/*
 * Copyright (C) 2017 Freie Universität Berlin
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @ingroup     tests
 * @{
 *
 * @file
 *
 * @author      Martine Lenders <m.lenders@fu-berlin.de>
 * @}
 */

#include <stdio.h>
#include <inttypes.h>

#include "msg.h"
#include "net/gnrc.h"
#include "net/gnrc/ipv6.h"
#include "net/gnrc/udp.h"
#include "net/gnrc/pktdump.h"
#include "timex.h"
#include "xtimer.h"
#include "od.h"
#include "fmt.h"
#define SERVER_MSG_QUEUE_SIZE   (8U)
#define SERVER_PRIO             (THREAD_PRIORITY_MAIN - 1)
#define SERVER_STACKSIZE        (THREAD_STACKSIZE_MAIN + THREAD_EXTRA_STACKSIZE_PRINTF)
#define SERVER_RESET            (0x8fae)

static gnrc_netreg_entry_t server = GNRC_NETREG_ENTRY_INIT_PID(0, KERNEL_PID_UNDEF);

static char server_stack[SERVER_STACKSIZE];
static msg_t server_queue[SERVER_MSG_QUEUE_SIZE];
static kernel_pid_t server_pid = KERNEL_PID_UNDEF;
static uint8_t send_count = 0;

static void *_eventloop(void *arg)
{
    (void)arg;
    msg_t msg, reply;
    //unsigned int rcv_count = 0;

    /* setup the message queue */
    msg_init_queue(server_queue, SERVER_MSG_QUEUE_SIZE);

    reply.content.value = (uint32_t)(-ENOTSUP);
    reply.type = GNRC_NETAPI_MSG_TYPE_ACK;
    gnrc_pktsnip_t * pkt;
    while (1) {
        msg_receive(&msg);

        switch (msg.type) {
            case GNRC_NETAPI_MSG_TYPE_RCV:
                //printf("Packets received: %d\r\n", ++rcv_count);
                //
                pkt = msg.content.ptr;
                //puts("data");
                print_str(pkt->data);
                printf("\r\n");
                //
                gnrc_pktbuf_release(msg.content.ptr);
                break;
            case GNRC_NETAPI_MSG_TYPE_GET:
            case GNRC_NETAPI_MSG_TYPE_SET:
                msg_reply(&msg, &reply);
                break;
            case SERVER_RESET:
                //rcv_count = 0;
                break;
            default:
                break;
        }
    }

    /* never reached */
    return NULL;
}

static void send_data(char *addr_str, char *port_str, char *data)
{
    uint16_t port;
    ipv6_addr_t addr;

    /* parse destination address */
    if (ipv6_addr_from_str(&addr, addr_str) == NULL) {
        puts("Error: unable to parse destination address \r\n");
        return;
    }
    /* parse port */
    port = atoi(port_str);
    if (port == 0) {
        puts("Error: unable to parse destination port \r\n");
        return;
    }

    for (unsigned int i = 0; i < 1; i++) {
        gnrc_pktsnip_t *payload, *udp, *ip;
        //unsigned payload_size;
        /* allocate payload */
        //puts("add the packet \r\n");
        payload = gnrc_pktbuf_add(NULL, data, strlen(data), GNRC_NETTYPE_UNDEF);
        //puts("adding is done \r\n");
        if (payload == NULL) {
            puts("Error: unable to copy data to packet buffer \r\n");
            return;
        }
        /* store size for output */
        //payload_size = (unsigned)payload->size;
        /* allocate UDP header, set source port := destination port */
        udp = gnrc_udp_hdr_build(payload, port, port);
        if (udp == NULL) {
            puts("Error: unable to allocate UDP header \r\n");
            gnrc_pktbuf_release(payload);
            return;
        }
        /* allocate IPv6 header */
        ip = gnrc_ipv6_hdr_build(udp, NULL, &addr);
        if (ip == NULL) {
            puts("Error: unable to allocate IPv6 header \r\n");
            gnrc_pktbuf_release(udp);
            return;
        }
        /* send packet */
        if (!gnrc_netapi_dispatch_send(GNRC_NETTYPE_UDP, GNRC_NETREG_DEMUX_CTX_ALL, ip)) {
            puts("Error: unable to locate UDP thread \r\n");
            gnrc_pktbuf_release(ip);
            return;
        }
        /* access to `payload` was implicitly given up with the send operation above
         * => use temporary variable for output */
        //printf("Success: sent %u byte(s) to [%s]:%u\n", payload_size, addr_str,
        //       port);
        //xtimer_usleep(delay);
        puts("done\r\n");
    }
    //free(val);

}
static void send(char *addr_str, char *port_str, char *data_len_str, unsigned int num,
                 unsigned int delay)
{
    uint16_t port;
    ipv6_addr_t addr;
    size_t data_len;

    /* parse destination address */
    if (ipv6_addr_from_str(&addr, addr_str) == NULL) {
        puts("Error: unable to parse destination address");
        return;
    }
    /* parse port */
    port = atoi(port_str);
    if (port == 0) {
        puts("Error: unable to parse destination port");
        return;
    }

    data_len = atoi(data_len_str);
    if (data_len == 0) {
        puts("Error: unable to parse data_len");
        return;
    }

    for (unsigned int i = 0; i < num; i++) {
        gnrc_pktsnip_t *payload, *udp, *ip;
        /* allocate payload */
        payload = gnrc_pktbuf_add(NULL, NULL, data_len, GNRC_NETTYPE_UNDEF);
        if (payload == NULL) {
            puts("Error: unable to copy data to packet buffer");
            return;
        }
        memset(payload->data, send_count++, data_len);
        /* allocate UDP header, set source port := destination port */
        udp = gnrc_udp_hdr_build(payload, port, port);
        if (udp == NULL) {
            puts("Error: unable to allocate UDP header");
            gnrc_pktbuf_release(payload);
            return;
        }
        /* allocate IPv6 header */
        ip = gnrc_ipv6_hdr_build(udp, NULL, &addr);
        if (ip == NULL) {
            puts("Error: unable to allocate IPv6 header");
            gnrc_pktbuf_release(udp);
            return;
        }
        /* send packet */
        if (!gnrc_netapi_dispatch_send(GNRC_NETTYPE_UDP, GNRC_NETREG_DEMUX_CTX_ALL, ip)) {
            puts("Error: unable to locate UDP thread");
            gnrc_pktbuf_release(ip);
            return;
        }
        /* access to `payload` was implicitly given up with the send operation above
         * => use original variable for output */
        printf("Success: send %u byte to [%s]:%u\n", (unsigned)data_len, addr_str,
               port);
        xtimer_usleep(delay);
    }
}

static void start_server(char *port_str)
{
    uint16_t port;

    /* check if server is already running */
    if (server.target.pid != KERNEL_PID_UNDEF) {
        printf("Error: server already running on port %" PRIu32 "\n",
               server.demux_ctx);
        return;
    }
    /* parse port */
    port = atoi(port_str);
    if (port == 0) {
        puts("Error: invalid port specified");
        return;
    }
    if (server_pid <= KERNEL_PID_UNDEF) {
        server_pid = thread_create(server_stack, sizeof(server_stack), SERVER_PRIO,
                                   THREAD_CREATE_STACKTEST, _eventloop, NULL, "UDP server");
        if (server_pid <= KERNEL_PID_UNDEF) {
            puts("Error: can not start server thread");
            return;
        }
    }
    /* start server (which means registering pktdump for the chosen port) */
    gnrc_netreg_entry_init_pid(&server, port, server_pid);
    gnrc_netreg_register(GNRC_NETTYPE_UDP, &server);
    printf("Success: started UDP server on port %" PRIu16 "\n", port);
}

static void stop_server(void)
{
    msg_t msg = { .type = SERVER_RESET };
    /* check if server is running at all */
    if (server.target.pid == KERNEL_PID_UNDEF) {
        printf("Error: server was not running\n");
        return;
    }
    /* reset server state */
    msg_send(&msg, server.target.pid);
    /* stop server */
    gnrc_netreg_unregister(GNRC_NETTYPE_UDP, &server);
    gnrc_netreg_entry_init_pid(&server, 0, KERNEL_PID_UNDEF);
    puts("Success: stopped UDP server");
}

int udp_cmd(int argc, char **argv)
{
    if (argc < 2) {
        printf("usage: %s [send|server|reset]\n", argv[0]);
        return 1;
    }

    if (strcmp(argv[1], "send") == 0) {
        uint32_t num = 1;
        uint32_t delay = 1000000LU;
        if (argc < 5) {
            printf("usage: %s send <addr> <port> <bytes> [<num> [<delay in us>]]\n",
                   argv[0]);
            return 1;
        }
        if (argc > 5) {
            num = atoi(argv[5]);
        }
        if (argc > 6) {
            delay = atoi(argv[6]);
        }
        send(argv[2], argv[3], argv[4], num, delay);
    }
    else if (strcmp(argv[1], "send_data") == 0) {
        //uint32_t num = 1;
        //uint32_t delay = 1000000;

        if (argc < 4) {
            printf("usage: %s send data <addr> <port> <data> \n",
                   argv[0]);
            return 1;
        }
      //  printf("send_data : number of args %d \n",argc);
//        if (argc > 5) {
//            num = atoi(argv[5]);
//        }
//        if (argc > 6) {
//            delay = atoi(argv[6]);
//        }
        send_data(argv[2], argv[3], argv[4]);
    }
    else if (strcmp(argv[1], "server") == 0) {
        if (argc < 3) {
            printf("usage: %s server [start|stop]\n", argv[0]);
            return 1;
        }
        if (strcmp(argv[2], "start") == 0) {
            if (argc < 4) {
                printf("usage %s server start <port>\n", argv[0]);
                return 1;
            }
            start_server(argv[3]);
        }
        else if (strcmp(argv[2], "stop") == 0) {
            stop_server();
        }
        else {
            puts("error: invalid command");
        }
    }
    else if (strcmp(argv[1], "reset") == 0) {
        if (server_pid > KERNEL_PID_UNDEF) {
            msg_t msg = { .type = SERVER_RESET };
            msg_send(&msg, server_pid);
            send_count = (uint8_t)0;
        }
    }
    else {
        puts("error: invalid command");
    }
    return 0;
}
