/*
 * Copyright (C) 2015 Freie Universität Berlin
 *               2016 Inria
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @ingroup     drivers_dw1000
 * @{
 *
 * @file
 * @brief       Netdev adaption for the dw1000 driver
 *
 * @author      Hessam Mohammadmoradi <hmoradi@cs.uh.edu>
 *
 * @}
 */

#include <string.h>
#include <assert.h>
#include <errno.h>

#include "net/eui64.h"
#include "net/ieee802154.h"
#include "net/netdev.h"
#include "net/netdev/ieee802154.h"
#include "xtimer.h"

#include "dw1000.h"
#include "dw1000_netdev.h"
#include "dw1000_internal.h"
#include "dw1000_registers.h"
#include "net/gnrc/netdev.h"
#define ENABLE_DEBUG    (0)
#include "debug.h"


static int _send(netdev_t *netdev, const struct iovec *vector, unsigned count);
static int _recv(netdev_t *netdev, void *buf, size_t len, void *info);
static int _init(netdev_t *netdev);
static void _isr(netdev_t *netdev);
static int _get(netdev_t *netdev, netopt_t opt, void *val, size_t max_len);
static int _set(netdev_t *netdev, netopt_t opt, void *val, size_t len);
//done
const netdev_driver_t dw1000_driver = {
    .send = _send,
    .recv = _recv,
    .init = _init,
    .isr = _isr,
    .get = _get,
    .set = _set,
};
//ported
static void _irq_handler(void *arg)
{
    //puts("irq handler in dw1000 !!!!!!!! \r\n");
    netdev_t *dev = (netdev_t *)arg;
    if(dev->event_callback) {
      // puts("irq handler calling netdev callback  \r\n");
       dev->event_callback(dev, NETDEV_EVENT_ISR);
    }
}
//ported
static inline uint16_t to_u16(void *buf)
{
    return *((uint16_t *)buf);
}
//ported
static inline int16_t to_i16(void *buf)
{
    return *((int16_t *)buf);
}
//ported
static inline bool to_bool(void *buf)
{
    return *((bool *)buf);
}
//ported
static inline int w_u16(void *buf, uint16_t val)
{
    memcpy(buf, &val, sizeof(uint16_t));
    return sizeof(uint16_t);
}
//ported
static inline int w_i16(void *buf, int16_t val)
{
    memcpy(buf, &val, sizeof(int16_t));
    return sizeof(int16_t);
}
//ported
static inline int opt_state(void *buf, bool cond)
{
    *((netopt_enable_t *)buf) = !!(cond);
    return sizeof(netopt_enable_t);
}
//ported
static int _init(netdev_t *netdev)
{
    puts("dw1000 net dev init called \r\n");
    dw1000_t *dev = (dw1000_t *)netdev;


    /* initialize power and reset pins -> put the device into reset state */
   // gpio_init(dev->params.pin_reset, GPIO_OUT);
    //gpio_set(dev->params.pin_reset);

    gpio_init((GPIO_PIN(PORT_B,11)), GPIO_IN);
    gpio_init_int((GPIO_PIN(PORT_B,11)), GPIO_IN, GPIO_RISING, _irq_handler, netdev);

    /* initialize the chip select line and the SPI bus */
    //spi_init_cs(dev->params.spi, dev->params.pin_cs);
#ifdef MODULE_NETSTATS_L2
    memset(&netdev->stats, 0, sizeof(netstats_t));
#endif
    return dw1000_init((dw1000_t *)dev);
}
//ported
static void _isr(netdev_t *netdev)
{
    //puts("isr dw1000 net dev \r\n");
    //netdev->event_callback(netdev, NETDEV_EVENT_RX_COMPLETE);
    dwt_isr(netdev);
}
//ported
static int _send(netdev_t *netdev, const struct iovec *vector, unsigned count)
{
    //puts("dw1000 net dev send called\r\n");
    dw1000_t *dev = (dw1000_t *)netdev;
    return (int)dw1000_send(dev, vector, count);
}
//ported
static int _recv(netdev_t *netdev, void *buf, size_t len, void *info)
{
    //puts("dw1000 netdeve driver rec called \r\n");
    dw1000_t *dev = (dw1000_t *)netdev;
    return (int)dw1000_rx(dev, buf, len, info);
}
//partially ported
static int _get(netdev_t *netdev, netopt_t opt, void *val, size_t max_len)
{
    //DEBUG("dw10000 get is called \r\n");
    if (netdev == NULL) {
        return -ENODEV;
    }

    dw1000_t *dev = (dw1000_t *)netdev;

    int ext = netdev_ieee802154_get(&dev->netdev, opt, val, max_len);
    if (ext > 0) {
        return ext;
    }

    switch (opt) {

        case NETOPT_ADDRESS:
           // puts("dw1000 get is net op address \r\n");
            assert(max_len >= sizeof(uint16_t));
            dw1000_get_addr_short(dev, val);
            return sizeof(uint16_t);

        case NETOPT_ADDRESS_LONG:
          //  puts("dw1000 get iNETOPT_ADDRESS_LONG \r\n");
            assert(max_len >= 8);
            dw1000_get_addr_long(dev, val);
            return 8;

        case NETOPT_NID:
           // puts("dw1000 get NETOPT_NID \r\n");
            assert(max_len >= sizeof(uint16_t));
            return w_u16(val, dw1000_get_pan(dev));

        case NETOPT_CHANNEL:
          //  puts("dw1000 get NETOPT_CHANNEL \r\n");
            assert(max_len >= sizeof(uint16_t));
            return w_u16(val, dw1000_get_chan(dev));

        case NETOPT_TX_POWER:
           // puts("dw1000 get NETOPT_TX_POWER \r\n");
            assert(max_len >= sizeof(int16_t));
            return w_i16(val, dw1000_get_txpower(dev));

        case NETOPT_STATE:
          //  puts("dw1000 get NETOPT_STATE \r\n");
            assert(max_len >= sizeof(netopt_state_t));
            *((netopt_state_t *)val) = dw1000_get_state(dev);
            return sizeof(netopt_state_t);

        case NETOPT_IS_CHANNEL_CLR:
          //  puts("dw1000 get NETOPT_IS_CHANNEL_CLR \r\n");
            return 1;

        case NETOPT_AUTOACK:
         //   puts("dw1000 get NETOPT_AUTOACK \r\n");
            return opt_state(val, (dev->options & DW1000_OPT_AUTOACK));

        case NETOPT_CSMA:
         //   puts("dw1000 get NETOPT_CSMA \r\n");
            return opt_state(val, (dev->options & DW1000_OPT_CSMA));

        case NETOPT_PRELOADING:
         //   puts("dw1000 get NETOPT_PRELOADING \r\n");
            return opt_state(val, (dev->options & DW1000_OPT_PRELOADING));

        case NETOPT_PROMISCUOUSMODE:
         //   puts("dw1000 get NETOPT_PROMISCUOUSMODE \r\n");
            return opt_state(val, (dev->options & DW1000_OPT_PROMISCUOUS));

        case NETOPT_RX_START_IRQ:
         //   puts("dw1000 get NETOPT_RX_START_IRQ \r\n");
            return opt_state(val, (dev->options & DW1000_OPT_TELL_RX_START));

        case NETOPT_RX_END_IRQ:
        //    puts("dw1000 get NETOPT_RX_END_IRQ \r\n");
            return opt_state(val, (dev->options & DW1000_OPT_TELL_TX_END));

        case NETOPT_TX_START_IRQ:
        //    puts("dw1000 get NETOPT_TX_START_IRQ \r\n");
            return opt_state(val, (dev->options & DW1000_OPT_TELL_RX_START));

        case NETOPT_TX_END_IRQ:
        //    puts("dw1000 get NETOPT_TX_END_IRQ \r\n");
            return opt_state(val, (dev->options & DW1000_OPT_TELL_RX_END));

        default:
            return -ENOTSUP;
    }
}
//ported
static int _set(netdev_t *netdev, netopt_t opt, void *val, size_t val_len)
{
    if (netdev == NULL) {
        return -ENODEV;
    }

    dw1000_t *dev = (dw1000_t *)netdev;

    int ext = netdev_ieee802154_set(&dev->netdev, opt, val, val_len);

    switch (opt) {
        case NETOPT_ADDRESS:
            assert(val_len == 2);
            dw1000_set_addr_short(dev, (uint8_t *)val);
            return 2;

        case NETOPT_ADDRESS_LONG:
            assert(val_len == 8);
            dw1000_set_addr_long(dev, (uint8_t *)val);
            return 8;

        case NETOPT_NID:
            assert(val_len == sizeof(uint16_t));
            dw1000_set_pan(dev, to_u16(val));
            return sizeof(uint16_t);

        case NETOPT_CHANNEL:
            assert(val_len == sizeof(uint16_t));
            return dw1000_set_chan(dev, to_u16(val));

        case NETOPT_TX_POWER:
            assert(val_len == sizeof(int16_t));
            dw1000_set_txpower(dev, to_i16(val));
            return sizeof(int16_t);

        case NETOPT_STATE:
            assert(val_len == sizeof(netopt_state_t));
            return dw1000_set_state(dev, *((netopt_state_t *)val));

        case NETOPT_AUTOACK:
            return dw1000_set_option(dev, DW1000_OPT_AUTOACK, to_bool(val));

        case NETOPT_CSMA:
            return dw1000_set_option(dev, DW1000_OPT_CSMA, to_bool(val));

        case NETOPT_PRELOADING:
            return dw1000_set_option(dev, DW1000_OPT_PRELOADING, to_bool(val));

        case NETOPT_PROMISCUOUSMODE:
            return dw1000_set_option(dev, DW1000_OPT_PROMISCUOUS, to_bool(val));

        case NETOPT_RX_START_IRQ:
            return dw1000_set_option(dev, DW1000_OPT_TELL_RX_START, to_bool(val));

        case NETOPT_RX_END_IRQ:
            return dw1000_set_option(dev, DW1000_OPT_TELL_RX_END, to_bool(val));

        case NETOPT_TX_START_IRQ:
            return dw1000_set_option(dev, DW1000_OPT_TELL_TX_START, to_bool(val));

        case NETOPT_TX_END_IRQ:
            return dw1000_set_option(dev, DW1000_OPT_TELL_TX_END, to_bool(val));

        default:
            return ext;
    }
}
