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
 * @brief       Getter and setter functions for the dw1000 driver
 *
 * @author      Hessam Mohammadmoradi <hmoradi@cs.uh.edu>
 *
 * @}
 */

#include <string.h>
#include <errno.h>

#include "dw1000.h"
#include "dw1000_internal.h"
#include "dw1000_registers.h"
#include "periph/spi.h"

#define ENABLE_DEBUG    (0)
#include "debug.h"


void dw1000_get_addr_short(dw1000_t *dev, uint8_t *addr)
{
    uint16 temp;
    temp = dwt_read16bitoffsetreg(PANADR_ID, 0);
    *addr = temp;
}

void dw1000_set_addr_short(dw1000_t *dev, uint8_t *addr)
{
    uint8_t tmp[2];
    tmp[0] = addr[1];
    tmp[1] = addr[0];

    memcpy(dev->netdev.short_addr, addr, 2);

#ifdef MODULE_SIXLOWPAN
    /* https://tools.ietf.org/html/rfc4944#section-12 requires the first bit to
     * 0 for unicast addresses */
    dev->netdev.short_addr[0] &= 0x7F;
#endif
    dwt_setaddress16((uint16)(*tmp));
}

void dw1000_get_addr_long(dw1000_t *dev, uint8_t *addr)
{
    dwt_geteui(addr);
}

void dw1000_set_addr_long(dw1000_t *dev, uint8_t *addr)
{
    int i, j;
    uint8_t tmp[8];

    for (i = 0, j = 7; i < 8; i++, j--) {
        dev->netdev.long_addr[i] = addr[i];
        tmp[j] = addr[i];
    }
    dwt_seteui(tmp);
}

uint16_t dw1000_get_pan(dw1000_t *dev)
{
    return dwt_read16bitoffsetreg(PANADR_ID, 0);
}

void dw1000_set_pan(dw1000_t *dev, uint16_t pan)
{
    dev->netdev.pan = pan;
    dwt_setpanid(pan);
}

uint16_t dw1000_get_chan(dw1000_t *dev)
{
    puts("dw1000 get set: get chan is called the return value is wrong;\r\n");
    return 0;
}

int dw1000_set_chan(dw1000_t *dev, uint16_t chan)
{
    if ((chan < DW1000_CHAN_MIN) || (chan > DW1000_CHAN_MAX)) {
        puts("dw1000: set channel: given channel invalid\n");
        return -ENOTSUP;
    }
    dev->netdev.chan = chan;
    //todo chan needs to be written it to device !
    return DW1000_RET_CHAN_OK;

}

int16_t dw1000_get_txpower(dw1000_t *dev)
{
    DEBUG("dw1000 get set: get tx power is called the return value is wrong;\r\n");
    //not used
    return 0;
}

void dw1000_set_txpower(dw1000_t *dev, int16_t txpower)
{
//    if (txpower > DW1000_TXPOWER_MAX) {
//        txpower = DW1000_TXPOWER_MAX;
//    }
//    else if (txpower < DW10000_TXPOWER_MIN) {
//        txpower = DW1000_TXPOWER_MIN;
//    }
    //txctrl |= power_dbm_to_pa[txpower + 25];
    DEBUG("dw1000 get set: set tx power is called \r\n");
    dwt_write32bitreg(0x1E, txpower);
}

int dw1000_set_option(dw1000_t *dev, uint16_t option, bool state)
{


    /* set option field */
    if (state) {
        dev->options |= option;
        /* trigger option specific actions */
        switch (option) {
            case DW1000_OPT_AUTOACK:
                DEBUG("dw1000: set_opt: DW1000_OPT_AUTOACK (it does not workd without frame filtering being enabled \r\n");
                dwt_enableautoack(10); //give enough time to sender to go to receive mode (us)
                break;

            case DW1000_OPT_CSMA:
                DEBUG("dw1000: set_opt: DW1000_OPT_CSMA\r\n");
                /* TODO: There is CSMA checking in UWB !*/
                break;

            case DW1000_OPT_PROMISCUOUS:
                DEBUG("dw1000: set_opt: DW1000_OPT_PROMISCUOUS\r\n");
                dwt_enableframefilter(DWT_FF_NOTYPE_EN);
                break;

            case DW1000_OPT_PRELOADING:
                DEBUG("dw1000: set_opt: DW1000_OPT_PRELOADING\r\n");
                break;

            case DW1000_OPT_TELL_TX_START:
                DEBUG("dw1000: set_opt: DW1000_OPT_TELL_TX_START\r\n");
                //dwt_starttx(DWT_START_TX_IMMEDIATE);
                break;
            case DW1000_OPT_TELL_TX_END:
                DEBUG("dw1000: set_opt: DW1000_OPT_TELL_TX_END\r\n");
               // dwt_forcetrxoff();
                break;
            case DW1000_OPT_TELL_RX_START:
                DEBUG("dw1000: set_opt: DW1000_OPT_TELL_RX_START\r\n");
//                dwt_setrxtimeout(0); //reconfigure the timeout
//                dwt_setpreambledetecttimeout(0);
//                dwt_rxenable(DWT_START_RX_IMMEDIATE) ;
                break;
            case DW1000_OPT_TELL_RX_END:
                DEBUG("dw1000: set_opt: TX/RX START/END\r\n");
                //dwt_forcetrxoff();
                break;

            default:
                return -ENOTSUP;
        }
    }
    else {
        dev->options &= ~(option);
        /* trigger option specific actions */
        switch (option) {
            case DW1000_OPT_AUTOACK:
                DEBUG("dw1000: clr_opt: DW1000_OPT_AUTOACK\r\n");
                dwt_disableautoack();
                break;

            case DW1000_OPT_CSMA:
                DEBUG("DW1000: clr_opt: DW1000_OPT_CSMA\r\n");
                /* TODO: en/disable csma there is no CSMA in UWB */
                break;

            case DW1000_OPT_PROMISCUOUS:
                DEBUG("dw1000: clr_opt: DW1000_OPT_PROMISCUOUS\r\n");
                dwt_enableframefilter(DWT_FF_NOTYPE_EN); //ToDO actully we need to enable frame filtering
                break;

            case DW1000_OPT_PRELOADING:
                DEBUG("dw1000: clr_opt: DW1000_OPT_PRELOADING\r\n");
                break;

            case DW1000_OPT_TELL_TX_START:
                DEBUG("dw1000: clr_opt: clear DW1000_OPT_TELL_TX_START]\r\n");
                //dwt_starttx(DWT_START_TX_IMMEDIATE);//TODO immedaite send by default ?
                break;
            case DW1000_OPT_TELL_TX_END:
                DEBUG("dw1000: clr_opt: DW1000_OPT_TELL_TX_END\r\n");
                //dwt_forcetrxoff();
                break;
            case DW1000_OPT_TELL_RX_START:
                DEBUG("dw1000: clr_opt: DW1000_OPT_TELL_RX_START\r\n");
                break;
            case DW1000_OPT_TELL_RX_END:
                DEBUG("dw1000: clr_opt: TX/RX START/END\r\n");
                //dwt_forcetrxoff();
                break;

            default:
                return -ENOTSUP;
        }
    }
    return sizeof(netopt_enable_t);
}

int dw1000_set_state(dw1000_t *dev, netopt_state_t cmd)
{
//    if ((dw1000_get_state(dev) == NETOPT_STATE_OFF) &&
//        (cmd != NETOPT_STATE_OFF)) {
//        cc2420_en_xosc(dev);
//    }
    switch (cmd) {
        case NETOPT_STATE_OFF:
            //TODO dw1000 has deep sleep functionality but needs extra care
            //puts("dw1000: set_state: NETOPT_STATE_OFF\r\n");
            break;
        case NETOPT_STATE_SLEEP:
            //TODO dw1000 has sleep functionality but needs extra care
           // puts("dw1000: set_state: NETOPT_STATE_SLEEP\r\n");
            break;
        case NETOPT_STATE_IDLE:
           // puts("dw1000: set_state: NETOPT_STATE_IDLE\r\n");
            //dwt_forcetrxoff();
            break;
        case NETOPT_STATE_TX:
           // puts("dw1000: set_state: NETOPT_STATE_TX\r\n");
            dwt_setrxaftertxdelay(1) ;
            dwt_starttx(DWT_RESPONSE_EXPECTED); //go back to rec mode after sending
            //dwt_starttx(DWT_RESPONSE_EXPECTED);
            break;
        case NETOPT_STATE_RESET:
          //  puts("dw1000: set_state: NETOPT_STATE_RESET\r\n");
            dw1000_init(dev);
            break;
        case NETOPT_STATE_RX:
          //  puts("dw1000: set_state: NETOPT_STATE_RX\r\n");
            dwt_setrxtimeout(0); //reconfigure the timeout
            dwt_setpreambledetecttimeout(0);
            //dwt_setrxaftertxdelay(1) ;
            //dwt_starttx(DWT_RESPONSE_EXPECTED); //go back to rec mode after sending
            dwt_rxenable(DWT_START_RX_IMMEDIATE) ;
            DEBUG("dw1000: set_state: NETOPT_STATE_RX \r\n");
            break;
        default:
          //  puts("dw1000: set_state: called with invalid target state\r\n");
            return -ENOTSUP;
    }
    return sizeof(netopt_state_t);
}

netopt_state_t dw1000_get_state(dw1000_t *dev)
{
    DEBUG("dw1000 get state is called the return value is always idle \r\n");
    return NETOPT_STATE_IDLE; //TODO it is not true
//    uint8_t cur_state = dw1000_state(dev);
//
//    if (cur_state == 0) {
//        return NETOPT_STATE_OFF;
//    }
//    else if (cur_state == 1) {
//        return NETOPT_STATE_SLEEP;
//    }
//    else if (((cur_state >= 32) && (cur_state <=39)) || (cur_state == 56)) {
//        return NETOPT_STATE_TX;
//    }
//    else if ((cur_state >= 3) && (cur_state <= 6)) {
//        return NETOPT_STATE_IDLE;
//    }
//    else {
//        return NETOPT_STATE_RX;
//    }
}
