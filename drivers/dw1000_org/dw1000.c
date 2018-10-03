/*
 * Copyright (C) 2015-2016 Freie Universität Berlin
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
 * @brief       Implementation of public functions for dw1000 driver
 *
 * @author      Hessam Mohammadmoradi <hmoradi@cs.uh.edu>
 *
 * @}
 */

#include "luid.h"
#include "byteorder.h"
#include "net/ieee802154.h"
#include "net/gnrc.h"

#include "dw1000_internal.h"
#include "dw1000_netdev.h"
#include "dw1000_registers.h"
#include "od.h"
#define ENABLE_DEBUG (0)
#include "debug.h"
#include "od.h"
#pragma GCC optimize ("O3")
//static uint8 CIR[1025] = {0};
void dw1000_rxcallback(netdev_t *dev,const dwt_callback_data_t *rxd)
{
    //if we got a frame with a good CRC - RX OK
    if(rxd->event == DWT_SIG_RX_OKAY)
    {
        //dw_event.rxLength = rxd->datalength;

        //need to process the frame control bytes to figure out what type of frame we have received
        if(((rxd->fctrl[0] == 0x41) || (rxd->fctrl[0] == 0x61))
           &&
           ((rxd->fctrl[1] & 0xCC) == 0x88)) //short address
        {

//            fcode_index = FRAME_CRTL_AND_ADDRESS_S; //function code is in first byte after source address
//            srcAddr_index = FRAME_CTRLP + ADDR_BYTE_SIZE_S;
            //rxd_event = DWT_SIG_RX_OKAY;
        }
        else
        {
            //rxd_event = SIG_RX_UNKNOWN; //not supported - all TREK1000 frames are short addressed
        }

        //read RX timestamp
//        dwt_readrxtimestamp(rxTimeStamp) ;
//        dwt_readrxdata((uint8 *)&dw_event.msgu.frame[0], rxd->datalength, 0);  // Read Data Frame
//        dw_event.timeStamp32l =  (uint32)rxTimeStamp[0] + ((uint32)rxTimeStamp[1] << 8) + ((uint32)rxTimeStamp[2] << 16) + ((uint32)rxTimeStamp[3] << 24);
//        dw_event.timeStamp = rxTimeStamp[4];
//        dw_event.timeStamp <<= 32;
//        dw_event.timeStamp += dw_event.timeStamp32l;
//        dw_event.timeStamp32h = ((uint32)rxTimeStamp[4] << 24) + (dw_event.timeStamp32l >> 8);
//
//        dw_event.type = 0; //type will be added as part of adding to event queue
//        dw_event.type_save = rxd_event;
//        dw_event.type_pend = DWT_SIG_DW_IDLE;


    }
//    puts("received a packet printing CIR \r\n");
//    dwt_rxdiag_t log;
//    dwt_readdignostics(&log);
//    printf("maxnoise %u \r\n",log.maxNoise);
//    printf("firstPathAmp1 %u \r\n",log.firstPathAmp1);
//    printf("stdNoise %u \r\n",log.stdNoise);
//    printf("firstPathAmp2 %u \r\n",log.firstPathAmp2);
//    printf("firstPathAmp3 %u \r\n",log.firstPathAmp3);
//    printf("maxGrowthCIR %u \r\n",log.maxGrowthCIR);
//    printf("rxPreamCount %u \r\n",log.rxPreamCount);
//    printf("firstPath %d \r\n",(int)log.firstPath);
//
//    int index = 0;
//    int i;
//    while(index<4064){
//        for (i = 0; i <= 1024; i++) {
//            CIR[i]= 0;
//        }
//        dwt_readaccdata(CIR, 1024, index);
//        od_hex_dump(CIR,1024,OD_WIDTH_DEFAULT);
//        index += 1024;
//    }

    if(dev->event_callback) {
        DEBUG("rx call back  calling netdev callback with rx complete \r\n");

        dev->event_callback(dev, NETDEV_EVENT_RX_COMPLETE);
    }

}
#pragma GCC optimize ("O3")
void dw1000_txcallback(netdev_t *dev,const dwt_callback_data_t *txd)
{
    uint8 txevent = txd->event;
    if(txevent == DWT_SIG_TX_DONE)
    {
        //puts("dw1000 tx callback TX done \r\n");
        //dw1000_set_state(dev, NETOPT_STATE_RX);
    }
    else if(txevent == DWT_SIG_TX_AA_DONE)
    {
        DEBUG("tx callback tx auto ack sent !!!!!!!!!!!!!11 \r\n");
    }
    dw1000_set_state((dw1000_t *)dev, NETOPT_STATE_RX);
//    if(dev->event_callback) {
//        DEBUG("tx callback calling netdev callback   \r\n");
//        dev->event_callback(dev, NETDEV_EVENT_ISR);
//    }
    //puts("dw1000 tx callback \r\n");
    if(dev->event_callback) {
        DEBUG("tx call back  calling netdev callback with tx complete \r\n");

        dev->event_callback(dev, NETDEV_EVENT_TX_COMPLETE);
    }
}

//ported
void dw1000_setup(dw1000_t * dev, const dw1000_params_t *params)
{
    /* set pointer to the devices netdev functions */
    dev->netdev.netdev.driver = &dw1000_driver;
    /* pull in device configuration parameters */
    memcpy(&dev->params, params, sizeof(dw1000_params_t));
    //dev->state = DW1000_IDLE_MODE;
    /* reset device descriptor fields */
    dev->options = 0;
}
//ported
int dw1000_init(dw1000_t *dev)
{
    /* reset options and sequence number */
    set_SPI_pins(dev);
    int result;
    DEBUG("dw1000 soft reset \r\n");
    dwt_softreset();
    DEBUG("dw initizlie  \r\n");
    //this initialises DW1000 and uses specified configurations from OTP/ROM
    result = dwt_initialise(dev,DWT_LOADUCODE | DWT_LOADLDOTUNE | DWT_LOADTXCONFIG | DWT_LOADANTDLY| DWT_LOADXTALTRIM) ;

    if (DWT_SUCCESS != result)
    {
        puts("dw1000 initialise has failed \r\n");
        return (-1) ;   // device initialise has failed
    }
    int devID = dwt_readdevid();
    DEBUG("dw1000 device id %x \r\n",devID);
    if(DWT_DEVICE_ID != devID) //if the read of device ID fails, the DW1000 could be asleep
    {
        port_SPIx_clear_chip_select();  //CS low
        Sleep(1);   //200 us to wake up then waits 5ms for DW1000 XTAL to stabilise
        port_SPIx_set_chip_select();  //CS high
        Sleep(7);
        devID = dwt_readdevid();
        // SPI not working or Unsupported Device ID
        if (DWT_DEVICE_ID != devID) {
            puts("device id does not match SPI has problem \r\n");
            return (-1);
        }
        //clear the sleep bit - so that after the hard reset below the DW does not go into sleep
        dwt_softreset();
    }

    dwt_setautorxreenable(0); //disable auto RX re-enable
    dwt_setdblrxbuffmode(0); //disable double RX buffer
    dwt_enableframefilter(DWT_FF_NOTYPE_EN);
    // if using auto CRC check (DWT_INT_RFCG and DWT_INT_RFCE) are used instead of DWT_INT_RDFR flag
    // other errors which need to be checked (as they disable receiver) are
    //dwt_setinterrupt(DWT_INT_TFRS | DWT_INT_RFCG | (DWT_INT_SFDT | DWT_INT_RFTO /*| DWT_INT_RXPTO*/), 1);
    dwt_setinterrupt(DWT_INT_TFRS | DWT_INT_RFCG | (DWT_INT_ARFE | DWT_INT_RFSL | DWT_INT_SFDT | DWT_INT_RPHE | DWT_INT_RFCE | DWT_INT_RFTO | DWT_INT_RXPTO), 1);

    dwt_setcallbacks(dw1000_txcallback, dw1000_rxcallback);
    ///config the channel
    dwt_config_t configData;
    configData.chan =2 ;
    configData.rxCode =  4;
    configData.txCode = 4 ;
    configData.prf = DWT_PRF_16M;
   // configData.dataRate = DWT_BR_110K;
    configData.dataRate = DWT_BR_6M8;
    //configData.txPreambLength = DWT_PLEN_1024 ;
    configData.txPreambLength = DWT_PLEN_128;
   // configData.rxPAC = DWT_PAC32 ;
    configData.rxPAC = DWT_PAC8;
    configData.nsSFD = 0 ;
    configData.phrMode = DWT_PHRMODE_STD ;
    //configData.phrMode = DWT_PHRMODE_EXT ;
    configData.sfdTO = (1025 + 64 - 32);
    //configData.sfdTO = (129 + 8 - 8);
    dwt_configure(&configData, DWT_LOADXTALTRIM) ;

    ///
    //uint16_t reg;
    uint8_t addr[8];

    /* reset options and sequence number */
    dev->netdev.seq = 0;
    dev->netdev.flags = 0;

    /* set default address, channel, PAN ID, and TX power */
    luid_get(addr, sizeof(addr));
    /* make sure we mark the address as non-multicast and not globally unique */
    addr[0] &= ~(0x01);
    addr[0] |= 0x02;
    dw1000_set_addr_short(dev, &addr[6]);
    dw1000_set_addr_long(dev, addr);
    dw1000_set_pan(dev, DW1000_PANID_DEFAULT);
    dw1000_set_chan(dev, DW1000_CHAN_DEFAULT);
    dw1000_set_txpower(dev, DW1000_TXPOWER_DEFAULT);

    /* set default options */
    //dw1000_set_option(dev, DW1000_OPT_AUTOACK, true);
    //dw1000_set_option(dev, DW1000_OPT_CSMA, true);
    //dw1000_set_option(dev, DW1000_OPT_TELL_RX_END, true);
    //dw1000_set_option(dev, DW1000_OPT_TELL_RX_START, false);
    //dw1000_set_option(dev, DW1000_OPT_TELL_TX_END, true);
    //dw1000_set_option(dev, DW1000_OPT_TELL_TX_START, false);

#ifdef MODULE_NETSTATS_L2
    dw1000_set_option(dev, DW1000_OPT_TELL_RX_END, true);
#endif
    /* set default protocol*/
#ifdef MODULE_GNRC_SIXLOWPAN
    dev->netdev.proto = GNRC_NETTYPE_SIXLOWPAN;
#elif MODULE_GNRC
    dev->netdev.proto = GNRC_NETTYPE_UNDEF;
#endif
    /* go into RX state */
    dw1000_set_state(dev, NETOPT_STATE_RX);
    ////
    puts("dw1000 init done! \r\n");
    return 0;
}

//ported
size_t dw1000_send(dw1000_t *dev, const struct iovec *data, unsigned count)
{
    size_t pkt_len = 0;
    /* get and check the length of the packet */
    for (unsigned i = 0; i < count; i++) {
        pkt_len += data[i].iov_len;
    }
    uint8 * tmp;
    tmp = (uint8*) malloc(pkt_len);
    if (tmp == 0){
        puts("dw1000 send can not allocate memory \r\n");
        return -1;
    }

    unsigned offset = 0;
    for (int i = 0; i < count; i++) {
        memcpy(&tmp[offset], data[i].iov_base, data[i].iov_len);
        offset += data[i].iov_len;
        DEBUG("iov len is %d \r\n",data[i].iov_len);
        if (offset > 1024) {
            puts("dw1000 send is called with size \r\n");
            free(tmp);
            return -1;
        }
    }
    DEBUG("dw1000 send is called with size %d\r\n",offset);
    //od_hex_dump(tmp, offset, OD_WIDTH_DEFAULT);
    //DEBUG("==================================\r\n");
    //if(offset > 120)
    //    offset = 120;
    dwt_forcetrxoff();
    dwt_writetxdata(offset+2, tmp, 0) ;
    dwt_writetxfctrl(offset+2, 0);
    //dwt_setrxaftertxdelay(1) ;
    dwt_starttx(0); //go back to rec mode after sending
    free(tmp);
    DEBUG("dw1000 send is issued  \r\n");
    return offset;
}

//ported
size_t dw1000_tx_prepare(dw1000_t *dev, const struct iovec *data, unsigned count)
{

    dwt_writetxdata(count, (uint8_t *)data[0].iov_base, 0) ;
    dwt_writetxfctrl(count, 0);
    return count;
}

//ported
void dw1000_tx_exec(dw1000_t *dev)
{
    dwt_starttx(DWT_START_TX_IMMEDIATE);//todo mode need to be set for now is immediate send
}
//ported
int dw1000_rx(dw1000_t *dev, uint8_t *buf, size_t max_len, void *info)
{

    //DEBUG("dw1000 rx (read from rx buffer) is called \r\n");
    if (info != NULL) {
        netdev_ieee802154_rx_info_t *radio_info = info;
        radio_info->rssi = 65; //ToDO: fix this later
    }
    if (buf == NULL) {
        return rx_read_len()-2;
    }
    else {
        //puts("read the real data from receive buffer %d \r\n");
        dwt_readrxdata(buf, max_len, 0);
        //DEBUG("==================================\r\n");
        //od_hex_dump(buf, max_len, OD_WIDTH_DEFAULT);
        //DEBUG("==================================\r\n");
    }

    //DEBUG("after reading the buffer set device to rx mode \r\n");
    dw1000_set_state(dev, NETOPT_STATE_RX);//TODO how about sending the autoack
    return max_len;
}

int dw1000_startrx(uint16 time){
    puts("dw1000 start rx \r\n");
    dwt_setrxtimeout(0); //reconfigure the timeout
    dwt_setpreambledetecttimeout(0);
    dwt_rxenable(DWT_START_RX_IMMEDIATE) ;
    return 0;
}
void dw1000_stoprx(void){
    dwt_forcetrxoff();
}

