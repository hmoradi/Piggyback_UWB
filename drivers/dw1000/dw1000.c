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

#include <inttypes.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "fmt.h"
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
    printf("rx callback %d \r\n",rxd->event);
    if(rxd->event == DWT_SIG_RX_OKAY)
    {    
        uint8_t ts[5];
        uint8_t tmp[200];
        dwt_readrxdata(tmp, 126, 0);
        uint64_t Reply = 0;
        uint64_t Delay = 0;
        uint8_t src_addr[2] = {0x22,0x22};
        dwt_readrxtimestamp(ts);
        uint64_t curr_ts = dw1000_convert_ts_to_int(ts);
        dw1000_extract_ranging_info((dw1000_t*)dev,tmp,0,src_addr,&Reply,&Delay);
        dw1000_calc_dist((dw1000_t*)dev,src_addr,Reply,Delay,curr_ts);
        dw1000_update_ranging_info((dw1000_t*)dev,src_addr,FALSE,curr_ts);

        if(dev->event_callback) {
            DEBUG("rx call back  calling netdev callback with rx complete \r\n");

            dev->event_callback(dev, NETDEV_EVENT_RX_COMPLETE);
        }
    }

    dw1000_set_state((dw1000_t *)dev, NETOPT_STATE_RX);
}
#pragma GCC optimize ("O3")
void dw1000_txcallback(netdev_t *dev,const dwt_callback_data_t *txd)
{
    
    uint8 txevent = txd->event;
    printf("TX done %d \r\n",txevent);
    if(txevent == DWT_SIG_TX_DONE)
    {
        //printf("TX done \r\n");
        //puts("dw1000 tx callback TX done \r\n");
        //dw1000_set_state(dev, NETOPT_STATE_RX);
    }
    else if(txevent == DWT_SIG_TX_AA_DONE)
    {
        DEBUG("tx callback tx auto ack sent !!!!!!!!!!!!!11 \r\n");
    }
    dw1000_set_state((dw1000_t *)dev, NETOPT_STATE_RX);

    if(dev->event_callback) {
        DEBUG("tx call back  calling netdev callback with tx complete \r\n");

        dev->event_callback(dev, NETDEV_EVENT_TX_COMPLETE);
    }
    
}

//ported
void dw1000_setup(dw1000_t * dev, const dw1000_params_t *params)
{
    uint8_t dummy_addr[2] = {0x11,0x11};
    for(int i=0;i<Num_Elements;i++){
        memcpy(dev->ranging_info_array[i].short_addr,dummy_addr,2);
    }
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
    //result = dwt_initialise(dev,DWT_LOADUCODE | DWT_LOADLDOTUNE | DWT_LOADTXCONFIG | DWT_LOADANTDLY| DWT_LOADXTALTRIM) ;
    result = dwt_initialise(dev,DWT_LOADUCODE) ;
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
    //TDOD: get from params file 
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
    //dwt_configure(&configData, DWT_LOADXTALTRIM) ;
    dwt_configure(&configData,0) ;

    // disable antenna delay deduction.
    dwt_setrxantennadelay(0);
    dwt_settxantennadelay(0);
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
    uint8_t dest_addr[2] = {0x22,0x22};
    uint8_t src_addr[2] = {0x22,0x22};
    //insert ranging info
    uint32_t tx_timestamp = dwt_readsystimestamphi32();
    tx_timestamp += DELAY_TX_5;
    uint64_t tx_timestamp_64 = tx_timestamp;
    tx_timestamp_64 = tx_timestamp_64 << 8;
    offset = dw1000_insert_ranging_info(dev,tmp,offset,src_addr,dest_addr,tx_timestamp_64);
    dw1000_update_ranging_info(dev,dest_addr,TRUE,tx_timestamp_64);
    //

    
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
    
    dwt_forcetrxoff();
    dwt_writetxdata(offset+2, tmp, 0) ;
    dwt_writetxfctrl(offset+2, 0);
    
    //dwt_setrxaftertxdelay(1) ;
    //dwt_starttx(0); 

    uint8_t tx_mode = DWT_START_TX_DELAYED | DWT_RESPONSE_EXPECTED;
    dwt_setdelayedtrxtime(tx_timestamp);
    dwt_starttx(tx_mode);

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
        uint8_t tmp[200];
        dwt_readrxdata(tmp, max_len, 0);
        memcpy(buf,&tmp[11],max_len-11);
        
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

int dw1000_find_ranging_info(dw1000_t *dev,uint8_t * short_addr){
    
    for (int i=0;i<Num_Elements;i++){
        if(dev->ranging_info_array[i].short_addr[0] == short_addr[0] ){
            return i;
        }
    }
    return -1;
}

void dw1000_calc_dist(dw1000_t *dev,uint8_t * short_addr,uint64_t Reply_a,uint64_t Delay_a,uint64_t curr_ts){
    int index  = dw1000_find_ranging_info(dev,short_addr);
    if(index < 0)
        return;
    
    if(dev->ranging_info_array[index].tx_ts_valid == FALSE || dev->ranging_info_array[index].rx_ts_valid == FALSE){
        printf("not valid local time stamps \r\n");
        return;
    }
    
    if(Reply_a == 0 || Delay_a == 0){
        printf("not valid remote time stamps \r\n");
        return;
    }

    uint64_t Reply_b = (curr_ts - dev->ranging_info_array[index].last_tx_ts+ UINT40_MAX) % UINT40_MAX;
    uint64_t Delay_b = (dev->ranging_info_array[index].last_tx_ts - dev->ranging_info_array[index].last_rx_ts + UINT40_MAX)%UINT40_MAX ;
    uint64_t ToF = (Reply_a*Reply_b - Delay_b*Delay_a) / (Reply_a + Reply_b + Delay_a + Delay_b);
    
    //dev->ranging_info_array[index].tx_ts_valid = FALSE;
    //dev->ranging_info_array[index].rx_ts_valid = FALSE;
    
    char out[17] = "----------------";
    fmt_u64_hex(out, ToF);
    printf("measured ToF low: %lu\r\n",(uint32_t)ToF);
    printf("ToF hex %s \r\n",out);
    char Reply_a_s[17] = "----------------";
    fmt_u64_hex(Reply_a_s, Reply_a);
    printf("Reply_a hex %s \r\n",Reply_a_s);
    char Reply_b_s[17] = "----------------";
    fmt_u64_hex(Reply_b_s, Reply_b);
    printf("Reply_b hex %s \r\n",Reply_b_s);
    char Delay_a_s[17] = "----------------";
    fmt_u64_hex(Delay_a_s, Delay_a);
    printf("Delay_a hex %s \r\n",Delay_a_s);
    char Delay_b_s[17] = "----------------";
    fmt_u64_hex(Delay_b_s, Delay_b);
    printf("Delay_b hex %s \r\n",Delay_b_s);
    

    //TODO fix it later does not work for now
    float timeunit = (1.0/499.2e6/128.0);
    timeunit = timeunit * SPEED_OF_LIGHT;
    float distance = (uint32_t)ToF * timeunit;
    //printf("measured distance: %lf, %lf !!!!!!!!!!!!!!!!!!!\r\n",timeunit, distance);
    
    char buf[19];
    fmt_float(buf, distance, 3);
    printf("measured dist is: %s \r\n",buf);

}
uint64_t dw1000_convert_ts_to_int(uint8_t* ts){
    uint64_t _ts = 0;
    int j;
    for (j = 4 ; j >= 0 ; j --)
    {
            _ts = (_ts << 8) + ts[j] ;        // sum
    }
    return _ts;
}
void dw1000_update_ranging_info(dw1000_t *dev,uint8_t * short_addr,bool TX,uint64_t ts){
    
    //printf("dest addr %x \r\n",short_addr[0]);
    int index = dw1000_find_ranging_info(dev,short_addr);
    printf("update ranging info at index %d \r\n",index);
    if(index >= 0){
        if (TX){
            dev->ranging_info_array[index].tx_ts_valid = TRUE;
            dev->ranging_info_array[index].last_tx_ts = ts;
        }else{
            dev->ranging_info_array[index].rx_ts_valid = TRUE;
            dev->ranging_info_array[index].last_rx_ts = ts;           
        }
    }else{ //this is the first time communicating with this node
        uint8_t dummy_addr[2] = {0x11,0x11};
        index = dw1000_find_ranging_info(dev,dummy_addr);
        
        memcpy(dev->ranging_info_array[index].short_addr , short_addr,2);
        dev->ranging_info_array[index].rx_ts_valid = FALSE;
        dev->ranging_info_array[index].tx_ts_valid = FALSE;
        if(TX){
            dev->ranging_info_array[index].last_tx_ts = ts; 
            dev->ranging_info_array[index].tx_ts_valid = TRUE;
        }else{
            dev->ranging_info_array[index].last_rx_ts = ts;
            dev->ranging_info_array[index].rx_ts_valid = TRUE;
        }
    }
}

int dw1000_insert_ranging_info(dw1000_t* dev,uint8_t* buffer,int offset,uint8_t* src_addr,uint8_t* dest_addr,uint64_t curr_ts){
    
    int index  = dw1000_find_ranging_info(dev,dest_addr);
    //printf("insert ranging info from %x to %x at index %d \r\n",src_addr[0],dest_addr[0],index);
    uint64_t Reply = 0;
    uint64_t Delay = 0;
    if(dev->ranging_info_array[index].tx_ts_valid && dev->ranging_info_array[index].rx_ts_valid){
        Reply = (dev->ranging_info_array[index].last_rx_ts - dev->ranging_info_array[index].last_tx_ts+UINT40_MAX)%UINT40_MAX;
        Delay = (curr_ts - dev->ranging_info_array[index].last_rx_ts+UINT40_MAX)%UINT40_MAX;    
    }
    //dev->ranging_info_array[index].tx_ts_valid = FALSE;
    //dev->ranging_info_array[index].rx_ts_valid = FALSE;

    char out1[17] = "----------------";
    char out2[17] = "----------------";
    fmt_u64_hex(out1, Reply);
    fmt_u64_hex(out2, Delay);
    printf("Inserted Reply time:  %s and Delay time %s \r\n",out1,out2);
    
    buffer[offset] = src_addr[0];
    offset += 1;

    buffer[offset] =     (Reply >> 32) & 0xFF;
    buffer[offset + 1] = (Reply >> 24) & 0xFF;
    buffer[offset + 2] = (Reply >> 16) & 0xFF;
    buffer[offset + 3] = (Reply >> 8) & 0xFF;
    buffer[offset + 4] = Reply & 0xFF;
    offset += 5;
    
    buffer[offset] =     (Delay >> 32) & 0xFF;
    buffer[offset + 1] = (Delay >> 24) & 0xFF;
    buffer[offset + 2] = (Delay >> 16) & 0xFF;
    buffer[offset + 3] = (Delay >> 8) & 0xFF;
    buffer[offset + 4] = Delay & 0xFF;
    return offset+5;
}

void dw1000_extract_ranging_info(dw1000_t* dev,uint8_t* buffer,int offset,uint8_t* short_addr,uint64_t* Reply,uint64_t*Delay){
    //puts("extract ranging info \r\n");
    uint64_t _Reply = 0;
    uint64_t _Delay = 0;
    short_addr[0] = buffer[offset];
    offset += 1;

    int j;
    for (j = 0 ; j <= 4 ; j ++)
    {
            _Reply = (_Reply << 8) + buffer[offset+j] ;        // sum
    }
    
    offset += 5;
    for (j = 0 ; j <= 4 ; j ++)
    {
            _Delay = (_Delay << 8) + buffer[offset+j] ;        // sum
    }
    *Delay = _Delay;
    *Reply = _Reply;
    
    char out1[17] = "----------------";
    char out2[17] = "----------------";
    fmt_u64_hex(out1, _Reply);
    fmt_u64_hex(out2, _Delay);
    printf("Extracted Reply time:  %s and Delay time %s \r\n",out1,out2);
    return;
}
