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
#include "random.h"
#include "thread.h"
#define ENABLE_DEBUG (0)
#include "debug.h"
#include "od.h"

#pragma GCC optimize ("O3")
#define ID 0x05
mutex_t mode_mtx = MUTEX_INIT;
mutex_t auto_reply_mtx = MUTEX_INIT;

static uint8_t _mode = 1;  //0 active ranging , 1 passive ranging
char active_range_stack[THREAD_STACKSIZE_MAIN];
char sched_stack[THREAD_STACKSIZE_MAIN];
#define TIMEOUT         (5000UL * US_PER_MS)
#define SCHED_TIMEOUT         (10000UL * US_PER_MS)
#define RANGE_RATE 4  //4 packets per 10 seconds
static bool auto_reply = FALSE;
#define queue_len 10
static dw1000_queue_item queue[10];
static uint8_t write_index = 0;
static uint8_t read_index = 0;
static kernel_pid_t sched_pid;
static kernel_pid_t act_rang_pid;
static int data_packets = 0;
static int range_packets = 0;
static void time_evt(void *arg)
{
    thread_flags_set((thread_t *)arg, 0x1);
}
void* active_ranging(void* arg){
    printf("active ranging thread is started \r\n");
    xtimer_t timer;
    timer.callback = time_evt;
    timer.arg = (void *)sched_active_thread;
    dw1000_t* dev = (dw1000_t*)arg;
    //uint8_t state = 0; //0 send poll, 1 
    while(TRUE){
        mutex_lock(&mode_mtx);
        if(_mode == 1){
            mutex_unlock(&mode_mtx);
            break;

        }else{
            mutex_unlock(&mode_mtx);
            printf("sending in active ranging \r\n");
            mutex_lock(&auto_reply_mtx);
            auto_reply = TRUE;
            mutex_unlock(&auto_reply_mtx);
            dw1000_send(dev,NULL,0);
            xtimer_set(&timer, TIMEOUT);
            thread_flags_wait_any(0x1);
        } 
    }
    printf("active ranging thread is exiting \r\n");
    return NULL;
}
void* scheduler(void *arg){
    //(void) arg;
    printf("scheduler thread  starting \r\n");
    xtimer_t timer;
    timer.callback = time_evt;
    timer.arg = (void *)sched_active_thread;
    
    dw1000_t* dev = arg; 
    while(TRUE){
        mutex_lock(&mode_mtx);
        dev->datarate = data_packets;
        //dev->rangrate = range_packets / SCHED_TIMEOUT;
        data_packets = 0;
        range_packets = 0;
        printf("in scheduler data rate is %d and range rate is %d \r\n",dev->datarate,dev->rangrate);
        if(dev->datarate >= dev->rangrate){ //passive ranging
            printf("going to passive mode \r\n");
            _mode = 1;
        }else{ //active ranging
            printf("going to active mode \r\n");
            if(_mode == 1){
               _mode = 0; 
               act_rang_pid = thread_create(active_range_stack,
                      sizeof(active_range_stack),
                      THREAD_PRIORITY_MAIN - 1,
                      THREAD_CREATE_STACKTEST,
                      active_ranging,
                      dev,
                      "active_ranging");
            }
        }
        mutex_unlock(&mode_mtx);
        xtimer_set(&timer, SCHED_TIMEOUT);
        thread_flags_wait_any(0x1);
    }
    
    //printf("thread pid %d\n",pid );
    printf("scheduler thread is exiting \r\n");
    return NULL;
}
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
        uint8_t last_tx_seq_nb = 0;
        uint8_t last_rx_seq_nb = 0;
        uint8_t last_seq_nb = 0;
        uint8_t src_addr[2];
        uint8_t ele = tmp[0] & 0x7F;
        src_addr[0] = tmp[1];
        src_addr[1] = tmp[2];
        bool _auto = FALSE;
        if(tmp[0] & 0x80)
            _auto = TRUE;
        

        dwt_readrxtimestamp(ts);
        uint64_t curr_ts = dw1000_convert_ts_to_int(ts);
        timex_t now;
        xtimer_now_timex(&now);
        uint64_t now_64 = now.seconds;
        now_64 = now_64 << 40;
        curr_ts |= now_64;
        dw1000_extract_ranging_info((dw1000_t*)dev,tmp,0,src_addr,&Reply,&Delay,&last_tx_seq_nb,&last_rx_seq_nb,&last_seq_nb);
        dw1000_calc_dist((dw1000_t*)dev,src_addr,Reply,Delay,curr_ts,last_rx_seq_nb,last_tx_seq_nb);
        dw1000_update_ranging_info((dw1000_t*)dev,src_addr,curr_ts,last_seq_nb,last_tx_seq_nb);
        if(ele*16+4 < rx_read_len()-2){
            printf("packet has extra payload ^^^^^^^^^^^^^^^^^^^^^\r\n");
            if(dev->event_callback) {
                DEBUG("rx call back  calling netdev callback with rx complete \r\n");

                dev->event_callback(dev, NETDEV_EVENT_RX_COMPLETE);
            }
            dw1000_set_state((dw1000_t *)dev, NETOPT_STATE_RX);
        }else{
            printf("just ranging packet (total len %d)do not send it up to network \r\n",rx_read_len()-2);
            if(_auto){
                mutex_lock(&auto_reply_mtx);
                auto_reply = FALSE;
                mutex_unlock(&auto_reply_mtx);
                printf("packet has auto reply request \r\n");
                dw1000_send((dw1000_t*)dev,NULL,0);
            }
        }
    }

    
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
    /* reset device descriptor fields */
    dev->options = 0;
    dev->seq_nb = 0;
    dev->datarate = 0;
    dev->rangrate = RANGE_RATE;
    timex_t now;
    xtimer_now_timex(&now);
    dev->last_sec = now.seconds;
}
//ported
int dw1000_init(dw1000_t *dev)
{
    /* reset options and sequence number */
    set_SPI_pins(dev);
    int result;
    DEBUG("dw1000 soft reset \r\n");
    dwt_softreset();
    DEBUG("dw initialize  \r\n");
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
    
    uint8_t device_addr[8];
    dw1000_get_addr_long(dev, device_addr); 
    memcpy(&addr[6],&device_addr[6],2);
    //addr[7] = (uint8_t)random_uint32();
    addr[7] = ID;
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
    sched_pid = thread_create(sched_stack,
                  sizeof(sched_stack),
                  THREAD_PRIORITY_MAIN - 1,
                  THREAD_CREATE_STACKTEST,
                  scheduler,
                  dev,
                  "scheduler");
    return 0;

}

//ported
size_t dw1000_send(dw1000_t *dev, const struct iovec *data, unsigned count)
{
    mutex_lock(&mode_mtx);
    uint8_t local_mode = _mode;
    mutex_unlock(&mode_mtx);
    timex_t now;
    xtimer_now_timex(&now);
    if((local_mode == 0) && (count > 0)){ //active ranging and not ranging packet, just store it
        int local_offset = 0;
        //printf("in radio send number of iovec structs are %d \r\n",count);
        for (int i = 0; i < count; i++) {
            //printf("and size of iobase is %d \r\n",data[i].iov_len);
            memcpy(&(queue[write_index].data[local_offset]), data[i].iov_base, data[i].iov_len);
            local_offset += data[i].iov_len;
        }
        printf("in radio send: copy the message to queue at index %d with size %d \r\n",write_index,local_offset);
        queue[write_index].seq_nb = dev->seq_nb;
        queue[write_index].datalen = local_offset;
        queue[write_index].time = now.seconds;
        write_index = (write_index + 1) % queue_len;
        dev->seq_nb ++;
        data_packets ++;
        return 0;    
    }
    
    size_t pkt_len = 0;
    uint8_t tmp[200]={0};
    
    for (unsigned i = 0; i < count; i++) {
        pkt_len += data[i].iov_len;
    }

    unsigned offset = 0;
    uint8_t dest_addr[2] = {0xFF,0xFF};
    uint8_t src_addr[2];
    dw1000_get_addr_short(dev,src_addr);
    
    //insert ranging info
    uint32_t tx_timestamp = dwt_readsystimestamphi32();
    tx_timestamp += DELAY_TX_10;
    uint64_t tx_timestamp_64 = tx_timestamp;
    tx_timestamp_64 = tx_timestamp_64 << 8;
    uint64_t sec_64 = now.seconds;
    tx_timestamp_64 |= sec_64 << 40;
    
    bool _auto = FALSE;
    mutex_lock(&auto_reply_mtx);
    _auto = auto_reply;
    mutex_unlock(&auto_reply_mtx);
    
    offset = dw1000_insert_ranging_info(dev,tmp,offset,src_addr,dest_addr,tx_timestamp_64,_auto);
    dev->last_tx_ts[dev->seq_nb] = tx_timestamp_64;
    
    //printf("local mode is %d \r\n",local_mode);
    if(local_mode == 1){ // passive ranging
        //printf("in passive ranging copy the real message to send buffer \r\n");
        for (int i = 0; i < count; i++) {
            memcpy(&tmp[offset], data[i].iov_base, data[i].iov_len);
            offset += data[i].iov_len;
            DEBUG("iov len is %d \r\n",data[i].iov_len);
            if (offset > 1024) {
                printf("dw1000 send is called with over size %d \r\n",offset);
                //free(tmp);
                return -1;
            }
        }
        data_packets ++;
    }else{ //active ranging
        if(read_index != write_index){
            printf(" in radio send read from queued messages from index %d \r\n",read_index);
            memcpy(&tmp[offset], queue[read_index].data, queue[read_index].datalen);
            offset += queue[read_index].datalen;
            read_index = (read_index + 1) % queue_len;
        }
    }
    printf("packet is ready to send with total length of %d  \r\n",offset);
    dwt_forcetrxoff();
    int res = 0;
    res = dwt_writetxdata(offset+2, tmp, 0) ;
    res = dwt_writetxfctrl(offset+2, 0);
    uint8_t tx_mode = DWT_START_TX_DELAYED | DWT_RESPONSE_EXPECTED;
    dwt_setdelayedtrxtime(tx_timestamp);
    res = dwt_starttx(tx_mode);
    
    if(res == DWT_ERROR){
        printf("send failed &&&&&&&&&&&&\r\n");
        dwt_forcetrxoff();
        dw1000_set_state(dev, NETOPT_STATE_RX);
    }
    //free(tmp);
    DEBUG("dw1000 send is issued  \r\n");
    dev->seq_nb ++;
    //printf("sent %d bytes of data and packet len  %d \r\n",offset,pkt_len);
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
    dwt_starttx(DWT_START_TX_IMMEDIATE);
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
        uint8_t tmp[200];
        dwt_readrxdata(tmp, 2, 0);
        uint8_t ele = tmp[0]&0x7F;
        
        uint8_t ind = 16*ele + 4;
        return rx_read_len()-2-ind;
    }
    else {
        uint8_t tmp[200];
        dwt_readrxdata(tmp, max_len, 0);
        uint8_t ele = tmp[0]&0x7F;
        
        uint8_t ind = 16*ele + 4;
        printf("reading from %d for %d bytes\r\n",ind,max_len);
        if (ind > max_len){
            ind = 0;
        }
        memcpy(buf,&tmp[ind],max_len);
        
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
        //printf("saved addr is %x:%x \r\n",dev->ranging_info_array[i].short_addr[0],dev->ranging_info_array[i].short_addr[1]);
        //printf("and we are looking for %x:%x \r\n",short_addr[0],short_addr[1]);
        if(memcmp(dev->ranging_info_array[i].short_addr ,short_addr,2) == 0){
            //printf("and we found it at index %d \r\n",i);
            return i;
        }
    }
    return -1;
}

void dw1000_calc_dist(dw1000_t *dev,uint8_t * short_addr,uint64_t Reply_a,uint64_t Delay_a,uint64_t curr_ts,uint8_t last_rx_seq_nb,uint8_t last_tx_seq_nb){
    if(Reply_a == 0 || Delay_a == 0){
        printf("not valid remote time stamps \r\n");
        return;
    }
    int index  = dw1000_find_ranging_info(dev,short_addr);
    if(index < 0)
        return;
    
    
    char out1[17] = "----------------";
    fmt_u64_hex(out1, dev->last_tx_ts[last_tx_seq_nb]);
    char out2[17] = "----------------";
    fmt_u64_hex(out2, dev->ranging_info_array[index].last_rx_ts[last_rx_seq_nb]);
    char out3[17] = "----------------";
    fmt_u64_hex(out3, curr_ts);
    //printf("last tx with %d with ts %s and last rx %d with ts %s \r\n",last_tx_seq_nb,out1,last_rx_seq_nb,out2);
    //printf("current rx ts : %s \r\n",out3);
    
    //uint64_t Reply_b = (curr_ts - dev->last_tx_ts[last_tx_seq_nb]+ UINT40_MAX) % UINT40_MAX;
    //uint64_t Delay_b = (dev->last_tx_ts[last_tx_seq_nb] - dev->ranging_info_array[index].last_rx_ts[last_rx_seq_nb] + UINT40_MAX)%UINT40_MAX ;

    uint64_t Reply_b = dw1000_calc_time_diff(curr_ts, dev->last_tx_ts[last_tx_seq_nb]);
    uint64_t Delay_b = dw1000_calc_time_diff(dev->last_tx_ts[last_tx_seq_nb] , dev->ranging_info_array[index].last_rx_ts[last_rx_seq_nb]);
    
    uint64_t ToF = (Reply_a*Reply_b - Delay_b*Delay_a) / (Reply_a + Reply_b + Delay_a + Delay_b);

    
    // char out[17] = "----------------";
    // fmt_u64_hex(out, ToF);
    //printf("measured ToF low: %lu\r\n",(uint32_t)ToF);
    // printf("ToF hex %s \r\n",out);
    // char Reply_a_s[17] = "----------------";
    // fmt_u64_hex(Reply_a_s, Reply_a);
    // printf("Reply_a hex %s \r\n",Reply_a_s);
    // char Reply_b_s[17] = "----------------";
    // fmt_u64_hex(Reply_b_s, Reply_b);
    // printf("Reply_b hex %s \r\n",Reply_b_s);
    // char Delay_a_s[17] = "----------------";
    // fmt_u64_hex(Delay_a_s, Delay_a);
    // printf("Delay_a hex %s \r\n",Delay_a_s);
    // char Delay_b_s[17] = "----------------";
    // fmt_u64_hex(Delay_b_s, Delay_b);
    // printf("Delay_b hex %s \r\n",Delay_b_s);
    

    //TODO fix it later does not work for now
    float timeunit = (1.0/499.2e6/128.0);
    timeunit = timeunit * SPEED_OF_LIGHT;
    float distance = (uint32_t)ToF * timeunit;
    //printf("measured distance: %lf, %lf !!!!!!!!!!!!!!!!!!!\r\n",timeunit, distance);
    
    char buf[19];
    fmt_float(buf, distance, 3);
    printf("measured dist from %x:%x  is: %s \r\n",short_addr[0],short_addr[1],buf);

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
void dw1000_update_ranging_info(dw1000_t *dev,uint8_t * short_addr,uint64_t _time,uint8_t last_seq,uint8_t last_tx_seq){
    //printf("updating rang info \r\n");
    //printf("packet received form %x:%x \r\n",short_addr[0],short_addr[1]);
    int index = dw1000_find_ranging_info(dev,short_addr);
    if(index >= 0){  
        //printf("updating old entry at index %d \r\n",index);  
        dev->ranging_info_array[index].last_rx_ts[last_seq] = _time;
        dev->ranging_info_array[index].last_rx_seq_nb = last_seq;
        dev->ranging_info_array[index].last_tx_seq_nb = last_tx_seq;
    }else{ //this is the first time communicating with this node
        uint8_t dummy_addr[2] = {0x11,0x11};
        index = dw1000_find_ranging_info(dev,dummy_addr);
        //printf("create new item at index %d and last seq nb %d \r\n",index,last_seq);
        
        //memcpy(dev->ranging_info_array[index].short_addr , short_addr,2);
        dev->ranging_info_array[index].short_addr[0] = short_addr[0];
        dev->ranging_info_array[index].short_addr[1] = short_addr[1];
        dev->ranging_info_array[index].last_rx_ts[last_seq] = _time;
        dev->ranging_info_array[index].last_rx_seq_nb = last_seq;
        dev->ranging_info_array[index].last_tx_seq_nb = last_tx_seq;    
    }
}
uint64_t dw1000_calc_time_diff(uint64_t a,uint64_t b){
    uint64_t diff = 0;
    diff = ((a & 0x000000FFFFFFFFFF) - (b & 0x000000FFFFFFFFFF)+UINT40_MAX) %UINT40_MAX;
    uint32_t OS_diff = 0;
    OS_diff = (a >> 40) - (b >> 40);
    if (OS_diff < 0){
        OS_diff = -1*OS_diff;
    }
    if(OS_diff > 17){
        printf("timer overflow with dist %d !!!!!!!!!!!!!\r\n",(int)OS_diff);
        while(OS_diff > 17){
            diff += UINT40_MAX;
            OS_diff -=17;
        }
    }
    return diff;
}
int dw1000_insert_ranging_info(dw1000_t* dev,uint8_t* buffer,int offset,uint8_t* src_addr,uint8_t* dest_addr,uint64_t curr_ts,bool _auto_reply){
    //printf("inserting range info at offset %d \r\n",offset);
    buffer[offset] = 0;
    buffer[offset+1]= src_addr[0];
    buffer[offset+2]= src_addr[1];
    buffer[offset+3]= dev->seq_nb;
    //printf("inserting data from %x:%x with seq nb %d\r\n",src_addr[0],src_addr[1],dev->seq_nb);
    uint8_t old_offset = offset;
    offset += 4;
    if(dest_addr[0]==0xFF && dest_addr[1]==0xFF){
        uint8_t ele = 0;
        for (int i=0;i<Num_Elements;i++){
            if(dev->ranging_info_array[i].short_addr[0] == 0x11 && dev->ranging_info_array[i].short_addr[1] == 0x11){
                continue;
            }else{                
                offset = dw1000_copy_range_info(dev, i,buffer,offset,curr_ts);
                ele ++;
            }
        }
        buffer[old_offset] = ele;
    }else{
        int index  = dw1000_find_ranging_info(dev,dest_addr);    
        if (index >= 0){
            buffer[old_offset] = 1;
            offset = dw1000_copy_range_info(dev, index,buffer,offset,curr_ts);   
        }
    }
    if(_auto_reply){
        buffer[old_offset] |= 0x80; 
    }
    //printf("inserting is done and the len is %d \r\n",offset);
    return offset;
}
int dw1000_copy_range_info(dw1000_t* dev, int index,uint8_t* buffer,int offset,uint64_t curr_ts){
    //printf("copy range info to dest of %x:%x \r\n",dev->ranging_info_array[index].short_addr[0],dev->ranging_info_array[index].short_addr[1]);
    uint64_t Reply = 0;
    uint64_t Delay = 0;
    uint8_t last_tx_seq_nb = -1;
    uint8_t last_rx_seq_nb = -1;

    last_tx_seq_nb = dev->ranging_info_array[index].last_tx_seq_nb;
    last_rx_seq_nb = dev->ranging_info_array[index].last_rx_seq_nb;
    if(last_tx_seq_nb >=0 && last_rx_seq_nb >= 0){
        Reply = dw1000_calc_time_diff(dev->ranging_info_array[index].last_rx_ts[last_rx_seq_nb] , dev->last_tx_ts[last_tx_seq_nb]);
        Delay = dw1000_calc_time_diff(curr_ts , dev->ranging_info_array[index].last_rx_ts[last_rx_seq_nb]);    
    }
    buffer[offset] = dev->ranging_info_array[index].short_addr[0];
    buffer[offset+1] = dev->ranging_info_array[index].short_addr[1];
    
    offset += 2;
    buffer[offset] = last_tx_seq_nb;
    buffer[offset+1] = last_rx_seq_nb;
    offset += 2;

    buffer[offset] =     (Reply >> 40) & 0xFF;
    buffer[offset + 1] = (Reply >> 32) & 0xFF;
    buffer[offset + 2] = (Reply >> 24) & 0xFF;
    buffer[offset + 3] = (Reply >> 16) & 0xFF;
    buffer[offset + 4] = (Reply >> 8) & 0xFF;
    buffer[offset + 5] = Reply & 0xFF;
    offset += 6;
    
    buffer[offset] =     (Delay >> 40) & 0xFF;
    buffer[offset + 1] = (Delay >> 32) & 0xFF;
    buffer[offset + 2] = (Delay >> 24) & 0xFF;
    buffer[offset + 3] = (Delay >> 16) & 0xFF;
    buffer[offset + 4] = (Delay >> 8) & 0xFF;
    buffer[offset + 5] = Delay & 0xFF;
    return offset+6;
}
int dw1000_find_offset_in_broadcast(uint8_t* buffer,uint8_t * short_addr,int ele,int offset){
    
    //printf("looking for %x:%x at rec packet \r\n",short_addr[0],short_addr[1]);
    for (int i = 0;i<ele;i++){
        //printf("rec address is %x:%x \r\n",buffer[offset+(i*16)],buffer[1+offset+(i*16)]);
        if(memcmp(&buffer[offset+(i*16)],short_addr,2)==0){
            //printf("found my address at index %d \r\n",i*16);
            return offset+i*16;
        }
    }
    return -1;
}
void dw1000_extract_ranging_info(dw1000_t* dev,uint8_t* buffer,int offset,uint8_t* short_addr,uint64_t* Reply,uint64_t*Delay,uint8_t * last_tx_seq_nb,uint8_t* last_rx_seq_nb,uint8_t* rx_seq_nb){
    uint64_t _Reply = 0;
    uint64_t _Delay = 0;
    uint8_t _last_rx_seq_nb = -1;
    uint8_t _last_tx_seq_nb = -1;
    uint8_t _rx_seq_nb = -1;
    
    int ele = buffer[offset] & 0x7F;
    short_addr[0] = buffer[offset+1];
    short_addr[1] = buffer[offset+2];
    _rx_seq_nb = buffer[offset+3];
    offset += 4;
    //printf("extracting rang info \r\n");
    uint8_t my_addr[2];
    dw1000_get_addr_short(dev,my_addr);
    offset = dw1000_find_offset_in_broadcast(buffer,my_addr,ele,offset);
    if(offset >= 0){
        //printf("find my address in rec packet \r\n");
        offset +=2;
        _last_rx_seq_nb = buffer[offset];
        _last_tx_seq_nb = buffer[offset+1];
        
        offset += 2;

        int j;
        for (j = 0 ; j <= 5 ; j ++)
        {
            _Reply = (_Reply << 8) + buffer[offset+j] ;        
        }
        
        offset += 6;
        for (j = 0 ; j <= 5 ; j ++)
        {
            _Delay = (_Delay << 8) + buffer[offset+j] ;       
        }    
    }
    
    *Delay = _Delay;
    *Reply = _Reply;
    *last_rx_seq_nb = _last_rx_seq_nb;
    *last_tx_seq_nb = _last_tx_seq_nb;
    *rx_seq_nb = _rx_seq_nb;
    //printf("extracted rang info last rx %d last tx %d and seq %d \r\n",_last_rx_seq_nb,_last_tx_seq_nb,_rx_seq_nb);
    return;
}
