/*
 * Copyright (C) 2015 Freie Universität Berlin
 *
 * This file is subject to the terms and conditions of the GNU Lesser General
 * Public License v2.1. See the file LICENSE in the top level directory for more
 * details.
 */

/**
 * @defgroup    drivers_dw1000 DW1000 radio driver
 * @ingroup     drivers_netdev
 * @{
 *
 * @file
 * @brief       Interface definition for the dw1000 driver
 *
 * @author      Hessam Mohammadmoradi <hmoadi@cs.uh.edu>
 */

#ifndef DW1000_H
#define DW1000_H

#include <stdint.h>

#include "periph/spi.h"
#include "periph/gpio.h"

#include "net/netdev.h"
#include "net/netdev/ieee802154.h"

#ifdef __cplusplus
extern "C" {
#endif
#ifndef uint8
#ifndef _DECA_UINT8_
#define _DECA_UINT8_
typedef unsigned char uint8;
#endif
#endif

#ifndef uint16
#ifndef _DECA_UINT16_
#define _DECA_UINT16_
typedef unsigned short uint16;
#endif
#endif

#ifndef uint32
#ifndef _DECA_UINT32_
#define _DECA_UINT32_
typedef unsigned long uint32;
#endif
#endif

#ifndef int8
#ifndef _DECA_INT8_
#define _DECA_INT8_
typedef signed char int8;
#endif
#endif

#ifndef int16
#ifndef _DECA_INT16_
#define _DECA_INT16_
typedef signed short int16;
#endif
#endif

#ifndef int32
#ifndef _DECA_INT32_
#define _DECA_INT32_
typedef signed long int32;
#endif
#endif

typedef uint64_t        uint64 ;

typedef int64_t         int64 ;


#ifndef FALSE
#define FALSE               0
#endif

#ifndef TRUE
#define TRUE                1
#endif
/**
 * @brief   Maximum possible packet size in byte
 */
#define DW1000_PKT_MAXLEN       (127U)

/**
 * @brief   PAN ID configuration
 */
#define DW1000_PANID_DEFAULT    (0x0044U)

/**
  * @brief   Channel configuration
  * @{
  */
#define DW1000_CHAN_MIN         (1U)
#define DW1000_CHAN_MAX         (11U)
#define DW1000_CHAN_DEFAULT     (2U)
/** @} */

/**
 * @brief   Default TX power configuration [in dBm]
 * @{
 */
#define DW1000_TXPOWER_MIN      (-25)
#define DW1000_TXPOWER_MAX      (31)
#define DW1000_TXPOWER_DEFAULT  (15)
/** @} */

/**
 * @brief   A couple of return values used in this driver
 */
enum {
    DW1000_RET_CHAN_OK      = 2,
};

//// 100 ms in dw ticks 
#define DELAY_TX_100 0x17CDC00  
// 5 ms in dw ticks 
#define DELAY_TX_5 0x130B00  
//// 10 ms in dw ticks 
#define DELAY_TX_10 0x261600  
//// 20 ms in dw ticks 
#define DELAY_TX_20 0x4C2C00

#define DELAY_TX_1 0x3CF00
#define UINT40_MAX  0xFFFFFFFFFFUL
/**
 * @brief   Struct holding all parameters needed for device initialization
 * @{
 */
typedef struct dw1000_params {
    spi_t spi;              /**< SPI bus the device is connected to */
    spi_clk_t spi_clk;      /**< SPI speed to use */
    gpio_t pin_cs;          /**< pin connected to chip select */
//    gpio_t pin_fifo;        /**< pin connected to the FIFO interrupt pin */
//    gpio_t pin_fifop;       /**< pin connected to the FIFOP interrupt pin */
//    gpio_t pin_cca;         /**< pin connected to CCA */
//    gpio_t pin_sfd;         /**< pin connected to 'start of frame delimiter' */
//    gpio_t pin_vrefen;      /**< pin connected to the Vref enable pin */
    gpio_t pin_reset;       /**< pin connected to the reset pin */
    gpio_t pin_irq;
} dw1000_params_t;
/** @} */

/** @} *//**
 * @brief   keeps required ranging info 
 * @{
 */
typedef struct {
    //uint64_t last_tx_ts[255];
    uint64_t last_rx_ts[255];
    uint8_t  short_addr[2];
    uint8_t last_tx_seq_nb;
    uint8_t last_rx_seq_nb;
} dw1000_ranging_info_t;

/** @} */
/**
 * @brief   Device descriptor for DW1000 radio devices
 * @{
 */
#define Num_Elements 2
typedef struct {
    /* netdev fields */
    netdev_ieee802154_t netdev;   /**< netdev parent struct */   //maybee just use netdev_t
    /* device specific fields */
    dw1000_params_t params;       /**< hardware interface configuration */
    /* device state fields */
    uint8_t state;                /**< current state of the radio */
    uint16_t options;             /**< state of used options */
    dw1000_ranging_info_t  ranging_info_array[Num_Elements]; /**< array of ranging info structs */
    uint64_t last_tx_ts[255];
    uint8_t seq_nb;
} dw1000_t;






/**
 * @brief   Setup the device descriptor for the given device
 *
 * @param[out] dev          device descriptor
 * @param[in]  params       device parameters
 *
 * @return                  0 on success
 * @return                  -1 on error
 */
void dw1000_setup(dw1000_t *dev, const dw1000_params_t *params);

/**
 * @brief   Initialize a given DW1000 device
 *
 * @param[out] dev          device descriptor
 *
 * @return                  0 on success
 * @return                  <0 on error
 */
int dw1000_init(dw1000_t *dev);

/**
 * @brief   Trigger a hardware reset and configure radio with default values
 *
 * @param[in] dev           device to reset
 *
 * @return  TODO
 */
int dw1000_reset(dw1000_t *dev);


/**
 * @brief   Get the short address of the given device
 *
 * @param[in]  dev          device to read from
 * @param[out] addr         memory to write the 2 byte address into
 */
void dw1000_get_addr_short(dw1000_t *dev, uint8_t *addr);

/**
 * @brief   Set the short address of the given device
 *
 * @param[in] dev           device to write to
 * @param[in] addr          (2-byte) short address to set
 */
void dw1000_set_addr_short(dw1000_t *dev, uint8_t *addr);

/**
 * @brief   Get the configured long address of the given device
 *
 * @param[in]  dev           device to read from
 * @param[out] addr_long     buffer to save the read address
 *
 * @return                  the currently set (8-byte) long address
 */
void dw1000_get_addr_long(dw1000_t *dev, uint8_t *addr_long);

/**
 * @brief   Set the long address of the given device
 *
 * @param[in] dev           device to write to
 * @param[in] addr_long     (8-byte) long address to set
 */
void dw1000_set_addr_long(dw1000_t *dev, uint8_t *addr_long);

/**
 * @brief   Get the configured PAN ID of the given device
 *
 * @param[in] dev           device to read from
 *
 * @return                  the currently set PAN ID
 */
uint16_t dw1000_get_pan(dw1000_t *dev);

/**
 * @brief   Set the PAN ID of the given device
 *
 * @param[in] dev           device to write to
 * @param[in] pan           PAN ID to set
 */
void dw1000_set_pan(dw1000_t *dev, uint16_t pan);

/**
 * @brief   Get the configured channel of the given device
 *
 * @param[in] dev           device to read from
 *
 * @return                  the currently set channel
 */
uint16_t dw1000_get_chan(dw1000_t *dev);

/**
 * @brief   Set the channel of the given device
 *
 * @param[in] dev           device to write to
 * @param[in] chan          channel to set
 */
int dw1000_set_chan(dw1000_t *dev, uint16_t chan);

/**
 * @brief   Get the configured transmission power of the given device [in dBm]
 *
 * @param[in] dev           device to read from
 *
 * @return                  configured transmission power in dBm
 */
int16_t dw1000_get_txpower(dw1000_t *dev);

/**
 * @brief   Set the transmission power of the given device [in dBm]
 *
 * If the device does not support the exact dBm value given, it will set a value
 * as close as possible to the given value. If the given value is larger or
 * lower then the maximal or minimal possible value, the min or max value is
 * set, respectively.
 *
 * @param[in] dev           device to write to
 * @param[in] txpower       transmission power in dBm
 */
void dw1000_set_txpower(dw1000_t *dev, int16_t txpower);

/**
 * @brief   Enable or disable driver specific options
 *
 * @param[in] dev           device to set/clear option flag for
 * @param[in] option        option to enable/disable
 * @param[in] state         true for enable, false for disable
 */
int dw1000_set_option(dw1000_t *dev, uint16_t option, bool state);

/**
 * @brief   Set the state of the given device (trigger a state change)
 *
 * @param[in] dev           device to change state of
 * @param[in] state         the targeted new state
 */
int dw1000_set_state(dw1000_t *dev, netopt_state_t state);

/**
 * @brief   Get the state of the given device
 *
 * @param[in] dev           device to change state of
 *
 * @return                  the device's current state
 */
netopt_state_t dw1000_get_state(dw1000_t *dev);

/**
 * @brief   Convenience function for simply sending data
 *
 * @note This function ignores the PRELOADING option
 *
 * @param[in] dev           device to use for sending
 * @param[in] data          data to send (must include IEEE802.15.4 header)
 * @param[in] count         length of @p data
 *
 * @return                  number of bytes that were actually send
 * @return                  0 on error
 */
size_t dw1000_send(dw1000_t *dev, const struct iovec *data, unsigned count);

/**
 * @brief   Prepare for sending of data
 *
 * This function puts the given device into the TX state, so no receiving of
 * data is possible after it was called.
 *
 * @param[in] dev           device to prepare for sending
 * @param[in] data          data to prepare (must include IEEE802.15.4 header)
 * @param[in] count         length of @p data
 */
size_t dw1000_tx_prepare(dw1000_t *dev, const struct iovec *data, unsigned count);

/**
 * @brief   Trigger sending of data previously loaded into transmit buffer
 *
 * @param[in] dev           device to trigger
 */
void dw1000_tx_exec(dw1000_t *dev);

/**
 * @brief   Read a chunk of data from the receive buffer of the given device
 *
 * @param[in]  dev          device to read from
 * @param[out] buf          buffer to write data to
 * @param[in]  max_len      number of bytes to read from device
 * @param[in]  info         to be removed
 *
 * @return                  the number of bytes in the Rx FIFO
 * @return                  the number of bytes written to @p buf if present
 */
int dw1000_rx(dw1000_t *dev, uint8_t *buf, size_t max_len, void *info);
int dw1000_startrx(uint16 time);
void dw1000_stoprx(void);
//void dw1000_rxcallback(const dw1000_t *dev,const dwt_callback_data_t *rxd);
//void dw1000_rxcallback(const dw1000_t *dev,const dwt_callback_data_t *rxd);
void dw1000_extract_ranging_info(dw1000_t* dev,uint8_t* buffer,int offset,uint8_t* short_addr,uint64_t* Reply,uint64_t*Delay,uint8_t * last_tx_seq_nb,uint8_t* last_rx_seq_nb,uint8_t* rx_seq_nb);
int dw1000_insert_ranging_info(dw1000_t* dev,uint8_t* buffer,int offset,uint8_t* src_addr,uint8_t* dest_addr,uint64_t curr_ts);
void dw1000_update_ranging_info(dw1000_t *dev,uint8_t * short_addr,uint64_t ts,uint8_t last_seq,uint8_t last_tx_seq);
uint64_t dw1000_convert_ts_to_int(uint8_t* ts);
void dw1000_calc_dist(dw1000_t *dev,uint8_t * short_addr,uint64_t Reply_a,uint64_t Delay_a,uint64_t curr_ts,uint8_t last_rx_seq_nb,uint8_t last_tx_seq_nb);
int dw1000_find_ranging_info(dw1000_t *dev,uint8_t * short_addr);
#ifdef __cplusplus
}
#endif

#endif /* DW1000_H */
/** @} */
