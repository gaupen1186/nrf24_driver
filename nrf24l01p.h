/*!
****************************************************************************
* Copyright (C) A-Champs 2019
*
* nrf24l01p.h
* Date: 2019/2/28
* Revision: 1.0
* Author: Pan@a-champs.com
*
* Brief: xxx
* Usage: xxx
*
****************************************************************************
*/
#ifndef __NRF24L01P_H__
#define __NRF24L01P_H__

#include <stdbool.h>
#include <stdint.h>


/* C++ detection */
#ifdef __cplusplus
extern "C"
{
#endif

  /**
  Copyright (C) gaupen1186@gmail.com, 2018

  brief
  *
  *
  Changelog
  *
  *
  Dependencies
  *
  */

  /**
  * @defgroup NRF24P_Macros
  * @brief    Library defines
  * @{
  */

  /*====================================================
  */

// SI24R1 = true, nrf24l01p = false
#define __SI24R1__    false


/*******************************************************************************
*
*/

// dynamic paload length settings
#define ENABLE_DYNAMIC_PALOAD_LENGTH    false

#if !ENABLE_DYNAMIC_PALOAD_LENGTH
// static paload length settings
#define NRF24_STATIC_LENGTH     32
#endif


#define NRF24_MAX_PAYLOAD_SIZE 32

  // command
#define NRF24_CMD_RX_PAYLOAD 0x61  // Read RX-payload
#define NRF24_CMD_TX_PAYLOAD 0xA0  // Write TX-payload
#define NRF24_CMD_FLUSH_TX 0xE1    // Flush TX FIFO, used in TX mode
#define NRF24_CMD_FLUSH_RX 0xE2    // Flush RX FIFO, used in RX mode
#define NRF24_CMD_REUSE_TX_PL 0xE3 // Reuse last transmitted payload, Used for a PTX device
#define NRF24_CMD_RX_PL_WID 0x60   // Read RX payload width for the top

#define NRF24_CMD_TX_PAYLOAD_NOACK 0xB0 // Write TX palyload with no ack. Disables AUTOACK on this specific packet
#define NRF24_CMD_NOP 0xFF              // No Operation. Might be used to read the STATUS register

  // #define NRF24_ACTIVATE_MASK				0x50
  // #define NRF24_CMD_ACTIVATE				0x50

  /* NRF24+ registers*/
#define NRF24_REG_CONFIG 0x00     //Configuration Register
#define NRF24_REG_EN_AA 0x01      //Enable auto Acknowledgment Function
#define NRF24_REG_EN_RXADDR 0x02  //Enabled RX Addresses
#define NRF24_REG_SETUP_AW 0x03   //Setup of Address Widths (common for all data pipes)
#define NRF24_REG_SETUP_RETR 0x04 //Setup of Automatic Retransmission
#define NRF24_REG_RF_CH 0x05      //RF Channel
#define NRF24_REG_RF_SETUP 0x06   //RF Setup Register
#define NRF24_REG_STATUS 0x07     //Status Register
#define NRF24_REG_OBSERVE_TX 0x08 //Transmit observe register
#define NRF24_REG_RPD 0x09
#define NRF24_REG_RX_ADDR_P0 0x0A //Receive address data pipe 0. 5 Bytes maximum length.
#define NRF24_REG_RX_ADDR_P1 0x0B //Receive address data pipe 1. 5 Bytes maximum length.
#define NRF24_REG_RX_ADDR_P2 0x0C //Receive address data pipe 2. Only LSB
#define NRF24_REG_RX_ADDR_P3 0x0D //Receive address data pipe 3. Only LSB
#define NRF24_REG_RX_ADDR_P4 0x0E //Receive address data pipe 4. Only LSB
#define NRF24_REG_RX_ADDR_P5 0x0F //Receive address data pipe 5. Only LSB
#define NRF24_REG_TX_ADDR 0x10    //Transmit address. Used for a PTX device only
#define NRF24_REG_RX_PW_P0 0x11
#define NRF24_REG_RX_PW_P1 0x12
#define NRF24_REG_RX_PW_P2 0x13
#define NRF24_REG_RX_PW_P3 0x14
#define NRF24_REG_RX_PW_P4 0x15
#define NRF24_REG_RX_PW_P5 0x16
#define NRF24_REG_FIFO_STATUS 0x17 //FIFO Status Register
#define NRF24_REG_DYNPD 0x1C       //Enable dynamic payload length
#define NRF24_REG_FEATURE 0x1D

  /* Registers default values */
#define NRF24_REG_DEFAULT_VAL_CONFIG 0x08
#define NRF24_REG_DEFAULT_VAL_EN_AA 0x3F
#define NRF24_REG_DEFAULT_VAL_EN_RXADDR 0x03
#define NRF24_REG_DEFAULT_VAL_SETUP_AW 0x03
#define NRF24_REG_DEFAULT_VAL_SETUP_RETR 0x03
#define NRF24_REG_DEFAULT_VAL_RF_CH 0x02
#define NRF24_REG_DEFAULT_VAL_RF_SETUP 0x0E
#define NRF24_REG_DEFAULT_VAL_STATUS 0x0E
#define NRF24_REG_DEFAULT_VAL_OBSERVE_TX 0x00
#define NRF24_REG_DEFAULT_VAL_RPD 0x00
#define NRF24_REG_DEFAULT_VAL_RX_ADDR_P0_0 0xE7
#define NRF24_REG_DEFAULT_VAL_RX_ADDR_P0_1 0xE7
#define NRF24_REG_DEFAULT_VAL_RX_ADDR_P0_2 0xE7
#define NRF24_REG_DEFAULT_VAL_RX_ADDR_P0_3 0xE7
#define NRF24_REG_DEFAULT_VAL_RX_ADDR_P0_4 0xE7
#define NRF24_REG_DEFAULT_VAL_RX_ADDR_P1_0 0xC2
#define NRF24_REG_DEFAULT_VAL_RX_ADDR_P1_1 0xC2
#define NRF24_REG_DEFAULT_VAL_RX_ADDR_P1_2 0xC2
#define NRF24_REG_DEFAULT_VAL_RX_ADDR_P1_3 0xC2
#define NRF24_REG_DEFAULT_VAL_RX_ADDR_P1_4 0xC2
#define NRF24_REG_DEFAULT_VAL_RX_ADDR_P2 0xC3
#define NRF24_REG_DEFAULT_VAL_RX_ADDR_P3 0xC4
#define NRF24_REG_DEFAULT_VAL_RX_ADDR_P4 0xC5
#define NRF24_REG_DEFAULT_VAL_RX_ADDR_P5 0xC6
#define NRF24_REG_DEFAULT_VAL_TX_ADDR_0 0xE7
#define NRF24_REG_DEFAULT_VAL_TX_ADDR_1 0xE7
#define NRF24_REG_DEFAULT_VAL_TX_ADDR_2 0xE7
#define NRF24_REG_DEFAULT_VAL_TX_ADDR_3 0xE7
#define NRF24_REG_DEFAULT_VAL_TX_ADDR_4 0xE7
#define NRF24_REG_DEFAULT_VAL_RX_PW_P0 0x00
#define NRF24_REG_DEFAULT_VAL_RX_PW_P1 0x00
#define NRF24_REG_DEFAULT_VAL_RX_PW_P2 0x00
#define NRF24_REG_DEFAULT_VAL_RX_PW_P3 0x00
#define NRF24_REG_DEFAULT_VAL_RX_PW_P4 0x00
#define NRF24_REG_DEFAULT_VAL_RX_PW_P5 0x00
#define NRF24_REG_DEFAULT_VAL_FIFO_STATUS 0x11
#define NRF24_REG_DEFAULT_VAL_DYNPD 0x00
#define NRF24_REG_DEFAULT_VAL_FEATURE 0x00

  /* Configuration register*/
#define NRF24_MASK_RX_DR 6
#define NRF24_MASK_TX_DS 5
#define NRF24_MASK_MAX_RT 4
#define NRF24_EN_CRC 3
#define NRF24_CRCO 2
#define NRF24_PWR_UP 1
#define NRF24_PRIM_RX 0

  /* Enable auto acknowledgment*/
#define NRF24_ENAA_P5 5
#define NRF24_ENAA_P4 4
#define NRF24_ENAA_P3 3
#define NRF24_ENAA_P2 2
#define NRF24_ENAA_P1 1
#define NRF24_ENAA_P0 0

  /* Enable rx addresses */
#define NRF24_ERX_P5 5
#define NRF24_ERX_P4 4
#define NRF24_ERX_P3 3
#define NRF24_ERX_P2 2
#define NRF24_ERX_P1 1
#define NRF24_ERX_P0 0

  /* Setup of address width */
#define NRF24_AW 0 //2 bits

  /* Setup of auto re-transmission*/
#define NRF24_ARD 4 //4 bits
#define NRF24_ARC 0 //4 bits

  /* RF setup register*/
#define NRF24_CONT_WAVE 7
#define NRF24_PLL_LOCK 4
#define NRF24_RF_DR_LOW 5
#define NRF24_RF_DR_HIGH 3
#if __SI24R1__
#define NRF24_RF_PWR 0     // bits 0 ~2, si24R1 is 3 bits
#else   // NRF24L01P
#define NRF24_RF_PWR 1     // bits 1~2, rf24l01p is 2bits
#endif  // __SI24R1__

  /* General status register */
#define NRF24_RX_DR 6
#define NRF24_TX_DS 5
#define NRF24_MAX_RT 4
#define NRF24_RX_P_NO 1 //3 bits
#define NRF24_TX_FULL 0

  /* Transmit observe register */
#define NRF24_PLOS_CNT 4 //4 bits
#define NRF24_ARC_CNT 0  //4 bits

  /* FIFO status*/
#define NRF24_TX_REUSE 6
#define NRF24_TX_FIFO_FULL 5
#define NRF24_TX_EMPTY 4
#define NRF24_RX_FULL 1
#define NRF24_RX_EMPTY 0

  //Dynamic length
#define NRF24_DPL_P0 0
#define NRF24_DPL_P1 1
#define NRF24_DPL_P2 2
#define NRF24_DPL_P3 3
#define NRF24_DPL_P4 4
#define NRF24_DPL_P5 5

  /*******************************************************************************
  * NRF24P_Typedefs
  */

  /**
  * @brief  auto-acknowledgment enumeration
  */
  typedef enum
  {
    AUTO_ACK,
    NO_ACK
  }
  NRF24_ACK_t;

  /**
  * @brief  Data rate enumeration
  */
  typedef enum
  {
    NRF24_DataRate_2M,  /*!< Data rate set to 2Mbps */
    NRF24_DataRate_1M,  /*!< Data rate set to 1Mbps */
    NRF24_DataRate_250k /*!< Data rate set to 250kbps */
  }
  NRF24_DR_t;

  /**
  * @brief  Output power enumeration
  */
  typedef enum
  {
    NRF24_Power_0dBm,   // 0dBm
    NRF24_Power_N6dBm,  // -6dBm
    NRF24_Power_N12dBm, // -12dBm
#if __SI24R1__
    NRF24_Power_7dBm,  // 7dBm
    NRF24_Power_4dBm,  // 4dBm
    NRF24_Power_3dBm,  // 3dBm
    NRF24_Power_1dBm,  // 1dBm
    NRF24_Power_N4dBm, // -4dBm
#else   // NRF24L01P
    NRF24_Power_N18dBm, // -18dBm
#endif
  }
  NRF24_Power_t;

  /**
  * @brief  fifo status
  */
  typedef struct
  {
    bool is_tx_reuse;
    bool is_tx_full;
    bool is_tx_empty;
    bool is_rx_full;
    bool is_rx_empty;
  }
  NRF24_FifoStatus_t;

  /**
  * @brief  global status
  */
  typedef struct
  {
    bool is_tx_full; // TX FIFO full flag
    uint8_t rx_p_no; // Data pipe number for the payload available for reading from RX_FIFO
    // 0-5: Data Pipe Number, 6: not used, 7: RX FIFO empty
    bool is_max_rt; // Maximum number of TX retransmits interrupt Write 1 to clear bit
    bool is_tx_ds;  // Data Sent TX FIFO interrupt
    bool is_rx_dr;  // Data Ready RX FIFO interrupt
  }
  NRF24_status_t;

  /**
  * @brief  device mode enumeration
  */
  typedef enum
  {
    MODE_POWERDOWN,
    MODE_STANDBY,
    MODE_TX,
    MODE_RX,
  }
  NRF24_mode_t;

  /**
  * @brief  call back functions struct
  */
  typedef struct
  {
    void (*TX_done_cb)(void);             // tx done callback
    void (*TX_full_cb)(void);             // tx fifo full callback
    /* rx data ready callback
       data: received data, 3 packets max
       pipe_num: the pipe number of received data
    */
    void (*RX_data_ready_cb)(uint8_t *data, uint8_t size, uint8_t pipe_num);
    void (*Max_retry_cb)(void);         // max retry count callback
  }
  NRF24_cb_t;

  /**
  * @brief  device global struct
  */
  typedef struct
  {
    uint8_t Channel;     //Channel selected
    NRF24_Power_t Power; //Output power
    NRF24_DR_t DataRate; //Data rate
//    uint8_t rx_pipe;     // rx pipe number, 0~5
    uint8_t tx_addr[5];
    uint8_t rx0_addr[5];    // rx pipe 0 address
    uint8_t rx1_addr[5];    // rx pipe 1 address
    uint8_t rx2_addr;    // rx pipe 2 address
    uint8_t rx3_addr;    // rx pipe 3 address
    uint8_t rx4_addr;    // rx pipe 4 address
    uint8_t rx5_addr;    // rx pipe 5 address
    NRF24_ACK_t ack;
    uint8_t delay;       // Auto Retransmit Delay Time = ( delay +1 )* 250us
    uint8_t retry_count; // Auto Retransmit Count, 0~15, 0 is disabled
    NRF24_FifoStatus_t fifo_status;
    NRF24_status_t status;
    NRF24_mode_t mode; // current mode
    NRF24_cb_t *callback;
  }
  NRF24_t;

  /*******************************************************************************
  * Public functions prototype
  */
  void NRF24_Dump_Regs(uint8_t *regs);
  bool NRF24_SetChannel(NRF24_t *nrf24, uint8_t channel);
  bool NRF24_GetChannel(NRF24_t *nrf24, uint8_t *channel);
  bool NRF24_Set_DR_Power(NRF24_t *nrf24, NRF24_DR_t dataRate, NRF24_Power_t power);
  bool NRF24_Set_Constant_Output(NRF24_t *nrf24, uint8_t channel, NRF24_Power_t power);
  bool NRF24_Set_Pipe_AutoAck(NRF24_t *nrf24, bool enable, uint8_t pipe_num);
  bool NRF24_Set_Pipe_Enable(NRF24_t *nrf24, bool enable, uint8_t pipe_num);
  bool NRF24_SetRxAddress(NRF24_t *nrf24, uint8_t pipe_number, uint8_t *rx_addr);
  bool NRF24_GetRxAddress(uint8_t pipe_number, uint8_t *addr);
  bool NRF24_SetTxAddress(NRF24_t *nrf24, uint8_t *tx_addr);
  bool NRF24_GetTxAddress(uint8_t *addr);
  bool NRF24_SetAutoRetry(NRF24_t *nrf24, uint8_t delay, uint8_t count);
  bool NRF24_Flush_Tx(void);
  bool NRF24_Flush_Rx(void);
  bool NRF24_PowerDown(NRF24_t *nrf24);
  bool NRF24_PowerStandby(NRF24_t *nrf24);
  bool NRF24_PowerUpTx(NRF24_t *nrf24);
  bool NRF24_PowerUpRx(NRF24_t *nrf24);
  bool NRF24_GetMode(NRF24_t *nrf24, NRF24_mode_t *mode);
  bool NRF24_GetStatus(NRF24_t *nrf24, NRF24_status_t *p_status);
  bool NRF24_Get_Fifo_Status(NRF24_t *nrf24, NRF24_FifoStatus_t *fifo_status);
  bool NRF24_Get_Retry_Count(uint8_t *retry_cnt);
  bool NRF24_is_channel_free(NRF24_t *nrf24, uint8_t channel, bool *is_ch_free);
  bool NRF24_Init(NRF24_t *nrf24,
                  NRF24_ACK_t ack,
                  uint8_t delay,
                  uint8_t retry_count,
                  NRF24_cb_t *call_back);
  bool NRF24_Transmit(NRF24_t *nrf24, uint8_t *data, uint8_t size);
  void NRF24_Irq_callback(NRF24_t *nrf24);

  /* C++ detection */
#ifdef __cplusplus
}
#endif

#endif // __NRF24L01P_H__
