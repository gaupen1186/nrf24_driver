/**	
* |----------------------------------------------------------------------
* | Copyright (C) gaupen1186@gmail.com, 2018
* | 
* | This program is free software: you can redistribute it and/or modify
* | it under the terms of the GNU General Public License as published by
* | the Free Software Foundation, either version 3 of the License, or
* | any later version.
* |  
* | This program is distributed in the hope that it will be useful,
* | but WITHOUT ANY WARRANTY; without even the implied warranty of
* | MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
* | GNU General Public License for more details.
* | 
* | You should have received a copy of the GNU General Public License
* | along with this program.  If not, see <http://www.gnu.org/licenses/>.
* |----------------------------------------------------------------------
*/

/*******************************************************************************
*/
#include <stdint.h>
#include <string.h>

#include "nrf24_hal.h"
#include "nrf24l01p.h"

#define NRF24_CHECK_BIT(reg, bit) (reg & (1 << bit))
#define NRF24_SET_BIT(reg, bit) ((reg) |= (1 << bit))
#define NRF24_CLEAR_BIT(reg, bit) ((reg) &= ~(1 << bit))

/*******************************************************************************
* NRF structure
*/
static NRF24_t NRF24_Struct = { 0 };

/*******************************************************************************
* Private functions
*/
static void NRF24_SoftwareReset(void)
{
  uint8_t data[5];

  NRF24_WriteRegister(NRF24_REG_CONFIG, NRF24_REG_DEFAULT_VAL_CONFIG);
  NRF24_WriteRegister(NRF24_REG_EN_AA, NRF24_REG_DEFAULT_VAL_EN_AA);
  NRF24_WriteRegister(NRF24_REG_EN_RXADDR, NRF24_REG_DEFAULT_VAL_EN_RXADDR);
  NRF24_WriteRegister(NRF24_REG_SETUP_AW, NRF24_REG_DEFAULT_VAL_SETUP_AW);
  NRF24_WriteRegister(NRF24_REG_SETUP_RETR, NRF24_REG_DEFAULT_VAL_SETUP_RETR);
  NRF24_WriteRegister(NRF24_REG_RF_CH, NRF24_REG_DEFAULT_VAL_RF_CH);
  NRF24_WriteRegister(NRF24_REG_RF_SETUP, NRF24_REG_DEFAULT_VAL_RF_SETUP);
  NRF24_WriteRegister(NRF24_REG_STATUS, NRF24_REG_DEFAULT_VAL_STATUS);
  NRF24_WriteRegister(NRF24_REG_OBSERVE_TX, NRF24_REG_DEFAULT_VAL_OBSERVE_TX);
  NRF24_WriteRegister(NRF24_REG_RPD, NRF24_REG_DEFAULT_VAL_RPD);

  //P0
  data[0] = NRF24_REG_DEFAULT_VAL_RX_ADDR_P0_0;
  data[1] = NRF24_REG_DEFAULT_VAL_RX_ADDR_P0_1;
  data[2] = NRF24_REG_DEFAULT_VAL_RX_ADDR_P0_2;
  data[3] = NRF24_REG_DEFAULT_VAL_RX_ADDR_P0_3;
  data[4] = NRF24_REG_DEFAULT_VAL_RX_ADDR_P0_4;
  NRF24_WriteRegisterMulti(NRF24_REG_RX_ADDR_P0, data, 5);

  //P1
  data[0] = NRF24_REG_DEFAULT_VAL_RX_ADDR_P1_0;
  data[1] = NRF24_REG_DEFAULT_VAL_RX_ADDR_P1_1;
  data[2] = NRF24_REG_DEFAULT_VAL_RX_ADDR_P1_2;
  data[3] = NRF24_REG_DEFAULT_VAL_RX_ADDR_P1_3;
  data[4] = NRF24_REG_DEFAULT_VAL_RX_ADDR_P1_4;
  NRF24_WriteRegisterMulti(NRF24_REG_RX_ADDR_P1, data, 5);

  NRF24_WriteRegister(NRF24_REG_RX_ADDR_P2, NRF24_REG_DEFAULT_VAL_RX_ADDR_P2);
  NRF24_WriteRegister(NRF24_REG_RX_ADDR_P3, NRF24_REG_DEFAULT_VAL_RX_ADDR_P3);
  NRF24_WriteRegister(NRF24_REG_RX_ADDR_P4, NRF24_REG_DEFAULT_VAL_RX_ADDR_P4);
  NRF24_WriteRegister(NRF24_REG_RX_ADDR_P5, NRF24_REG_DEFAULT_VAL_RX_ADDR_P5);

  //TX
  data[0] = NRF24_REG_DEFAULT_VAL_TX_ADDR_0;
  data[1] = NRF24_REG_DEFAULT_VAL_TX_ADDR_1;
  data[2] = NRF24_REG_DEFAULT_VAL_TX_ADDR_2;
  data[3] = NRF24_REG_DEFAULT_VAL_TX_ADDR_3;
  data[4] = NRF24_REG_DEFAULT_VAL_TX_ADDR_4;
  NRF24_WriteRegisterMulti(NRF24_REG_TX_ADDR, data, 5);

  NRF24_WriteRegister(NRF24_REG_RX_PW_P0, NRF24_REG_DEFAULT_VAL_RX_PW_P0);
  NRF24_WriteRegister(NRF24_REG_RX_PW_P1, NRF24_REG_DEFAULT_VAL_RX_PW_P1);
  NRF24_WriteRegister(NRF24_REG_RX_PW_P2, NRF24_REG_DEFAULT_VAL_RX_PW_P2);
  NRF24_WriteRegister(NRF24_REG_RX_PW_P3, NRF24_REG_DEFAULT_VAL_RX_PW_P3);
  NRF24_WriteRegister(NRF24_REG_RX_PW_P4, NRF24_REG_DEFAULT_VAL_RX_PW_P4);
  NRF24_WriteRegister(NRF24_REG_RX_PW_P5, NRF24_REG_DEFAULT_VAL_RX_PW_P5);
  NRF24_WriteRegister(NRF24_REG_FIFO_STATUS, NRF24_REG_DEFAULT_VAL_FIFO_STATUS);
  NRF24_WriteRegister(NRF24_REG_DYNPD, NRF24_REG_DEFAULT_VAL_DYNPD);
  NRF24_WriteRegister(NRF24_REG_FEATURE, NRF24_REG_DEFAULT_VAL_FEATURE);

  NRF24_Struct.mode = MODE_POWERDOWN;
}

static inline bool NRF24_ClearInterrupts(void)
{
  bool ret = NRF24_WriteRegister(NRF24_REG_STATUS, 0x70);
  return ret;
}

/*******************************************************************************
* Public functions
*/
// regs length must be larger than 0x1E
void NRF24_Dump_Regs(uint8_t *regs)
{
  uint8_t i;
  for (i = 0; i <= 0x1D; i++)
    regs[i] = NRF24_ReadRegister(i);
}

static uint8_t NRF24_Get_Received_Size(void)
{
  uint8_t size;
  bool ret = NRF24_SPI_RecvMulti(NRF24_CMD_RX_PL_WID, &size, 1);
  return size;
}

// before call this function, you need call NRF24_Get_Received_Size() first.
static bool NRF24_ReceiveData(uint8_t *data, uint8_t size)
{
  /* Read payload */
  bool ret = NRF24_SPI_RecvMulti(NRF24_CMD_RX_PAYLOAD, data, size);
  return ret;

  /* Reset status register, clear RX_DR interrupt flag */
  // NRF24_WriteRegister(NRF24_REG_STATUS, (1 << NRF24_RX_DR));
}

/*
bool NRF24_SetAddressWidths( NRF24_AW_t widths )
{
  bool ret;
  if( widths == NRF24_AW_3BYTES )
  {
    NRF24_Struct.aw = NRF24_AW_3BYTES;
  }
  else( widths == NRF24_AW_4BYTES )
  {
    NRF24_Struct.aw = NRF24_AW_4BYTES;
  }
  else
  {
    NRF24_Struct.aw = NRF24_AW_5BYTES;
  }

  ret = NRF24_WriteRegister(NRF24_REG_SETUP_AW, NRF24_Struct.aw );
  return ret;
}
*/

bool NRF24_SetChannel(uint8_t channel)
{
  bool ret = true;
  /* Store new channel setting */
  NRF24_Struct.Channel = channel;
  /* Write channel */
  ret = NRF24_WriteRegister(NRF24_REG_RF_CH, channel);

  return ret;
}

bool NRF24_Set_DR_Power(NRF24_DR_t dataRate, NRF24_Power_t power)
{
  uint8_t tmp = 0;
  NRF24_Struct.DataRate = dataRate;
  NRF24_Struct.Power = power;

  if (dataRate == NRF24_DataRate_2M)
  {
    NRF24_SET_BIT(tmp, NRF24_RF_DR_HIGH);
  }
  else if (dataRate == NRF24_DataRate_250k)
  {
    NRF24_SET_BIT(tmp, NRF24_RF_DR_LOW);
  }

#ifdef __SI24R1__
  if (power == NRF24_Power_7dBm)
    tmp |= 7 << NRF24_RF_PWR;
  else if (power == NRF24_Power_4dBm)
    tmp |= 6 << NRF24_RF_PWR;
  else if (power == NRF24_Power_3dBm)
    tmp |= 5 << NRF24_RF_PWR;
  else if (power == NRF24_Power_1dBm)
    tmp |= 4 << NRF24_RF_PWR;
  else if (power == NRF24_Power_0dBm)
    tmp |= 3 << NRF24_RF_PWR;
  else if (power == NRF24_Power_N4dBm)
    tmp |= 2 << NRF24_RF_PWR;
  else if (power == NRF24_Power_N6dBm)
    tmp |= 1 << NRF24_RF_PWR;
  else if (power == NRF24_Power_N12dBm)
    tmp |= 0 << NRF24_RF_PWR;
#elif define __NRF24L01P__
  if (power == NRF24_Power_0dBm)
    tmp |= 3 << NRF24_RF_PWR;
  else if (power == NRF24_Power_N6dBm)
    tmp |= 2 << NRF24_RF_PWR;
  else if (power == NRF24_Power_N12dBm)
    tmp |= 1 << NRF24_RF_PWR;
  else if (power == NRF24_Power_N18dBm)
    tmp |= 0 << NRF24_RF_PWR;
#endif

  bool ret = NRF24_WriteRegister(NRF24_REG_RF_SETUP, tmp);
  return ret;
}

// Constant carrier wave output for testing
bool NRF24_Set_Constant_Output(uint8_t channel, NRF24_Power_t power)
{
  uint8_t tmp = 0;
  
  if( NRF24_PowerStandby() != true )
    return false;
  
#ifdef __SI24R1__
  if (power == NRF24_Power_7dBm)
    tmp |= 7 << NRF24_RF_PWR;
  else if (power == NRF24_Power_4dBm)
    tmp |= 6 << NRF24_RF_PWR;
  else if (power == NRF24_Power_3dBm)
    tmp |= 5 << NRF24_RF_PWR;
  else if (power == NRF24_Power_1dBm)
    tmp |= 4 << NRF24_RF_PWR;
  else if (power == NRF24_Power_0dBm)
    tmp |= 3 << NRF24_RF_PWR;
  else if (power == NRF24_Power_N4dBm)
    tmp |= 2 << NRF24_RF_PWR;
  else if (power == NRF24_Power_N6dBm)
    tmp |= 1 << NRF24_RF_PWR;
  else if (power == NRF24_Power_N12dBm)
    tmp |= 0 << NRF24_RF_PWR;
#elif define __NRF24L01P__
  if (power == NRF24_Power_0dBm)
    tmp |= 3 << NRF24_RF_PWR;
  else if (power == NRF24_Power_N6dBm)
    tmp |= 2 << NRF24_RF_PWR;
  else if (power == NRF24_Power_N12dBm)
    tmp |= 1 << NRF24_RF_PWR;
  else if (power == NRF24_Power_N18dBm)
    tmp |= 0 << NRF24_RF_PWR;
#endif

  if( NRF24_SetChannel(channel) != true )
    return false;
  
  NRF24_SET_BIT( tmp, NRF24_CONT_WAVE );
  NRF24_SET_BIT( tmp, NRF24_PLL_LOCK );
  
  bool ret = NRF24_WriteRegister(NRF24_REG_RF_SETUP, tmp);

  return ret;
}

/*
  pipe_number = 0 ~ 5
  if pipe_number = 2~5, this func set the first 4 byte address to 
  reg REG_RX_ADDR_P1, and then set the 5th byte to the REG_RX_ADDR_Px
*/
bool NRF24_SetRxAddress(uint8_t pipe_number, uint8_t *rx_addr)
{
  bool ret;

  NRF24_Struct.rx_pipe = pipe_number;
  memcpy(NRF24_Struct.rx_address, rx_addr, 5);

  // read back enable status of pipe x
  uint8_t en_rx_px = NRF24_ReadRegister(NRF24_REG_EN_RXADDR);
  // read back auto ack of pipe x
  uint8_t enaa_px = NRF24_ReadRegister(NRF24_REG_EN_AA);

  // set pipe 2~5 high 4 bytes address to reg NRF24_REG_RX_ADDR_P1
  if( pipe_number != 0 && pipe_number != 1 )
  {
    if( NRF24_WriteRegisterMulti(NRF24_REG_RX_ADDR_P1, rx_addr, 4) != true )
      return false;
  }
  
  switch (pipe_number)
  {
  case 0:
    // higher 4 bytes address as same as NRF24_REG_RX_ADDR_P0
    ret = NRF24_WriteRegisterMulti(NRF24_REG_RX_ADDR_P0, rx_addr, 5);
    NRF24_SET_BIT(en_rx_px, NRF24_ERX_P0);
    break;

  case 1:
    ret = NRF24_WriteRegisterMulti(NRF24_REG_RX_ADDR_P1, rx_addr, 5);
    NRF24_SET_BIT(en_rx_px, NRF24_ERX_P1);
    break;

  case 2:
    ret = NRF24_WriteRegisterMulti(NRF24_REG_RX_ADDR_P2, &rx_addr[4], 1);
    NRF24_SET_BIT(en_rx_px, NRF24_ERX_P2);
    break;

  case 3:
    ret = NRF24_WriteRegisterMulti(NRF24_REG_RX_ADDR_P3, &rx_addr[4], 1);
    NRF24_SET_BIT(en_rx_px, NRF24_ERX_P3);
    break;

  case 4:
    ret = NRF24_WriteRegisterMulti(NRF24_REG_RX_ADDR_P4, &rx_addr[4], 1);
    NRF24_SET_BIT(en_rx_px, NRF24_ERX_P4);
    break;

  case 5:
    ret = NRF24_WriteRegisterMulti(NRF24_REG_RX_ADDR_P5, &rx_addr[4], 1);
    NRF24_SET_BIT(en_rx_px, NRF24_ERX_P5);
    break;
  }
  if (ret != true)
    return false;

  /* Enable RX pipe x */
  ret = NRF24_WriteRegister(NRF24_REG_EN_RXADDR, en_rx_px);
  if (ret != true)
    return false;

  // disable pipe x auto ack
  if (NRF24_Struct.ack == NO_ACK)
  {
    NRF24_CLEAR_BIT(enaa_px, pipe_number);
  }
  // enable pipe x auto ack
  else
  {
    NRF24_SET_BIT(enaa_px, pipe_number);
  }
  if (NRF24_WriteRegister(NRF24_REG_EN_AA, enaa_px) != true)
    return false;

  return true;
}

bool NRF24_SetTxAddress(uint8_t *tx_addr)
{
  memcpy(NRF24_Struct.tx_address, tx_addr, 5);

  bool ret = NRF24_WriteRegisterMulti(NRF24_REG_TX_ADDR, tx_addr, 5);
  return ret;
}

/*
  delay = 0~15 ( 250us ~ 4000us ), delay time = ( delay +1 )* 250us
  count = 0~15, Auto Retransmit Count, 0 is disabled
*/
bool NRF24_SetAutoRetry(uint8_t delay, uint8_t count)
{
  NRF24_Struct.delay = delay;
  NRF24_Struct.retry_count = count;
  bool ret = NRF24_WriteRegister(NRF24_REG_SETUP_RETR, (delay << 4) | count);
  return ret;
}

/* Flush FIFOs */
bool NRF24_Flush_Tx(void)
{
  bool ret = NRF24_SPI_SendMulti(NRF24_CMD_FLUSH_TX, NULL, 0);
  return ret;
}

bool NRF24_Flush_Rx(void)
{
  bool ret = NRF24_SPI_SendMulti(NRF24_CMD_FLUSH_RX, NULL, 0);
  return ret;
}

bool NRF24_PowerDown(void)
{
  uint8_t tmp = NRF24_ReadRegister(NRF24_REG_CONFIG);
  NRF24_CLEAR_BIT(tmp, NRF24_PWR_UP);
  bool ret = NRF24_WriteRegister(NRF24_REG_CONFIG, tmp);
  NRF24_Struct.mode = MODE_POWERDOWN;
  return ret;
}

bool NRF24_PowerStandby(void)
{
  uint8_t tmp = NRF24_ReadRegister(NRF24_REG_CONFIG);

  // from RX or TX mode to standby mode
  if (NRF24_CHECK_BIT(tmp, NRF24_PWR_UP) == 1)
  {
    nrf24_set_CE_Low();
    NRF24_Struct.mode = MODE_STANDBY;
    return true;
  }
  //else: from shutdown mode to standby mode
  nrf24_set_CE_Low();
  NRF24_Struct.mode = MODE_STANDBY;
  bool ret = NRF24_WriteRegister(NRF24_REG_CONFIG, NRF24_SET_BIT(tmp, NRF24_PWR_UP));
  nrf24_delay_ms(3);
  return ret;
}

bool NRF24_PowerUpTx(void)
{
  bool ret;
  NRF24_ClearInterrupts();

  uint8_t tmp = NRF24_ReadRegister(NRF24_REG_CONFIG);

  // shutdown mode -> standby mode
  if (NRF24_CHECK_BIT(tmp, NRF24_PWR_UP) == 0)
  {
    ret = NRF24_PowerStandby();
    nrf24_delay_ms(3);
    if (ret != true)
      return false;
  }

  // TX mode or RX mode -> RX mode
  if (nrf24_get_CE() == 1)
  {
    // RX mode -> standby mode
    if (NRF24_CHECK_BIT(tmp, NRF24_PRIM_RX) == 1)
    {
      if (NRF24_PowerStandby() != true)
        return false;
    }
    // TX mode, do nothing
    else
      return true;
  }

  // set PRIM_RX = 0
  NRF24_SET_BIT(tmp, NRF24_PWR_UP);
  NRF24_CLEAR_BIT(tmp, NRF24_PRIM_RX);
  if (NRF24_WriteRegister(NRF24_REG_CONFIG, tmp) != true)
    return false;

  // standby mode -> TX mode
  nrf24_set_CE_High();
  NRF24_Struct.mode = MODE_TX;
  // nrf24_delay_us(15);
  // nrf24_set_CE_Low();
  // NRF24_Struct.mode = MODE_STANDBY;
  return true;
}

bool NRF24_PowerUpRx(void)
{
  bool ret;
  NRF24_ClearInterrupts();

  uint8_t tmp = NRF24_ReadRegister(NRF24_REG_CONFIG);

  // shutdown mode -> standby mode
  if (NRF24_CHECK_BIT(tmp, NRF24_PWR_UP) == 0)
  {
    ret = NRF24_PowerStandby();
    nrf24_delay_ms(3);
    if (ret != true)
      return false;
  }

  // TX mode or RX mode -> RX mode
  if (nrf24_get_CE() == 1)
  {
    // TX mode -> standby mode
    if (NRF24_CHECK_BIT(tmp, NRF24_PRIM_RX) == 0)
    {
      if (NRF24_PowerStandby() != true)
        return false;
    }
    // RX mode already, do nothing
    else
      return true;
  }

  // set PRIM_RX = 1
  NRF24_SET_BIT(tmp, NRF24_PWR_UP);
  NRF24_SET_BIT(tmp, NRF24_PRIM_RX);
  if (NRF24_WriteRegister(NRF24_REG_CONFIG, tmp) != true)
    return false;

  // standby mode -> RX mode
  nrf24_set_CE_High();
  NRF24_Struct.mode = MODE_RX;
  // nrf24_delay_us(15);
  return true;
}

NRF24_mode_t NRF24_GetMode(void)
{
  uint8_t reg_val = NRF24_ReadRegister(NRF24_REG_CONFIG);

  // shutdown mode
  if (NRF24_CHECK_BIT(reg_val, NRF24_PWR_UP) == 0)
  {
    NRF24_Struct.mode = MODE_POWERDOWN;
  }
  else
  {
    // standby mode
    if (nrf24_get_CE() == 0)
    {
      NRF24_Struct.mode = MODE_STANDBY;
    }
    else
    {
      // receive mode
      if (NRF24_CHECK_BIT(reg_val, NRF24_PRIM_RX) == 1)
      {
        NRF24_Struct.mode = MODE_RX;
      }
      // transmit mode
      else
      {
        NRF24_Struct.mode = MODE_TX;
      }
    }
  }

  return NRF24_Struct.mode;
}

NRF24_status_t NRF24_GetStatus(void)
{
  /* First received byte is always status register */
  // uint8_t status = NRF24_SPI_RecvMulti(NRF24_CMD_NOP, NULL, 0);
  uint8_t status = NRF24_ReadRegister(NRF24_REG_STATUS);

  // RF fifo
  if (NRF24_CHECK_BIT(status, NRF24_TX_FULL))
    NRF24_Struct.status.is_tx_full = true;
  else
    NRF24_Struct.status.is_tx_full = false;

  if (NRF24_CHECK_BIT(status, NRF24_MAX_RT))
    NRF24_Struct.status.is_max_rt = true;
  else
    NRF24_Struct.status.is_max_rt = false;

  if (NRF24_CHECK_BIT(status, NRF24_TX_DS))
  {
    NRF24_Struct.status.is_tx_ds = true;
  }
  else
    NRF24_Struct.status.is_tx_ds = false;

  if (NRF24_CHECK_BIT(status, NRF24_RX_DR))
    NRF24_Struct.status.is_rx_dr = true;
  else
    NRF24_Struct.status.is_rx_dr = false;

  NRF24_Struct.status.rx_p_no = (status >> 1) & 0x07;

  return NRF24_Struct.status;
}

NRF24_FifoStatus_t NRF24_Get_Fifo_Status(void)
{
  uint8_t fifo_status = NRF24_ReadRegister(NRF24_REG_FIFO_STATUS);

  if (NRF24_CHECK_BIT(fifo_status, NRF24_RX_EMPTY))
    NRF24_Struct.fifo_status.is_rx_empty = true;
  else
    NRF24_Struct.fifo_status.is_rx_empty = false;

  if (NRF24_CHECK_BIT(fifo_status, NRF24_RX_FULL))
    NRF24_Struct.fifo_status.is_rx_full = true;
  else
    NRF24_Struct.fifo_status.is_rx_full = false;

  if (NRF24_CHECK_BIT(fifo_status, NRF24_TX_EMPTY))
    NRF24_Struct.fifo_status.is_tx_empty = true;
  else
    NRF24_Struct.fifo_status.is_tx_empty = false;

  if (NRF24_CHECK_BIT(fifo_status, NRF24_TX_FIFO_FULL))
    NRF24_Struct.fifo_status.is_tx_full = true;
  else
    NRF24_Struct.fifo_status.is_tx_full = false;

  if (NRF24_CHECK_BIT(fifo_status, NRF24_TX_REUSE))
    NRF24_Struct.fifo_status.is_tx_reuse = true;
  else
    NRF24_Struct.fifo_status.is_tx_reuse = false;

  return NRF24_Struct.fifo_status;
}
uint8_t NRF24_Get_Retry_Count(void)
{
  /* Low 4 bits */
  uint8_t cnt = NRF24_ReadRegister(NRF24_REG_OBSERVE_TX) & 0x0F;
  return cnt;
}

/*
  check a channel is free or not
  return true, the channel is free, received power is above -64dBm ( Si24R1 is -60dBm )
  return false, the channel is busy, received power is less than -64dBm ( Si24R1 is -60dBm )
*/
bool NRF24_is_channel_free(uint8_t channel)
{
  if (channel != NRF24_Struct.Channel)
  {
    NRF24_PowerStandby();
    NRF24_SetChannel(channel);
  }

  if (NRF24_Struct.mode != MODE_RX)
  {
    NRF24_PowerUpRx();
  }

  // wait 170us at least
  nrf24_delay_us(180);

  if (NRF24_ReadRegister(NRF24_REG_RPD) == 1)
    return false;

  return true;
}

/*
  before call NRF24_Init(), you need initialize NRF24_Struct, register callback functions for it.
  If AUTO_ACK enabled, the rx pipe 0 is not recommended as the first main rx pipe.
  please refer struct declaration NRF24_t in nrf24l01p.h

  uint8_t channel,
  NRF24_Power_t power,    // tx output power
  NRF24_DR_t dataRate,    // data rate
  uint8_t rx_pipe,        // rx pipe number, 0~5
  uint8_t *tx_address,    // tx address, 5 bytes
  uint8_t *rx_address,    // rx address, 5 bytes
  NRF24_ACK_t ack,        // if use NO_ACK mode, the next param delay & retry_count are invalid
  uint8_t delay,          // Auto Retransmit Delay Time = ( delay +1 )* 250us
  uint8_t retry_count     // Auto Retransmit Count, 0~15, 0 is disabled
  NRF24_cb_t *call_back   // interrupts call back functions
*/
bool NRF24_Init(uint8_t channel,
                NRF24_Power_t power,
                NRF24_DR_t dataRate,
                uint8_t rx_pipe,
                uint8_t *tx_address,
                uint8_t *rx_address,
                NRF24_ACK_t ack,
                uint8_t delay,
                uint8_t retry_count,
                NRF24_cb_t *call_back)
{
  nrf24_hal_init();
  //  nrf24_set_CE_Low();
  // Fill structure
  NRF24_Struct.mode = MODE_POWERDOWN;
  NRF24_Struct.status.is_max_rt = 0;
  NRF24_Struct.status.is_rx_dr = 0;
  NRF24_Struct.status.is_tx_ds = 0;
  NRF24_Struct.status.is_tx_full = 0;
  NRF24_Struct.status.rx_p_no = 0;

  NRF24_Struct.callback = call_back;

  /* Reset nRF24L01+ to power on registers values */
  NRF24_SoftwareReset();

  /* Go to Standby mode */
  if (NRF24_PowerStandby() != true)
    return false;

  /* Channel select */
  if (NRF24_SetChannel(channel) != true)
    return false;
  /* Set RF settings */
  if (NRF24_Set_DR_Power(dataRate, power) != true)
    return false;

  // no ack settings
  if (ack == NO_ACK)
  {
    NRF24_Struct.ack = NO_ACK;
    NRF24_SetAutoRetry(delay, 0);
    // Dynamic length enable
    NRF24_WriteRegister(NRF24_REG_FEATURE, 0x05);
    // Config CRC as 2bytes, disable max retry interrupt
    NRF24_WriteRegister(NRF24_REG_CONFIG, (1 << NRF24_EN_CRC) | (1 << NRF24_CRCO) | (1 << NRF24_MASK_MAX_RT));
  }
  // auto ack settings
  else
  {
    NRF24_Struct.ack = AUTO_ACK;
    // set rx pipe 0 address as same as the tx address
    //    NRF24_SetRxAddress( 0, tx_address );
    /* Auto retransmit delay */
    NRF24_SetAutoRetry(delay, retry_count);
    // Dynamic length enable
    NRF24_WriteRegister(NRF24_REG_FEATURE, 0x04);
    // Config CRC as 2bytes, enable max retry interrupt
    NRF24_WriteRegister(NRF24_REG_CONFIG, (1 << NRF24_EN_CRC) | (1 << NRF24_CRCO) | (0 << NRF24_MASK_MAX_RT));
  }

  // set address width 5 bytes
  NRF24_WriteRegister(NRF24_REG_SETUP_AW, 0x03);

  // set rx address
  NRF24_SetRxAddress(rx_pipe, rx_address);

  // set tx address
  NRF24_SetTxAddress(tx_address);

  // enable dynamic payload length for all pipes
  NRF24_WriteRegister(NRF24_REG_DYNPD, (1 << NRF24_DPL_P0) | (1 << NRF24_DPL_P1) | (1 << NRF24_DPL_P2) | (1 << NRF24_DPL_P3) | (1 << NRF24_DPL_P4) | (1 << NRF24_DPL_P5));

  /* Clear FIFOs */
  NRF24_Flush_Tx();
  NRF24_Flush_Rx();

  /* Clear interrupts */
  NRF24_ClearInterrupts();

  /* Go to Standby mode */
  if (NRF24_PowerStandby() != true)
    return false;

  /* Return OK */
  return true;
}

/*
  data: the data that you want to send
  size：the length of data
*/
bool NRF24_Transmit(uint8_t *data, uint8_t size)
{
  //  NRF24_mode_t mode = NRF24_GetMode();
  //  NRF24_status_t status = NRF24_GetStatus();
  NRF24_FifoStatus_t fifo_stat = NRF24_Get_Fifo_Status();

  // when tx fifo is full, retrun false
  if (fifo_stat.is_tx_full == 1)
  {
    NRF24_ClearInterrupts();
    if (NRF24_Flush_Tx() != true)
      return false;
  }

  // switch to standby mode
  if (NRF24_PowerStandby() != true)
    return false;

  uint8_t cmd;
  if (NRF24_Struct.ack == AUTO_ACK)
  {
    // update rx & tx adress
    if (NRF24_SetRxAddress(NRF24_Struct.rx_pipe, NRF24_Struct.rx_address) != true)
      return false;
    if (NRF24_SetTxAddress(NRF24_Struct.tx_address) != true)
      return false;

    // set rx pipe 0 address as same as tx address
    if (NRF24_WriteRegisterMulti(NRF24_REG_RX_ADDR_P0, NRF24_Struct.tx_address, 5) != true)
      return false;

    cmd = NRF24_CMD_TX_PAYLOAD;
  }
  else
    cmd = NRF24_CMD_TX_PAYLOAD_NOACK;
  // fill TX fifo
  if (NRF24_SPI_SendMulti(cmd, data, size) != true)
    return false;
  //switch to TX mode and start transmit now
  if (NRF24_PowerUpTx() != true)
    return false;

  return true;
}

// Called by nrf24 IRQ pin interrupt function only
void NRF24_Irq_callback(void)
{
  NRF24_status_t status = NRF24_GetStatus();
  NRF24_FifoStatus_t fifo_status = NRF24_Get_Fifo_Status();

  // tx done
  if (status.is_tx_ds == true)
  {
    NRF24_Struct.status.is_tx_ds = false;
    NRF24_Struct.callback->TX_done_cb();
  }
  // max tx retry count
  if (status.is_max_rt == true)
  {
    NRF24_Struct.status.is_max_rt = false;
    nrf24_set_CE_Low();
    NRF24_Struct.mode = MODE_STANDBY;
    NRF24_Struct.callback->Max_retry_cb();
  }
  // tx fifo is full
  if (status.is_tx_full == true)
  {
    NRF24_Struct.status.is_tx_full = false;
    // if needed, you must clear tx fifo with NRF24_Flush_Tx()
    NRF24_Struct.callback->TX_full_cb();
  }
  // rx fifo is filled by data
  if (status.is_rx_dr == true)
  {
    NRF24_Struct.mode = MODE_RX;
    NRF24_Struct.status.is_rx_dr = false;

    uint8_t i = 0;
    NRF24_fifo_t fifo[3] = {0};
    // get fifo status and data size, then read data back
    while (NRF24_Get_Fifo_Status().is_rx_empty != true)
    {
      fifo[i].size = NRF24_Get_Received_Size();
      if (fifo[i].size > 0)
      {
        if (NRF24_ReceiveData(fifo[i].packet, fifo[i].size) != true)
          return;
      }
    }
    NRF24_Struct.callback->RX_data_ready_cb(fifo, status.rx_p_no);
  }

  NRF24_ClearInterrupts();
}
