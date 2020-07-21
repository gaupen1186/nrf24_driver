//﻿/**
//* |----------------------------------------------------------------------
//* | Copyright (C) gaupen1186@gmail.com, 2018
//* |
//* | This program is free software: you can redistribute it and/or modify
//* | it under the terms of the GNU General Public License as published by
//* | the Free Software Foundation, either version 3 of the License, or
//* | any later version.
//* |
//* | This program is distributed in the hope that it will be useful,
//* | but WITHOUT ANY WARRANTY; without even the implied warranty of
//* | MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//* | GNU General Public License for more details.
//* |
//* | You should have received a copy of the GNU General Public License
//* | along with this program.  If not, see <http://www.gnu.org/licenses/>.
//* |----------------------------------------------------------------------
//*/

/*******************************************************************************
*/
#include <stdint.h>
#include <string.h>

#include "nrf24_hal.h"
#include "nrf24l01p.h"


#ifndef __FILENAME__
#define __FILENAME__ (strrchr(__FILE__, '\\') ? (strrchr(__FILE__, '\\') + 1) : __FILE__)
#endif // __FILENAME__

#define NRF24_CHECK_BIT(reg, bit) ((reg >> bit) & 0x01)
#define NRF24_SET_BIT(reg, bit) ((reg) |= (1 << bit))
#define NRF24_CLEAR_BIT(reg, bit) ((reg) &= ~(1 << bit))

/*******************************************************************************
* NRF structure
*/
// static NRF24_t NRF24_Struct = { 0 };

/*******************************************************************************
* Private functions
*/
static bool NRF24_SoftwareReset(NRF24_t *nrf24)
{
  uint8_t data[5];
  bool ret = true;

  if(NRF24_WriteRegister(NRF24_REG_CONFIG, NRF24_REG_DEFAULT_VAL_CONFIG) != true)
    ret = false;
  if(NRF24_WriteRegister(NRF24_REG_EN_AA, NRF24_REG_DEFAULT_VAL_EN_AA) != true)
    ret = false;
  if(NRF24_WriteRegister(NRF24_REG_EN_RXADDR, NRF24_REG_DEFAULT_VAL_EN_RXADDR) != true)
    ret = false;
  if(NRF24_WriteRegister(NRF24_REG_SETUP_AW, NRF24_REG_DEFAULT_VAL_SETUP_AW) != true)
    ret = false;
  if(NRF24_WriteRegister(NRF24_REG_SETUP_RETR, NRF24_REG_DEFAULT_VAL_SETUP_RETR) != true)
    ret = false;
  if(NRF24_WriteRegister(NRF24_REG_RF_CH, NRF24_REG_DEFAULT_VAL_RF_CH) != true)
    ret = false;
  if(NRF24_WriteRegister(NRF24_REG_RF_SETUP, NRF24_REG_DEFAULT_VAL_RF_SETUP) != true)
    ret = false;
  if(NRF24_WriteRegister(NRF24_REG_STATUS, NRF24_REG_DEFAULT_VAL_STATUS) != true)
    ret = false;
  if(NRF24_WriteRegister(NRF24_REG_OBSERVE_TX, NRF24_REG_DEFAULT_VAL_OBSERVE_TX) != true)
    ret = false;
  if(NRF24_WriteRegister(NRF24_REG_RPD, NRF24_REG_DEFAULT_VAL_RPD) != true)
    ret = false;

  //P0
  data[0] = NRF24_REG_DEFAULT_VAL_RX_ADDR_P0_0;
  data[1] = NRF24_REG_DEFAULT_VAL_RX_ADDR_P0_1;
  data[2] = NRF24_REG_DEFAULT_VAL_RX_ADDR_P0_2;
  data[3] = NRF24_REG_DEFAULT_VAL_RX_ADDR_P0_3;
  data[4] = NRF24_REG_DEFAULT_VAL_RX_ADDR_P0_4;
  if(NRF24_WriteRegisterMulti(NRF24_REG_RX_ADDR_P0, data, 5) != true)
    ret = false;

  //P1
  data[0] = NRF24_REG_DEFAULT_VAL_RX_ADDR_P1_0;
  data[1] = NRF24_REG_DEFAULT_VAL_RX_ADDR_P1_1;
  data[2] = NRF24_REG_DEFAULT_VAL_RX_ADDR_P1_2;
  data[3] = NRF24_REG_DEFAULT_VAL_RX_ADDR_P1_3;
  data[4] = NRF24_REG_DEFAULT_VAL_RX_ADDR_P1_4;
  if(NRF24_WriteRegisterMulti(NRF24_REG_RX_ADDR_P1, data, 5) != true)
    ret = false;

  if(NRF24_WriteRegister(NRF24_REG_RX_ADDR_P2, NRF24_REG_DEFAULT_VAL_RX_ADDR_P2) != true)
    ret = false;
  if(NRF24_WriteRegister(NRF24_REG_RX_ADDR_P3, NRF24_REG_DEFAULT_VAL_RX_ADDR_P3) != true)
    ret = false;
  if(NRF24_WriteRegister(NRF24_REG_RX_ADDR_P4, NRF24_REG_DEFAULT_VAL_RX_ADDR_P4) != true)
    ret = false;
  if(NRF24_WriteRegister(NRF24_REG_RX_ADDR_P5, NRF24_REG_DEFAULT_VAL_RX_ADDR_P5) != true)
    ret = false;

  //TX
  data[0] = NRF24_REG_DEFAULT_VAL_TX_ADDR_0;
  data[1] = NRF24_REG_DEFAULT_VAL_TX_ADDR_1;
  data[2] = NRF24_REG_DEFAULT_VAL_TX_ADDR_2;
  data[3] = NRF24_REG_DEFAULT_VAL_TX_ADDR_3;
  data[4] = NRF24_REG_DEFAULT_VAL_TX_ADDR_4;
  if(NRF24_WriteRegisterMulti(NRF24_REG_TX_ADDR, data, 5) != true)
    ret = false;

  if(NRF24_WriteRegister(NRF24_REG_RX_PW_P0, NRF24_REG_DEFAULT_VAL_RX_PW_P0) != true)
    ret = false;
  if(NRF24_WriteRegister(NRF24_REG_RX_PW_P1, NRF24_REG_DEFAULT_VAL_RX_PW_P1) != true)
    ret = false;
  if(NRF24_WriteRegister(NRF24_REG_RX_PW_P2, NRF24_REG_DEFAULT_VAL_RX_PW_P2) != true)
    ret = false;
  if(NRF24_WriteRegister(NRF24_REG_RX_PW_P3, NRF24_REG_DEFAULT_VAL_RX_PW_P3) != true)
    ret = false;
  if(NRF24_WriteRegister(NRF24_REG_RX_PW_P4, NRF24_REG_DEFAULT_VAL_RX_PW_P4) != true)
    ret = false;
  if(NRF24_WriteRegister(NRF24_REG_RX_PW_P5, NRF24_REG_DEFAULT_VAL_RX_PW_P5) != true)
    ret = false;
  if(NRF24_WriteRegister(NRF24_REG_FIFO_STATUS, NRF24_REG_DEFAULT_VAL_FIFO_STATUS) != true)
    ret = false;
  if(NRF24_WriteRegister(NRF24_REG_DYNPD, NRF24_REG_DEFAULT_VAL_DYNPD) != true)
    ret = false;
  if(NRF24_WriteRegister(NRF24_REG_FEATURE, NRF24_REG_DEFAULT_VAL_FEATURE) != true)
    ret = false;

  if(ret == true)
  {
    nrf24->mode = MODE_POWERDOWN;
  }

  return ret;
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
  {
    if(NRF24_ReadRegister(i, &regs[i]) != true)
    {
      NRF24_LOG_ERROR("Func:[%s] Failed! line = %d\r\n",
                    (uint32_t)__func__, (uint32_t)__LINE__);
    }
  }
}

#if ENABLE_DYNAMIC_PALOAD_LENGTH
static uint8_t NRF24_Get_Received_Size(void)
{
  uint8_t size = 0;
  if(NRF24_SPI_RecvMulti(NRF24_CMD_RX_PL_WID, &size, 1) != true)
  {
    NRF24_LOG_ERROR("func:[%s] failed! %s, %d.\r\n", (uint32_t)__func__, (uint32_t)__FILENAME__, __LINE__);
  }

  return size;
}
#endif // ENABLE_DYNAMIC_PALOAD_LENGTH

// before call this function, you need call NRF24_Get_Received_Size() first.
static bool NRF24_ReceiveData(uint8_t *data, uint8_t size)
{
  if(data == NULL || size == 0 || size > 32)
  {
    NRF24_LOG_ERROR("func:[%s] params error! %s, line %d\r\n",
                  (uint32_t)__func__, (uint32_t)__FILENAME__, (uint32_t)__LINE__);
    return false;
  }

  /* Read payload */
  if(NRF24_SPI_RecvMulti(NRF24_CMD_RX_PAYLOAD, data, size) != true)
  {
    NRF24_LOG_ERROR("func:[%s] failed! %s, %d.\r\n",
                    (uint32_t)__func__, (uint32_t)__FILENAME__, __LINE__);
    return false;
  }

//  NRF24_LOG_WARNING("NRF24_ReceiveData, SIZE= %d\r\n", size);
  return true;

  /* Reset status register, clear RX_DR interrupt flag */
  // NRF24_WriteRegister(NRF24_REG_STATUS, (1 << NRF24_RX_DR));
}

/*
bool NRF24_SetAddressWidths(NRF24_t *nrf24, NRF24_AW_t widths )
{
  bool ret;
  if( widths == NRF24_AW_3BYTES )
  {
    nrf24->aw = NRF24_AW_3BYTES;
  }
  else( widths == NRF24_AW_4BYTES )
  {
    nrf24->aw = NRF24_AW_4BYTES;
  }
  else
  {
    nrf24->aw = NRF24_AW_5BYTES;
  }

  ret = NRF24_WriteRegister(NRF24_REG_SETUP_AW, nrf24->aw );
  return ret;
}
*/

bool NRF24_SetChannel(NRF24_t *nrf24, uint8_t channel)
{
  bool ret = true;
  if(nrf24->Channel == channel)
    return true;

  // goto standby before writing to the configuration registers
  if( NRF24_PowerStandby(nrf24) != true )
  {
    NRF24_LOG_ERROR("func:[%s] failed! %s, %d.\r\n", (uint32_t)__func__, (uint32_t)__FILENAME__, __LINE__);
    return false;
  }

  /* Store new channel setting */
  nrf24->Channel = channel;

  /* Write channel */
  ret = NRF24_WriteRegister(NRF24_REG_RF_CH, channel);
  if(ret != true)
  {
    NRF24_LOG_ERROR("func:[%s] failed! %s, %d.\r\n", (uint32_t)__func__, (uint32_t)__FILENAME__, __LINE__);
  }

  return ret;
}

bool NRF24_GetChannel(NRF24_t *nrf24, uint8_t *channel)
{
  /* Read channel */
  uint8_t ch;

  if(NRF24_ReadRegister(NRF24_REG_RF_CH, &ch) != true)
  {
    NRF24_LOG_ERROR("Func:[%s] Failed! line = %d\r\n",
                  (uint32_t)__func__, (uint32_t)__LINE__);
    return false;
  }

  if(nrf24->Channel != ch)
  {
    nrf24->Channel = ch;
  }

  *channel = ch;
  return true;
}

bool NRF24_Set_DR_Power(NRF24_t *nrf24, NRF24_DR_t dataRate, NRF24_Power_t power)
{
  // goto standby before writing to the configuration registers
  if( NRF24_PowerStandby(nrf24) != true )
  {
    NRF24_LOG_ERROR("func:[%s] failed! %s, %d.\r\n", (uint32_t)__func__, (uint32_t)__FILENAME__, __LINE__);
    return false;
  }

  uint8_t tmp = 0;
  nrf24->DataRate = dataRate;
  nrf24->Power = power;

  if (dataRate == NRF24_DataRate_2M)
  {
    NRF24_SET_BIT(tmp, NRF24_RF_DR_HIGH);
  }
  else if (dataRate == NRF24_DataRate_250k)
  {
    NRF24_SET_BIT(tmp, NRF24_RF_DR_LOW);
  }

#if __SI24R1__
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
#else   // NRF24L01P
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
  if(ret != true)
  {
    NRF24_LOG_ERROR("func:[%s] failed! %s, %d.\r\n", (uint32_t)__func__, (uint32_t)__FILENAME__, __LINE__);
  }

  return ret;
}

// Constant carrier wave output for testing, you need to call NRF24_PowerUpTx after this function
bool NRF24_Set_Constant_Output(NRF24_t *nrf24, uint8_t channel, NRF24_Power_t power)
{
  uint8_t tmp = 0;

  // goto standby before writing to the configuration registers
  if( NRF24_PowerStandby(nrf24) != true )
  {
    NRF24_LOG_ERROR("func:[%s] failed! %s, %d.\r\n", (uint32_t)__func__, (uint32_t)__FILENAME__, __LINE__);
    return false;
  }

#if __SI24R1__
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
#else   // NRF24L01P
  if (power == NRF24_Power_0dBm)
    tmp |= 3 << NRF24_RF_PWR;
  else if (power == NRF24_Power_N6dBm)
    tmp |= 2 << NRF24_RF_PWR;
  else if (power == NRF24_Power_N12dBm)
    tmp |= 1 << NRF24_RF_PWR;
  else if (power == NRF24_Power_N18dBm)
    tmp |= 0 << NRF24_RF_PWR;
#endif

  if( NRF24_SetChannel(nrf24, channel) != true )
    return false;

  NRF24_SET_BIT( tmp, NRF24_CONT_WAVE );
  NRF24_SET_BIT( tmp, NRF24_PLL_LOCK );

  bool ret = NRF24_WriteRegister(NRF24_REG_RF_SETUP, tmp);
  if(ret != true)
  {
    NRF24_LOG_ERROR("func:[%s] failed! %s, %d.\r\n", (uint32_t)__func__, (uint32_t)__FILENAME__, __LINE__);
  }

  nrf24->Power = power;

  return ret;
}


/* enable or disable pipe x Auto ACK
*/
bool NRF24_Set_Pipe_AutoAck(NRF24_t *nrf24, bool enable, uint8_t pipe_num)
{
  // goto standby before writing to the configuration registers
  if( NRF24_PowerStandby(nrf24) != true )
  {
    NRF24_LOG_ERROR("func:[%s] failed! %s, %d.\r\n",
                    (uint32_t)__func__, (uint32_t)__FILENAME__, __LINE__);
    return false;
  }

  // read back auto ack of pipe x
  uint8_t enaa_px;
  if(NRF24_ReadRegister(NRF24_REG_EN_AA, &enaa_px) != true)
  {
    NRF24_LOG_ERROR("Func:[%s] Failed! line = %d\r\n",
                  (uint32_t)__func__, (uint32_t)__LINE__);
    return false;
  }

  if(enable == true)
  {
    NRF24_SET_BIT(enaa_px, pipe_num);
  }
  else
  {
    NRF24_CLEAR_BIT(enaa_px, pipe_num);
  }
  // write auto ack register
  if (NRF24_WriteRegister(NRF24_REG_EN_AA, enaa_px) != true)
  {
    NRF24_LOG_ERROR("func:[%s] failed! %s, %d.\r\n", (uint32_t)__func__, (uint32_t)__FILENAME__, __LINE__);
    return false;
  }

  return true;
}

/* enable or disable pipe x
*/
bool NRF24_Set_Pipe_Enable(NRF24_t *nrf24, bool enable, uint8_t pipe_num)
{
  // goto standby before writing to the configuration registers
  if( NRF24_PowerStandby(nrf24) != true )
  {
    NRF24_LOG_ERROR("func:[%s] failed! %s, %d.\r\n",
                    (uint32_t)__func__, (uint32_t)__FILENAME__, __LINE__);
    return false;
  }

  // read back enable status of pipe x
  uint8_t en_rx_px;
  if(NRF24_ReadRegister(NRF24_REG_EN_RXADDR, &en_rx_px) != true)
  {
    NRF24_LOG_ERROR("Func:[%s] Failed! line = %d\r\n",
                  (uint32_t)__func__, (uint32_t)__LINE__);
    return false;
  }

  if(enable == true)
  {
    NRF24_SET_BIT(en_rx_px, pipe_num);
  }
  else
  {
    NRF24_CLEAR_BIT(en_rx_px, pipe_num);
  }
  /* Write to reg */
  bool ret = NRF24_WriteRegister(NRF24_REG_EN_RXADDR, en_rx_px);
  if (ret != true)
  {
    NRF24_LOG_ERROR("func:[%s] failed! %s, %d.\r\n", (uint32_t)__func__, (uint32_t)__FILENAME__, __LINE__);
    return false;
  }

  return true;
}

/*
  pipe_number = 0 ~ 5, *rx_data is a 5 bytes arry
  if pipe_number = 2~5, this func will only set the first byte of *rx_addr
*/
bool NRF24_SetRxAddress(NRF24_t *nrf24, uint8_t pipe_number, uint8_t *rx_addr)
{
  uint8_t reg, *data, size;

  switch (pipe_number)
  {
  case 0:
    {
      if(memcmp(nrf24->rx0_addr, rx_addr, 5) != 0)
      {
        memcpy(nrf24->rx0_addr, rx_addr, 5);
        reg = NRF24_REG_RX_ADDR_P0;
        data = nrf24->rx0_addr;
        size = 5;
      }
      else
        return true;
    }break;

  case 1:
    {
      if(memcmp(nrf24->rx1_addr, rx_addr, 5) != 0)
      {
        memcpy(nrf24->rx1_addr, rx_addr, 5);
        reg = NRF24_REG_RX_ADDR_P1;
        data = nrf24->rx1_addr;
        size = 5;
      }
      else
        return true;
    }break;

  case 2:
    {
      if(nrf24->rx2_addr != rx_addr[0])
      {
        nrf24->rx2_addr = rx_addr[0];
        reg = NRF24_REG_RX_ADDR_P2;
        data = &nrf24->rx2_addr;
        size = 1;
      }
      else
        return true;
    }break;

  case 3:
    {
      if(nrf24->rx3_addr != rx_addr[0])
      {
        nrf24->rx3_addr = rx_addr[0];
        reg = NRF24_REG_RX_ADDR_P3;
        data = &nrf24->rx3_addr;
        size = 1;
      }
      else
        return true;
    }break;

  case 4:
    {
      if(nrf24->rx4_addr != rx_addr[0])
      {
        nrf24->rx4_addr = rx_addr[0];
        reg = NRF24_REG_RX_ADDR_P4;
        data = &nrf24->rx4_addr;
        size = 1;
      }
      else
        return true;
    }break;

  case 5:
    {
      if(nrf24->rx5_addr != rx_addr[0])
      {
        nrf24->rx5_addr = rx_addr[0];
        reg = NRF24_REG_RX_ADDR_P5;
        data = &nrf24->rx5_addr;
        size = 1;
      }
      else
        return true;
    }break;
  }

  // goto standby before writing to the configuration registers
  if( NRF24_PowerStandby(nrf24) != true )
  {
    NRF24_LOG_ERROR("func:[%s] failed! %s, %d.\r\n", (uint32_t)__func__, (uint32_t)__FILENAME__, __LINE__);
    return false;
  }

  // write data to reg
  if (NRF24_WriteRegisterMulti(reg, data, size) != true)
  {
    NRF24_LOG_ERROR("func:[%s] failed! %s, %d.\r\n", (uint32_t)__func__, (uint32_t)__FILENAME__, __LINE__);
    return false;
  }

  return true;
}

bool NRF24_GetRxAddress(uint8_t pipe_number, uint8_t *addr)
{
  bool ret;

  switch (pipe_number)
  {
  case 0:
    // higher 4 bytes address as same as NRF24_REG_RX_ADDR_P0
    ret = NRF24_ReadRegisterMulti(NRF24_REG_RX_ADDR_P0, addr, 5);
    break;

  case 1:
    ret = NRF24_ReadRegisterMulti(NRF24_REG_RX_ADDR_P1, addr, 5);
    break;

  case 2:
    ret = NRF24_ReadRegisterMulti(NRF24_REG_RX_ADDR_P2, &addr[4], 1);
    break;

  case 3:
    ret = NRF24_ReadRegisterMulti(NRF24_REG_RX_ADDR_P3, &addr[4], 1);
    break;

  case 4:
    ret = NRF24_ReadRegisterMulti(NRF24_REG_RX_ADDR_P4, &addr[4], 1);
    break;

  case 5:
    ret = NRF24_ReadRegisterMulti(NRF24_REG_RX_ADDR_P5, &addr[4], 1);
    break;
  }

  return ret;
}

bool NRF24_SetTxAddress(NRF24_t *nrf24, uint8_t *tx_addr)
{
   if(memcmp(nrf24->tx_addr, tx_addr, 5) == 0)
     return true;

  memcpy(nrf24->tx_addr, tx_addr, 5);

  // goto standby before writing to the configuration registers
  if( NRF24_PowerStandby(nrf24) != true )
  {
    NRF24_LOG_ERROR("func:[%s] failed! %s, %d.\r\n", (uint32_t)__func__, (uint32_t)__FILENAME__, __LINE__);
    return false;
  }

  bool ret = NRF24_WriteRegisterMulti(NRF24_REG_TX_ADDR, tx_addr, 5);
  if(ret != true)
  {
    NRF24_LOG_ERROR("func:[%s] failed! %s, %d.\r\n", (uint32_t)__func__, (uint32_t)__FILENAME__, __LINE__);
    return false;
  }

//  NRF24_LOG_INFO("NRF24_SetTxAddress = 0x%x, 0x%x, 0x%x, 0x%x, 0x%x\r\n",
//                 tx_addr[0], tx_addr[1], tx_addr[2], tx_addr[3], tx_addr[4]);
  return ret;
}

bool NRF24_GetTxAddress(uint8_t *addr)
{
  bool ret;
  ret = NRF24_ReadRegisterMulti(NRF24_REG_TX_ADDR, addr, 5);
  if(ret != true)
  {
    return false;
  }
  return true;
}

/*
  delay = 0~15 ( 250us ~ 4000us ), delay time = ( delay +1 )* 250us
  count = 0~15, Auto Retransmit Count, 0 is disabled
*/
bool NRF24_SetAutoRetry(NRF24_t *nrf24, uint8_t delay, uint8_t count)
{
  // goto standby before writing to the configuration registers
  if( NRF24_PowerStandby(nrf24) != true )
  {
    NRF24_LOG_ERROR("func:[%s] failed! %s, %d.\r\n", (uint32_t)__func__, (uint32_t)__FILENAME__, __LINE__);
    return false;
  }

  nrf24->delay = delay;
  nrf24->retry_count = count;
  bool ret = NRF24_WriteRegister(NRF24_REG_SETUP_RETR, (delay << 4) | count);
  if(ret != true)
  {
    NRF24_LOG_ERROR("func:[%s] failed! %s, %d.\r\n", (uint32_t)__func__, (uint32_t)__FILENAME__, __LINE__);
    return false;
  }

  return ret;
}

/* Flush FIFOs */
bool NRF24_Flush_Tx(void)
{
  bool ret = NRF24_SPI_SendMulti(NRF24_CMD_FLUSH_TX, NULL, 0);
  if(ret != true)
  {
    NRF24_LOG_ERROR("func:[%s] failed! %s, %d.\r\n", (uint32_t)__func__, (uint32_t)__FILENAME__, __LINE__);
    return false;
  }

  return ret;
}

bool NRF24_Flush_Rx(void)
{
  bool ret = NRF24_SPI_SendMulti(NRF24_CMD_FLUSH_RX, NULL, 0);
  if(ret != true)
  {
    NRF24_LOG_ERROR("func:[%s] failed! %s, %d.\r\n", (uint32_t)__func__, (uint32_t)__FILENAME__, __LINE__);
    return false;
  }

  return ret;
}

bool NRF24_PowerDown(NRF24_t *nrf24)
{
  uint8_t tmp;
  if(NRF24_ReadRegister(NRF24_REG_CONFIG, &tmp) != true)
  {
    NRF24_LOG_ERROR("Func:[%s] Failed! line = %d\r\n",
                  (uint32_t)__func__, (uint32_t)__LINE__);
    return false;
  }

  NRF24_CLEAR_BIT(tmp, NRF24_PWR_UP);
  bool ret = NRF24_WriteRegister(NRF24_REG_CONFIG, tmp);
  if(ret != true)
  {
    NRF24_LOG_ERROR("func:[%s] failed! %s, %d.\r\n", (uint32_t)__func__, (uint32_t)__FILENAME__, __LINE__);
    return false;
  }

  nrf24->mode = MODE_POWERDOWN;
  return ret;
}

bool NRF24_PowerStandby(NRF24_t *nrf24)
{
  uint8_t tmp;
  if(NRF24_ReadRegister(NRF24_REG_CONFIG, &tmp) != true)
  {
    NRF24_LOG_ERROR("Func:[%s] Failed! line = %d\r\n",
                  (uint32_t)__func__, (uint32_t)__LINE__);
    return false;
  }

  // from RX or TX mode to standby mode
  if (NRF24_CHECK_BIT(tmp, NRF24_PWR_UP) == 1)
  {
    nrf24_set_CE_Low();
    nrf24->mode = MODE_STANDBY;
    return true;
  }
  //else: from shutdown mode to standby mode
  nrf24_set_CE_Low();
  nrf24->mode = MODE_STANDBY;
  bool ret = NRF24_WriteRegister(NRF24_REG_CONFIG, NRF24_SET_BIT(tmp, NRF24_PWR_UP));
  if(ret != true)
  {
    NRF24_LOG_ERROR("func:[%s] failed! %s, %d.\r\n", (uint32_t)__func__, (uint32_t)__FILENAME__, __LINE__);
    return false;
  }
  nrf24_delay_ms(3);
  return ret;
}

bool NRF24_PowerUpTx(NRF24_t *nrf24)
{
  bool ret;
//  NRF24_ClearInterrupts();

  uint8_t tmp;
  if(NRF24_ReadRegister(NRF24_REG_CONFIG, &tmp) != true)
  {
    NRF24_LOG_ERROR("Func:[%s] Failed! line = %d\r\n",
                  (uint32_t)__func__, (uint32_t)__LINE__);
    return false;
  }

  // shutdown mode -> standby mode
  if (NRF24_CHECK_BIT(tmp, NRF24_PWR_UP) == 0)
  {
    ret = NRF24_PowerStandby(nrf24);
    nrf24_delay_ms(3);
    if (ret != true)
    {
      NRF24_LOG_ERROR("func:[%s] failed! %s, %d.\r\n", (uint32_t)__func__, (uint32_t)__FILENAME__, __LINE__);
      return false;
    }
  }

  // TX mode or RX mode -> RX mode
  if (nrf24_get_CE() == 1)
  {
    // RX mode -> standby mode
    if (NRF24_CHECK_BIT(tmp, NRF24_PRIM_RX) == 1)
    {
      if (NRF24_PowerStandby(nrf24) != true)
      {
        NRF24_LOG_ERROR("func:[%s] failed! %s, %d.\r\n", (uint32_t)__func__, (uint32_t)__FILENAME__, __LINE__);
        return false;
      }
    }
    // TX mode, do nothing
    else
      return true;
  }

  // set PRIM_RX = 0
  NRF24_SET_BIT(tmp, NRF24_PWR_UP);
  NRF24_CLEAR_BIT(tmp, NRF24_PRIM_RX);
  if (NRF24_WriteRegister(NRF24_REG_CONFIG, tmp) != true)
  {
    NRF24_LOG_ERROR("func:[%s] failed! %s, %d.\r\n", (uint32_t)__func__, (uint32_t)__FILENAME__, __LINE__);
    return false;
  }

  // standby mode -> TX mode
  nrf24_set_CE_High();
  nrf24->mode = MODE_TX;
//  nrf24_delay_us(15);
//  nrf24_set_CE_Low();
//  nrf24->mode = MODE_STANDBY;
  return true;
}

bool NRF24_PowerUpRx(NRF24_t *nrf24)
{
  bool ret;
//  NRF24_ClearInterrupts();

  uint8_t tmp;
  if(NRF24_ReadRegister(NRF24_REG_CONFIG, &tmp) != true)
  {
    NRF24_LOG_ERROR("Func:[%s] Failed! line = %d\r\n",
                  (uint32_t)__func__, (uint32_t)__LINE__);
    return false;
  }

  // shutdown mode -> standby mode
  if (NRF24_CHECK_BIT(tmp, NRF24_PWR_UP) == 0)
  {
    ret = NRF24_PowerStandby(nrf24);
    nrf24_delay_ms(3);
    if (ret != true)
    {
      NRF24_LOG_ERROR("func:[%s] failed! %s, %d.\r\n", (uint32_t)__func__, (uint32_t)__FILENAME__, __LINE__);
      return false;
    }
  }

  // TX mode or RX mode -> RX mode
  if (nrf24_get_CE() == 1)
  {
    // TX mode -> standby mode
    if (NRF24_CHECK_BIT(tmp, NRF24_PRIM_RX) == 0)
    {
      if (NRF24_PowerStandby(nrf24) != true)
      {
        NRF24_LOG_ERROR("func:[%s] failed! %s, %d.\r\n", (uint32_t)__func__, (uint32_t)__FILENAME__, __LINE__);
        return false;
      }
    }
    // RX mode already, do nothing
    else
      return true;
  }

  // set PRIM_RX = 1
  NRF24_SET_BIT(tmp, NRF24_PWR_UP);
  NRF24_SET_BIT(tmp, NRF24_PRIM_RX);
  if (NRF24_WriteRegister(NRF24_REG_CONFIG, tmp) != true)
  {
    NRF24_LOG_ERROR("func:[%s] failed! %s, %d.\r\n", (uint32_t)__func__, (uint32_t)__FILENAME__, __LINE__);
    return false;
  }

  // standby mode -> RX mode
  nrf24_set_CE_High();
  nrf24->mode = MODE_RX;
  // nrf24_delay_us(15);
  return true;
}

bool NRF24_GetMode(NRF24_t *nrf24, NRF24_mode_t *mode)
{
  uint8_t reg_val;
  if(NRF24_ReadRegister(NRF24_REG_CONFIG, &reg_val) != true)
  {
    NRF24_LOG_ERROR("Func:[%s] Failed! line = %d\r\n",
                  (uint32_t)__func__, (uint32_t)__LINE__);
    return false;
  }

  // shutdown mode
  if (NRF24_CHECK_BIT(reg_val, NRF24_PWR_UP) == 0)
  {
    nrf24->mode = MODE_POWERDOWN;
  }
  else
  {
    // standby mode
    if (nrf24_get_CE() == 0)
    {
      nrf24->mode = MODE_STANDBY;
    }
    else
    {
      // receive mode
      if (NRF24_CHECK_BIT(reg_val, NRF24_PRIM_RX) == 1)
      {
        nrf24->mode = MODE_RX;
      }
      // transmit mode
      else
      {
        nrf24->mode = MODE_TX;
      }
    }
  }

  *mode = nrf24->mode;
  return true;
}

bool NRF24_GetStatus(NRF24_t *nrf24, NRF24_status_t *p_status)
{
  /* First received byte is always status register */
  // uint8_t status = NRF24_SPI_RecvMulti(NRF24_CMD_NOP, NULL, 0);
  uint8_t status;
  if(NRF24_ReadRegister(NRF24_REG_STATUS, &status) != true)
  {
    NRF24_LOG_ERROR("Func:[%s] Failed! line = %d\r\n",
                  (uint32_t)__func__, (uint32_t)__LINE__);
    return false;
  }

  // RF fifo
  if (NRF24_CHECK_BIT(status, NRF24_TX_FULL))
    nrf24->status.is_tx_full = true;
  else
    nrf24->status.is_tx_full = false;

  if (NRF24_CHECK_BIT(status, NRF24_MAX_RT))
    nrf24->status.is_max_rt = true;
  else
    nrf24->status.is_max_rt = false;

  if (NRF24_CHECK_BIT(status, NRF24_TX_DS))
  {
    nrf24->status.is_tx_ds = true;
  }
  else
    nrf24->status.is_tx_ds = false;

  if (NRF24_CHECK_BIT(status, NRF24_RX_DR))
    nrf24->status.is_rx_dr = true;
  else
    nrf24->status.is_rx_dr = false;

  nrf24->status.rx_p_no = (status >> 1) & 0x07;

  *p_status = nrf24->status;
  return true;
}

bool NRF24_Get_Fifo_Status(NRF24_t *nrf24, NRF24_FifoStatus_t *status)
{
  uint8_t fifo_status;
  if(NRF24_ReadRegister(NRF24_REG_FIFO_STATUS, &fifo_status) != true)
  {
    NRF24_LOG_ERROR("Func:[%s] Failed! line = %d\r\n",
                  (uint32_t)__func__, (uint32_t)__LINE__);
    return false;
  }

  if (NRF24_CHECK_BIT(fifo_status, NRF24_RX_EMPTY))
    nrf24->fifo_status.is_rx_empty = true;
  else
    nrf24->fifo_status.is_rx_empty = false;

  if (NRF24_CHECK_BIT(fifo_status, NRF24_RX_FULL))
    nrf24->fifo_status.is_rx_full = true;
  else
    nrf24->fifo_status.is_rx_full = false;

  if (NRF24_CHECK_BIT(fifo_status, NRF24_TX_EMPTY))
    nrf24->fifo_status.is_tx_empty = true;
  else
    nrf24->fifo_status.is_tx_empty = false;

  if (NRF24_CHECK_BIT(fifo_status, NRF24_TX_FIFO_FULL))
    nrf24->fifo_status.is_tx_full = true;
  else
    nrf24->fifo_status.is_tx_full = false;

  if (NRF24_CHECK_BIT(fifo_status, NRF24_TX_REUSE))
    nrf24->fifo_status.is_tx_reuse = true;
  else
    nrf24->fifo_status.is_tx_reuse = false;

  *status = nrf24->fifo_status;
  return true;
}

bool NRF24_Get_Retry_Count(uint8_t *retry_cnt)
{
  /* Low 4 bits */
  uint8_t cnt;
  if(NRF24_ReadRegister(NRF24_REG_OBSERVE_TX, &cnt) != true)
  {
    NRF24_LOG_ERROR("Func:[%s] Failed! line = %d\r\n",
                  (uint32_t)__func__, (uint32_t)__LINE__);
    return false;
  }

  *retry_cnt = cnt & 0x0F;
  return true;
}

/*
  check a channel is free or not
  return true, the channel is free, received power is above -64dBm ( Si24R1 is -60dBm )
  return false, the channel is busy, received power is less than -64dBm ( Si24R1 is -60dBm )
*/
bool NRF24_is_channel_free(NRF24_t *nrf24, uint8_t channel, bool *is_ch_free)
{
  if (channel != nrf24->Channel)
  {
    if(NRF24_PowerStandby(nrf24) != true || NRF24_SetChannel(nrf24, channel) != true)
    {
      NRF24_LOG_ERROR("Func:[%s] Failed! line = %d\r\n",
                    (uint32_t)__func__, (uint32_t)__LINE__);
      return false;
    }
  }

  if (nrf24->mode != MODE_RX)
  {
    if(NRF24_PowerUpRx(nrf24) != true)
    {
      NRF24_LOG_ERROR("Func:[%s] Failed! line = %d\r\n",
                    (uint32_t)__func__, (uint32_t)__LINE__);
      return false;
    }
  }

  // wait 170us at least
  nrf24_delay_us(180);

  uint8_t reg_val;
  if(NRF24_ReadRegister(NRF24_REG_RPD, &reg_val) != true)
  {
    NRF24_LOG_ERROR("Func:[%s] Failed! line = %d\r\n",
                  (uint32_t)__func__, (uint32_t)__LINE__);
    return false;
  }

  if(reg_val == 1)
    *is_ch_free = false;
  else
    *is_ch_free = true;

  return true;
}

/*
  before call NRF24_Init(), you need initialize NRF24_t *nrf24, register callback functions for it.
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
bool NRF24_Init(NRF24_t *nrf24,
                NRF24_ACK_t ack,
                uint8_t delay,
                uint8_t retry_count,
                NRF24_cb_t *call_back)
{
  uint8_t reg_feature;
  const uint8_t DEFAULT_RX_P0_ADDR[5] = {0xE7, 0xE7, 0xE7, 0xE7, 0xE7};
  const uint8_t DEFAULT_RX_P1_ADDR[5] = {0xC2, 0xC2, 0xC2, 0xC2, 0xC2};

  nrf24_hal_init();
  //  nrf24_set_CE_Low();
  // Fill structure
  nrf24->mode = MODE_POWERDOWN;
  nrf24->status.is_max_rt = 0;
  nrf24->status.is_rx_dr = 0;
  nrf24->status.is_tx_ds = 0;
  nrf24->status.is_tx_full = 0;
  nrf24->status.rx_p_no = 0;
  memcpy(nrf24->rx0_addr, DEFAULT_RX_P0_ADDR, 5);
  memcpy(nrf24->rx1_addr, DEFAULT_RX_P1_ADDR, 5);
  nrf24->rx2_addr = 0xC3;
  nrf24->rx3_addr = 0xC4;
  nrf24->rx4_addr = 0xC5;
  nrf24->rx5_addr = 0xC6;

  nrf24->callback = call_back;

  /* Reset nRF24L01+ to power on registers values */
  if(NRF24_SoftwareReset(nrf24) != true)
  {
    NRF24_LOG_ERROR("Func:[%s] Failed! line = %d\r\n",
                  (uint32_t)__func__, (uint32_t)__LINE__);
    return false;
  }

  /* Go to Standby mode */
  if (NRF24_PowerStandby(nrf24) != true)
  {
    NRF24_LOG_ERROR("Func:[%s] Failed! line = %d\r\n",
                  (uint32_t)__func__, (uint32_t)__LINE__);
    return false;
  }

//  /* Channel select */
//  if (NRF24_SetChannel(nrf24, channel) != true)
//    return false;
//  /* Set RF settings */
//  if (NRF24_Set_DR_Power(nrf24, dataRate, power) != true)
//    return false;

  // no ack settings
  if (ack == NO_ACK)
  {
    nrf24->ack = NO_ACK;
    if(NRF24_SetAutoRetry(nrf24, 0, 0) != true)
    {
      NRF24_LOG_ERROR("Func:[%s] Failed! line = %d\r\n",
                    (uint32_t)__func__, (uint32_t)__LINE__);
      return false;
    }

#if ENABLE_DYNAMIC_PALOAD_LENGTH
    reg_feature = 0x05;
#else
    reg_feature = 0x01;
#endif // ENABLE_DYNAMIC_PALOAD_LENGTH

    /* !!! disable CRC !!! CRC reg values will be modified if enable, the reason is not clear yet!
      disable max retry interrupt.
    */
    if(NRF24_WriteRegister(NRF24_REG_CONFIG, (0 << NRF24_EN_CRC) | (0 << NRF24_CRCO) | (1 << NRF24_MASK_MAX_RT)) != true)
    {
      NRF24_LOG_ERROR("Func:[%s] Failed! line = %d\r\n",
                    (uint32_t)__func__, (uint32_t)__LINE__);
      return false;
    }
  }
  // auto ack settings
  else
  {
    nrf24->ack = AUTO_ACK;
    // set rx pipe 0 address as same as the tx address
    //    NRF24_SetRxAddress( 0, tx_address );
    /* Auto retransmit delay */
    if(NRF24_SetAutoRetry(nrf24, delay, retry_count) != true)
    {
      NRF24_LOG_ERROR("Func:[%s] Failed! line = %d\r\n",
                    (uint32_t)__func__, (uint32_t)__LINE__);
      return false;
    }

#if ENABLE_DYNAMIC_PALOAD_LENGTH
    reg_feature = 0x04;
#else
    reg_feature = 0x00;
#endif // ENABLE_DYNAMIC_PALOAD_LENGTH

    // Config CRC as 2bytes, enable max retry interrupt
    if(NRF24_WriteRegister(NRF24_REG_CONFIG, (1 << NRF24_EN_CRC) | (1 << NRF24_CRCO) | (0 << NRF24_MASK_MAX_RT)) != true)
    {
      NRF24_LOG_ERROR("Func:[%s] Failed! line = %d\r\n",
                    (uint32_t)__func__, (uint32_t)__LINE__);
      return false;
    }
  }

  // Dynamic length settings
  if(NRF24_WriteRegister(NRF24_REG_FEATURE, reg_feature) != true)
    {
      NRF24_LOG_ERROR("Func:[%s] Failed! line = %d\r\n",
                    (uint32_t)__func__, (uint32_t)__LINE__);
      return false;
    }

  // settings static payload length
#if !ENABLE_DYNAMIC_PALOAD_LENGTH
  uint8_t ret = NRF24_WriteRegister(NRF24_REG_RX_PW_P0, NRF24_STATIC_LENGTH);
  ret += NRF24_WriteRegister(NRF24_REG_RX_PW_P1, NRF24_STATIC_LENGTH);
  ret += NRF24_WriteRegister(NRF24_REG_RX_PW_P2, NRF24_STATIC_LENGTH);
  ret += NRF24_WriteRegister(NRF24_REG_RX_PW_P3, NRF24_STATIC_LENGTH);
  ret += NRF24_WriteRegister(NRF24_REG_RX_PW_P4, NRF24_STATIC_LENGTH);
  ret += NRF24_WriteRegister(NRF24_REG_RX_PW_P5, NRF24_STATIC_LENGTH);
  if(ret != 6)
    {
      NRF24_LOG_ERROR("Func:[%s] Failed! line = %d\r\n",
                    (uint32_t)__func__, (uint32_t)__LINE__);
      return false;
    }
#endif // ENABLE_DYNAMIC_PALOAD_LENGTH

  // set address width 5 bytes
  if(NRF24_WriteRegister(NRF24_REG_SETUP_AW, 0x03) != true)
    {
      NRF24_LOG_ERROR("Func:[%s] Failed! line = %d\r\n",
                    (uint32_t)__func__, (uint32_t)__LINE__);
      return false;
    }

//  // set rx address
//  if(NRF24_SetRxAddress(nrf24, rx_pipe, rx_address) != true)
//    {
//      NRF24_LOG_ERROR("Func:[%s] Failed! line = %d\r\n",
//                    (uint32_t)__func__, (uint32_t)__LINE__);
//      return false;
//    }
//
//  // set tx address
//  if(NRF24_SetTxAddress(nrf24, tx_address) != true)
//    {
//      NRF24_LOG_ERROR("Func:[%s] Failed! line = %d\r\n",
//                    (uint32_t)__func__, (uint32_t)__LINE__);
//      return false;
//    }

  // enable dynamic payload length for all pipes
  if(NRF24_WriteRegister(NRF24_REG_DYNPD, (1 << NRF24_DPL_P0) | (1 << NRF24_DPL_P1) | (1 << NRF24_DPL_P2) | (1 << NRF24_DPL_P3) | (1 << NRF24_DPL_P4) | (1 << NRF24_DPL_P5)) != true)
    {
      NRF24_LOG_ERROR("Func:[%s] Failed! line = %d\r\n",
                    (uint32_t)__func__, (uint32_t)__LINE__);
      return false;
    }

  /* Clear FIFOs */
  if(NRF24_Flush_Tx() != true)
    {
      NRF24_LOG_ERROR("Func:[%s] Failed! line = %d\r\n",
                    (uint32_t)__func__, (uint32_t)__LINE__);
      return false;
    }
  if(NRF24_Flush_Rx() != true)
    {
      NRF24_LOG_ERROR("Func:[%s] Failed! line = %d\r\n",
                    (uint32_t)__func__, (uint32_t)__LINE__);
      return false;
    }

  /* Clear interrupts */
  if(NRF24_ClearInterrupts() != true)
    {
      NRF24_LOG_ERROR("Func:[%s] Failed! line = %d\r\n",
                    (uint32_t)__func__, (uint32_t)__LINE__);
      return false;
    }

  /* Go to Standby mode */
  if (NRF24_PowerStandby(nrf24) != true)
    {
      NRF24_LOG_ERROR("Func:[%s] Failed! line = %d\r\n",
                    (uint32_t)__func__, (uint32_t)__LINE__);
      return false;
    }

  /* Return OK */
  return true;
}

/*
  data: the data that you want to send
  size：the length of data
*/
bool NRF24_Transmit(NRF24_t *nrf24, uint8_t *data, uint8_t size)
{
//  NRF24_mode_t mode = NRF24_GetMode(nrf24);
//  NRF24_status_t status = NRF24_GetStatus(nrf24);
  NRF24_FifoStatus_t fifo_stat;
  if(NRF24_Get_Fifo_Status(nrf24, &fifo_stat) != true)
  {
    NRF24_LOG_ERROR("Func:[%s] Failed! line = %d\r\n",
                  (uint32_t)__func__, (uint32_t)__LINE__);
    return false;
  }

  // when tx fifo is full, retrun false
  if (fifo_stat.is_tx_full == 1)
  {
    NRF24_LOG_WARNING("is_tx_full when NRF24_Transmit.\r\n");
//    NRF24_ClearInterrupts();
    if (NRF24_Flush_Tx() != true)
    {
      NRF24_LOG_ERROR("func:[%s] failed! %s, %d.\r\n", (uint32_t)__func__, (uint32_t)__FILENAME__, __LINE__);
      return false;
    }
  }

  // switch to standby mode
  if (NRF24_PowerStandby(nrf24) != true)
  {
    NRF24_LOG_ERROR("func:[%s] failed! %s, %d.\r\n", (uint32_t)__func__, (uint32_t)__FILENAME__, __LINE__);
    return false;
  }

  uint8_t cmd;
  if (nrf24->ack == AUTO_ACK)
  {
//    // update rx & tx adress
//    if (NRF24_SetRxAddress(nrf24, nrf24->rx_pipe, nrf24->rx_address) != true)
//    {
//      NRF24_LOG_ERROR("func:[%s] failed! %s, %d.\r\n", (uint32_t)__func__, (uint32_t)__FILENAME__, __LINE__);
//      return false;
//    }
//    if (NRF24_SetTxAddress(nrf24, nrf24->tx_address) != true)
//    {
//      NRF24_LOG_ERROR("func:[%s] failed! %s, %d.\r\n", (uint32_t)__func__, (uint32_t)__FILENAME__, __LINE__);
//      return false;
//    }
//
//    // set rx pipe 0 address as same as tx address
//    if (NRF24_WriteRegisterMulti(NRF24_REG_RX_ADDR_P0, nrf24->tx_address, 5) != true)
//    {
//      NRF24_LOG_ERROR("func:[%s] failed! %s, %d.\r\n", (uint32_t)__func__, (uint32_t)__FILENAME__, __LINE__);
//      return false;
//    }

    cmd = NRF24_CMD_TX_PAYLOAD;
  }
  else
    cmd = NRF24_CMD_TX_PAYLOAD_NOACK;
  // fill TX fifo
  if (NRF24_SPI_SendMulti(cmd, data, size) != true)
  {
    NRF24_LOG_ERROR("func:[%s] failed! %s, %d.\r\n", (uint32_t)__func__, (uint32_t)__FILENAME__, __LINE__);
    return false;
  }
  //switch to TX mode and start transmit now
  if (NRF24_PowerUpTx(nrf24) != true)
  {
    NRF24_LOG_ERROR("func:[%s] failed! %s, %d.\r\n", (uint32_t)__func__, (uint32_t)__FILENAME__, __LINE__);
    return false;
  }

  return true;
}

// Called by nrf24 IRQ pin interrupt function only
void NRF24_Irq_callback(NRF24_t *nrf24)
{
  if(nrf24->mode == MODE_POWERDOWN)
    return;

  NRF24_status_t status;
  if(NRF24_GetStatus(nrf24, &status) != true)
  {
    NRF24_LOG_ERROR("Func:[%s] Failed! line = %d\r\n",
                  (uint32_t)__func__, (uint32_t)__LINE__);
    NRF24_ClearInterrupts();
    return;
  }

//  NRF24_FifoStatus_t fifo_status = NRF24_Get_Fifo_Status(nrf24);

  // tx done
  if (status.is_tx_ds == true)
  {
    nrf24->status.is_tx_ds = false;
    nrf24->callback->TX_done_cb();
  }
  // max tx retry count
  if (status.is_max_rt == true)
  {
    nrf24->status.is_max_rt = false;
    nrf24_set_CE_Low();
    nrf24->mode = MODE_STANDBY;
    nrf24->callback->Max_retry_cb();
  }
  // tx fifo is full
  if (status.is_tx_full == true)
  {
    NRF24_Flush_Tx();
    NRF24_PowerStandby(nrf24);
    // if needed, you must clear tx fifo with NRF24_Flush_Tx()
    nrf24->status.is_tx_full = false;
    nrf24->callback->TX_full_cb();
  }
  // rx fifo is filled by data
  if (status.is_rx_dr == true)
  {
    nrf24->mode = MODE_RX;
    nrf24->status.is_rx_dr = false;

    NRF24_FifoStatus_t fifo_state;
    uint8_t i = 0;
    for(i = 0; i < 3; i++)
    {
      // get fifo status and data size, then read data back
      if(NRF24_Get_Fifo_Status(nrf24, &fifo_state) != true)
      {
        NRF24_LOG_ERROR("Func:[%s] Failed! line = %d\r\n",
                        (uint32_t)__func__, (uint32_t)__LINE__);
        NRF24_ClearInterrupts();
        return;
      }

      if(fifo_state.is_rx_empty != true)
      {
        uint8_t size = 0;
        uint8_t data[NRF24_MAX_PAYLOAD_SIZE];
        memset(data, 0x00, sizeof(data));

#if ENABLE_DYNAMIC_PALOAD_LENGTH
        size = NRF24_Get_Received_Size();
        if(size > 32)
        {
          NRF24_Flush_Rx();
          break;
        }
        if(size > 0)
        {
          if(NRF24_ReceiveData(data, size) != true)
          {
            NRF24_LOG_ERROR("NRF24_ReceiveData Failed! Line= %d\r\n", (uint32_t)__LINE__);
            break;
          }
        }
#else
        size = NRF24_STATIC_LENGTH;
        if(NRF24_ReceiveData(data, size) != true)
        {
          NRF24_LOG_ERROR("NRF24_ReceiveData Failed! Line= %d\r\n", (uint32_t)__LINE__);
          break;
        }
#endif // ENABLE_DYNAMIC_PALOAD_LENGTH

        nrf24->callback->RX_data_ready_cb(data, size, status.rx_p_no);
      }
    }
  }

  NRF24_ClearInterrupts();
}
