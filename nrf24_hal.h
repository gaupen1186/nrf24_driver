/*
  Copyright (C) gaupen1186@gmail.com, 2018
*/

#ifndef __NRF24_HAL_H__
#define __NRF24_HAL_H__

#include <stdbool.h>
#include "nordic_common.h"
#include "nrf.h"
#include "nrf_drv_spi.h"
#include "nrf_delay.h"

/* Instruction Mnemonics */
#define NRF24_REGISTER_MASK 0x1F

#define NRF24_READ_REGISTER_MASK(reg) (0x00 | (NRF24_REGISTER_MASK & reg))  //Last 5 bits will indicate reg. address
#define NRF24_WRITE_REGISTER_MASK(reg) (0x20 | (NRF24_REGISTER_MASK & reg)) //Last 5 bits will indicate reg. address

void nrf24_delay_us(uint32_t us);
void nrf24_delay_ms(uint32_t ms);
void nrf24_set_CSN_Low(void);
void nrf24_set_CSN_High(void);
void nrf24_set_CE_Low(void);
void nrf24_set_CE_High(void);
uint8_t nrf24_get_CE(void);

bool NRF24_SPI_SendMulti(uint8_t cmd, uint8_t *data, uint8_t size);
bool NRF24_SPI_RecvMulti(uint8_t cmd, uint8_t *data, uint8_t size);
bool NRF24_SPI_Send(uint8_t value);
uint8_t NRF24_SPI_Receive(uint8_t cmd);

uint8_t NRF24_ReadRegister(uint8_t reg);
bool NRF24_ReadRegisterMulti(uint8_t reg, uint8_t *data, uint8_t count);
bool NRF24_WriteRegister(uint8_t reg, uint8_t data);
bool NRF24_WriteRegisterMulti(uint8_t reg, uint8_t *data, uint8_t count);

void nrf24_hal_init(void);

#endif // __NRF24_HAL_H__
