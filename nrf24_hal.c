/*
  Copyright (C) gaupen1186@gmail.com, 2018
*/

#include "nrf24_hal.h"
#include "nrfx.h"
#include "nrf_drv_gpiote.h"
#include "nrf_drv_spi.h"
#include "nrf_delay.h"
#include "nrf_error.h"
#include "nrf_log.h"

#include "boards.h"
#include "sdk_config.h"


/********************************************************************/
#define ERROR_CODE         \
  NRF_LOG_ERROR(__FILE__); \
  NRF_LOG_ERROR("LINE = [%d]\r\n", __LINE__)

/********************************************************************/
// #define NRF24_DISABLE_IRQ   NVIC_DisableIRQ( GPIOTE_IRQn )
// #define NRF24_ENABLE_IRQ    NVIC_EnableIRQ( GPIOTE_IRQn )
#define NRF24_DISABLE_IRQ   __disable_irq()
#define NRF24_ENABLE_IRQ    __enable_irq()

/********************************************************************/

static const nrf_drv_spi_t m_spi_nrf = NRF_DRV_SPI_INSTANCE(0); /**< SPI instance. */

inline void nrf24_delay_us(uint32_t us)
{
  nrf_delay_us(us);
}

inline void nrf24_delay_ms(uint32_t ms)
{
  nrf_delay_ms(ms);
}

inline void nrf24_set_CSN_Low(void)
{
  nrf_gpio_pin_write(NRF24_SPIM1_CS_PIN, 0);
}

inline void nrf24_set_CSN_High(void)
{
  nrf_gpio_pin_write(NRF24_SPIM1_CS_PIN, 1);
}

inline void nrf24_set_CE_Low(void)
{
  nrf_gpio_pin_write(NRF24_CE_PIN, 0);
}

inline void nrf24_set_CE_High(void)
{
  nrf_gpio_pin_write(NRF24_CE_PIN, 1);
}

inline uint8_t nrf24_get_CE(void)
{
  return nrf_gpio_pin_read(NRF24_CE_PIN);
}

bool NRF24_SPI_SendMulti(uint8_t cmd, uint8_t *data, uint8_t size)
{
  uint8_t tx[33] = {0};
  uint8_t rx = 0; // status register
  ret_code_t err_code;

  tx[0] = cmd;
  if (0 < size <= 32)
  {
    memcpy(&tx[1], data, size);
  }
  nrf24_set_CSN_Low();
  NRF24_DISABLE_IRQ;
  err_code = nrf_drv_spi_transfer(&m_spi_nrf,
                                  tx, 1 + size,
                                  &rx, 1);
  NRF24_ENABLE_IRQ;

  nrf24_set_CSN_High();
  if (err_code != NRF_SUCCESS)
  {
    ERROR_CODE;
    return false;
  }
  return true;
}

bool NRF24_SPI_RecvMulti(uint8_t cmd, uint8_t *data, uint8_t size)
{
  uint8_t rx[33] = {0}; // rx[0] is status register
  ret_code_t err_code;

  nrf24_set_CSN_Low();
  NRF24_DISABLE_IRQ;
  err_code = nrf_drv_spi_transfer(&m_spi_nrf,
                                  &cmd, 1,
                                  rx, 1 + size);
  NRF24_ENABLE_IRQ;

  nrf24_set_CSN_High();
  if (err_code != NRF_SUCCESS)
  {
    ERROR_CODE;
    return false;
  }
  if (0 < size <= 32)
  {
    memcpy(data, &rx[1], size);
  }
  else
    data[0] = rx[0];

  return true;
}

inline bool NRF24_ReadRegisterMulti(uint8_t reg, uint8_t *data, uint8_t count)
{
  bool ret = NRF24_SPI_RecvMulti(NRF24_READ_REGISTER_MASK(reg), data, count);
  return ret;
}

inline bool NRF24_WriteRegisterMulti(uint8_t reg, uint8_t *data, uint8_t count)
{
  bool ret = NRF24_SPI_SendMulti(NRF24_WRITE_REGISTER_MASK(reg), data, count);
  return ret;
}

inline uint8_t NRF24_ReadRegister(uint8_t reg)
{
  uint8_t data = 0;
  NRF24_SPI_RecvMulti(NRF24_READ_REGISTER_MASK(reg), &data, 1);
  return data;
}

inline bool NRF24_WriteRegister(uint8_t reg, uint8_t data)
{
  bool ret = NRF24_SPI_SendMulti(NRF24_WRITE_REGISTER_MASK(reg), &data, 1);
  return ret;
}

void nrf24_hal_init(void)
{
  ret_code_t err_code;

  // io init
  nrf_gpio_cfg(NRF24_CE_PIN,
               NRF_GPIO_PIN_DIR_OUTPUT,
               NRF_GPIO_PIN_INPUT_CONNECT,
               NRF_GPIO_PIN_NOPULL,
               NRF_GPIO_PIN_S0S1,
               NRF_GPIO_PIN_NOSENSE);
  nrf_gpio_cfg_output(NRF24_SPIM1_CS_PIN);
  nrf_gpio_pin_write(NRF24_SPIM1_CS_PIN, 1);
  nrf_gpio_pin_write(NRF24_CE_PIN, 1);

  if (!nrf_drv_gpiote_is_init())
  {
    err_code = nrf_drv_gpiote_init();
    APP_ERROR_CHECK(err_code);
  }

  extern void nrf24_irq_pin_handler_callback(nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t action);

  nrf_drv_gpiote_in_config_t in_config = GPIOTE_CONFIG_IN_SENSE_HITOLO(true);
  err_code = nrf_drv_gpiote_in_init(NRF24_IRQ_PIN, &in_config, nrf24_irq_pin_handler_callback);
  APP_ERROR_CHECK(err_code);
  nrf_drv_gpiote_in_event_enable(NRF24_IRQ_PIN, true);

  // spi init
  uint32_t error_code;
  const nrf_drv_spi_config_t spi_cfg = {
      .sck_pin = NRF24_SPIM1_SCK_PIN,
      .mosi_pin = NRF24_SPIM1_MOSI_PIN,
      .miso_pin = NRF24_SPIM1_MISO_PIN,
      .ss_pin = NRF_DRV_SPI_PIN_NOT_USED,
      .irq_priority = SPI_DEFAULT_CONFIG_IRQ_PRIORITY,
      .orc = 0,
      .frequency = NRF_DRV_SPI_FREQ_8M,
      .mode = NRF_DRV_SPI_MODE_0,
      .bit_order = NRF_DRV_SPI_BIT_ORDER_MSB_FIRST,
  };
  error_code = nrf_drv_spi_init(&m_spi_nrf, &spi_cfg, NULL, NULL);
  if (error_code != NRF_SUCCESS)
  {
    ERROR_CODE;
  }
}
