# Nordic nRF24L01+ and Si24R1 Driver

# 项目介绍
适用于ROXs2，重新编写的 nrf24l01p（包括兼容芯片 Si24R1）的驱动程序，以及测试例程

基于nordic nRF5_SDK 15.0.0_a53641a

# 如何使用

### 1. 根据硬件平台修改硬件接口层
- 修改头文件 nrf24_hal.c 和 nrf24_hal.h 包含内容的头文件
- 修改 nrf24_hal.c 中的 NRF24_DISABLE_IRQ 和 NRF24_ENABLE_IRQ
- 修改以下函数的实现方式
    nrf24_delay_us
    nrf24_delay_ms
    nrf24_set_CSN_Low
    nrf24_set_CSN_High
    nrf24_set_CE_Low
    nrf24_set_CE_High
    nrf24_get_CE
    NRF24_SPI_SendMulti
    NRF24_SPI_RecvMulti
    NRF24_ReadRegisterMulti
    NRF24_WriteRegisterMulti
    nrf24_hal_init
### 2. 初始化
- 在你的工程中编写4个回调函数，原型位于 nrf24l01p.h 中的 NRF24_cb_t 类型
- 创建 NRF24_cb_t 类型的回调函数集
- 在外部 IO 中断函数中调用 NRF24_Irq_callback() 函数
- 在 main()函数中调用初始化函数 NRF24_Init
### 3. 发送数据
- 使用函数 NRF24_Transmit 发送数据，发送完成后会调用你自己编写的发送完成回调函数
### 4. 接收数据
- 使用函数 NRF24_PowerUpRx 进入接收模式
- 一旦接收到有效的数据，则会自动进入你自己编写的接收完成回调函数，在接收完成回调函数内，你需要保存接收到的数据，以便进一步处理
- 接收完成后，你可以使用函数 NRF24_Powerxxx 手动切换到其他模式，否则继续保持在接收模式

# 使用例程
包含头文件，定义变量
````
#include <stdbool.h>
#include "nrf24_hal.h"
#include "nrf24l01p.h"

bool is_tx_ok, is_tx_full,is_rx_ok, is_max_retry;
uint8_t tx_addr[] = {0x11, 0x22, 0x33, 0x44, 0x55};
uint8_t rx_addr[] = {0x55, 0x44, 0x33, 0x22, 0x11};
uint8_t received[3] = {0};
uint8_t tx_buffer = { "\r\nThis is a test line.\r\n" }
````
定义回调函数
````
void tx_done_callback(void)
{
  is_tx_ok = true;
  // do someting what you want
}
void tx_full_callback(void)
{
  is_tx_full = true;
  // do someting what you want
}
void rx_data_ready_callback(NRF24_fifo_t *data, uint8_t pipe_num)
{
  uint8_t i = 0;
  while (data[i].size > 0)
  {
    memcpy( &received[i], data[i].packet, data[i].size );
    i++;
    if (i == 3)
      break;
  }
  is_rx_ok = true;
}
void max_retry_callback(void)
{
  is_max_retry = true;
}
void nrf24_irq_pin_handler()
{
  NRF24_Irq_callback();
}
````
初始化和发送
````
NRF24_cb_t nrf24_callback = {tx_done_callback, tx_full_callback，
                             rx_data_ready_callback, max_retry_callback};

int main(void)
{
  bool ret = NRF24_Init( 20, // channel 20 = 2420 MHz
                         NRF24_Power_0dBm,
                         NRF24_DataRate_2M,
                         1, // rx pipe number is 1
                         tx_addr,
                         rx_addr,
                         AUTO_ACK,
                         9, // ( 9 +1 )* 250us = 2500 us
                         6,
                         &nrf24_callback ); // retry 5 times
  if (ret == false)
  {
    LOG_ERROR("nrf24 INIT error!");
  }
  is_rx_ok = true;
  while(true)
  {
    // received a packet
    if( is_rx_ok == true )
    {
      is_rx_ok = false;
      /* do something what you want
        ......
      */
      // send a packet
      if( NRF24_Transmit( &tx_buffer, sizeof(tx_buffer) ) == false )
      {
        LOG_ERROR("nrf24 transmit error!");
      }
    }
    // error process
    if( is_tx_full == true )
    {
      is_tx_full = false;
      NRF24_Flush_Tx();
      is_rx_ok = true;
    }
  }
}
````

# 注意事项
- 初始化的时候，只能先指定一个接收管道的地址。如果需要使用多个接收管道，你需要在初始化完成后单独使用函数 NRF24_SetRxAddress 来开启多个管道
- 此驱动代码固定使用 5 bytes 地址，其它长度地址暂未实现
- 自动应答和非自动应答模式，仅支持一种，若需改变应答模式，需重新调用初始化函数 NRF24_Init

# 参与贡献

1. Fork 本项目
2. 新建分支
3. 提交代码
4. 新建 Pull Request

# 版权
    Copyright (C) gaupen1186@gmail.com, 2018
