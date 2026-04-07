#ifndef __NRF24L01_H
#define __NRF24L01_H

#include <stdint.h>
#include "spi.h" // 包含 E203 的底层驱动

// 类型定义兼容
#ifndef u8
#define u8 uint8_t
#endif

// ========================================================================
// NRF24L01 寄存器操作命令
// ========================================================================
#define NRF_READ_REG    0x00  // 读配置寄存器,低5位为寄存器地址
#define NRF_WRITE_REG   0x20  // 写配置寄存器,低5位为寄存器地址
#define RD_RX_PLOAD     0x61  // 读RX有效数据,1~32字节
#define WR_TX_PLOAD     0xA0  // 写TX有效数据,1~32字节
#define FLUSH_TX        0xE1  // 清除TX FIFO寄存器
#define FLUSH_RX        0xE2  // 清除RX FIFO寄存器
#define REUSE_TX_PL     0xE3  // 重新使用上一包数据
#define NOP             0xFF  // 空操作

// ========================================================================
// NRF24L01 寄存器地址
// ========================================================================
#define CONFIG          0x00  // 配置寄存器地址
#define EN_AA           0x01  // 使能自动应答
#define EN_RXADDR       0x02  // 接收地址使能
#define SETUP_AW        0x03  // 设置地址宽度(3-5字节)
#define SETUP_RETR      0x04  // 建立自动重发
#define RF_CH           0x05  // RF通道
#define RF_SETUP        0x06  // RF寄存器
#define STATUS          0x07  // 状态寄存器
#define OBSERVE_TX      0x08  // 发送检测寄存器
#define CD              0x09  // 载波检测
#define RX_ADDR_P0      0x0A  // 数据通道0接收地址
#define RX_ADDR_P1      0x0B  // 数据通道1接收地址
#define RX_ADDR_P2      0x0C  // 数据通道2接收地址
#define RX_ADDR_P3      0x0D  // 数据通道3接收地址
#define RX_ADDR_P4      0x0E  // 数据通道4接收地址
#define RX_ADDR_P5      0x0F  // 数据通道5接收地址
#define TX_ADDR         0x10  // 发送地址寄存器
#define RX_PW_P0        0x11  // 接收数据通道0有效数据宽度(1~32字节)
#define RX_PW_P1        0x12  // 接收数据通道1有效数据宽度(1~32字节)
#define RX_PW_P2        0x13  // 接收数据通道2有效数据宽度(1~32字节)
#define RX_PW_P3        0x14  // 接收数据通道3有效数据宽度(1~32字节)
#define RX_PW_P4        0x15  // 接收数据通道4有效数据宽度(1~32字节)
#define RX_PW_P5        0x16  // 接收数据通道5有效数据宽度(1~32字节)
#define FIFO_STATUS     0x17  // FIFO状态寄存器

// ========================================================================
// 状态标志位
// ========================================================================
#define MAX_TX  		0x10  // 达到最大发送次数中断
#define TX_OK   		0x20  // TX发送完成中断
#define RX_OK   		0x40  // 接收到数据中断

// ========================================================================
// 24L01发送接收数据宽度定义
// ========================================================================
#define TX_ADR_WIDTH    5     // 5字节的地址宽度
#define RX_ADR_WIDTH    5     // 5字节的地址宽度
#define TX_PLOAD_WIDTH  32    // 32字节的用户数据宽度
#define RX_PLOAD_WIDTH  32    // 32字节的用户数据宽度

// ========================================================================
// 函数声明
// ========================================================================
void NRF24L01_Init(void);
u8 NRF24L01_Check(void);
u8 NRF24L01_Write_Reg(u8 regaddr, u8 data);
u8 NRF24L01_Read_Reg(u8 regaddr);
u8 NRF24L01_Read_Buf(u8 regaddr, u8 *pBuf, u8 datalen);
u8 NRF24L01_Write_Buf(u8 regaddr, u8 *pBuf, u8 datalen);
u8 NRF24L01_TxPacket(u8 *txbuf);
u8 NRF24L01_RxPacket(u8 *rxbuf);
void NRF24L01_RX_Mode(void);
void NRF24L01_TX_Mode(void);

#endif
