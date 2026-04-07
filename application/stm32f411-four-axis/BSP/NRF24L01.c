#include "NRF24L01.h"
#include "spi.h"
#include <string.h>

// 定义常量
const uint8_t TX_ADDRESS[TX_ADR_WIDTH] = {0xE1,0xE2,0xE3,0xE4,0xE5};
const uint8_t RX_ADDRESS[RX_ADR_WIDTH] = {0xE1,0xE2,0xE3,0xE4,0xE5};

/**
 * @brief 初始化 NRF24L01
 */
void NRF24L01_Init(void)
{
    // 1. 初始化底层 SPI 和 GPIO
    // 这一步会设置 PADDIR，让引脚可以被拉低
    spi_Init();

    // 2. 检查芯片是否存在
    // 如果卡死在这里，请检查接线或 Verilog 引脚绑定
    while(NRF24L01_Check() != 0) {
        // 等待...
        for(volatile int k=0; k<10000; k++);
    }

    // 3. 默认进入接收模式
    NRF24L01_RX_Mode();
}

/**
 * @brief 检测 NRF24L01 连接状态
 * @return 0:成功, 1:失败
 */
uint8_t NRF24L01_Check(void)
{
    uint8_t buf[5] = {0xA5, 0xA5, 0xA5, 0xA5, 0xA5};
    uint8_t buf1[5] = {0};
    uint8_t i;

    // 写入 TX_ADDR 寄存器
    NRF24L01_Write_Buf(NRF_WRITE_REG + TX_ADDR, buf, 5);
    // 读回
    NRF24L01_Read_Buf(TX_ADDR, buf1, 5);

    // 比较
    for(i = 0; i < 5; i++) {
        if(buf1[i] != 0xA5) break;
    }

    if(i != 5) return 1; // 失败
    return 0;            // 成功
}

/**
 * @brief SPI 写寄存器
 */
uint8_t NRF24L01_Write_Reg(uint8_t regaddr, uint8_t data)
{
    uint8_t status;
    SPI_CS_LOW;                 // 拉低 CSN
    status = SPI_ReadWrite_Byte(regaddr);
    SPI_ReadWrite_Byte(data);
    SPI_CS_HIGH;                // 拉高 CSN
    return status;
}

/**
 * @brief SPI 读寄存器
 */
uint8_t NRF24L01_Read_Reg(uint8_t regaddr)
{
    uint8_t reg_val;
    SPI_CS_LOW;
    SPI_ReadWrite_Byte(regaddr);
    reg_val = SPI_ReadWrite_Byte(0xFF); // 发送 Dummy Byte
    SPI_CS_HIGH;
    return reg_val;
}

/**
 * @brief 读 Buffer
 */
uint8_t NRF24L01_Read_Buf(uint8_t regaddr, uint8_t *pBuf, uint8_t datalen)
{
    uint8_t status, i;
    SPI_CS_LOW;
    status = SPI_ReadWrite_Byte(regaddr);
    for(i = 0; i < datalen; i++) {
        pBuf[i] = SPI_ReadWrite_Byte(0xFF);
    }
    SPI_CS_HIGH;
    return status;
}

/**
 * @brief 写 Buffer
 */
uint8_t NRF24L01_Write_Buf(uint8_t regaddr, uint8_t *pBuf, uint8_t datalen)
{
    uint8_t status, i;
    SPI_CS_LOW;
    status = SPI_ReadWrite_Byte(regaddr);
    for(i = 0; i < datalen; i++) {
        SPI_ReadWrite_Byte(*pBuf++);
    }
    SPI_CS_HIGH;
    return status;
}

/**
 * @brief 发送数据包
 */
uint8_t NRF24L01_TxPacket(uint8_t *txbuf)
{
    uint8_t state;

    NRF_CE_LOW;
    NRF24L01_Write_Buf(WR_TX_PLOAD, txbuf, TX_PLOAD_WIDTH);
    NRF_CE_HIGH; // 脉冲触发发送

    // 简单延时等待发送结束
    for(volatile int k=0; k<5000; k++);

    state = NRF24L01_Read_Reg(STATUS);
    NRF24L01_Write_Reg(NRF_WRITE_REG + STATUS, state); // 清除中断

    if(state & MAX_TX) {
        NRF24L01_Write_Reg(FLUSH_TX, 0xFF);
        return MAX_TX;
    }
    if(state & TX_OK) {
        return 0; // SUCCESS
    }
    return 1; // FAILED
}

/**
 * @brief 接收数据包
 */
uint8_t NRF24L01_RxPacket(uint8_t *rxbuf)
{
    uint8_t state;
    state = NRF24L01_Read_Reg(STATUS);
    NRF24L01_Write_Reg(NRF_WRITE_REG + STATUS, state);

    if(state & RX_OK) {
        NRF24L01_Read_Buf(RD_RX_PLOAD, rxbuf, RX_PLOAD_WIDTH);
        NRF24L01_Write_Reg(FLUSH_RX, 0xFF);
        return 0; // SUCCESS
    }
    return 1; // FAILED
}

/**
 * @brief 配置为接收模式
 */
void NRF24L01_RX_Mode(void)
{
    NRF_CE_LOW; // 进入待机，准备配置

    // 1. 写接收地址
    NRF24L01_Write_Buf(NRF_WRITE_REG + RX_ADDR_P0, (uint8_t*)RX_ADDRESS, RX_ADR_WIDTH);

    // 2. 禁止自动应答 (EN_AA = 0x00) - 关键修改！匹配遥控器的 0x00
    NRF24L01_Write_Reg(NRF_WRITE_REG + EN_AA, 0x00);

    // 3. 允许接收地址 (只允许通道0)
    NRF24L01_Write_Reg(NRF_WRITE_REG + EN_RXADDR, 0x01);

    // 4. 设置频道 (RF_CH = 1) - 匹配遥控器
    NRF24L01_Write_Reg(NRF_WRITE_REG + RF_CH, 1);

    // 5. 设置数据包长度 (32字节)
    NRF24L01_Write_Reg(NRF_WRITE_REG + RX_PW_P0, 32);

    // 6. 射频设置 (1Mbps, 0dBm) - 匹配遥控器的 0x07
    NRF24L01_Write_Reg(NRF_WRITE_REG + RF_SETUP, 0x07);

    // 7. 配置寄存器 (开启CRC, 16位CRC, 接收模式, 上电)
    // 遥控器是 0x0E (发送), 我们要是 0x0F (接收)
    NRF24L01_Write_Reg(NRF_WRITE_REG + CONFIG, 0x0F);

    NRF_CE_HIGH; // 拉高 CE，开始监听空气中的信号
}

/**
 * @brief 配置为发送模式
 */
void NRF24L01_TX_Mode(void)
{
    NRF_CE_LOW;
    NRF24L01_Write_Buf(NRF_WRITE_REG + TX_ADDR, (uint8_t*)TX_ADDRESS, TX_ADR_WIDTH);
    NRF24L01_Write_Buf(NRF_WRITE_REG + RX_ADDR_P0, (uint8_t*)RX_ADDRESS, RX_ADR_WIDTH);
    NRF24L01_Write_Reg(NRF_WRITE_REG + EN_AA, 0x01);
    NRF24L01_Write_Reg(NRF_WRITE_REG + EN_RXADDR, 0x01);
    NRF24L01_Write_Reg(NRF_WRITE_REG + SETUP_RETR, 0x1A);
    NRF24L01_Write_Reg(NRF_WRITE_REG + RF_CH, 1);
    NRF24L01_Write_Reg(NRF_WRITE_REG + RF_SETUP, 0x07);
    NRF24L01_Write_Reg(NRF_WRITE_REG + CONFIG, 0x0E); // PWR_UP, PRIM_RX=0
    NRF_CE_HIGH;
}
