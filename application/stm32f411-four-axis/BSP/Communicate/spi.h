#ifndef __SPI_H
#define __SPI_H

#include <stdint.h>
#include <stddef.h> // for NULL
#include "hbird_sdk_soc.h" // 包含 GPIO_TypeDef 和 GPIOA 的定义

// ==========================================================
// 1. 引脚掩码定义 (对应 GPIOA 的位)
// ==========================================================
// 请确认这些位与你的 FPGA 约束文件 (.xdc) 一致
#define MASK_CSN     (1UL << 4)  // Bit 2
#define MASK_MOSI    (1UL << 5)  // Bit 3
#define MASK_MISO    (1UL << 6)  // Bit 4 (输入)
#define MASK_SCK     (1UL << 7)  // Bit 5
#define MASK_CE      (1UL << 8)  // Bit 6
#define MASK_IRQ     (1UL << 9)  // Bit 7 (输入)

// ==========================================================
// 2. 宏定义：操作 GPIO
// ==========================================================

// CSN 片选 (输出)
#define SPI_CS_HIGH     gpio_write(GPIOA, MASK_CSN, 1)
#define SPI_CS_LOW      gpio_write(GPIOA, MASK_CSN, 0)

// SCK 时钟 (输出)
#define SPI_SCK_HIGH    gpio_write(GPIOA, MASK_SCK, 1)
#define SPI_SCK_LOW     gpio_write(GPIOA, MASK_SCK, 0)

// MOSI 主出从入 (输出)
#define SPI_MOSI_HIGH   gpio_write(GPIOA, MASK_MOSI, 1)
#define SPI_MOSI_LOW    gpio_write(GPIOA, MASK_MOSI, 0)

// CE 无线使能 (输出)
#define NRF_CE_HIGH     gpio_write(GPIOA, MASK_CE, 1)
#define NRF_CE_LOW      gpio_write(GPIOA, MASK_CE, 0)

// MISO 主入从出 (读取)
#define SPI_MISO_READ   gpio_read(GPIOA, MASK_MISO)

// IRQ 中断 (读取)
#define NRF_IRQ_READ    gpio_read(GPIOA, MASK_IRQ)

// ==========================================================
// 3. 函数声明
// ==========================================================
void spi_Init(void);
uint8_t SPI_ReadWrite_Byte(uint8_t tx_data);

#endif
