#include "spi.h"
#include "hbird_sdk_soc.h"

// 简单的微秒级延时
static void spi_delay(void)
{
    // 根据主频调整循环次数，保证 SPI 速率不过快
    volatile int i = 40;
    while(i--);
}


// =================================================================
// 2. SPI 初始化 (解决无法拉低的核心)
// =================================================================

void spi_Init(void)
{
    // --------------------------------------------------------
    // A. 关闭硬件复用功能 (IOFCFG)
    // --------------------------------------------------------
    // 将相关引脚的 IOF 配置位清零，确保它们作为普通 GPIO 使用
    GPIOA->IOFCFG &= ~(MASK_CSN | MASK_SCK | MASK_MOSI | MASK_MISO | MASK_CE | MASK_IRQ);

    // --------------------------------------------------------
    // B. 配置输入/输出方向 (PADDIR) -> 这一步最重要！
    // --------------------------------------------------------
    // PADDIR 置 1 为输出，置 0 为输入

    // 1. 设置输出引脚 (CSN, SCK, MOSI, CE)
    GPIOA->PADDIR |= (MASK_CSN | MASK_SCK | MASK_MOSI | MASK_CE);

    // 2. 设置输入引脚 (MISO, IRQ)
    GPIOA->PADDIR &= ~(MASK_MISO | MASK_IRQ);

    // --------------------------------------------------------
    // C. 设置初始电平 (PADOUT)
    // --------------------------------------------------------
    SPI_CS_HIGH;    // 不选中
    NRF_CE_LOW;     // 待机模式
    SPI_SCK_LOW;    // 时钟空闲低
    SPI_MOSI_LOW;   // 数据空闲低

    // 注意：你的结构体中没有 Pull-up (上拉) 寄存器定义。
    // 如果 FPGA 顶层 Verilog 已经加了 PULLUP，这里就不需要软件配置了。
    // 如果必须软件配置上拉，通常在 offset 0x10 处，你可以用指针强行访问，但暂时先这样。
}

// =================================================================
// 3. SPI 字节读写 (Mode 0)
// =================================================================

uint8_t SPI_ReadWrite_Byte(uint8_t tx_data)
{
    uint8_t rx_data = 0;
    uint8_t i;

    // 循环 8 次，高位在前 (MSB First)
    for(i = 0; i < 8; i++)
    {
        // 1. 准备数据 (MOSI) - 在 SCK 上升沿之前
        if(tx_data & 0x80) {
            SPI_MOSI_HIGH;
        } else {
            SPI_MOSI_LOW;
        }
        tx_data <<= 1; // 准备下一位

        spi_delay(); // 建立时间

        // 2. SCK 拉高 (Slave 采样 MOSI)
        SPI_SCK_HIGH;

        spi_delay(); // 保持时间

        // 3. 此时 Master 读取 MISO (Slave 已经输出了数据)
        rx_data <<= 1;
        if(SPI_MISO_READ) {
            rx_data |= 1;
        }

        // 4. SCK 拉低
        SPI_SCK_LOW;

        spi_delay(); // 周期结束
    }

    return rx_data;
}
