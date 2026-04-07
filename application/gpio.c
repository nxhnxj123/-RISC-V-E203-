#include "hbird_sdk_soc.h"
#include "i2c.h"
#include "gpio.h"

void i2c_GPIO_Init(void)
{
	uint32_t pins = (1 << 18) | (1 << 19);

	    // 1. 强制关闭 GPIO 18/19 的硬件复用功能，回归普通 GPIO 模式
	    // 这是解决 0xFF 的核心一步！
	    GPIOA->IOFCFG &= ~pins;

	    // 2. 设置为输出方向
	    // 根据头文件定义，PADDIR 控制方向
	    GPIOA->PADDIR |= pins;

	    // 3. 初始电平拉高 (I2C 空闲状态)
	    GPIOA->PADOUT |= pins;

}
