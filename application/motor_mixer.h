#ifndef MOTOR_MIXER_H
#define MOTOR_MIXER_H

#include <stdint.h>

// 限制 PWM 输出范围 (假设硬件是 8-bit PWM, 0-255)
#define MAX_PWM 255
#define MIN_PWM 0
// 1. 确保基地址存在 (E203 GPIOA 基地址通常是 0x10012000)
#ifndef HBIRD_GPIO_BASE
#define HBIRD_GPIO_BASE 0x10012000
#endif

// 2. 定义寄存器偏移量
#ifndef GPIO_INPUT_VAL
#define GPIO_INPUT_VAL  0x00  // 输入值
#endif

#ifndef GPIO_INPUT_EN
#define GPIO_INPUT_EN   0x04  // 输入使能
#endif

#ifndef GPIO_OUTPUT_EN
#define GPIO_OUTPUT_EN  0x08  // 输出使能
#endif

#ifndef GPIO_OUTPUT_VAL
#define GPIO_OUTPUT_VAL 0x0C  // 输出值 (这就是你缺少的那个！)
#endif

#ifndef GPIO_IOF_EN
#define GPIO_IOF_EN     0x38  // 硬件功能复用使能 (PWM模式需要操作这个)
#endif

#ifndef GPIO_IOF_SEL
#define GPIO_IOF_SEL    0x3C  // 硬件功能选择
#endif

// 3. 定义寄存器读写宏
// 如果 SDK 里没有 _REG32，手动定义它
#ifndef _REG32
#define _REG32(addr, offset) (*(volatile uint32_t *)((uintptr_t)(addr) + (offset)))
#endif

// 定义方便使用的宏 GPIOA_REG(OFFSET)
#ifndef GPIOA_REG
#define GPIOA_REG(offset)  _REG32(HBIRD_GPIO_BASE, offset)
#endif
// 定义电机混控结构体
typedef struct {
    int16_t m1; // 右前 (CCW)
    int16_t m2; // 右后 (CW)
    int16_t m3; // 左后 (CCW)
    int16_t m4; // 左前 (CW)
} Motor_t;

// 函数声明
void Motor_Mix_And_Write(int32_t throttle, int32_t pitch_pid, int32_t roll_pid, int32_t yaw_pid);

#endif
