//#include "motor_mixer.h"
//#include <stdio.h>
//#include "hbird_sdk_soc.h"
//
//// ====================================================================
//// 电调参数定义 (标准航模电调)
//// ====================================================================
//#define ESC_MIN_THROTTLE 1000  // 1000us (停转)
//#define ESC_MAX_THROTTLE 2000  // 2000us (满速)
//#define ESC_ARM_THRESHOLD 1050 // 起转阈值
//
//// 这里的 PWM0_BASE 也就是 0x10015000 (根据 HBIRD 文档)
//// 如果你的 SDK 里没有定义 PWM0_REG，请确保包含正确的头文件
//// 通常 E203 PWM 寄存器定义如下：
//// CMP0 -> 通道0 (GPIO 0)
//// CMP1 -> 通道1 (GPIO 1)
//// CMP2 -> 通道2 (GPIO 2)
//// CMP3 -> 通道3 (GPIO 3)
//
//// 辅助限幅函数
//static int32_t constrain_pwm(int32_t val) {
//    if (val > ESC_MAX_THROTTLE) return ESC_MAX_THROTTLE;
//    if (val < ESC_MIN_THROTTLE) return ESC_MIN_THROTTLE;
//    return val;
//}
//
///**
// * @brief 电机混控与硬件写入函数
// *
// * @param throttle  油门基础值 (范围: 1000 - 2000)
// * @param pitch_pid 俯仰角 PID 输出 (范围建议: -500 到 +500)
// * @param roll_pid  横滚角 PID 输出 (范围建议: -500 到 +500)
// * @param yaw_pid   航向角 PID 输出 (范围建议: -500 到 +500)
// */
////void Motor_Mix_And_Write(int32_t throttle, int32_t pitch_pid, int32_t roll_pid, int32_t yaw_pid) {
////
////    // 1. 安全检查：如果油门极低，强制锁定电机
////    if (throttle < 1020) {
////        // 直接设置比较值为 1000 (或 0，取决于你的 PWM 计数器配置)
////        // 假设 PWM 周期是 20ms，计数器单位是 1us
////        PWM0_REG(PWM_CMP0) = 1000;
////        PWM0_REG(PWM_CMP1) = 1000;
////        PWM0_REG(PWM_CMP2) = 1000;
////        PWM0_REG(PWM_CMP3) = 1000;
////        return;
////    }
////
////    int32_t m1, m2, m3, m4;
////
////    // ====================================================================
////    // 2. 混控算法 (X型四轴)
////    // 根据你的机架实际电机转向，可能需要修改符号 (+/-)
////    // 假设：
////    // M1: 右前 (CW)   -> GPIO 0
////    // M2: 右后 (CCW)  -> GPIO 1
////    // M3: 左后 (CW)   -> GPIO 2
////    // M4: 左前 (CCW)  -> GPIO 3
////    // ====================================================================
////
////    // 基本混控公式
////    m1 = throttle - pitch_pid - roll_pid - yaw_pid; // 右前
////    m2 = throttle + pitch_pid - roll_pid + yaw_pid; // 右后
////    m3 = throttle + pitch_pid + roll_pid - yaw_pid; // 左后
////    m4 = throttle - pitch_pid + roll_pid + yaw_pid; // 左前
////
////    // 3. 限幅 (Clamping)
////    m1 = constrain_pwm(m1);
////    m2 = constrain_pwm(m2);
////    m3 = constrain_pwm(m3);
////    m4 = constrain_pwm(m4);
////
////    // ====================================================================
////    // 4. 写入硬件 PWM 寄存器
////    // 注意：这里假设 PWM 硬件已经初始化好，且 1个计数单位 = 1us
////    // 如果你的 PWM 初始化频率不同，这里的值需要按比例缩放
////    // ====================================================================
////
////    // 直接写入 E203 的 PWM 比较寄存器 (Comparator)
////    // 这些寄存器决定了高电平持续的时间
////    PWM0_REG(PWM_CMP0) = (uint16_t)m1; // 对应 motor_pwm[0]
////    PWM0_REG(PWM_CMP1) = (uint16_t)m2; // 对应 motor_pwm[1]
////    PWM0_REG(PWM_CMP2) = (uint16_t)m3; // 对应 motor_pwm[2]
////    PWM0_REG(PWM_CMP3) = (uint16_t)m4; // 对应 motor_pwm[3]
////}
