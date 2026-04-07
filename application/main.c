#include <stdint.h>
#include <string.h>
#include <stdio.h>
#include "hbird_sdk_soc.h"
#include "NRF24L01.h"
#include "spi.h"
#include "i2c.h"
#include "mpu6050.h"
#include "PID.h"
#include "wdvhc_IMU.h"

// =========================================================================
// 1. 硬件控制宏与全局变量
// =========================================================================
#define PWM_BASE       0x10042000
#define PWM_CTRL_REG   (*(volatile uint32_t *)(PWM_BASE + 0x00))
#define PWM_M01_REG    (*(volatile uint32_t *)(PWM_BASE + 0x04))
#define PWM_M23_REG    (*(volatile uint32_t *)(PWM_BASE + 0x08))
#define PWM_MODE_MAX   0x1
#define PWM_MODE_MIN   0x2
#define PWM_MODE_RUN   0x3

// 引入外部姿态与PID变量
extern int32_t Pitch, Roll, Yaw;
extern int16_t Gyro[3];
uint8_t fly_flag = 0;
uint8_t GL_flag = 1;

// 目标姿态
int32_t target_throttle = 0;
int32_t target_pitch_angle = 0;
int32_t target_roll_angle = 0;
int32_t target_yaw_rate = 0;
int32_t final_target_yaw_angle = 0; // 航向锁目标
int32_t final_target_yaw_angle_HR = 0; // HR 意为 High Resolution (高精度)，单位是 0.001 度
// =========================================================================
// 2. 辅助函数
// =========================================================================
void calibrate_esc(void) {
    printf("\r\n========== ESC Calibration Started ==========\r\n");
    printf("1. [SAFETY] PLEASE REMOVE PROPELLERS!\r\n");
    printf("2. Ensure the battery is DISCONNECTED.\r\n");

    // --- 第一步：设置为运行模式并输出最大脉宽 (2000us) ---
    PWM_CTRL_REG = PWM_MODE_MAX;
//    // 这里的 1000 对应你硬件中的 2000us (最大值)
//    set_drone_motors(1200, 1200, 1200, 1200);

    printf("3. [ACTION] Connect the battery NOW.\r\n");
    printf("Waiting for ESC 'Beep-Beep' (Maximum throttle confirmation)...\r\n");

    // 等待 5 秒钟。在这期间，你需要插上电池。
    // 电调检测到高电平油门后，通常会发出两声短促的嘀嘀声。
    for (int i = 2; i > 0; i--) {
        printf("Time remaining: %d s\n", i);
        delay_ms_volatile(1000);
    }

    // --- 第二步：输出最小脉宽 (1000us) ---
    printf("4. Sending Minimum Throttle (1000us)...\r\n");
    // 这里的 0 对应你硬件中的 1000us (最小值)
    PWM_CTRL_REG = PWM_MODE_MIN;

    printf("Waiting for ESC 'Beep---' (Calibration success/Battery cell count)...\r\n");

    // 等待电调保存设置。电调通常会发出一长声或根据电池节数发出几声短响。
    delay_ms_volatile(3000);

    printf("========== ESC Calibration Finished ==========\r\n");
    printf("Now you can restart the drone or proceed to flight mode.\r\n");
}

void set_drone_motors(uint16_t m0, uint16_t m1, uint16_t m2, uint16_t m3) {
    if(m0 > 1200) m0 = 1200; if(m1 > 1200) m1 = 1200;
    if(m2 > 1200) m2 = 1200; if(m3 > 1200) m3 = 1200;
    PWM_M01_REG = (m0 << 16) | m1;
    PWM_M23_REG = (m2 << 16) | m3;
}

#define LIMIT(x, min, max) ((x) < (min) ? (min) : ((x) > (max) ? (max) : (x)))

/**
 * @brief 根据用户提供的日志修正的解析函数
 */
void parse_rc_data(uint8_t *buf) {
    // 映射逻辑来自用户提供的 Log 数据
    uint16_t rc_roll  = (buf[4] << 8)  | buf[5];  // 右摇杆左右
    uint16_t rc_pitch = (buf[6] << 8)  | buf[7];  // 右摇杆上下
    uint16_t rc_yaw   = (buf[10] << 8)  | buf[11];  // 左摇杆左右
    uint16_t rc_thr   = (buf[8] << 8) | buf[9]; // 左摇杆上下 (油门)

    // 1. 油门映射 1000~2000 -> 0~1000
    target_throttle = LIMIT((int32_t)rc_thr - 1000, 0, 1000);
//    printf("RC Raw -> R:%d P:%d Y:%d T:%d\n", rc_roll, rc_pitch, rc_yaw, rc_thr);
    // 2. 姿态映射 1000~2000 -> -25度 到 +25度
    // 死区处理 (1500附近20us不动作)
    int32_t p_off = (int32_t)rc_pitch - 1503;
    int32_t r_off = (int32_t)rc_roll - 1393;
    int32_t y_off = (int32_t)rc_yaw - 1506;

    target_pitch_angle = (uint32_t)abs(p_off) < 20 ? 0 : (p_off * 25 / 500);
    target_roll_angle  = (uint32_t)abs(r_off) < 20 ? 0 : (r_off * 25 / 500);

    // 3. 偏航映射为角速度 (Degree/Second)
    target_yaw_rate    = (uint32_t)abs(y_off) < 30 ? 0 : (y_off * 150 / 500);
}

// 保存陀螺仪三个轴的零点偏移量
int16_t Gyro_Offset[3] = {0, 0, 0};
// 保存角度的初始偏移量（用于消除桌面不平或安装不平的误差）
int32_t Angle_Offset[2] = {0, 0}; // [0]:Pitch, [1]:Roll

/**
  * @brief  高级 IMU 校准函数 (必须在无人机绝对静止时调用)
  *         利用现有的解算函数采集数据，确保校准数据和实际控制数据格式完全一致
  */
void Calibrate_IMU_Offsets(void)
{
    long gyro_sum[3] = {0, 0, 0};
    long angle_sum[2] = {0, 0};
    int sample_count = 500; // 采样500次，耗时约 2 秒

    printf("\r\n--- Starting IMU Calibration ---\r\n");
    printf("KEEP THE DRONE ABSOLUTELY STILL AND LEVEL!\r\n");

    // 给系统和滤波器 1 秒钟的收敛稳定时间
    for(int i = 0; i < 500; i++) {
        delay_ms_volatile(4);
    }

    printf("Sampling...\r\n");
    for(int i = 0; i < sample_count; i++)
    {
        wdvhc_get_data_fixed(1); // 刷新外部变量 Pitch, Roll, Yaw, Gyro

        gyro_sum[0] += Gyro[0];
        gyro_sum[1] += Gyro[1];
        gyro_sum[2] += Gyro[2];

        angle_sum[0] += (Pitch >> 16);
        angle_sum[1] += (Roll >> 16);

        delay_ms_volatile(4); // 模拟主循环频率
    }

    // 计算平均值得到零偏
    Gyro_Offset[0] = gyro_sum[0] / sample_count;
    Gyro_Offset[1] = gyro_sum[1] / sample_count;
    Gyro_Offset[2] = gyro_sum[2] / sample_count;

    Angle_Offset[0] = angle_sum[0] / sample_count;
    Angle_Offset[1] = angle_sum[1] / sample_count;

    printf("Calibration Done!\r\n");
//    printf("Gyro Offsets X:%d Y:%d Z:%d\r\n", Gyro_Offset[0], Gyro_Offset[1], Gyro_Offset[2]);
//    printf("Angle Offsets Pitch:%d Roll:%d\r\n", Angle_Offset[0], Angle_Offset[1]);
}
// =========================================================================
// 3. 主程序
// =========================================================================
int main(void) {
    uint8_t rx_buf[32];
    uint32_t failsafe_cnt = 0;
//    calibrate_esc();
    // --- 初始化流程 ---
    printf("System Booting...\n");

    MPU6050_Init();
    NRF24L01_Init();
    delay_ms_volatile(2000);    // 等待电调确认
    printf("Calibrating IMU... Do not move!\n");
    Calibrate_IMU_Offsets();

    printf("Armed and Ready!\n");
    PWM_CTRL_REG = PWM_MODE_MIN;
    PWM_CTRL_REG = PWM_MODE_RUN;
    set_drone_motors(0, 0, 0, 0); // 初始推力为 0

    // --- 主控循环 (250Hz / 4ms) ---
    while (1) {
        // 1. 获取遥控器数据
        if (NRF24L01_RxPacket(rx_buf) == 0) {
            if (rx_buf[0] == 0xAA && rx_buf[1] == 0xAF) {
                parse_rc_data(rx_buf);
                failsafe_cnt = 0;
            }
        } else {
            failsafe_cnt++;
        }
        // 2. 姿态解算
        wdvhc_get_data_fixed(1);
        int32_t cur_pitch = (Pitch >> 16) - Angle_Offset[0];
        int32_t cur_roll  = (Roll >> 16) - Angle_Offset[1];
        int32_t cur_yaw   = Yaw >> 16; // 偏航通常不需要修正绝对零点

                // [修改点 3]：减去陀螺仪零偏，消除旋转漂移
        int16_t cal_gyro_x = Gyro[0] - Gyro_Offset[0];
        int16_t cal_gyro_y = Gyro[1] - Gyro_Offset[1];
        int16_t cal_gyro_z = Gyro[2] - Gyro_Offset[2];

        // 3. 安全与状态检查
               if (target_throttle > 50 && failsafe_cnt < 100) {
                   // 起飞瞬间锁定 Yaw (放大 1000 倍)
                   if (fly_flag == 0) {
                       final_target_yaw_angle_HR = cur_yaw * 1000;
                       Reset_PID_Integrals();
                   }
                   fly_flag = 1;
               } else {
                   fly_flag = 0;
                   // 地面时实时跟随 (放大 1000 倍)
                   final_target_yaw_angle_HR = cur_yaw * 1000;
                   set_drone_motors(0,0,0,0);
                   Reset_PID_Integrals();
                   delay_ms_volatile(4);
                   continue;
               }

               final_target_yaw_angle_HR += (target_yaw_rate * 4);

                       // 还原回普通度数，供 PID 计算使用
                       final_target_yaw_angle = final_target_yaw_angle_HR / 1000;

                       // --- 注意偏航角 360 度回环问题 (可选，但强烈建议加上) ---
                       // 虽然 int32_t 很大，但机头转多了可能会溢出，或者与 cur_yaw 产生 ±360 度的跳变
                       // 这是一个简单的限制：
                       if (final_target_yaw_angle - cur_yaw > 180) {
                           final_target_yaw_angle_HR -= 360 * 1000;
                           final_target_yaw_angle -= 360;
                       } else if (final_target_yaw_angle - cur_yaw < -180) {
                           final_target_yaw_angle_HR += 360 * 1000;
                           final_target_yaw_angle += 360;
                       }

        // 5. 串级 PID 计算
        // ---------------------------------------------------------
        // 横滚 Roll (X轴)
        int32_t rate_target_x = Gyro_PID_X(target_roll_angle, cur_roll); // 外环角度
        int32_t out_roll      = DG_PID_X(rate_target_x, cal_gyro_x);        // 内环角速度

        // 俯仰 Pitch (Y轴)
        int32_t rate_target_y = Gyro_PID_Y(target_pitch_angle, cur_pitch);
        int32_t out_pitch     = DG_PID_Y(rate_target_y, cal_gyro_y);

        // 偏航 Yaw (Z轴)
        int32_t rate_target_z = Gyro_PID_Z(final_target_yaw_angle, cur_yaw);
        int16_t gyro_z = cal_gyro_z;
        if (abs(gyro_z) < 3) gyro_z = 0; // 过滤微小噪声
        int32_t out_yaw       = DG_PID_Z(rate_target_z, gyro_z);

        // 6. 电机混控 (X型)
        // M0:左前(CW), M1:右前(CCW), M2:左后(CCW), M3:右后(CW)

        int32_t m0 = target_throttle - out_pitch + out_roll + out_yaw;
        int32_t m1 = target_throttle - out_pitch - out_roll - out_yaw;
        int32_t m2 = target_throttle + out_pitch + out_roll - out_yaw;
        int32_t m3 = target_throttle + out_pitch - out_roll + out_yaw;

//        printf("T_Roll:%d | C_Roll:%d | Rate_X:%d | cGyro_Y:%d || Out_R:%d\n",
//                       target_roll_angle, // 目标角度 (遥控器指令)
//                       cur_roll,          // 当前角度 (滤波器解算结果)
//                       rate_target_x,     // 外环输出的目标角速度
//                       cal_gyro_y,        // 当前实际角速度
//                       out_roll);         // 最终给电机的输出


        // 7. 输出
        set_drone_motors(
            LIMIT(m0, 0, 1200),
            LIMIT(m1, 0, 1200),
            LIMIT(m2, 0, 1200),
            LIMIT(m3, 0, 1200)
        );

        // 8. 频率控制 (确保 4ms 周期)
        // 建议此处使用硬件定时器，若无则使用 delay
        delay_ms_volatile(4);
    }
}
