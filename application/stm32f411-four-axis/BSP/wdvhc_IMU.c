#include "hbird_sdk_soc.h"
#include <stdint.h>
#include <math.h> // 仅用于最后输出
#include <stdlib.h> // 必须包含，用于 abs() 函数
#include "wdvhc_IMU.h"

// =============================================
// 1. 定点数宏 (Q16.16)
// =============================================
#define Q_SHIFT         16
#define FIX_ONE         (1 << Q_SHIFT)
#define FIX_TWO         (2 << Q_SHIFT)
#define F2FIX(f)        ((int32_t)((f) * 65536.0f))
#define FIX2F(x)        ((float)(x) / 65536.0f)
#define FIX_MUL(a, b)   (int32_t)((((int64_t)(a)) * ((int64_t)(b))) >> Q_SHIFT)

// =============================================
// 2. 参数调整区 (关键)
// =============================================
// 死区：原始陀螺仪数据小于此值强制为0，消除静止漂移
#define GYRO_DEADBAND   6

// 加速度计低通滤波系数 (0~65536). 越小越平滑，但反应越慢
// 4000 约为 0.06，滤波效果强，能稳定 Rol
#define ACC_LPF_FACTOR  4000

// =============================================
// 3. 全局变量
// =============================================
int16_t Gyro[3], Acc[3];
int32_t Pitch, Roll, Yaw;

// 四元数
int32_t q0 = FIX_ONE, q1 = 0, q2 = 0, q3 = 0;

// 积分误差
int32_t exInt = 0, eyInt = 0, ezInt = 0;

// 校准偏移量
int32_t offset_ax = 0;
int32_t offset_ay = 0;
int32_t offset_gx = 0;
int32_t offset_gy = 0;
int32_t offset_gz = 0;

// 滤波后的加速度值 (保持静态，用于下一次计算)
int32_t acc_filt_x = 0;
int32_t acc_filt_y = 0;
int32_t acc_filt_z = 0;

uint8_t is_calibrated = 0;

extern void MPU6050ReadAcc(int16_t *acc);
extern void MPU6050ReadGyro(int16_t *gyro);

// 简单的延时函数
void simple_delay_ms(int ms) {
    for(int i=0; i<ms; i++) {
        for(int j=0; j<4000; j++) { __asm("nop"); }
    }
}

// =============================================
// 4. 核心算法 (Mahony)
// =============================================
#define FIX_HALFT       (262)       // 0.004s
// 调大 Ki (积分增益)，让角度更快回归到加速度计测出的"水平面"
#define FIX_KP          (2 << 16)   // Kp = 2.0
#define FIX_KI          (500)       // Ki = 0.0075 (增大积分作用，修正偏差)

// 整数 sqrt (牛顿法)
uint32_t isqrt(uint32_t n) {
    if (n == 0) return 0;
    uint32_t x = n;
    uint32_t y = (x + 1) >> 1;
    while (y < x) { x = y; y = (x + n / x) >> 1; }
    return x;
}

// Q16.16 倒数平方根
int32_t fix_invSqrt(int32_t x) {
    if (x <= 0) return 0;
    uint32_t root = isqrt((uint32_t)x);
    if (root == 0) return 0;
    return (1 << 24) / root;
}

void ImuUpdate_Fixed(int32_t gx, int32_t gy, int32_t gz, int32_t ax, int32_t ay, int32_t az) {
    int32_t norm;
    int32_t vx, vy, vz;
    int32_t ex, ey, ez;

    if (ax == 0 && ay == 0 && az == 0) return;

    // 1. 加速度归一化
    int32_t sq_sum = FIX_MUL(ax, ax) + FIX_MUL(ay, ay) + FIX_MUL(az, az);
    norm = fix_invSqrt(sq_sum);
    ax = FIX_MUL(ax, norm);
    ay = FIX_MUL(ay, norm);
    az = FIX_MUL(az, norm);

    // 2. 提取重力分量
    vx = FIX_MUL(FIX_TWO, FIX_MUL(q1, q3) - FIX_MUL(q0, q2));
    vy = FIX_MUL(FIX_TWO, FIX_MUL(q0, q1) + FIX_MUL(q2, q3));
    vz = FIX_MUL(q0, q0) - FIX_MUL(q1, q1) - FIX_MUL(q2, q2) + FIX_MUL(q3, q3);

    // 3. 误差计算
    ex = FIX_MUL(ay, vz) - FIX_MUL(az, vy);
    ey = FIX_MUL(az, vx) - FIX_MUL(ax, vz);
    ez = FIX_MUL(ax, vy) - FIX_MUL(ay, vx);

    // 4. 误差积分
    exInt += FIX_MUL(ex, FIX_KI);
    eyInt += FIX_MUL(ey, FIX_KI);
    ezInt += FIX_MUL(ez, FIX_KI);

    // 5. 修正角速度
    gx += FIX_MUL(ex, FIX_KP) + exInt;
    gy += FIX_MUL(ey, FIX_KP) + eyInt;
    gz += FIX_MUL(ez, FIX_KP) + ezInt;

    // 6. 四元数更新
    int32_t pa = -FIX_MUL(q1, gx) - FIX_MUL(q2, gy) - FIX_MUL(q3, gz);
    int32_t pb =  FIX_MUL(q0, gx) + FIX_MUL(q2, gz) - FIX_MUL(q3, gy);
    int32_t pc =  FIX_MUL(q0, gy) - FIX_MUL(q1, gz) + FIX_MUL(q3, gx);
    int32_t pd =  FIX_MUL(q0, gz) + FIX_MUL(q1, gy) - FIX_MUL(q2, gx);

    q0 += FIX_MUL(pa, FIX_HALFT);
    q1 += FIX_MUL(pb, FIX_HALFT);
    q2 += FIX_MUL(pc, FIX_HALFT);
    q3 += FIX_MUL(pd, FIX_HALFT);

    // 7. 归一化
    sq_sum = FIX_MUL(q0, q0) + FIX_MUL(q1, q1) + FIX_MUL(q2, q2) + FIX_MUL(q3, q3);
    norm = fix_invSqrt(sq_sum);
    q0 = FIX_MUL(q0, norm);
    q1 = FIX_MUL(q1, norm);
    q2 = FIX_MUL(q2, norm);
    q3 = FIX_MUL(q3, norm);

    // 8. 输出欧拉角
    float fq0 = FIX2F(q0), fq1 = FIX2F(q1), fq2 = FIX2F(q2), fq3 = FIX2F(q3);
    float pitch_rad = asinf(-2.0f * (fq1*fq3 - fq0*fq2));
    float roll_rad  = atan2f(2.0f * (fq0*fq1 + fq2*fq3), fq0*fq0 - fq1*fq1 - fq2*fq2 + fq3*fq3);
    float yaw_rad   = atan2f(2.0f * (fq1*fq2 + fq0*fq3), fq0*fq0 + fq1*fq1 - fq2*fq2 - fq3*fq3);

    Pitch = F2FIX(pitch_rad * 57.29578f);
    Roll  = F2FIX(roll_rad  * 57.29578f);
    Yaw   = F2FIX(yaw_rad   * 57.29578f);
}

// =============================================
// 5. 自动校准 (增强版)
// =============================================
void perform_calibration(void)
{
    // 【重要】启动延时 1秒
    // 请在这1秒内松开手，让板子在桌面上彻底静止
    simple_delay_ms(100);

    int32_t sum_ax = 0, sum_ay = 0;
    int32_t sum_gx = 0, sum_gy = 0, sum_gz = 0;
    int i;
    int calib_count = 500; // 采样500次，提高平均值精度

    for(i = 0; i < calib_count; i++)
    {
        MPU6050ReadAcc(Acc);
        MPU6050ReadGyro(Gyro);

        sum_ax += Acc[0];
        sum_ay += Acc[1];
        // Z轴不去皮

        sum_gx += Gyro[0];
        sum_gy += Gyro[1];
        sum_gz += Gyro[2];

        simple_delay_ms(2);
    }

    offset_ax = sum_ax / calib_count;
    offset_ay = sum_ay / calib_count;

    offset_gx = sum_gx / calib_count;
    offset_gy = sum_gy / calib_count;
    offset_gz = sum_gz / calib_count;

    // 初始化滤波变量
    acc_filt_x = 0;
    acc_filt_y = 0;
    acc_filt_z = (16384 << 2); // 假设Z轴初始为1g

    // 重置算法状态
    q0 = FIX_ONE; q1 = 0; q2 = 0; q3 = 0;
    exInt = 0; eyInt = 0; ezInt = 0;

    is_calibrated = 1;
}

// =============================================
// 6. 数据处理入口
// =============================================
void wdvhc_get_data_fixed(uint8_t on)
{
    if (!on) return;

    if (is_calibrated == 0) {
        perform_calibration();
        return;
    }

    MPU6050ReadAcc(Acc);
    MPU6050ReadGyro(Gyro);

    // 1. 去皮 (减去校准偏移)
    int32_t ax_raw = Acc[0] - offset_ax;
    int32_t ay_raw = Acc[1] - offset_ay;
    int32_t az_raw = Acc[2]; // Z轴不去皮

    int32_t gx_raw = Gyro[0] - offset_gx;
    int32_t gy_raw = Gyro[1] - offset_gy;
    int32_t gz_raw = Gyro[2] - offset_gz;

    // 2. 【核心修复】陀螺仪死区处理
    // 如果数值很小，强制为0，消除静止漂移
    if (abs(gx_raw) < GYRO_DEADBAND) gx_raw = 0;
    if (abs(gy_raw) < GYRO_DEADBAND) gy_raw = 0;
    if (abs(gz_raw) < GYRO_DEADBAND) gz_raw = 0;

    // 3. 单位转换 -> Q16.16
    // 加速度: 1g=16384 -> Q16.16(1.0)=65536.  Rate=4
    int32_t ax_new = ax_raw << 2;
    int32_t ay_new = ay_raw << 2;
    int32_t az_new = az_raw << 2;

    // 4. 【核心修复】加速度计低通滤波 (Low Pass Filter)
    // 消除 Rol 在 -1.0 ~ -2.6 之间的剧烈跳动
    // Formula: Filt = Filt + Factor * (New - Filt)
    acc_filt_x += FIX_MUL(ACC_LPF_FACTOR, (ax_new - acc_filt_x));
    acc_filt_y += FIX_MUL(ACC_LPF_FACTOR, (ay_new - acc_filt_y));
    acc_filt_z += FIX_MUL(ACC_LPF_FACTOR, (az_new - acc_filt_z));

    // Gyro: 1rad/s = 939LSB -> Q16.16(1.0)=65536. Rate=70
    int32_t gx_fix = gx_raw * 70;
    int32_t gy_fix = gy_raw * 70;
    int32_t gz_fix = gz_raw * 70;

    // 5. 传入算法 (使用滤波后的加速度数据)
    ImuUpdate_Fixed(gx_fix, gy_fix, gz_fix, acc_filt_x, acc_filt_y, acc_filt_z);
}
