#include "PID.h"
#include "hbird_sdk_soc.h"
#include <stdint.h>
#include <stdlib.h> // for abs()

// =========================================================================
// 定点数配置 (Q12格式)
// 1.0f 对应 4096
// =========================================================================
#define Q_SHIFT    12
#define Q_FACTOR   (1 << Q_SHIFT) // 4096

// 辅助宏：将浮点常数转换为定点整数
// 例如: F2I(0.72) -> 2949
#define F2I(x)     ((int32_t)((x) * Q_FACTOR))

// 绝对值宏
#define ABS(x)     ((x) > 0 ? (x) : -(x))

// 全局调试变量
int32_t iiii = 0;

// =========================================================================
// PID 参数定义 (全部转换为 int32_t)
// =========================================================================

/* --- 角速度环 X轴 --- */
int32_t DG_x_Kp = F2I(0.0);
int32_t DG_x_Ki = F2I(0.0);
int32_t DG_x_Kd = F2I(0.0);

/* --- 角速度环 Y轴 --- */
int32_t DG_y_Kp = F2I(5.0);
int32_t DG_y_Ki = F2I(0.0);
int32_t DG_y_Kd = F2I(0.0);

/* --- 角度环 X轴 --- */
int32_t Gyro_x_Kp = F2I(0.0);
int32_t Gyro_x_Ki = F2I(0.0);
int32_t Gyro_x_Kd = F2I(2.0);

/* --- 角度环 Y轴 --- */
int32_t Gyro_y_Kp = F2I(0.0);
int32_t Gyro_y_Ki = F2I(0.0);
int32_t Gyro_y_Kd = F2I(0.0);

/* --- 转向环 Z轴 --- */
int32_t DG_z_Kp = F2I(0.0);
int32_t DG_z_Ki = F2I(0.0);
int32_t DG_z_Kd = F2I(0.0);

int32_t Gyro_z_Kp = F2I(0.0);
int32_t Gyro_z_Ki = F2I(0.0);
int32_t Gyro_z_Kd = F2I(0.0);

/* --- 高度环 --- */
int32_t Altitude_Kp = F2I(0.0);
int32_t Altitude_Ki = F2I(0.0);
int32_t Altitude_Kd = F2I(0.0);

// 注意：Kd=1600比较大，1600*4096 = 6,553,600，int32存得下
int32_t Alt_SP_WZS_Kp = F2I(0.0);
int32_t Alt_SP_WZS_Ki = F2I(0.0);
int32_t Alt_SP_WZS_Kd = F2I(0.0);

/* --- 方向控制 --- */
int32_t Direct_X_Kp = F2I(0.0);
int32_t Direct_X_Ki = F2I(0.0);
int32_t Direct_X_Kd = F2I(0.0);

int32_t Direct_Y_Kp = F2I(0.0);
int32_t Direct_Y_Ki = F2I(0.0);
int32_t Direct_Y_Kd = F2I(0.0);

int32_t Direct_X_SP_Kp = F2I(0.0);
int32_t Direct_X_SP_Ki = F2I(0.0);
int32_t Direct_X_SP_Kd = F2I(0.0);

int32_t Direct_Y_SP_Kp = F2I(0.0);
int32_t Direct_Y_SP_Ki = F2I(0.0);
int32_t Direct_Y_SP_Kd = F2I(0.0);

int32_t ACC_Z_Kp = F2I(0.0);
int32_t ACC_Z_Kd = F2I(0.0);

int32_t Gyro_alt_Kp = F2I(0.0);

// 角速度环状态
static int32_t DG_X_Last_error = 0, DG_X_error_sum = 0;
static int32_t DG_Y_Last_error = 0, DG_Y_error_sum = 0;
static int32_t DG_Z_Last_error = 0, DG_Z_error_sum = 0;

// 角度环状态
static int32_t Gyro_X_Last_error = 0, Gyro_X_error_sum = 0;
static int32_t Gyro_Y_Last_error = 0, Gyro_Y_error_sum = 0;
static int32_t Gyro_Z_Last_error = 0, Gyro_Z_error_sum = 0;

// 高度与垂直速度状态
static int32_t Altitude_Last_error = 0, Altitude_error_sum = 0;
static int32_t Alt_SP_WZS_Last_error = 0, Alt_SP_WZS_error_sum = 0;

// 方向控制状态
static int32_t Direct_X_Last_error = 0;
static int32_t Direct_Y_Last_error = 0;
static int32_t Last_Direct_X_SP_error = 0, Direct_X_SP_error_sum = 0;
static int32_t Last_Direct_Y_SP_error = 0, Direct_Y_SP_error_sum = 0;

static int32_t ACC_Z_Last_error = 0;
// =========================================================================
// 核心计算函数
// =========================================================================
void Reset_PID_Integrals(void) {
    // 清空所有轴的积分累加
    DG_X_error_sum = 0; DG_Y_error_sum = 0; DG_Z_error_sum = 0;
    Gyro_X_error_sum = 0; Gyro_Y_error_sum = 0; Gyro_Z_error_sum = 0;
    Altitude_error_sum = 0; Alt_SP_WZS_error_sum = 0;
    Direct_X_SP_error_sum = 0; Direct_Y_SP_error_sum = 0;

    // 清空所有轴的上一次误差 (防止 D 项在恢复瞬间产生跳变)
    DG_X_Last_error = 0; DG_Y_Last_error = 0; DG_Z_Last_error = 0;
    Gyro_X_Last_error = 0; Gyro_Y_Last_error = 0; Gyro_Z_Last_error = 0;
    Altitude_Last_error = 0; Alt_SP_WZS_Last_error = 0;
    Direct_X_Last_error = 0; Direct_Y_Last_error = 0;
    Last_Direct_X_SP_error = 0; Last_Direct_Y_SP_error = 0;
    ACC_Z_Last_error = 0;
}
// 通用限幅函数
int32_t constrain_int(int32_t amt, int32_t low, int32_t high) {
    if (amt < low) return low;
    else if (amt > high) return high;
    else return amt;
}

// ************************************************
// 角速度环
// ************************************************

// 假设 Ek_Distance 和 Now_Distance 已经是整数（例如放大过的值或者原始传感器数据）
int32_t DG_PID_X(int32_t Ek_Distance, int32_t Now_Distance)
{
    int32_t DG_X_error;
    int64_t DG_X_pwm_calc; // 使用64位防止中间计算溢出
    int32_t DG_X_pwm;

    if(fly_flag == 1)
    {
        DG_X_error = Ek_Distance - Now_Distance;
        DG_X_error_sum += DG_X_error;

        // 计算 PID: P*Error + I*Sum + D*Diff
        // 注意：系数是Q12格式，所以乘法结果是 Q12 格式
        DG_X_pwm_calc = (int64_t)DG_x_Kp * DG_X_error
                      + (int64_t)DG_x_Ki * DG_X_error_sum
                      + (int64_t)DG_x_Kd * (DG_X_error - DG_X_Last_error);

        // 将 Q12 格式还原为普通整数：右移 12 位
        DG_X_pwm = (int32_t)(DG_X_pwm_calc >> Q_SHIFT);

        DG_X_Last_error = DG_X_error;

        // 输出限幅
        DG_X_pwm = constrain_int(DG_X_pwm, -500, 500);
    }
    else
    {
        DG_X_error_sum = 0;
        DG_X_pwm = 0;
        DG_X_Last_error = 0;
    }
    return DG_X_pwm;
}

int32_t DG_PID_Y(int32_t Ek_Distance, int32_t Now_Distance)
{
    int32_t DG_Y_error;
    int64_t DG_Y_pwm_calc;
    int32_t DG_Y_pwm;

    if(fly_flag == 1)
    {
        DG_Y_error = Ek_Distance - Now_Distance;
        DG_Y_error_sum += DG_Y_error;

        DG_Y_pwm_calc = (int64_t)DG_y_Kp * DG_Y_error
                      + (int64_t)DG_y_Ki * DG_Y_error_sum
                      + (int64_t)DG_y_Kd * (DG_Y_error - DG_Y_Last_error);

        DG_Y_pwm = (int32_t)(DG_Y_pwm_calc >> Q_SHIFT);

        DG_Y_Last_error = DG_Y_error;
        DG_Y_pwm = constrain_int(DG_Y_pwm, -500, 500);
    }
    else
    {
        DG_Y_error_sum = 0;
        DG_Y_pwm = 0;
        DG_Y_Last_error = 0;
    }
    return DG_Y_pwm;
}

// ************************************************
// 角度环
// ************************************************

// ************************************************
// 角度环 (已添加防地面 windup 保护)
// ************************************************

int32_t Gyro_PID_X(int32_t Ek_Distance, int32_t Now_Distance)
{
    int32_t Gyro_X_error;
    int64_t Gyro_X_pwm_calc;
    int32_t Gyro_X_pwm;

    if (fly_flag == 1)
    {
        Gyro_X_error = Ek_Distance - Now_Distance;
        Gyro_X_error_sum += Gyro_X_error;

        // 积分限幅
        Gyro_X_error_sum = constrain_int(Gyro_X_error_sum, -100, 100);

        // 注意：如果你后续给 Gyro_X_Ki 赋了非零值，这里需要加上 I 项的计算
        // 目前你的 Gyro_x_Ki 是 0，所以这里只有 P 和 D
        Gyro_X_pwm_calc = (int64_t)Gyro_x_Kp * Gyro_X_error
                        + (int64_t)Gyro_x_Ki * Gyro_X_error_sum  // 建议加上这行，以备将来调参
                        + (int64_t)Gyro_x_Kd * (Gyro_X_error - Gyro_X_Last_error);

        Gyro_X_pwm = (int32_t)(Gyro_X_pwm_calc >> Q_SHIFT);
        Gyro_X_Last_error = Gyro_X_error;
    }
    else
    {
        // 核心修复：地面状态时清空历史状态，输出为0
        Gyro_X_error_sum = 0;
        Gyro_X_Last_error = 0;
        Gyro_X_pwm = 0;
    }

    return Gyro_X_pwm;
}

int32_t Gyro_PID_Y(int32_t Ek_Distance, int32_t Now_Distance)
{
    int32_t Gyro_Y_error;
    int64_t Gyro_Y_pwm_calc;
    int32_t Gyro_Y_pwm;

    if (fly_flag == 1)
    {
        Gyro_Y_error = Ek_Distance - Now_Distance;
        Gyro_Y_error_sum += Gyro_Y_error;

        Gyro_Y_error_sum = constrain_int(Gyro_Y_error_sum, -100, 100);

        Gyro_Y_pwm_calc = (int64_t)Gyro_y_Kp * Gyro_Y_error
                        + (int64_t)Gyro_y_Ki * Gyro_Y_error_sum
                        + (int64_t)Gyro_y_Kd * (Gyro_Y_error - Gyro_Y_Last_error);

        Gyro_Y_pwm = (int32_t)(Gyro_Y_pwm_calc >> Q_SHIFT);
        Gyro_Y_Last_error = Gyro_Y_error;
    }
    else
    {
        Gyro_Y_error_sum = 0;
        Gyro_Y_Last_error = 0;
        Gyro_Y_pwm = 0;
    }

    return Gyro_Y_pwm;
}

int32_t Gyro_PID_Z(int32_t Ek_Distance, int32_t Now_Distance)
{
    int32_t Gyro_Z_error;
    int64_t Gyro_Z_pwm_calc;
    int32_t Gyro_Z_pwm;

    if (fly_flag == 1)
    {
        Gyro_Z_error = Ek_Distance - Now_Distance;
        Gyro_Z_error_sum += Gyro_Z_error;

        Gyro_Z_error_sum = constrain_int(Gyro_Z_error_sum, -100, 100);

        Gyro_Z_pwm_calc = (int64_t)Gyro_z_Kp * Gyro_Z_error
                        + (int64_t)Gyro_z_Ki * Gyro_Z_error_sum
                        + (int64_t)Gyro_z_Kd * (Gyro_Z_error - Gyro_Z_Last_error);

        Gyro_Z_pwm = (int32_t)(Gyro_Z_pwm_calc >> Q_SHIFT);
        Gyro_Z_Last_error = Gyro_Z_error;
    }
    else
    {
        Gyro_Z_error_sum = 0;
        Gyro_Z_Last_error = 0;
        Gyro_Z_pwm = 0;
    }

    return Gyro_Z_pwm;
}


// 转向环 Z轴
int32_t DG_PID_Z(int32_t Ek_Distance, int32_t Now_Distance)
{
    int32_t DG_Z_error;
    int64_t DG_Z_pwm_calc;
    int32_t DG_Z_pwm;

    if(fly_flag == 1)
    {
        DG_Z_error = Ek_Distance - Now_Distance;
        DG_Z_error_sum += DG_Z_error;

        DG_Z_pwm_calc = (int64_t)DG_z_Kp * DG_Z_error
                      + (int64_t)DG_z_Ki * DG_Z_error_sum
                      + (int64_t)DG_z_Kd * (DG_Z_error - DG_Z_Last_error);

        DG_Z_pwm = (int32_t)(DG_Z_pwm_calc >> Q_SHIFT);
        DG_Z_Last_error = DG_Z_error;

        DG_Z_pwm = constrain_int(DG_Z_pwm, -500, 500);
    }
    else
    {
        DG_Z_error_sum = 0;
        DG_Z_pwm = 0;
        DG_Z_Last_error = 0;
    }
    return DG_Z_pwm;
}

// ************************************************
// 高度环
// ************************************************

int32_t Altitude_PID(int32_t Ek_Distance, int32_t Now_Distance)
{
    int32_t Altitude_error;
    int64_t Altitude_sp_calc;
    int32_t Altitude_sp;

    if((Gyro_Alt_mode_flag == 1 || Altitude_mode_flag == 1 || Direct_Alt_mode_flag == 1) && (fly_flag == 1 && GL_flag == 1))
    {
        Altitude_error = Ek_Distance - Now_Distance;

        // 5.2f -> 5 (假设输入没有放大), 如果输入是厘米，直接用整数比较
        if(ABS(Altitude_error) < 5)
            Altitude_error_sum += Altitude_error;

        Altitude_error_sum = constrain_int(Altitude_error_sum, -10, 10);

        Altitude_sp_calc = (int64_t)Altitude_Kp * Altitude_error
                         + (int64_t)Altitude_Ki * Altitude_error_sum
                         + (int64_t)Altitude_Kd * (Altitude_error - Altitude_Last_error);

        Altitude_sp = (int32_t)(Altitude_sp_calc >> Q_SHIFT);
        Altitude_sp = constrain_int(Altitude_sp, -2, 3); // 范围检查

        Altitude_Last_error = Altitude_error;
    }
    else
    {
        Altitude_error_sum = 0;
        Altitude_sp = 0;
        Altitude_Last_error = 0;
    }
    return Altitude_sp;
}

// 垂直速度环
int32_t Alt_SP_WZS_curb(int32_t Ek_Distance, int32_t Now_Distance)
{
    int32_t Alt_SP_WZS_error;
    int64_t Alt_SP_WZS_pwm_calc;
    int32_t Alt_SP_WZS_pwm;

    if(fly_flag == 1 && GL_flag == 1)
    {
        // 0.1f * (diff) -> 使用整数除法: diff / 10
        Alt_SP_WZS_error = (Ek_Distance - Now_Distance) / 10;

        Alt_SP_WZS_error = constrain_int(Alt_SP_WZS_error, -10, 10);
        Alt_SP_WZS_error_sum += Alt_SP_WZS_error;
        Alt_SP_WZS_error_sum = constrain_int(Alt_SP_WZS_error_sum, -200, 200);

        Alt_SP_WZS_pwm_calc = (int64_t)Alt_SP_WZS_Kp * Alt_SP_WZS_error
                            + (int64_t)Alt_SP_WZS_Ki * Alt_SP_WZS_error_sum
                            + (int64_t)Alt_SP_WZS_Kd * (Alt_SP_WZS_error - Alt_SP_WZS_Last_error);

        Alt_SP_WZS_pwm = (int32_t)(Alt_SP_WZS_pwm_calc >> Q_SHIFT);
        Alt_SP_WZS_Last_error = Alt_SP_WZS_error;

        Alt_SP_WZS_pwm = constrain_int(Alt_SP_WZS_pwm, -200, 200);
    }
    else
    {
        Alt_SP_WZS_error_sum = 0;
        Alt_SP_WZS_pwm = 0;
        Alt_SP_WZS_Last_error = 0;
    }
    return Alt_SP_WZS_pwm;
}

// ************************************************
// 方向控制
// ************************************************

int32_t Direct_X_PID(int32_t Ek_Distance, int32_t Now_Distance)
{
    int32_t Direct_X_error;
    int64_t Direct_X_sp_calc;
    int32_t Direct_X_sp;

    if(fly_flag == 1 && GL_flag == 1)
    {
        Direct_X_error = Ek_Distance - Now_Distance;

        Direct_X_sp_calc = (int64_t)Direct_X_Kp * Direct_X_error
                         + (int64_t)Direct_X_Kd * (Direct_X_error - Direct_X_Last_error);

        Direct_X_sp = (int32_t)(Direct_X_sp_calc >> Q_SHIFT);
        Direct_X_sp = constrain_int(Direct_X_sp, -5, 5);
        Direct_X_Last_error = Direct_X_error;
    }
    else
    {
        Direct_X_sp = 0;
        Direct_X_Last_error = 0;
    }
    return Direct_X_sp;
}

int32_t Direct_Y_PID(int32_t Ek_Distance, int32_t Now_Distance)
{
    int32_t Direct_Y_error;
    int64_t Direct_Y_sp_calc;
    int32_t Direct_Y_sp;

    if(fly_flag == 1 && GL_flag == 1)
    {
        Direct_Y_error = Ek_Distance - Now_Distance;

        Direct_Y_sp_calc = (int64_t)Direct_Y_Kp * Direct_Y_error
                         + (int64_t)Direct_Y_Kd * (Direct_Y_error - Direct_Y_Last_error);

        Direct_Y_sp = (int32_t)(Direct_Y_sp_calc >> Q_SHIFT);
        Direct_Y_sp = constrain_int(Direct_Y_sp, -5, 5);
        Direct_Y_Last_error = Direct_Y_error;
    }
    else
    {
        Direct_Y_sp = 0;
        Direct_Y_Last_error = 0;
    }
    return Direct_Y_sp;
}

int32_t Direct_X_SPcurb(int32_t Ek_Speed, int32_t Now_Speed)
{
    int32_t Direct_X_SP_error;
    int64_t Direct_X_SP_calc;
    int32_t Direct_X_SP;

    if(fly_flag == 1 && GL_flag == 1)
    {
        Direct_X_SP_error = Ek_Speed - Now_Speed;
        Direct_X_SP_error = constrain_int(Direct_X_SP_error, -300, 300);

        // 0.1 * error -> error / 10
        Direct_X_SP_error_sum += (Direct_X_SP_error / 10);
        Direct_X_SP_error_sum = constrain_int(Direct_X_SP_error_sum, -200, 200);

        Direct_X_SP_calc = (int64_t)Direct_X_SP_Kp * Direct_X_SP_error
                         + (int64_t)Direct_X_SP_Ki * Direct_X_SP_error_sum
                         + (int64_t)Direct_X_SP_Kd * (Direct_X_SP_error - Last_Direct_X_SP_error);

        Direct_X_SP = (int32_t)(Direct_X_SP_calc >> Q_SHIFT);
        Last_Direct_X_SP_error = Direct_X_SP_error;

        Direct_X_SP = constrain_int(Direct_X_SP, -5, 5);
    }
    else
    {
        Direct_X_SP_error_sum = 0;
        Direct_X_SP = 0;
        Last_Direct_X_SP_error = 0;
    }
    return Direct_X_SP;
}

int32_t Direct_Y_SPcurb(int32_t Ek_Speed, int32_t Now_Speed)
{
    int32_t Direct_Y_SP_error;
    int64_t Direct_Y_SP_calc;
    int32_t Direct_Y_SP;

    if(fly_flag == 1 && GL_flag == 1)
    {
        Direct_Y_SP_error = Ek_Speed - Now_Speed;
        Direct_Y_SP_error = constrain_int(Direct_Y_SP_error, -300, 300);

        Direct_Y_SP_error_sum += (Direct_Y_SP_error / 10);
        Direct_Y_SP_error_sum = constrain_int(Direct_Y_SP_error_sum, -200, 200);

        iiii = Direct_Y_SP_error_sum; // 全局变量

        Direct_Y_SP_calc = (int64_t)Direct_Y_SP_Kp * Direct_Y_SP_error
                         + (int64_t)Direct_Y_SP_Ki * Direct_Y_SP_error_sum
                         + (int64_t)Direct_Y_SP_Kd * (Direct_Y_SP_error - Last_Direct_Y_SP_error);

        Direct_Y_SP = (int32_t)(Direct_Y_SP_calc >> Q_SHIFT);
        Last_Direct_Y_SP_error = Direct_Y_SP_error;

        Direct_Y_SP = constrain_int(Direct_Y_SP, -5, 5);
    }
    else
    {
        Direct_Y_SP_error_sum = 0;
        Direct_Y_SP = 0;
        Last_Direct_Y_SP_error = 0;
    }
    return Direct_Y_SP;
}

int32_t ACC_Z_PID(int32_t Ek_Distance, int32_t Now_Distance)
{
    int32_t ACC_Z_error;
    int64_t ACC_Z_pwm_calc;
    int32_t ACC_Z_pwm;

    ACC_Z_error = Ek_Distance - Now_Distance;

    ACC_Z_pwm_calc = (int64_t)ACC_Z_Kp * ACC_Z_error
                   + (int64_t)ACC_Z_Kd * (ACC_Z_error - ACC_Z_Last_error);

    ACC_Z_pwm = (int32_t)(ACC_Z_pwm_calc >> Q_SHIFT);
    ACC_Z_pwm = constrain_int(ACC_Z_pwm, -50, 50);

    ACC_Z_Last_error = ACC_Z_error;
    return ACC_Z_pwm;
}

// ************************************************
// 倾角油门补偿 (整数快速近似算法)
// ************************************************

// 计算 1 - cos(angle_deg) 的 Q12 定点数近似值
// 物理近似：1 - cos(x) ≈ x^2 / 2 (小角度)
// 角度单位是度。
// 近似公式: result_Q12 = (angle^2 * 10) >> 4
// 推导: 45度时, 1-cos(45)=0.293. 0.293 * 4096 = 1200
//       用公式: (45*45*10)/16 = 20250/16 = 1265. (误差很小，完全可用)
int32_t OneMinusCos_Q12(int32_t angle_deg)
{
    // 取绝对值
    angle_deg = ABS(angle_deg);

    // 如果角度大于90度，简单返回最大值 (实际上无人机一般不会倾斜这么大)
    if(angle_deg > 90) return Q_FACTOR;

    // 计算近似值: (deg * deg * 10) / 16
    // 乘法可能溢出32位，所以用64位暂存
    int64_t temp = (int64_t)angle_deg * angle_deg * 10;
    return (int32_t)(temp >> 4);
}

int32_t Gyro_Alt_PID(int32_t Pitch_Now_Distance, int32_t Roll_Now_Distance)
{
    int32_t OneMinusCosVal;
    int64_t GA_pwm_calc;
    int32_t GA_pwm;
    int32_t max_angle;

    // 找出倾角较大的轴
    if(ABS(Pitch_Now_Distance) > ABS(Roll_Now_Distance))
        max_angle = ABS(Pitch_Now_Distance);
    else
        max_angle = ABS(Roll_Now_Distance);

    // 计算 1 - cos(angle) 的 Q12 值
    OneMinusCosVal = OneMinusCos_Q12(max_angle);

    // Gyro_alt_Kp 已经是 Q12 格式
    // YML_pwm 假设是普通整数 (1000-2000)
    // OneMinusCosVal 是 Q12 格式
    // 公式: Kp * (pwm - 200) * error
    // 结果: Q12 * int * Q12 = Q24. 需要右移 24 位变回 int?
    // 不，我们只希望保留一个 Q12 精度给最终输出吗？通常 PWM 是整数。
    // 所以结果应该右移 2*Q_SHIFT = 24位。

    // 假设 YML_pwm 是外部变量
    extern int32_t YML_pwm;

    GA_pwm_calc = (int64_t)Gyro_alt_Kp * (YML_pwm - 200) * OneMinusCosVal;

    // 右移 24 位 (因为乘了两个 Q12 变量)
    GA_pwm = (int32_t)(GA_pwm_calc >> (2 * Q_SHIFT));

    GA_pwm = constrain_int(GA_pwm, 0, 50); // 通常补偿是正的

    return GA_pwm;
}
