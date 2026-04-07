#ifndef __PID_H
#define __PID_H

#include <stdint.h>

// =========================================================================
// 定点数配置 (与 PID.c 保持一致)
// =========================================================================
#define Q_SHIFT    12
#define Q_FACTOR   (1 << Q_SHIFT) // 4096

// 辅助宏
#define F2I(x)     ((int32_t)((x) * Q_FACTOR))

// =========================================================================
// 全局变量声明 (全部改为 int32_t)
// =========================================================================

// 调试变量
extern int32_t iiii;

// 外部标志位 (确保定义这些变量的地方也是整数类型)
extern uint8_t fly_flag;
extern uint8_t GL_flag;
extern uint8_t Gyro_Alt_mode_flag;
extern uint8_t Altitude_mode_flag;
extern uint8_t Direct_Alt_mode_flag;

// 油门值
extern int32_t YML_pwm;

// ---------------- PID 参数 (Q12定点数) ----------------

/* 角速度环 */
extern int32_t DG_x_Kp; extern int32_t DG_x_Ki; extern int32_t DG_x_Kd;
extern int32_t DG_y_Kp; extern int32_t DG_y_Ki; extern int32_t DG_y_Kd;
extern int32_t DG_z_Kp; extern int32_t DG_z_Ki; extern int32_t DG_z_Kd;

/* 角度环 */
extern int32_t Gyro_x_Kp; extern int32_t Gyro_x_Ki; extern int32_t Gyro_x_Kd;
extern int32_t Gyro_y_Kp; extern int32_t Gyro_y_Ki; extern int32_t Gyro_y_Kd;
extern int32_t Gyro_z_Kp; extern int32_t Gyro_z_Ki; extern int32_t Gyro_z_Kd;

/* 高度环 */
extern int32_t Altitude_Kp; extern int32_t Altitude_Ki; extern int32_t Altitude_Kd;
extern int32_t Alt_SP_WZS_Kp; extern int32_t Alt_SP_WZS_Ki; extern int32_t Alt_SP_WZS_Kd;

/* 方向控制 */
extern int32_t Direct_X_Kp; extern int32_t Direct_X_Ki; extern int32_t Direct_X_Kd;
extern int32_t Direct_Y_Kp; extern int32_t Direct_Y_Ki; extern int32_t Direct_Y_Kd;

extern int32_t Direct_X_SP_Kp; extern int32_t Direct_X_SP_Ki; extern int32_t Direct_X_SP_Kd;
extern int32_t Direct_Y_SP_Kp; extern int32_t Direct_Y_SP_Ki; extern int32_t Direct_Y_SP_Kd;

extern int32_t ACC_Z_Kp; extern int32_t ACC_Z_Kd;
extern int32_t Gyro_alt_Kp;

// =========================================================================
// 函数声明 (全部改为 int32_t)
// =========================================================================
void Reset_PID_Integrals(void);
/* 角速度环 PID */
int32_t DG_PID_X(int32_t Ek_Distance, int32_t Now_Distance);
int32_t DG_PID_Y(int32_t Ek_Distance, int32_t Now_Distance);
int32_t DG_PID_Z(int32_t Ek_Distance, int32_t Now_Distance);

/* 角度环 PID */
int32_t Gyro_PID_X(int32_t Ek_Distance, int32_t Now_Distance);
int32_t Gyro_PID_Y(int32_t Ek_Distance, int32_t Now_Distance);
int32_t Gyro_PID_Z(int32_t Ek_Distance, int32_t Now_Distance);

/* 高度环 PID */
int32_t Altitude_PID(int32_t Ek_Distance, int32_t Now_Distance);
int32_t Alt_SP_WZS_curb(int32_t Ek_Distance, int32_t Now_Distance);

/* 方向控制 PID */
int32_t Direct_X_PID(int32_t Ek_Distance, int32_t Now_Distance);
int32_t Direct_Y_PID(int32_t Ek_Distance, int32_t Now_Distance);

int32_t Direct_X_SPcurb(int32_t Ek_Speed, int32_t Now_Speed);
int32_t Direct_Y_SPcurb(int32_t Ek_Speed, int32_t Now_Speed);

int32_t ACC_Z_PID(int32_t Ek_Distance, int32_t Now_Distance);

/* 倾角补偿 */
int32_t Gyro_Alt_PID(int32_t Pitch_Now_Distance, int32_t Roll_Now_Distance);

#endif /* __PID_H */
