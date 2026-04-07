#ifndef __WDVHC_IMU_H
#define __WDVHC_IMU_H

#include <stdint.h>

/* --- 传感器原始数据与解算结果 --- */
// 修正：extern 声明数组时应指定大小（通常为3，代表X/Y/Z）
uint32_t isqrt(uint32_t n);
int32_t fix_invSqrt(int32_t x);
void ImuUpdate_Fixed(int32_t gx, int32_t gy, int32_t gz, int32_t ax, int32_t ay, int32_t az);
void perform_calibration(void);
void wdvhc_get_data_fixed(uint8_t on);


#endif /* __WDVHC_IMU_H */
