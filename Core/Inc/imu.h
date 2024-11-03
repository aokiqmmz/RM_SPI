//
// Created by h on 2024/11/3.
//

#ifndef IMU_H
#define IMU_H
#include <stdint.h>

float linearMapping(int in, int in_min, int in_max, float out_min, float out_max);
void BMI088_ACCEL_NS_L(void);
void BMI088_ACCEL_NS_R(void);
void BMI088_GYRO_NS_L(void);
void BMI088_GYRO_NS_R(void);
static void BMI088_read_single_reg(uint8_t reg, uint8_t *return_data);
static void BMI088_write_single_reg(uint8_t reg, uint8_t data);
void read_gyro_rate(void);
void BMI088_Init(void);
void read_accel_rate(void);

#endif //IMU_H
