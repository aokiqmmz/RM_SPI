//
// Created by h on 2024/11/3.
//

#include "imu.h"
#include "main.h"
#include "spi.h"
#include "stm32f4xx_hal_conf.h"



float linearMapping(int in, int in_min, int in_max, float out_min, float out_max) {
    float k = (out_max - out_min) / (in_max - in_min);
    float out = k * (in - in_min) + out_min;
    return out;
}

void BMI088_ACCEL_NS_L(void) {
    HAL_GPIO_WritePin(CS1_Accel_GPIO_Port, CS1_Accel_Pin, GPIO_PIN_RESET);
}
void BMI088_ACCEL_NS_H(void) {
    HAL_GPIO_WritePin(CS1_Accel_GPIO_Port, CS1_Accel_Pin, GPIO_PIN_SET);
}
void BMI088_GYRO_NS_L(void) {
    HAL_GPIO_WritePin(CS1_Gyro_GPIO_Port, CS1_Gyro_Pin, GPIO_PIN_RESET);
}
void BMI088_GYRO_NS_H(void) {
    HAL_GPIO_WritePin(CS1_Gyro_GPIO_Port, CS1_Gyro_Pin, GPIO_PIN_SET);
}

void BMI088_WriteReg(uint8_t reg, uint8_t write_data) {
    reg = reg & 0x7F;
    BMI088_ACCEL_NS_L();
    HAL_SPI_Transmit(&hspi1, &reg, 1, HAL_MAX_DELAY);
    while (HAL_SPI_GetState(&hspi1) == HAL_SPI_STATE_BUSY_TX);
    HAL_SPI_Transmit(&hspi1, &write_data, 1, HAL_MAX_DELAY);
    while (HAL_SPI_GetState(&hspi1) == HAL_SPI_STATE_BUSY_TX);
    BMI088_ACCEL_NS_H();
}

void BMI088_Init() {
    BMI088_ACCEL_NS_L();
    BMI088_WriteReg(0x7E, 0xB6);
    BMI088_ACCEL_NS_H();

    BMI088_GYRO_NS_L();
    BMI088_WriteReg(0x14, 0xB6);
    BMI088_GYRO_NS_H();

    BMI088_ACCEL_NS_L();
    BMI088_WriteReg(0x7D, 0x04);
    BMI088_ACCEL_NS_H();
}

#define Reg_GYRO_RATE_X_LSB 0x02
#define Reg_GYRO_RATE_X_MSB 0x03
#define Reg_GYRO_RATE_Y_LSB 0x04
#define Reg_GYRO_RATE_Y_MSB 0x05
#define Reg_GYRO_RATE_Z_LSB 0x06
#define Reg_GYRO_RATE_Z_MSB 0x07

#define Reg_ACCEL_RATE_X_LSB 0x12
#define Reg_ACCEL_RATE_X_MSB 0x13
#define Reg_ACCEL_RATE_Y_LSB 0x14
#define Reg_ACCEL_RATE_Y_MSB 0x15
#define Reg_ACCEL_RATE_Z_LSB 0x16
#define Reg_ACCEL_RATE_Z_MSB 0x17


typedef struct {
    uint8_t RATE_X_LSB;
    uint8_t RATE_X_MSB;
    uint8_t RATE_Y_LSB;
    uint8_t RATE_Y_MSB;
    uint8_t RATE_Z_LSB;
    uint8_t RATE_Z_MSB;
    int16_t RATE_X;
    int16_t RATE_Y;
    int16_t RATE_Z;
    float angular_rate_x;
    float angular_rate_y;
    float angular_rate_z;
} gyro;
gyro gyro_c;

typedef struct{
  uint8_t RATE_X_LSB;
  uint8_t RATE_X_MSB;
  uint8_t RATE_Y_LSB;
  uint8_t RATE_Y_MSB;
  uint8_t RATE_Z_LSB;
  uint8_t RATE_Z_MSB;
  int16_t RATE_X;
  int16_t RATE_Y;
  int16_t RATE_Z;
  float accel_x;
  float accel_y;
  float accel_z;
} accel;
accel accel_c;


static void BMI088_read_gyro_single_reg(uint8_t reg, uint8_t *return_data) {
    BMI088_GYRO_NS_L();
    uint8_t reg_send = reg|0x80;
    HAL_SPI_Transmit(&hspi1, &reg_send, 1, 100);
    HAL_SPI_Receive(&hspi1, return_data, 1, 100);
    BMI088_GYRO_NS_H();
}

static void BMI088_read_acc_single_reg(uint8_t reg, uint8_t *return_data) {
    BMI088_ACCEL_NS_L();
    uint8_t reg_send = reg|0x80;
    HAL_SPI_Transmit(&hspi1, &reg_send, 1, 100);
    HAL_SPI_Receive(&hspi1, return_data, 1, 100);
    BMI088_ACCEL_NS_H();
}

static void BMI088_write_gyro_single_reg(uint8_t reg, uint8_t data) {
  	BMI088_GYRO_NS_L();
	uint8_t reg_send = reg|0x7F;
	HAL_SPI_Transmit(&hspi1, &reg_send, 1, 100);
	HAL_SPI_Transmit(&hspi1, &data, 1, 100);
    BMI088_GYRO_NS_H();
}

static void BMI088_write_acc_single_reg(uint8_t reg, uint8_t data) {
  	BMI088_ACCEL_NS_L();
	uint8_t reg_send = reg|0x7F;
	HAL_SPI_Transmit(&hspi1, &reg_send, 1, 100);
	HAL_SPI_Transmit(&hspi1, &data, 1, 100);
    BMI088_ACCEL_NS_H();
}

/*
static void BMI088_read_single_reg(uint8_t reg, uint8_t *return_data) {
  HAL_SPI_TransmitReceive(&hspi1, reg|0x80, return_data, 1, 100);
}

static void BMI088_write_single_reg(uint8_t reg, uint8_t data) {
  HAL_SPI_TransmitReceive(&hspi1, reg&0x7F, NULL, 1, 100);
  HAL_SPI_TransmitReceive(&hspi1, &data, NULL, 1, 100);
}
*/

void read_gyro_rate(void) {
    BMI088_read_gyro_single_reg(Reg_GYRO_RATE_X_LSB, &gyro_c.RATE_X_LSB);
    BMI088_read_gyro_single_reg(Reg_GYRO_RATE_X_MSB, &gyro_c.RATE_X_MSB);
    BMI088_read_gyro_single_reg(Reg_GYRO_RATE_Y_LSB, &gyro_c.RATE_Y_LSB);
    BMI088_read_gyro_single_reg(Reg_GYRO_RATE_Y_MSB, &gyro_c.RATE_Y_MSB);
    BMI088_read_gyro_single_reg(Reg_GYRO_RATE_Z_LSB, &gyro_c.RATE_Z_LSB);
    BMI088_read_gyro_single_reg(Reg_GYRO_RATE_Z_MSB, &gyro_c.RATE_Z_MSB);
    gyro_c.RATE_X = ((int16_t)gyro_c.RATE_X_MSB)<<8 | gyro_c.RATE_X_LSB;
    gyro_c.RATE_Y = ((int16_t)gyro_c.RATE_Y_MSB)<<8 | gyro_c.RATE_Y_LSB;
    gyro_c.RATE_Z = ((int16_t)gyro_c.RATE_Z_MSB)<<8 | gyro_c.RATE_Z_LSB;
    gyro_c.angular_rate_x = linearMapping(gyro_c.RATE_X, -32767, 32767, -2000, 2000);
    gyro_c.angular_rate_y = linearMapping(gyro_c.RATE_Y, -32767, 32767, -2000, 2000);
    gyro_c.angular_rate_z = linearMapping(gyro_c.RATE_Z, -32767, 32767, -2000, 2000);
}

void read_accel_rate(void){
	BMI088_read_acc_single_reg(Reg_ACCEL_RATE_X_LSB, &accel_c.RATE_X_LSB);
	BMI088_read_acc_single_reg(Reg_ACCEL_RATE_X_MSB, &accel_c.RATE_X_MSB);
	BMI088_read_acc_single_reg(Reg_ACCEL_RATE_Y_LSB, &accel_c.RATE_Y_LSB);
	BMI088_read_acc_single_reg(Reg_ACCEL_RATE_Y_MSB, &accel_c.RATE_Y_MSB);
	BMI088_read_acc_single_reg(Reg_ACCEL_RATE_Z_LSB, &accel_c.RATE_Z_LSB);
	BMI088_read_acc_single_reg(Reg_ACCEL_RATE_Z_MSB, &accel_c.RATE_Z_MSB);
	accel_c.RATE_X = ((int16_t)accel_c.RATE_X_MSB) << 8 | accel_c.RATE_X_LSB;
	accel_c.RATE_Y = ((int16_t)accel_c.RATE_Y_MSB) << 8 | accel_c.RATE_Y_LSB;
	accel_c.RATE_Z = ((int16_t)accel_c.RATE_Z_MSB) << 8 | accel_c.RATE_Z_LSB;
	accel_c.accel_x = linearMapping(accel_c.RATE_X, -32767, 32767, -12, 12);
    accel_c.accel_y = linearMapping(accel_c.RATE_Y, -32767, 32767, -12, 12);
    accel_c.accel_z = linearMapping(accel_c.RATE_Z, -32767, 32767, -12, 12);
}