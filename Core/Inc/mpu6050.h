/**
 * @file    mpu6050.h
 * @brief   MPU6050 六轴传感器驱动(DMA 异步 + 在线零偏追踪 + 自动重校准)
 *
 * 硬件: I2C1 (PB6-SCL / PB7-SDA), 100 kHz, DMA 异步 14 字节读取.
 *
 * 使用流程:
 *   1. MPU6050_Init()           上电配置, 唤醒, 设置 500°/s 量程
 *   2. MPU6050_CalibrateYaw(n)  静止采集 n 组 gyroZ 作为零偏基准
 *   3. MPU6050_StartReadDMA()   启动一次 DMA 14 字节读(非阻塞)
 *   4. DMA 完成后 HAL_I2C_MemRxCpltCallback → MPU6050_OnDMAComplete()
 *   5. MPU6050_IntegrateYaw(dt) 积分偏航角
 *   6. MPU6050_GetYaw()         获取当前偏航角(度)
 */
#ifndef __MPU6050_H
#define __MPU6050_H

#include "main.h"
#include <stdint.h>
#include <stdbool.h>

/* ====== MPU6050 寄存器 ====== */
#define MPU6050_ADDR         (0x68 << 1)
#define MPU6050_REG_PWR1     0x6B
#define MPU6050_REG_GYRO_CFG 0x1B
#define MPU6050_REG_SMPLRT   0x19
#define MPU6050_REG_DLPF     0x1A
#define MPU6050_REG_ACC_XH   0x3B
#define MPU6050_REG_WHO_AM_I 0x75

/* ====== 校准缓存 ====== */
#define MPU6050_CALIB_BUF    256

/* ====== 零偏追踪默认 ====== */
#define MPU6050_BIAS_DECAY_LONG   0.00001f    /* 极缓慢衰减, 等效时延 >20s, 保留持续可重复零偏 */
#define MPU6050_BIAS_LEARN_RATE   0.001f      /* short期学习率, 1~2s 快速收敛 */
#define MPU6050_AUTO_BIAS_THRESH  0.3f        /* deg/s: z轴振幅低于此视为静止, 开启追踪 */
#define MPU6050_AUTO_BIAS_STD     0.15f       /* deg/s: z轴标准差低于此视为稳定, 触发重校准 */
#define MPU6050_AUTO_BIAS_SHORT  200         /* 短窗: 200 样本(200ms), 稳定后触发热校准 */
#define MPU6050_AUTO_BIAS_LONG    500         /* 长窗: 500 样本(500ms), 持续稳定后更新基准 */

/* ====== 初始化 ====== */
uint8_t MPU6050_Init(void);

/* ====== 校准 ====== */
void MPU6050_CalibrateYaw(uint16_t samples);

/* ====== DMA 异步读 ====== */
void MPU6050_StartReadDMA(void);

/* ====== DMA 完成回调(ISR 中调用) ====== */
void MPU6050_OnDMAComplete(void);

/* ====== 偏航积分 ====== */
void MPU6050_IntegrateYaw(float dt);

/* ====== 查询 ====== */
float MPU6050_GetYaw(void);       /* 偏航角(度) */
bool  MPU6050_IsReady(void);      /* 数据是否有效 */
float MPU6050_GetTempC(void);     /* 温度(摄氏度) */
float MPU6050_GetGyroZ(void);     /* 原始角速度(dps) */

/* ====== 兼容 Robot DMP API (包装层) ====== */
uint8_t mpu_dmp_init(void);
uint8_t mpu_dmp_get_data(float *pitch, float *roll, float *yaw);

#endif /* __MPU6050_H */
