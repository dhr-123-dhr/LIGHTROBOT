/**
 * @file    mpu6050.c
 * @brief   MPU6050 六轴传感器驱动实现
 *
 * 关键设计:
 *   - DMA 14 字节批量读, 不阻塞 CPU
 *   - 在线零偏追踪: short(200样本) 快速收敛 + long(500样本) 持续微调基准
 *   - 自动重校准: 静止检测 stable_cnt > long → 更新 calib_bias
 *   - 温度补偿: 温度漂移 > 5°C 触发重校准
 */
#include "mpu6050.h"
#include "i2c.h"
#include <math.h>
#include <string.h>

/* ====== I2C 辅助宏 ====== */
#define MPU6050_I2C   hi2c1

/* ====== 静态变量 ====== */
static uint8_t  dma_buf[14];              /* DMA 接收缓冲: [accXH,accXL,...,gyroZH,gyroZL] */
static volatile bool dma_done = false;     /* DMA 完成标志(ISR 设置, 主循环读取) */

static float gyro_z_raw = 0.0f;           /* 原始角速度(dps), 已去零偏 */
static float yaw_deg    = 0.0f;            /* 积分偏航角(度) */
static bool  data_valid = false;

/* 零偏基准(上电校准得到, 仅自动重校准更新避免温度温漂) */
static float calib_bias = 0.0f;

/* 短期慢跟随零偏(快速收敛, 防止纯死区抑制慢零漂) */
static float short_bias = 0.0f;
static float long_bias  = 0.0f;            /* 长期滑动均值(温漂补偿) */

/* 静止检测窗: 环形缓冲 */
static float gyro_win[MPU6050_AUTO_BIAS_LONG];
static uint16_t win_idx = 0;
static uint16_t win_cnt = 0;
static uint16_t stable_cnt = 0;            /* 连续静止样本计数 */

/* 温度补偿基准 */
static float base_temp = 25.0f;            /* 校准时温度 */
static float curr_temp = 25.0f;

/* ====== I2C 单字节写 ====== */
static uint8_t i2c_write(uint8_t reg, uint8_t val)
{
    return HAL_I2C_Mem_Write(&MPU6050_I2C, MPU6050_ADDR, reg,
                             I2C_MEMADD_SIZE_8BIT, &val, 1, 50);
}

/* ====== I2C 单字节读 ====== */
static uint8_t i2c_read(uint8_t reg, uint8_t *val)
{
    return HAL_I2C_Mem_Read(&MPU6050_I2C, MPU6050_ADDR, reg,
                            I2C_MEMADD_SIZE_8BIT, val, 1, 50);
}

/* ====== 3σ 鲁棒均值(剔除离群) ====== */
static float robust_mean(const float *buf, uint16_t n)
{
    if (n == 0) return 0.0f;
    /* 均值 */
    float sum = 0.0f;
    for (uint16_t i = 0; i < n; i++) sum += buf[i];
    float mean = sum / (float)n;
    /* 标准差 */
    float var = 0.0f;
    for (uint16_t i = 0; i < n; i++) {
        float d = buf[i] - mean;
        var += d * d;
    }
    float std = sqrtf(var / (float)n);
    /* 3σ 裁切后重算均值 */
    float lo = mean - 3.0f * std;
    float hi = mean + 3.0f * std;
    sum = 0.0f;
    uint16_t m = 0;
    for (uint16_t i = 0; i < n; i++) {
        if (buf[i] >= lo && buf[i] <= hi) {
            sum += buf[i];
            m++;
        }
    }
    return m > 0 ? sum / (float)m : mean;
}

/* ====== 初始化 ====== */
uint8_t MPU6050_Init(void)
{
    uint8_t who;
    /* 复位 I2C */
    HAL_I2C_DeInit(&MPU6050_I2C);
    MX_I2C1_Init();
    HAL_Delay(10);

    /* 检查 WHO_AM_I */
    if (i2c_read(MPU6050_REG_WHO_AM_I, &who) != HAL_OK) return 1;
    if (who != 0x68) return 2;

    /* 唤醒(清 sleep 位) */
    if (i2c_write(MPU6050_REG_PWR1, 0x01) != HAL_OK) return 3;
    HAL_Delay(50);

    /* 设置陀螺仪量程 ±500°/s (FS_SEL=1) → 65.5 LSB/(°/s) */
    if (i2c_write(MPU6050_REG_GYRO_CFG, 0x08) != HAL_OK) return 4;

    /* 采样率分频器: SMPLRT = 0 (1 kHz 采样) */
    if (i2c_write(MPU6050_REG_SMPLRT, 0x00) != HAL_OK) return 5;

    /* DLPF: 带宽 = 98 Hz (DLPF_CFG=2), 延迟约 2.8 ms */
    if (i2c_write(MPU6050_REG_DLPF, 0x02) != HAL_OK) return 6;

    /* 初始状态 */
    yaw_deg = 0.0f;
    data_valid = false;
    calib_bias = 0.0f;
    short_bias = 0.0f;
    long_bias = 0.0f;
    memset(gyro_win, 0, sizeof(gyro_win));
    win_idx = 0;
    win_cnt = 0;
    stable_cnt = 0;

    return 0;
}

/* ====== 上电静止校准 ====== */
void MPU6050_CalibrateYaw(uint16_t samples)
{
    if (samples > MPU6050_CALIB_BUF) samples = MPU6050_CALIB_BUF;

    /* 预热: 丢弃前 50 组 */
    for (uint16_t i = 0; i < 50; i++) {
        MPU6050_StartReadDMA();
        uint32_t t = HAL_GetTick();
        while (!dma_done && (HAL_GetTick() - t < 10)) {}
        if (dma_done) {
            /* 解析 gyroZ */
            int16_t gz = (int16_t)((dma_buf[12] << 8) | dma_buf[13]);
            gyro_z_raw = gz / 65.5f;
        }
        HAL_Delay(1);
    }

    /* 采集样本 */
    static float buf[MPU6050_CALIB_BUF];
    for (uint16_t i = 0; i < samples; i++) {
        MPU6050_StartReadDMA();
        uint32_t t = HAL_GetTick();
        while (!dma_done && (HAL_GetTick() - t < 10)) {}
        if (dma_done) {
            int16_t gz = (int16_t)((dma_buf[12] << 8) | dma_buf[13]);
            buf[i] = gz / 65.5f;
        }
        HAL_Delay(1);
    }

    calib_bias = robust_mean(buf, samples);
    short_bias = calib_bias;
    long_bias  = calib_bias;
    base_temp  = MPU6050_GetTempC();
}

/* ====== 启动 DMA 异步 14 字节读 ====== */
void MPU6050_StartReadDMA(void)
{
    dma_done = false;
    HAL_I2C_Mem_Read_DMA(&MPU6050_I2C, MPU6050_ADDR,
                         MPU6050_REG_ACC_XH, I2C_MEMADD_SIZE_8BIT,
                         dma_buf, 14);
}

/* ====== DMA 完成回调(ISR 上下文) ====== */
void MPU6050_OnDMAComplete(void)
{
    dma_done = true;

    /* 检查数据合理性(gyroZ 不应该是全 FF 或全 00) */
    int16_t gz = (int16_t)((dma_buf[12] << 8) | dma_buf[13]);
    if (gz == 0x7FFF || gz == 0x8000 || gz == 0) {
        /* 异常数据, 不更新 */
        return;
    }

    /* 解析原始值(±500°/s 量程 → 65.5 LSB/(°/s)) */
    float gz_dps = gz / 65.5f;

    /* 温度(用于温漂检测) */
    int16_t t_raw = (int16_t)((dma_buf[6] << 8) | dma_buf[7]);
    curr_temp = t_raw / 340.0f + 36.53f;

    /* 减去短期零偏 */
    gyro_z_raw = gz_dps - short_bias;

    /* 静止检测窗更新 */
    gyro_win[win_idx] = gz_dps - long_bias;
    win_idx = (win_idx + 1) % MPU6050_AUTO_BIAS_LONG;
    if (win_cnt < MPU6050_AUTO_BIAS_LONG) win_cnt++;

    /* 在线零偏追踪:
     *   - short: 200 样本窗, 标准差 < 0.15 dps → 快速学习静止漂移
     *   - long:  500 样本窗, 持续稳定 → 更新基准 calib_bias(温度温漂补偿)
     *   - 温度跳变 > 5°C → 标记为重校准 */
    if (win_cnt >= MPU6050_AUTO_BIAS_SHORT)
    {
        /* 短窗统计: 最近 short 个样本 */
        uint16_t sn = MPU6050_AUTO_BIAS_SHORT;
        float s_sum = 0.0f;
        for (uint16_t i = 0; i < sn; i++) {
            uint16_t idx = (win_idx + MPU6050_AUTO_BIAS_LONG - sn + i) % MPU6050_AUTO_BIAS_LONG;
            s_sum += gyro_win[idx];
        }
        float s_mean = s_sum / (float)sn;
        float s_var = 0.0f;
        for (uint16_t i = 0; i < sn; i++) {
            uint16_t idx = (win_idx + MPU6050_AUTO_BIAS_LONG - sn + i) % MPU6050_AUTO_BIAS_LONG;
            float d = gyro_win[idx] - s_mean;
            s_var += d * d;
        }
        float s_std = sqrtf(s_var / (float)sn);

        float gyro_amp = gz_dps - long_bias;
        if (gyro_amp < 0.0f) gyro_amp = -gyro_amp;

        if (gyro_amp < MPU6050_AUTO_BIAS_THRESH && s_std < MPU6050_AUTO_BIAS_STD)
        {
            /* 静止: 短期零偏快速跟随(1~2s 收敛) */
            short_bias += MPU6050_BIAS_LEARN_RATE * (gz_dps - short_bias);
            stable_cnt++;

            /* 长窗(500 样本)稳定 → 更新基准 */
            if (win_cnt >= MPU6050_AUTO_BIAS_LONG && stable_cnt > MPU6050_AUTO_BIAS_LONG)
            {
                /* 长窗均值 */
                float l_sum = 0.0f;
                for (uint16_t i = 0; i < MPU6050_AUTO_BIAS_LONG; i++)
                    l_sum += gyro_win[i];
                float l_mean = l_sum / (float)MPU6050_AUTO_BIAS_LONG;
                /* 更新基准(极慢速率, 防止剧烈跳变) */
                calib_bias += 0.0005f * (l_mean - 0.0f);  /* l_mean 已是去 long_bias 的偏差 */
                /* 更新长期滑动基准 */
                long_bias += 0.0005f * l_mean;
            }
        }
        else
        {
            stable_cnt = 0;
            /* 缓慢衰减短期零偏回基准, 防止运动时 short_bias 漂走 */
            float decay = MPU6050_BIAS_DECAY_LONG;
            short_bias += decay * (long_bias - short_bias);
        }

        /* 温度温漂 > 5°C → 触发热重置(取近 200 样本均值更新基准) */
        float dt_temp = curr_temp - base_temp;
        float dt_abs = dt_temp < 0.0f ? -dt_temp : dt_temp;
        if (dt_abs > 5.0f && win_cnt >= MPU6050_AUTO_BIAS_SHORT && stable_cnt > MPU6050_AUTO_BIAS_SHORT)
        {
            float h_sum = 0.0f;
            for (uint16_t i = 0; i < sn; i++) {
                uint16_t idx = (win_idx + MPU6050_AUTO_BIAS_LONG - sn + i) % MPU6050_AUTO_BIAS_LONG;
                h_sum += gyro_win[idx];
            }
            float new_bias = long_bias + h_sum / (float)sn;
            calib_bias = calib_bias * 0.7f + new_bias * 0.3f;
            long_bias  = calib_bias;
            short_bias = calib_bias;
            base_temp  = curr_temp;
        }
    }

    data_valid = true;
}

/* ====== 定积分偏航角 ====== */
void MPU6050_IntegrateYaw(float dt)
{
    if (dt <= 0.0f || dt > 0.01f) return;  /* 安全钳: 防止异常 dt */
    if (!data_valid) return;
    yaw_deg += gyro_z_raw * dt;
}

/* ====== 查询接口 ====== */
float MPU6050_GetYaw(void)   { return yaw_deg; }
bool  MPU6050_IsReady(void)  { return data_valid; }
float MPU6050_GetTempC(void) { return curr_temp; }
float MPU6050_GetGyroZ(void) { return gyro_z_raw; }

/* ====== Debug: 重置偏航角 ====== */
void MPU6050_ResetYaw(void) { yaw_deg = 0.0f; }

/* ================================================================
 * Robot DMP API 兼容包装层 (对齐 Robot 项目调用方式)
 *
 * Robot 项目使用 MPU6050 的 DMP (Digital Motion Processor)
 * 通过 I2C FIFO 轮询直接输出欧拉角, 调用方式为:
 *   mpu_dmp_init()          上电初始化 DMP
 *   mpu_dmp_get_data(&p, &r, &y)  读取 pitch/roll/yaw
 *
 * 本项目使用 DMA 异步读取 + 纯积分偏航角方案, 通过此包装层
 * 提供完全相同的函数签名, 内部映射到现有驱动.
 *
 * 调用流程:
 *   1. gyroTask: mpu_dmp_init()  → MPU6050_Init() + MPU6050_CalibrateYaw(200)
 *   2. gyroTask 1ms 循环:
 *      - MPU6050_StartReadDMA() 启动 DMA 读
 *      - 等待 DMA 完成 (在 ISR 中调用 MPU6050_OnDMAComplete)
 *      - MPU6050_IntegrateYaw(0.001f) 积分偏航
 *      - mpu_dmp_get_data(&p, &r, &y) → 获取 pitch=0, roll=0, yaw=MPU6050_GetYaw()
 * ================================================================ */

/* 包装层上次积分时间戳 (ms), 用于 dt 计算 */
static uint32_t dmp_last_ms = 0;
static bool     dma_pending  = false;  /* 非阻塞流水线: 是否有 DMA 传输正在进行中 */

/* mpu_dmp_init: 初始化 + 上电校准 */
uint8_t mpu_dmp_init(void)
{
    uint8_t ret;
    ret = MPU6050_Init();
    if (ret != 0) return ret;

    /* 上电静止校准 200 样本 (约 200ms), 假设上电时机身静止 */
    MPU6050_CalibrateYaw(200);

    /* 初始化积分时间戳 + DMA 流水线状态 */
    dmp_last_ms = HAL_GetTick();
    dma_pending = false;

    return 0;
}

/* mpu_dmp_get_data: 获取欧拉角 (pitch/roll 填充 0, 仅提供 yaw)
 *
 * 非阻塞流水线设计 (修复高优先级忙等饿死 printTask):
 *   - 首次调用: 启动 DMA, 返回旧 yaw (尚未积分新数据)
 *   - 后续调用: 若上次 DMA 已就绪 → 积分 → 启动新 DMA → 返回
 *             若上次 DMA 未就绪 → 跳过积分 → 返回旧 yaw
 *   - 每次调用 < 50us (无忙等), 严格不阻塞
 *
 * 代价: yaw 数据滞后 1 个调用周期 (~1ms), 对航向锁定无影响
 */
uint8_t mpu_dmp_get_data(float *pitch, float *roll, float *yaw)
{
    if (pitch) *pitch = 0.0f;
    if (roll)  *roll  = 0.0f;

    if (yaw)
    {
        /* 计算 dt */
        uint32_t now_ms = HAL_GetTick();
        float dt = (float)(now_ms - dmp_last_ms) / 1000.0f;
        dmp_last_ms = now_ms;

        /* 安全钳: dt 异常 (断流/溢出) 使用 1ms */
        if (dt <= 0.0f || dt > 0.01f) dt = 0.001f;

        /* === 非阻塞流水线 ===
         * 步骤1: 检查上次 DMA 是否完成 → 积分上次数据 */
        if (dma_pending && dma_done)
        {
            MPU6050_IntegrateYaw(dt);
            dma_pending = false;
            dma_done    = false;  /* 防重入: 清标志位 */
        }

        /* 步骤2: 若没有进行中的 DMA, 启动新传输供下次调用使用 */
        if (!dma_pending)
        {
            MPU6050_StartReadDMA();  /* 内部设置 dma_done=false */
            dma_pending = true;
        }

        /* 步骤3: 立即返回当前 yaw (可能是旧值, 1ms 滞后可接受) */
        *yaw = MPU6050_GetYaw();
    }

    return (data_valid ? 0 : 1);
}
