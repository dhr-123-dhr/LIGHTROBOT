#ifndef __CHASSIS_H
#define __CHASSIS_H

#include "main.h"
#include "MotorCtrl.h"
#include <stdbool.h>

/* ====== 底盘机械参数 (双驱差速) ====== */
#define WHEEL_DIAMETER      85.0f     /* 轮径 D (mm)                 */
#define WHEEL_TRACK         82.0f     /* 轮距 L (mm)                 */
#define STEPS_PER_REV       3200      /* 每圈步数 (1.8°, 16细分)     */
#define PI                  3.14159265f

/* ====== 步数计算宏 ====== */
/* 直线距离 → 步数 */
#define DIST_TO_STEPS(d_mm)  ((d_mm) / (PI * WHEEL_DIAMETER) * (float)STEPS_PER_REV)
/* 旋转角度 → 步数 (差速旋转: arc = r*theta, r=WHEEL_TRACK=转弯半径) */
#define ANGLE_TO_STEPS(deg)  ((deg) / 180.0f * WHEEL_TRACK / WHEEL_DIAMETER * (float)STEPS_PER_REV)

/* ====== 航向锁定 PD 参数 ====== */
#define CH_YAW_KP            3.0f     /* 航向偏差比例系数 (1/s) */
#define CH_YAW_KD            0.5f     /* 航向偏差微分系数 (s)   */
#define CH_YAW_DEADBAND      0.5f     /* 航向死区 (deg)         */
#define CH_ROTATION_THRESH   0.02f    /* 视为旋转模式的最小角速度命令 (相对值) */

/* ====== 运动模式枚举 ====== */
typedef enum {
    CH_MODE_IDLE = 0,
    CH_MODE_MOVING,     /* 直线运动: 锁头(yaw PD 修正) */
    CH_MODE_TURNING,    /* 原地旋转: 不锁头(陀螺仪只读不控) */
} Chassis_Mode;

/* ====== API: 初始化 ====== */
void Chassis_Init(void);

/* ====== API: 非阻塞运动命令 (发起-查询模型) ====== */
void Chassis_Move(float distance_mm);   /* 正=前进, 负=后退 */
void Chassis_Rotate(float angle_deg);   /* 正=左转, 负=右转 */

/* ====== API: 查询 ====== */
uint8_t Chassis_IsBusy(void);           /* 运动中返回 1, 空闲返回 0 */
bool   Chassis_IsTurning(void);         /* 是否处于旋转模式 */

/* ====== API: 急停 ====== */
void Chassis_EmergencyStop(void);

/* ====== 1ms 控制层 tick (由 chassisTask 调用) ====== */
void chassis_tick(void);

/* ====== 陀螺仪注入 (由 gyroTask 在 DMA 完成后调用) ====== */
void chassis_feed_gyro(float yaw_deg);

/* ====== 查询位姿和速度 (上位机 100ms 日志用) ====== */
void chassis_get_pose(float *x, float *y, float *theta);   /* 世界系位姿 (mm, mm, deg) */
void chassis_get_wheel_speed(float *vL, float *vR);        /* 左右轮目标线速度 (mm/s) */
float chassis_get_cmd_speed(void);                          /* 目标线速度 (mm/s) */

/* ====== 重置里程计 ====== */
void chassis_reset_odom(void);

#endif /* __CHASSIS_H */