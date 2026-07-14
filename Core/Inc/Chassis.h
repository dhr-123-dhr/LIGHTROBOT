#ifndef __CHASSIS_H
#define __CHASSIS_H

#include "main.h"
#include "MotorCtrl.h"

/* ---- 底盘机械参数 (按实际结构修改) ------------------------------ */
#define WHEEL_DIAMETER      65.0f     /* 轮径 D (mm)                 */
#define WHEEL_TRACK         150.0f    /* 轮距 L (mm)                 */
#define STEPS_PER_REV       3200      /* 每圈步数 (1.8°, 16细分)     */
#define PI                  3.14159265f

/* ---- 步数计算宏 ------------------------------------------------ */
/* 直线距离 → 步数 */
#define DIST_TO_STEPS(d_mm)  ((d_mm) / (PI * WHEEL_DIAMETER) * (float)STEPS_PER_REV)
/* 旋转角度 → 步数 (差速旋转，两轮反向各走一半) */
#define ANGLE_TO_STEPS(deg)  ((deg) / 360.0f * PI * WHEEL_TRACK / (PI * WHEEL_DIAMETER) * (float)STEPS_PER_REV)

/* API ------------------------------------------------------------ */
void Chassis_Init(void);
void Chassis_Move(float distance_mm);       /* 正=前进, 负=后退      */
void Chassis_Rotate(float angle_deg);       /* 正=左转, 负=右转      */
uint8_t Chassis_IsBusy(void);               /* 两电机均停止返回 0    */
void Chassis_EmergencyStop(void);           /* 立即停止两电机         */

#endif /* __CHASSIS_H */