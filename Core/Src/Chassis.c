#include "Chassis.h"
#include "MotorCtrl.h"

/* 初始化底盘 ------------------------------------------------------ */
void Chassis_Init(void)
{
    MotorCtrl_Init();
}

/* 直线运动 -------------------------------------------------------- */
void Chassis_Move(float distance_mm)
{
    float steps = DIST_TO_STEPS(distance_mm);
    if (steps < 0.0f) steps = -steps;

    uint8_t dir;
    if (distance_mm >= 0.0f) {
        dir = 0;    /* 前进：两轮同步正转 */
    } else {
        dir = 1;    /* 后退：两轮同步反转 */
    }

    MotorCtrl_Start(MOTOR_LEFT,  steps, dir);
    MotorCtrl_Start(MOTOR_RIGHT, steps, dir);
}

/* 原地旋转 -------------------------------------------------------- */
void Chassis_Rotate(float angle_deg)
{
    float steps = ANGLE_TO_STEPS(angle_deg);
    if (steps < 0.0f) steps = -steps;

    if (angle_deg > 0.0f) {
        /* 左转：左轮后退，右轮前进 */
        MotorCtrl_Start(MOTOR_LEFT,  steps, 1);
        MotorCtrl_Start(MOTOR_RIGHT, steps, 0);
    } else {
        /* 右转：左轮前进，右轮后退 */
        MotorCtrl_Start(MOTOR_LEFT,  steps, 0);
        MotorCtrl_Start(MOTOR_RIGHT, steps, 1);
    }
}

/* 查询底盘是否运动中 ---------------------------------------------- */
uint8_t Chassis_IsBusy(void)
{
    return MotorCtrl_IsBusy(MOTOR_LEFT) || MotorCtrl_IsBusy(MOTOR_RIGHT);
}

/* 紧急停止 -------------------------------------------------------- */
void Chassis_EmergencyStop(void)
{
    /* 左轮 */
    HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_1);
    motor[MOTOR_LEFT].state         = STOP;
    motor[MOTOR_LEFT].current_speed = 0.0f;
    motor[MOTOR_LEFT].current_accel = 0.0f;

    /* 右轮 */
    HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_1);
    motor[MOTOR_RIGHT].state         = STOP;
    motor[MOTOR_RIGHT].current_speed = 0.0f;
    motor[MOTOR_RIGHT].current_accel = 0.0f;
}