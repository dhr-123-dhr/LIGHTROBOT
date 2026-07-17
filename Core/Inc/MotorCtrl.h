#ifndef __MOTORCTRL_H
#define __MOTORCTRL_H

#include "main.h"
#include "tim.h"

/* 梯形加减速参数 (可调) -------------------------------------------- */
#define MAX_SPEED       5100.0f     /* 最大速度 (步/秒), 1.5转/秒    */
#define MIN_SPEED       100.0f      /* 最小启动速度 (跳过低速死区)    */
#define MAX_ACCEL       1440.0f     /* 最大加速度 (步/秒²)           */
#define MAX_DELTA_SPEED 800.0f      /* 每 tick 最大速度变化 (防突变)  */
#define TIMER_CLK_HZ    1000000.0f  /* 计数器时钟 1 MHz (84MHz/84)   */
#define UPDATE_FREQ     1000u       /* 更新频率 1 kHz                */

/* 电机索引 -------------------------------------------------------- */
#define MOTOR_LEFT  0
#define MOTOR_RIGHT 1

/* 运动状态枚举 (梯形加减速) ---------------------------------------- */
typedef enum {
    STOP = 0,
    ACCEL,          /* 匀加速段                       */
    CONST_SPEED,    /* 匀速段                         */
    DECEL           /* 匀减速段                       */
} MotionState;

/* 单电机控制结构体 ------------------------------------------------ */
typedef struct {
    MotionState     state;
    float           last_speed;         /* 前一拍速度 (步/秒, 限幅用) */
    float           current_speed;      /* 当前速度 (步/秒)    */
    float           current_accel;      /* 当前加速度 (步/秒²)  */
    float           target_steps;       /* 目标步数            */
    float           current_steps;      /* 已走步数            */
    uint8_t         direction;          /* 0=正转, 1=反转      */
    TIM_HandleTypeDef *htim;            /* 定时器句柄          */
    uint32_t        channel;            /* PWM 通道            */
    GPIO_TypeDef    *dir_port;          /* DIR 引脚端口        */
    uint16_t        dir_pin;            /* DIR 引脚编号        */
} MotorCtrl_t;

/* 全局电机实例 ---------------------------------------------------- */
extern MotorCtrl_t motor[2];

/* API ------------------------------------------------------------ */
void MotorCtrl_Init(void);
void MotorCtrl_Start(uint8_t motor_id, float target_steps, uint8_t direction);
void MotorCtrl_Stop(uint8_t motor_id);
void MotorCtrl_Update(void);            /* SysTick ISR 中调用      */
uint8_t MotorCtrl_IsBusy(uint8_t motor_id);

#endif /* __MOTORCTRL_H */