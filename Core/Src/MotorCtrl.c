/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file    MotorCtrl.c
 * @brief   双步进电机 PWM 脉冲发生 + 梯形加减速状态机
 *
 * 1kHz SysTick 驱动
 *   - 双预装载 (ARPE + OCxPE) 在 tim.c 初始化时已使能
 *   - __HAL_TIM_SET_AUTORELOAD / __HAL_TIM_SET_COMPARE 直接写影子寄存器
 *     (不写 UG, 不重置 CNT, 由硬件在溢出时自动装载)
 *   - 停止用 CCR=0 软停止, 不立即关定时器 (避免截断当前脉冲)
 *   - 速度变化率限幅 (防突变丢步)
 *
 * 梯形加减速核心公式: v² = v0² + 2·a·s
 *   - 减速判据: 剩余步数 ≤ v²/(2a) (等价于 project 的 REAL_DEC_STEPS)
 *   - 从 MIN_SPEED 起步跳过低速死区
 ******************************************************************************
 */
/* USER CODE END Header */
#include "MotorCtrl.h"
#include <math.h>

MotorCtrl_t motor[2];

/* PWM 已启动标志 -------------------------------------------------- */
static uint8_t pwm_started[2] = {0, 0};

/* 内部辅助: 取绝对值 ---------------------------------------------- */
static float fabsf_local(float v)
{
    return (v < 0.0f) ? -v : v;
}

/* 电机初始化 ------------------------------------------------------ */
void MotorCtrl_Init(void)
{
    /* 电机1 (TIM2 CH1, PA5 STP, PA7 DIR) */
    motor[MOTOR_LEFT].state = STOP;
    motor[MOTOR_LEFT].last_speed = 0.0f;
    motor[MOTOR_LEFT].current_speed = 0.0f;
    motor[MOTOR_LEFT].current_accel = 0.0f;
    motor[MOTOR_LEFT].target_steps = 0.0f;
    motor[MOTOR_LEFT].current_steps = 0.0f;
    motor[MOTOR_LEFT].direction = 0;
    motor[MOTOR_LEFT].pending_cnt = 0;
    motor[MOTOR_LEFT].htim = &htim2;
    motor[MOTOR_LEFT].channel = TIM_CHANNEL_1;
    motor[MOTOR_LEFT].dir_port = GPIOA;
    motor[MOTOR_LEFT].dir_pin = GPIO_PIN_7;

    /* 右电机 (TIM3 CH1, PA6 STP, PC4 DIR) */
    motor[MOTOR_RIGHT].state = STOP;
    motor[MOTOR_RIGHT].last_speed = 0.0f;
    motor[MOTOR_RIGHT].current_speed = 0.0f;
    motor[MOTOR_RIGHT].current_accel = 0.0f;
    motor[MOTOR_RIGHT].target_steps = 0.0f;
    motor[MOTOR_RIGHT].current_steps = 0.0f;
    motor[MOTOR_RIGHT].direction = 0;
    motor[MOTOR_RIGHT].pending_cnt = 0;
    motor[MOTOR_RIGHT].htim = &htim3;
    motor[MOTOR_RIGHT].channel = TIM_CHANNEL_1;
    motor[MOTOR_RIGHT].dir_port = GPIOC;
    motor[MOTOR_RIGHT].dir_pin = GPIO_PIN_4;

    /* PSC 已在 MX_TIM2_Init / MX_TIM3_Init 中配置为 83 */
    /* ARPE + OCxPE 已在 tim.c Init 中使能 */
}

/* 启动电机运动 ---------------------------------------------------- */
void MotorCtrl_Start(uint8_t motor_id, float target_steps, uint8_t direction)
{
    if (motor_id > 1) return;

    MotorCtrl_t *m = &motor[motor_id];

    m->target_steps = target_steps;
    m->current_steps = 0.0f;
    m->direction = direction;
    m->last_speed = 0.0f;
    m->current_speed = MIN_SPEED;  /* 从 MIN_SPEED 起步, 跳过低速死区 */
    m->current_accel = MAX_ACCEL;
    m->pending_cnt = 0;
    m->state = ACCEL;

    /* 设置方向引脚: 0=正转(SET), 1=反转(RESET) */
    HAL_GPIO_WritePin(m->dir_port, m->dir_pin,
                      direction ? GPIO_PIN_RESET : GPIO_PIN_SET);

    /* 预启动 PWM */
    if (!pwm_started[motor_id])
    {
        HAL_TIM_PWM_Start(m->htim, m->channel);
        pwm_started[motor_id] = 1;
    }
}

/* 软停止: CCR=0, 等脉冲完结后关定时器 ----------------------------- */
void MotorCtrl_Stop(uint8_t motor_id)
{
    if (motor_id > 1) return;

    MotorCtrl_t *m = &motor[motor_id];

    if (m->state == STOP || m->state == STOP_PENDING) return;


    __HAL_TIM_SET_COMPARE(m->htim, m->channel, 0);
    m->state = STOP_PENDING;
    m->pending_cnt = 0;
    m->current_speed = 0.0f;
    m->current_accel = 0.0f;
}

/* 单电机梯形加减速状态机 (在 SysTick 1kHz ISR 中调用) ------------- */
static void MotorCtrl_UpdateSingle(MotorCtrl_t *m)
{
    float dt = 1.0f / (float)UPDATE_FREQ;  /* 0.001s */
    float remaining, decel_req;
    float target_speed, delta, d_abs;
    uint32_t arr, ccr;
    uint8_t id = (m == &motor[MOTOR_LEFT]) ? MOTOR_LEFT : MOTOR_RIGHT;

    /* ---- STOP_PENDING: 等待当前脉冲完结后安全关断 ---- */
    if (m->state == STOP_PENDING)
    {
        m->pending_cnt++;
        /* 3ms 后安全关断 (覆盖最长周期: 1/MIN_SPEED ≈ 10ms 时脉冲
         * 已因 CCR=0 而自然停止, 此处延后关定时器) */
        if (m->pending_cnt >= 3)
        {
            HAL_TIM_PWM_Stop(m->htim, m->channel);
            pwm_started[id] = 0;
            m->state = STOP;
        }
        return;
    }

    if (m->state == STOP)
        return;

    /* ---- 计算剩余步数 ---- */
    remaining = m->target_steps - m->current_steps;
    if (remaining <= 0.0f)
    {
        /* 到达目标: 软停止 */
        MotorCtrl_Stop(id);
        return;
    }

    /* ---- 梯形减速判据: v²/(2a) ---- */
    decel_req = (m->current_speed * m->current_speed) / (2.0f * MAX_ACCEL);

    /* ---- 状态切换判断 ---- */
    if (remaining > decel_req)
    {
        if (m->current_speed < MAX_SPEED)
        {
            m->state = ACCEL;
            m->current_accel = MAX_ACCEL;
        }
        else
        {
            m->state = CONST_SPEED;
            m->current_accel = 0.0f;
        }
    }
    else
    {
        m->state = DECEL;
        m->current_accel = -MAX_ACCEL;
    }

    /* ---- 计算目标速度 (梯形规划) ---- */
    target_speed = m->current_speed + m->current_accel * dt;

    /* ---- 速度变化率限幅 (借鉴 Robot stepper.c 防突变) ---- */
    delta = target_speed - m->last_speed;
    d_abs = fabsf_local(delta);
    if (d_abs > MAX_DELTA_SPEED)
    {
        target_speed = m->last_speed + (delta < 0.0f ? -MAX_DELTA_SPEED : MAX_DELTA_SPEED);
    }

    /* ---- 速度钳位 ---- */
    if (target_speed > MAX_SPEED)  target_speed = MAX_SPEED;
    if (target_speed < MIN_SPEED)  target_speed = MIN_SPEED;

    m->last_speed    = target_speed;
    m->current_speed = target_speed;

    /* ---- 更新已走步数 ---- */
    m->current_steps += m->current_speed * dt;

    /* ---- 防止过冲 ---- */
    if (m->current_steps >= m->target_steps)
    {
        m->current_steps = m->target_steps;
        MotorCtrl_Stop(id);
        return;
    }

    /* ---- 速度 → PWM 频率 (ARR = TIMER_CLK / speed - 1) ----
     * 借鉴 Robot stepper.c: 写入影子寄存器 (ARPE + OCxPE 已在 Init 使能)
     * 不写 UG, 不重置 CNT, 硬件在溢出时自动装载新值 */
    arr = (uint32_t)(TIMER_CLK_HZ / m->current_speed);
    /* 安全钳位: ARR 至少 2 (保证 50% 占空比有整数解 CCR=1) */
    if (arr < 2)  arr = 2;
    /* 16 位定时器上限 */
    if (arr > 65535) arr = 65535;

    /* ARR - 1 因为 ARR 寄存器是 "周期 - 1" */
    arr = arr - 1;

    ccr = arr >> 1;  /* 50% 占空比 */

    /* 写入影子寄存器 (HAL 宏直接写 TIMx->ARR / TIMx->CCRx)
     * ARPE=1 → 下次溢出时自动加载到活动寄存器 */
    __HAL_TIM_SET_AUTORELOAD(m->htim, arr);
    __HAL_TIM_SET_COMPARE(m->htim, m->channel, ccr);
}

/* 更新所有电机 (SysTick 中断中调用) -------------------------------- */
void MotorCtrl_Update(void)
{
    MotorCtrl_UpdateSingle(&motor[MOTOR_LEFT]);
    MotorCtrl_UpdateSingle(&motor[MOTOR_RIGHT]);
}

/* 查询电机是否忙 -------------------------------------------------- */
uint8_t MotorCtrl_IsBusy(uint8_t motor_id)
{
    if (motor_id > 1) return 0;
    return (motor[motor_id].state != STOP);
}