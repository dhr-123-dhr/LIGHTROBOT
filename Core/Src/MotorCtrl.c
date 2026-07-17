/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file    MotorCtrl.c
 * @brief   双步进电机 PWM 脉冲发生 + 梯形加减速状态机 + 速度直控模式
 *
 * 1kHz SysTick 驱动
 *   位置模式: 梯形加减速 (ACCEL→CONST_SPEED→DECEL), 固定步数到达停车
 *   速度直控模式: 1ms tick 持续写入目标速度, 增量 PID/PD 修正.
 *     - 该模式不关心步数, 由上层(chassis)持续调用 MotorCtrl_SetSpeed() 保速.
 *     - 方向和目标速度由上层直接控制, 电机层仅做速率平滑限幅.
 *     - 用于 chassis 1ms tick 中的 yaw PD 航向修正.
 *
 *   双预装载 (ARPE + OCxPE) 在 tim.c 初始化时已使能
 *   - __HAL_TIM_SET_AUTORELOAD / __HAL_TIM_SET_COMPARE 直接写影子寄存器
 *   - 不写 UG, 不重置 CNT, 由硬件在溢出时自动装载
 *   - 停止用 CCR=0 软停止, 不立即关定时器 (避免截断当前脉冲)
 ******************************************************************************
 */
/* USER CODE END Header */
#include "MotorCtrl.h"
#include <math.h>

MotorCtrl_t motor[2];

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
    motor[MOTOR_LEFT].target_speed_spd = 0.0f;
    motor[MOTOR_LEFT].current_accel = 0.0f;
    motor[MOTOR_LEFT].target_steps = 0.0f;
    motor[MOTOR_LEFT].current_steps = 0.0f;
    motor[MOTOR_LEFT].direction = 0;
    motor[MOTOR_LEFT].htim = &htim2;
    motor[MOTOR_LEFT].channel = TIM_CHANNEL_1;
    motor[MOTOR_LEFT].dir_port = GPIOA;
    motor[MOTOR_LEFT].dir_pin = GPIO_PIN_7;

    /* 右电机 (TIM3 CH1, PA6 STP, PC4 DIR) */
    motor[MOTOR_RIGHT].state = STOP;
    motor[MOTOR_RIGHT].last_speed = 0.0f;
    motor[MOTOR_RIGHT].current_speed = 0.0f;
    motor[MOTOR_RIGHT].target_speed_spd = 0.0f;
    motor[MOTOR_RIGHT].current_accel = 0.0f;
    motor[MOTOR_RIGHT].target_steps = 0.0f;
    motor[MOTOR_RIGHT].current_steps = 0.0f;
    motor[MOTOR_RIGHT].direction = 0;
    motor[MOTOR_RIGHT].htim = &htim3;
    motor[MOTOR_RIGHT].channel = TIM_CHANNEL_1;
    motor[MOTOR_RIGHT].dir_port = GPIOC;
    motor[MOTOR_RIGHT].dir_pin = GPIO_PIN_4;

    /* PSC 已在 MX_TIM2_Init / MX_TIM3_Init 中配置为 83 */
    /* ARPE + OCxPE 已在 tim.c Init 中使能 */
}

/* 启动电机运动 (位置模式: 梯形加减速) ----------------------------- */
void MotorCtrl_Start(uint8_t motor_id, float target_steps, uint8_t direction)
{
    if (motor_id > 1) return;

    MotorCtrl_t *m = &motor[motor_id];

    /* 如果之前是速度直控模式, 先软停止 */
    if (m->state == SPEED_CTRL) {
        MotorCtrl_Stop(motor_id);
    }

    m->target_steps = target_steps;
    m->current_steps = 0.0f;
    m->direction = direction;
    /* 短距离限幅: 目标很近时自动降起始速度, 防止冲出 */
    {
        float v0 = MIN_SPEED;
        float v_short = sqrtf(2.0f * MAX_ACCEL * target_steps * 0.4f);
        if (v_short < v0) v0 = v_short;
        if (v0 < 1.0f) v0 = 1.0f;
        m->last_speed    = v0;
        m->current_speed = v0;
    }
    m->current_accel = MAX_ACCEL;
    m->state = ACCEL;

    /* 设置方向引脚: 0=正转(SET), 1=反转(RESET) */
    HAL_GPIO_WritePin(m->dir_port, m->dir_pin,
                      direction ? GPIO_PIN_RESET : GPIO_PIN_SET);

    /* 先写初始 ARR/CCR, 再启动 PWM */
    {
        uint32_t init_arr = (uint32_t)(TIMER_CLK_HZ / m->current_speed) - 1;
        uint32_t init_ccr = init_arr >> 1;
        __HAL_TIM_SET_AUTORELOAD(m->htim, init_arr);
        __HAL_TIM_SET_COMPARE(m->htim, m->channel, init_ccr);
    }

    HAL_TIM_PWM_Start(m->htim, m->channel);
}

/* 速度直控模式: 持续保速, 不关心步数 (1ms tick 底盘 yaw PD 用) ---- */
void MotorCtrl_SetSpeed(uint8_t motor_id, float speed_steps_per_s)
{
    if (motor_id > 1) return;

    MotorCtrl_t *m = &motor[motor_id];
    float abs_speed = fabsf_local(speed_steps_per_s);

    /* 已在速度直控模式且目标速度降至0: 让 SPEED_CTRL 内部斜坡减速, 不立即停 */
    if (abs_speed < 1.0f) {
        if (m->state == SPEED_CTRL) {
            m->target_speed_spd = 0.0f;  /* 让 Update 内部 MAX_ACCEL 斜坡归零 */
            return;
        }
        MotorCtrl_Stop(motor_id);
        return;
    }

    /* 方向 (正=0=前进, 负=1=后退) */
    uint8_t new_dir = (speed_steps_per_s < 0.0f) ? 1 : 0;

    /* 如果之前处于 STOP 或位置模式, 初始化进入速度直控 */
    if (m->state == STOP || m->state == ACCEL || m->state == CONST_SPEED || m->state == DECEL) {
        m->state = SPEED_CTRL;
        m->direction = new_dir;
        m->target_speed_spd = abs_speed;
        /* 起步平滑: 长距离可达 MIN_SPEED, 短程限幅 */
        if (m->current_speed < MIN_SPEED) {
            m->current_speed = MIN_SPEED;
            m->last_speed = MIN_SPEED;
        }
        /* 累计步数清零 (速度模式不关心步数) */
        m->current_steps = 0.0f;
        m->target_steps = 0.0f;
        /* 设置方向 */
        HAL_GPIO_WritePin(m->dir_port, m->dir_pin,
                          new_dir ? GPIO_PIN_RESET : GPIO_PIN_SET);
        /* 启动 PWM */
        {
            uint32_t init_arr = (uint32_t)(TIMER_CLK_HZ / m->current_speed) - 1;
            uint32_t init_ccr = init_arr >> 1;
            __HAL_TIM_SET_AUTORELOAD(m->htim, init_arr);
            __HAL_TIM_SET_COMPARE(m->htim, m->channel, init_ccr);
        }
        HAL_TIM_PWM_Start(m->htim, m->channel);
        return;
    }

    /* 已在速度直控模式: 更新目标速度 (方向反转先软停再重设) */
    if (new_dir != m->direction) {
        /* 方向反转: 先减到 0 再换向 */
        float slow = m->current_speed - MAX_ACCEL * (1.0f / UPDATE_FREQ);
        if (slow < MIN_SPEED) slow = MIN_SPEED;
        if (m->current_speed > slow) {
            m->target_speed_spd = slow;
            m->direction = new_dir;
        } else {
            /* 速度已降到最小, 直接换向 */
            m->target_speed_spd = abs_speed;
            m->direction = new_dir;
            HAL_GPIO_WritePin(m->dir_port, m->dir_pin,
                              new_dir ? GPIO_PIN_RESET : GPIO_PIN_SET);
        }
    } else {
        m->target_speed_spd = abs_speed;
    }
}

/* 软停止: CCR=0, 不关定时器 --------------------------------------- */
void MotorCtrl_Stop(uint8_t motor_id)
{
    if (motor_id > 1) return;

    MotorCtrl_t *m = &motor[motor_id];

    if (m->state == STOP) return;

    __HAL_TIM_SET_COMPARE(m->htim, m->channel, 0);
    m->state = STOP;
    m->current_speed = 0.0f;
    m->current_accel = 0.0f;
    m->target_speed_spd = 0.0f;
}

/* 写 PWM 寄存器 (临界区保护) --------------------------------------- */
static void MotorCtrl_WritePWM(MotorCtrl_t *m, float speed_sp)
{
    /* 安全钳: 速度不低于 1 步/秒 */
    if (speed_sp < 1.0f) speed_sp = 1.0f;
    if (speed_sp > MAX_SPEED) speed_sp = MAX_SPEED;

    uint32_t arr = (uint32_t)(TIMER_CLK_HZ / speed_sp);
    if (arr < 3) arr = 3;
    if (arr > 65535) arr = 65535;
    arr = arr - 1;

    uint32_t ccr = arr >> 1;

    __disable_irq();
    __HAL_TIM_SET_AUTORELOAD(m->htim, arr);
    __HAL_TIM_SET_COMPARE(m->htim, m->channel, ccr);
    __enable_irq();
}

/* 单电机更新 (位置模式梯形 或 速度直控) --------------------------- */
static void MotorCtrl_UpdateSingle(MotorCtrl_t *m)
{
    float dt = 1.0f / (float)UPDATE_FREQ;  /* 0.001s */
    uint8_t id = (m == &motor[MOTOR_LEFT]) ? MOTOR_LEFT : MOTOR_RIGHT;

    if (m->state == STOP)
        return;

    /* ====== 速度直控模式 ====== */
    if (m->state == SPEED_CTRL)
    {
        float target = m->target_speed_spd;
        /* 限幅平滑: 速度变化率不超过 MAX_DELTA_SPEED/tick */
        float delta = target - m->current_speed;
        float d_abs = fabsf_local(delta);
        if (d_abs > MAX_DELTA_SPEED) {
            target = m->current_speed + (delta < 0.0f ? -MAX_DELTA_SPEED : MAX_DELTA_SPEED);
        }
        /* 加速度限幅 */
        if (target > m->current_speed + MAX_ACCEL * dt)
            target = m->current_speed + MAX_ACCEL * dt;
        if (target < m->current_speed - MAX_ACCEL * dt)
            target = m->current_speed - MAX_ACCEL * dt;

        if (target > MAX_SPEED) target = MAX_SPEED;
        /* 低速不立即停: 等 current_speed 自然降到 0, 斜坡减速 */
        if (target < 0.0f) target = 0.0f;
        if (target < 1.0f && m->current_speed < 1.0f) {
            MotorCtrl_Stop(id);
            return;
        }

        m->last_speed = target;
        m->current_speed = target;
        m->current_steps += m->current_speed * dt;  /* 里程计数 (不参与刹车逻辑) */

        MotorCtrl_WritePWM(m, m->current_speed);
        return;
    }

    /* ====== 位置模式: 梯形加减速 ====== */
    float remaining, decel_req;
    float target_speed, delta, d_abs;

    remaining = m->target_steps - m->current_steps;
    if (remaining <= 0.0f) {
        MotorCtrl_Stop(id);
        return;
    }

    decel_req = (m->current_speed * m->current_speed) / (2.0f * MAX_ACCEL)
              + 0.5f * m->current_speed * dt;

    if (remaining <= decel_req) {
        m->state = DECEL;
        m->current_accel = -MAX_ACCEL;
    } else if (m->state == ACCEL && m->current_speed >= MAX_SPEED) {
        m->state = CONST_SPEED;
        m->current_accel = 0.0f;
    }

    target_speed = m->current_speed + m->current_accel * dt;

    delta = target_speed - m->last_speed;
    d_abs = fabsf_local(delta);
    if (d_abs > MAX_DELTA_SPEED) {
        target_speed = m->last_speed + (delta < 0.0f ? -MAX_DELTA_SPEED : MAX_DELTA_SPEED);
    }

    /* 减速过零死锁保护 */
    if (m->state == DECEL && target_speed <= 0.0f) {
        m->current_steps = m->target_steps;
        m->current_speed = 0.0f;
        m->current_accel = 0.0f;
        MotorCtrl_Stop(id);
        return;
    }

    if (target_speed > MAX_SPEED)  target_speed = MAX_SPEED;
    if (target_speed < MIN_SPEED)  target_speed = MIN_SPEED;

    m->last_speed    = target_speed;
    m->current_speed = target_speed;
    m->current_steps += m->current_speed * dt;

    if (m->current_steps >= m->target_steps) {
        m->current_steps = m->target_steps;
        MotorCtrl_Stop(id);
        return;
    }

    MotorCtrl_WritePWM(m, m->current_speed);
}

/* 更新所有电机 (SysTick 1kHz ISR 中调用) --------------------------- */
void MotorCtrl_Update(void)
{
    MotorCtrl_UpdateSingle(&motor[MOTOR_LEFT]);
    MotorCtrl_UpdateSingle(&motor[MOTOR_RIGHT]);
}

/* 查询 ------------------------------------------------------------ */
uint8_t MotorCtrl_IsBusy(uint8_t motor_id)
{
    if (motor_id > 1) return 0;
    return (motor[motor_id].state != STOP);
}

float MotorCtrl_GetCurrentSteps(uint8_t motor_id)
{
    if (motor_id > 1) return 0.0f;
    return motor[motor_id].current_steps;
}

float MotorCtrl_GetCurrentSpeed(uint8_t motor_id)
{
    if (motor_id > 1) return 0.0f;
    /* 速度直控返回 current_speed (方向修正含方向位) */
    float s = motor[motor_id].current_speed;
    if (motor[motor_id].direction == 1) s = -s;
    return s;
}