#include "MotorCtrl.h"

/* 电机实例 -------------------------------------------------------- */
MotorCtrl_t motor[2];

/* 定时器句柄引用 (由 CubeMX 生成) ---------------------------------- */
extern TIM_HandleTypeDef htim2;   /* 电机1 STP: PA5, TIM2_CH1 */
extern TIM_HandleTypeDef htim3;   /* 电机2 STP: PA6, TIM3_CH1 */

/* 初始化电机控制参数 ---------------------------------------------- */
void MotorCtrl_Init(void)
{
    /* ---- 电机1: 左轮 ---- */
    motor[MOTOR_LEFT].state         = STOP;
    motor[MOTOR_LEFT].current_speed = 0.0f;
    motor[MOTOR_LEFT].current_accel = 0.0f;
    motor[MOTOR_LEFT].target_steps  = 0.0f;
    motor[MOTOR_LEFT].current_steps = 0.0f;
    motor[MOTOR_LEFT].direction     = 0;
    motor[MOTOR_LEFT].htim          = &htim2;
    motor[MOTOR_LEFT].channel       = TIM_CHANNEL_1;
    motor[MOTOR_LEFT].dir_port      = GPIOA;
    motor[MOTOR_LEFT].dir_pin       = GPIO_PIN_7;

    /* 配置 TIM2 预分频器为 1 MHz */
    __HAL_TIM_SET_PRESCALER(&htim2, TIMER_PSC);
    HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_1);

    /* ---- 电机2: 右轮 ---- */
    motor[MOTOR_RIGHT].state         = STOP;
    motor[MOTOR_RIGHT].current_speed = 0.0f;
    motor[MOTOR_RIGHT].current_accel = 0.0f;
    motor[MOTOR_RIGHT].target_steps  = 0.0f;
    motor[MOTOR_RIGHT].current_steps = 0.0f;
    motor[MOTOR_RIGHT].direction     = 0;
    motor[MOTOR_RIGHT].htim          = &htim3;
    motor[MOTOR_RIGHT].channel       = TIM_CHANNEL_1;
    motor[MOTOR_RIGHT].dir_port      = GPIOC;
    motor[MOTOR_RIGHT].dir_pin       = GPIO_PIN_4;

    /* 配置 TIM3 预分频器为 1 MHz */
    __HAL_TIM_SET_PRESCALER(&htim3, TIMER_PSC);
    HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_1);
}

/* 启动指定电机 ---------------------------------------------------- */
void MotorCtrl_Start(uint8_t motor_id, float target_steps, uint8_t direction)
{
    if (motor_id > 1) return;
    if (target_steps <= 0.0f) return;

    MotorCtrl_t *m = &motor[motor_id];

    m->target_steps  = target_steps;
    m->current_steps = 0.0f;
    m->current_speed = 0.0f;
    m->current_accel = 0.0f;
    m->state         = ACCEL_JERK_UP;
    m->direction     = direction;

    /* 设置方向引脚 */
    if (direction == 0) {
        HAL_GPIO_WritePin(m->dir_port, m->dir_pin, GPIO_PIN_SET);
    } else {
        HAL_GPIO_WritePin(m->dir_port, m->dir_pin, GPIO_PIN_RESET);
    }
}

/* 查询电机是否运动中 ---------------------------------------------- */
uint8_t MotorCtrl_IsBusy(uint8_t motor_id)
{
    if (motor_id > 1) return 0;
    return (motor[motor_id].state != STOP);
}

/* S型曲线速度更新 (每个电机独立计算) ------------------------------ */
static void MotorCtrl_UpdateSingle(MotorCtrl_t *m)
{
    if (m->state == STOP) return;

    float dt = 1.0f / (float)UPDATE_FREQ;

    /* 判断是否需要进入减速阶段 */
    float remaining = m->target_steps - m->current_steps;
    float speed_sq  = m->current_speed * m->current_speed;
    float decel_req = speed_sq / (2.0f * MAX_ACCEL);  /* 匀减速所需步数 */

    /* ---- 加速阶段 ---- */
    if (m->current_speed < MAX_SPEED && remaining > decel_req) {
        switch (m->state) {
        case ACCEL_JERK_UP:
            m->current_accel += MAX_JERK * dt;
            if (m->current_accel >= MAX_ACCEL) {
                m->current_accel = MAX_ACCEL;
                m->state = ACCEL_CONST;
            }
            break;

        case ACCEL_CONST:
            /* 预测下一拍是否超速，提前进入减加速段 */
            if (m->current_speed + m->current_accel * dt >= MAX_SPEED) {
                m->state = ACCEL_JERK_DOWN;
            }
            break;

        case ACCEL_JERK_DOWN:
            m->current_accel -= MAX_JERK * dt;
            if (m->current_accel <= 0.0f) {
                m->current_accel = 0.0f;
                m->current_speed = MAX_SPEED;
                m->state = CONST_SPEED;
            }
            break;

        default:
            break;
        }
    }
    /* ---- 减速阶段 ---- */
    else if (remaining <= decel_req || m->current_speed >= MAX_SPEED) {
        switch (m->state) {
        case ACCEL_JERK_UP:
        case ACCEL_CONST:
        case ACCEL_JERK_DOWN:
        case CONST_SPEED:
            /* 进入减速 */
            m->current_accel = -MAX_JERK * dt;
            m->state = DECEL_JERK_UP;
            break;

        case DECEL_JERK_UP:
            m->current_accel -= MAX_JERK * dt;
            if (m->current_accel <= -MAX_ACCEL) {
                m->current_accel = -MAX_ACCEL;
                m->state = DECEL_CONST;
            }
            break;

        case DECEL_CONST:
            if (m->current_speed + m->current_accel * dt <= 0.0f) {
                m->state = DECEL_JERK_DOWN;
            }
            break;

        case DECEL_JERK_DOWN:
            m->current_accel += MAX_JERK * dt;
            if (m->current_accel >= 0.0f) {
                /* 停止 */
                m->current_accel = 0.0f;
                m->current_speed = 0.0f;
                m->state = STOP;
                HAL_TIM_PWM_Stop(m->htim, m->channel);
                return;
            }
            break;

        default:
            break;
        }
    }

    /* 更新速度与位移 */
    m->current_speed += m->current_accel * dt;
    if (m->current_speed < 0.0f) m->current_speed = 0.0f;
    if (m->current_speed > MAX_SPEED) m->current_speed = MAX_SPEED;

    m->current_steps += m->current_speed * dt;

    /* 防止过冲 */
    if (m->current_steps >= m->target_steps) {
        m->current_steps = m->target_steps;
        m->current_speed = 0.0f;
        m->current_accel = 0.0f;
        m->state = STOP;
        HAL_TIM_PWM_Stop(m->htim, m->channel);
        return;
    }

    /* 更新 PWM 频率: period = 1/f, ARR = period * 1e6 - 1 */
    if (m->current_speed > 0.0f) {
        float period   = 1.0f / m->current_speed;
        uint32_t arr   = (uint32_t)(period * TIMER_CLK_HZ) - 1;

        /* 硬件保护: 最小频率约 15 Hz, 最大由 ARR 决定 */
        if (arr < 10) arr = 10;
        if (arr > 65535) arr = 65535;

        __HAL_TIM_SET_AUTORELOAD(m->htim, arr);
        __HAL_TIM_SET_COMPARE(m->htim, m->channel, arr / 2);
        HAL_TIM_PWM_Start(m->htim, m->channel);
    }
}

/* 在 SysTick ISR (1kHz) 中调用 ----------------------------------- */
void MotorCtrl_Update(void)
{
    MotorCtrl_UpdateSingle(&motor[MOTOR_LEFT]);
    MotorCtrl_UpdateSingle(&motor[MOTOR_RIGHT]);
}