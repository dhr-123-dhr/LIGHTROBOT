/**
 * @file    Chassis.c
 * @brief   双驱差速底盘控制层 (1ms tick)
 *
 * 核心架构:
 *   - 直线运动: 速度直控模式 (MotorCtrl_SetSpeed) + yaw PD 航向锁定
 *     - 陀螺仪注入航向, PD 控制器差速修正两轮, 锁住车头直线
 *   - 原地旋转: 位置模式梯形加减速 (MotorCtrl_Start)
 *     - 旋转期间不锁头(陀螺仪只读不控), 不阻塞 1ms tick
 *   - 里程计: 世界坐标系 2D 积分 (x, y, theta)
 *     - 直线运动用电机 step 编码器 + 航向融合
 *     - 旋转时 theta 由 step 编码器积分 + 陀螺仪校核
 *
 * 关键: 1ms 严格节拍, 无阻塞操作. 打印/指令编排交给低优先级任务.
 */

#include "Chassis.h"
#include "MotorCtrl.h"
#include <math.h>

/* ====== 电机 step -> 线速度/角速度 转换 ====== */
/* 轮子每圈走 π*D mm, 每圈 STEPS_PER_REV 步 */
#define STEP_TO_MM   ((PI * WHEEL_DIAMETER) / (float)STEPS_PER_REV)   /* 1 step → mm */

/* 差速旋转: 轮速差 (step/s) → 角速度 (deg/s)
 * omega = (vR - vL) / L  (rad/s)
 * omega_deg = omega * 180 / PI
 * v_linear = (stepR - stepL) * STEP_TO_MM   (mm/s)
 * omega_deg = (step_diff * STEP_TO_MM) / WHEEL_TRACK * (180/PI) */
#define STEP_DIFF_TO_DEGPS(step_diff)  (((step_diff) * STEP_TO_MM) / WHEEL_TRACK * (180.0f / PI))

/* ====== 静态变量 ====== */
static Chassis_Mode chassis_mode = CH_MODE_IDLE;

/* 目标: 位置模式用 */
static float target_distance_mm = 0.0f;   /* 剩余直线距离 (mm) */
static float target_angle_deg   = 0.0f;   /* 剩余旋转角度 (deg), 未使用(位置模式直接包给 MotorCtrl) */

/* 速度直控目标 */
static float cmd_linear_speed  = 0.0f;    /* 目标线速度 (mm/s), 正=前进 */
static float cmd_angular_speed = 0.0f;    /* 目标角速度 (deg/s), 正=左转 */

/* 里程计: 世界坐标系位姿 */
static float pose_x     = 0.0f;   /* mm */
static float pose_y     = 0.0f;   /* mm */
static float pose_theta = 0.0f;   /* deg, 0=初始头向+X */

/* 航向锁定用参数 */
static float ref_yaw_hold = 0.0f;   /* 直线运动时的参考航向 (deg), 进入直线模式时锁定 */
static float last_yaw_err = 0.0f;   /* 上一拍航向误差 (deg), 用于微分项 */
static bool  gyro_has_data = false; /* 陀螺仪是否有有效数据 */

/* 梯形减速: 直线运动末尾用 MAX_ACCEL 统一斜坡 */
static bool  chassis_decel_phase = false;   /* 是否已进入减速区 */

/* 底盘 tick 计时器 (用于 1ms dt) */
static uint32_t chassis_tick_cnt = 0;

/* 内部函数声明 */
static float fabsf_ch(float v) { return (v < 0.0f) ? -v : v; }
static float wrap_180(float deg);

/* ====== 初始化 ====== */
void Chassis_Init(void)
{
    MotorCtrl_Init();
    chassis_mode = CH_MODE_IDLE;
    target_distance_mm = 0.0f;
    target_angle_deg   = 0.0f;
    cmd_linear_speed   = 0.0f;
    cmd_angular_speed  = 0.0f;
    pose_x     = 0.0f;
    pose_y     = 0.0f;
    pose_theta = 0.0f;
    ref_yaw_hold = 0.0f;
    last_yaw_err = 0.0f;
    gyro_has_data = false;
    chassis_tick_cnt = 0;
}

/* ====== 发起直线运动 (非阻塞) ====== */
void Chassis_Move(float distance_mm)
{
    /* 停止当前运动 */
    Chassis_EmergencyStop();

    if (fabsf_ch(distance_mm) < 0.1f) return;

    target_distance_mm = fabsf_ch(distance_mm);
    chassis_decel_phase = false;

    /* 进入直线模式 */
    chassis_mode = CH_MODE_MOVING;

    /* 锁定当前航向为参考 (有陀螺仪用陀螺仪, 否则用里程计) */
    ref_yaw_hold = pose_theta;

    /* 目标线速度 (mm/s):
     *   最大速度 = MAX_SPEED * STEP_TO_MM (步/秒 → mm/s)
     *   额定速度取 80% 最大速度 */
    float max_lin = MAX_SPEED * STEP_TO_MM;   /* ~ 5100 * 0.083 ≈ 424 mm/s */
    cmd_linear_speed = max_lin * 0.8f;         /* ~ 340 mm/s */
    if (distance_mm < 0.0f) cmd_linear_speed = -cmd_linear_speed;
    cmd_angular_speed = 0.0f;   /* 直线: 期望零角速 */
}

/* ====== 发起原地旋转 (非阻塞, 位置模式梯形加减速) ====== */
void Chassis_Rotate(float angle_deg)
{
    Chassis_EmergencyStop();

    if (fabsf_ch(angle_deg) < 0.1f) return;

    /* 进入旋转模式: 直接包给 MotorCtrl 梯形加减速位置模式 */
    chassis_mode = CH_MODE_TURNING;
    target_angle_deg = angle_deg;

    float steps = ANGLE_TO_STEPS(fabsf_ch(angle_deg));

    if (angle_deg > 0.0f) {
        /* 左转: 左轮后退, 右轮前进 */
        MotorCtrl_Start(MOTOR_LEFT,  steps, 1);
        MotorCtrl_Start(MOTOR_RIGHT, steps, 0);
    } else {
        /* 右转: 左轮前进, 右轮后退 */
        MotorCtrl_Start(MOTOR_LEFT,  steps, 0);
        MotorCtrl_Start(MOTOR_RIGHT, steps, 1);
    }
}

/* ====== 查询忙 ====== */
uint8_t Chassis_IsBusy(void)
{
    if (chassis_mode == CH_MODE_IDLE) return 0;
    if (chassis_mode == CH_MODE_TURNING) {
        return MotorCtrl_IsBusy(MOTOR_LEFT) || MotorCtrl_IsBusy(MOTOR_RIGHT);
    }
    /* 直线模式: 目标距离走完或手动停止 */
    return (chassis_mode == CH_MODE_MOVING) ? 1 : 0;
}

bool Chassis_IsTurning(void)
{
    return (chassis_mode == CH_MODE_TURNING);
}

/* ====== 急停 ====== */
void Chassis_EmergencyStop(void)
{
    MotorCtrl_Stop(MOTOR_LEFT);
    MotorCtrl_Stop(MOTOR_RIGHT);
    chassis_mode = CH_MODE_IDLE;
    cmd_linear_speed = 0.0f;
    cmd_angular_speed = 0.0f;
    target_distance_mm = 0.0f;
    chassis_decel_phase = false;
}

/* ====== 陀螺仪注入 (gyroTask 异步调用, 1ms 节拍) ====== */
void chassis_feed_gyro(float yaw_deg)
{
    if (!gyro_has_data) {
        /* 首次注入: 校准里程计航向 */
        pose_theta = yaw_deg;
        ref_yaw_hold = yaw_deg;
        gyro_has_data = true;
    } else {
        /* 将陀螺仪角度融合到里程计 (旋转模式不融合, 直线模式融合) */
        if (chassis_mode == CH_MODE_MOVING || chassis_mode == CH_MODE_IDLE) {
            /* 低通滤波融合: 90% 陀螺仪 + 10% 里程计, 防止漂移 */
            pose_theta = pose_theta * 0.05f + yaw_deg * 0.95f;
        }
    }
}

/* ====== 1ms 控制层 tick ====== */
void chassis_tick(void)
{
    float dt = 0.001f;   /* 1ms */
    chassis_tick_cnt++;

    /* ====== 旋转模式: MotorCtrl 位置模式自行管理, 仅更新里程计 ====== */
    if (chassis_mode == CH_MODE_TURNING)
    {
        float stepL = MotorCtrl_GetCurrentSteps(MOTOR_LEFT);
        float stepR = MotorCtrl_GetCurrentSteps(MOTOR_RIGHT);

        /* 差速旋转: theta 变化率
         *   vL (mm/s) = stepL_speed * STEP_TO_MM, 方向: 左转时左轮-右轮+
         *   角速度 (deg/s) = (vR - vL) / WHEEL_TRACK * 180/PI
         *   step 增量换算 */
        static float last_stepL_turn = 0.0f;
        static float last_stepR_turn = 0.0f;

        float dL = stepL - last_stepL_turn;
        float dR = stepR - last_stepR_turn;
        last_stepL_turn = stepL;
        last_stepR_turn = stepR;

        float mmL = dL * STEP_TO_MM;
        float mmR = dR * STEP_TO_MM;
        /* mmR - mmL 是因为右轮前进左轮后退时正角速度
         * 在 rotating 中 MotorCtrl_Start 设方向: 左转时左轮 dir=1(负) 右轮 dir=0(正)
         * 但方向已包含在 step 符号中 (MotorCtrl_GetCurrentSteps 不带方向)
         * 我们这里用 MotorCtrl_GetCurrentSpeed 带方向:
         *   左转: 左轮 = -speed, 右轮 = +speed → 角速度 ∝ (右-左) > 0 即正角速度 */
        float omega = (mmR - mmL) / WHEEL_TRACK * (180.0f / PI);  /* deg/s * dt = deg */

        /* 根据运动方向判断角速度符号:
         *   左转 (target_angle > 0): 左轮后退(-), 右轮前进(+) → mmR - mmL > 0 → omega > 0
         *   右转 (target_angle < 0): 左轮前进(+), 右轮后退(-) → mmR - mmL < 0 → omega < 0 */
        float dtheta_deg = omega * dt;  /* 实际步数应该很少, 1ms 内可能 0 步 */

        /* 里程计更新 theta (旋转模式: 陀螺仪不控, 但可以融合补偿) */
        if (gyro_has_data) {
            /* 融合 70% 步进里程计 + 30% 陀螺仪微分 */
            pose_theta = pose_theta * 0.3f + (pose_theta + dtheta_deg) * 0.7f;
        } else {
            pose_theta += dtheta_deg;
        }
        pose_theta = wrap_180(pose_theta);

        /* 检查旋转完成: 两电机都停 */
        if (!MotorCtrl_IsBusy(MOTOR_LEFT) && !MotorCtrl_IsBusy(MOTOR_RIGHT)) {
            chassis_mode = CH_MODE_IDLE;
            last_stepL_turn = 0.0f;
            last_stepR_turn = 0.0f;
        }
        return;
    }

    /* ====== 直线模式: 速度直控 + yaw PD 航向锁定 ====== */
    if (chassis_mode == CH_MODE_MOVING)
    {
        /* yaw PD 控制器: 保持车头方向 */
        float yaw_correction = 0.0f;
        if (gyro_has_data) {
            float yaw_err = ref_yaw_hold - pose_theta;
            yaw_err = wrap_180(yaw_err);

            /* 死区: 航向误差 < 0.5° 不修正, 防止抖振 */
            if (fabsf_ch(yaw_err) < CH_YAW_DEADBAND) {
                yaw_err = 0.0f;
            }

            /* PD 控制 */
            float yaw_diff = (yaw_err - last_yaw_err) / dt;  /* 微分 (deg/s) */
            yaw_correction = CH_YAW_KP * yaw_err + CH_YAW_KD * yaw_diff;
            last_yaw_err = yaw_err;
        }

        /* 限幅: 角速度修正不超过 MAX_SPEED 的 30% */
        float max_corr = MAX_SPEED * 0.3f * STEP_DIFF_TO_DEGPS(1.0f);  /* 转换为步/秒的当量 */
        /* 换个表述: max_corr = (MAX_SPEED * 0.3) 步/秒 对应的线速度差 */
        float max_corr_steps = MAX_SPEED * 0.3f;

        /* yaw_correction 单位是步/秒的差速
         * 转换: deg/s 的角速度修正 * (WHEEL_TRACK / 2 / STEP_TO_MM)  → 轮速差 (步/秒)
         * 简化: 修正角速度 (deg/s) * (WHEEL_TRACK / 2) / STEP_TO_MM / (180/PI)
         *      = yaw_correction(deg/s) * (WHEEL_TRACK * PI) / (360 * STEP_TO_MM) */
        float yaw_to_step_diff = yaw_correction * (WHEEL_TRACK * PI) / (360.0f * STEP_TO_MM);
        if (yaw_to_step_diff > max_corr_steps)  yaw_to_step_diff = max_corr_steps;
        if (yaw_to_step_diff < -max_corr_steps) yaw_to_step_diff = -max_corr_steps;

        /* 目标轮速 = 线速度对应的步频 + 差速修正
         *  线速度 (mm/s) → 步/秒:  v_linear / (PI*D) * STEPS_PER_REV  = v_linear / STEP_TO_MM
         *  左轮 = v_center - diff
         *  右轮 = v_center + diff   */
        float center_step_sp = cmd_linear_speed / STEP_TO_MM;  /* 步/秒 */
        float left_target  = center_step_sp - yaw_to_step_diff;
        float right_target = center_step_sp + yaw_to_step_diff;

        /* 限幅: 不超过 MAX_SPEED */
        if (left_target  > MAX_SPEED)  left_target  = MAX_SPEED;
        if (left_target  < -MAX_SPEED) left_target  = -MAX_SPEED;
        if (right_target > MAX_SPEED)  right_target = MAX_SPEED;
        if (right_target < -MAX_SPEED) right_target = -MAX_SPEED;

        /* 下发速度 (MotorCtrl_SetSpeed 持续保速) */
        MotorCtrl_SetSpeed(MOTOR_LEFT,  left_target);
        MotorCtrl_SetSpeed(MOTOR_RIGHT, right_target);

        /* 里程计积分:
         *   线速度 → 位移 = v * dt
         *   位移在世界坐标系分解:
         *     dx = v * cos(theta) * dt
         *     dy = v * sin(theta) * dt
         *   其中 v = (vL + vR) / 2 * STEP_TO_MM
         */
        float dL = MotorCtrl_GetCurrentSpeed(MOTOR_LEFT);
        float dR = MotorCtrl_GetCurrentSpeed(MOTOR_RIGHT);
        float v_center = (dL + dR) * 0.5f * STEP_TO_MM;  /* mm/s */

        float theta_rad = pose_theta * (PI / 180.0f);
        float dx = v_center * cosf(theta_rad) * dt;
        float dy = v_center * sinf(theta_rad) * dt;

        /* 航向变化 (陀螺仪已注入, 这里用编码器补充以防陀螺仪未就绪) */
        if (!gyro_has_data) {
            float dtheta = (dR - dL) * STEP_TO_MM / WHEEL_TRACK * (180.0f / PI) * dt;
            pose_theta += dtheta;
            pose_theta = wrap_180(pose_theta);
        }

        pose_x += dx;
        pose_y += dy;

        /* 减速区计算: v²/(2a) → 当前速度所需的刹车距离 (mm) */
        float cur_abs_speed = fabsf_ch(v_center);
        float decel_dist = (cur_abs_speed * cur_abs_speed) / (2.0f * MAX_ACCEL * STEP_TO_MM);

        /* 进入减速区: 梯形斜坡减速 — 每 tick 用 MAX_ACCEL 逐步降低 cmd_linear_speed,
           与位置模式 DECEL 用同一个减速度常数, 保持运动平滑统一 */
        if (target_distance_mm <= decel_dist || chassis_decel_phase) {
            chassis_decel_phase = true;
            float decel_step = MAX_ACCEL * STEP_TO_MM * dt;  /* mm/s 每拍减量 */

            if (fabsf_ch(cmd_linear_speed) > decel_step) {
                cmd_linear_speed += (cmd_linear_speed > 0.0f ? -decel_step : decel_step);
            } else {
                cmd_linear_speed = 0.0f;
            }
        }

        /* 距离追踪 (用实际速度递减) */
        target_distance_mm -= cur_abs_speed * dt;

        /* 速度降到 10 mm/s 以下且距离耗尽 → 完成停止 */
        if (target_distance_mm <= 0.0f && cur_abs_speed < 10.0f) {
            Chassis_EmergencyStop();
        }
        return;
    }

    /* ====== 空闲模式: 仅更新里程计 (防止陀螺仪漂移) ====== */
    if (chassis_mode == CH_MODE_IDLE)
    {
        /* 无运动, 不更新里程计 */
        /* 但如果有陀螺仪数据, 持续融合微小漂移 */
        /* 不做任何操作, 保持位置记忆 */
    }
}

/* ====== 查询位姿 (世界系 mm, mm, deg) ====== */
void chassis_get_pose(float *x, float *y, float *theta)
{
    if (x) *x = pose_x;
    if (y) *y = pose_y;
    if (theta) *theta = pose_theta;
}

/* ====== 查询目标线速度 (mm/s) ====== */
float chassis_get_cmd_speed(void)
{
    return cmd_linear_speed;
}

/* ====== 查询左右轮目标线速度 (mm/s) ====== */
void chassis_get_wheel_speed(float *vL, float *vR)
{
    if (vL) {
        float s = MotorCtrl_GetCurrentSpeed(MOTOR_LEFT);
        *vL = s * STEP_TO_MM;
    }
    if (vR) {
        float s = MotorCtrl_GetCurrentSpeed(MOTOR_RIGHT);
        *vR = s * STEP_TO_MM;
    }
}

/* ====== 重置里程计 ====== */
void chassis_reset_odom(void)
{
    pose_x = 0.0f;
    pose_y = 0.0f;
    pose_theta = 0.0f;
    ref_yaw_hold = 0.0f;
    last_yaw_err = 0.0f;
}

/* ====== 角度归一化到 [-180, 180) ====== */
static float wrap_180(float deg)
{
    while (deg > 180.0f)  deg -= 360.0f;
    while (deg < -180.0f) deg += 360.0f;
    return deg;
}