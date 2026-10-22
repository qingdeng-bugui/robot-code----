#include "Shoot.h"
#include "Function.h"
#include "Imu.h"
#include "Motor.h"
#include "PID.h"
#include "Serial.h"
#include "Servo.h"
#include "main.h"
#include <math.h>

/* 点位表：每一段都是相对上一个点位的位移，戳完会回到该点位，所以段与段之间可以直接累加。
 * 字段：转角 / 前移 / 左移 / 云台角 / 是否判可戳。
 * 转角写的是增量（正数左转、负数右转）；运行时累加成期望航向，
 * 转向用 Imu_TurnTo(期望航向)，整段平移再用 Shoot_HoldOmega(期望航向) 持续纠偏。 */
static const Shoot_Leg_t SHOOT_LEGS[] =
    {
        /* 转角(deg)  前移(mm)        左移(mm)              云台角             判可戳 */
        {0.0f, SHOOT_FWD_MM, SHOOT_LEFT_MM, SHOOT_GIMBAL_DEG, 1U},                   /* 循迹终点 -> 7 号位 */
        {0.0f, SHOOT_FWD2_MM, -(SHOOT_RIGHT2_MM), SHOOT_GIMBAL2_DEG, 1U},            /* -> 6 号位：前21cm、右3cm */
        {0.0f, 0.0f, -(SHOOT_RIGHT3_MM), SHOOT_GIMBAL2_DEG, 0U},                     /* 探路：向右 25cm */
        {0.0f, SHOOT_FWD3_MM, 0.0f, SHOOT_GIMBAL2_DEG, 0U},                          /* 探路：向前 30cm */
        {SHOOT_TURN3_DEG, SHOOT_FWD4_MM, SHOOT_LEFT4_MM, SHOOT_GIMBAL3_DEG, 1U},     /* 左转 40°，再按 117/17 走 -> 3 号位 */
        {SHOOT_TURN4_DEG, -(SHOOT_BACK4_MM), 0.0f, SHOOT_GIMBAL4_DEG, 0U},           /* 右转 40° 转回 0° 后先后退 5cm */
        {0.0f, SHOOT_FWD4B_MM, -(SHOOT_RIGHT4_MM), SHOOT_GIMBAL4_DEG, 1U},           /* 右移 13cm + 前进 10cm -> 4 号位 */
        {0.0f, SHOOT_FWD_TO1_MM, 0.0f, SHOOT_GIMBAL1_DEG, 1U},                       /* 前进 25cm -> 1 号位 */
        {0.0f, 0.0f, -(SHOOT_RIGHT1_MM), SHOOT_GIMBAL_TO2_DEG, 0U},                  /* 右移 10cm */
        {0.0f, SHOOT_FWD1_MM, -(SHOOT_RIGHT1B_MM), SHOOT_GIMBAL_TO2_DEG, 0U},        /* 斜着走：前15cm + 右15cm */
        {SHOOT_TURN2_DEG, SHOOT_FWD2B_MM, SHOOT_LEFT2_MM, SHOOT_GIMBAL_TO2_DEG, 1U}, /* 掉头 180°，再前 1cm + 左 6cm -> 2 号位 */
        {0.0f, 0.0f, -(SHOOT_RIGHT2B_MM), SHOOT_GIMBAL_TO2_DEG, 0U},                 /* 右移 15cm */
        {SHOOT_TURN5A_DEG, SHOOT_FWD5_MM, 0.0f, SHOOT_GIMBAL_TO2_DEG, 0U},           /* 右转 30° 后直行 45cm */
        {SHOOT_TURN5_DEG, SHOOT_FWD5B_MM, 0.0f, SHOOT_GIMBAL5_DEG, 1U},              /* 左转 120° 转到正东，前进 27cm -> 5 号位 */
        {SHOOT_TURN8_DEG, SHOOT_FWD8_MM, 0.0f, SHOOT_GIMBAL8_DEG, 1U}                /* 右转 60°，前进 27cm -> 8 号位 */
};

#define SHOOT_LEG_COUNT (sizeof(SHOOT_LEGS) / sizeof(SHOOT_LEGS[0]))

static float Shoot_Wrap180(float a)
{
    a = fmodf(a + 180.0f, 360.0f);
    if (a < 0.0f)
    {
        a += 360.0f;
    }
    return a - 180.0f;
}

/* 四个轮子的角速度 -> 车体线速度（cm/s），麦轮正解 */
static void Shoot_BodySpeed(float *vx, float *vy)
{
    float lf = PID_Get_Current_Speed(1U) * (PI / 180.0f) * WHEEL_R;
    float rf = PID_Get_Current_Speed(2U) * (PI / 180.0f) * WHEEL_R;
    float lr = PID_Get_Current_Speed(3U) * (PI / 180.0f) * WHEEL_R;
    float rr = PID_Get_Current_Speed(4U) * (PI / 180.0f) * WHEEL_R;

    *vx = (lf + rf + lr + rr) * 0.25f;
    *vy = (-lf + rf + lr - rr) * 0.25f;
}

/* 航向修正角速度，锁在 target_deg */
static float Shoot_HoldOmega(float target_deg)
{
    float err;
    float omega;

    if (Imu_IsOnline() == 0U)
    {
        return 0.0f;
    }

    err = Shoot_Wrap180(target_deg - Imu_GetYaw());
    omega = SHOOT_HOLD_GAIN * err;

    if (omega > SHOOT_HOLD_OMEGA)
    {
        omega = SHOOT_HOLD_OMEGA;
    }
    else if (omega < -SHOOT_HOLD_OMEGA)
    {
        omega = -SHOOT_HOLD_OMEGA;
    }

    return omega;
}

static void Shoot_Stop(void)
{
    Car_Move(0.0f, 0.0f, 0.0f);
}

/* 轻点一脚反向刹车，削掉惯性前冲 */
static void Shoot_Brake(void)
{
    Set_Target_Speed(1U, -120.0f);
    Set_Target_Speed(2U, -120.0f);
    Set_Target_Speed(3U, -120.0f);
    Set_Target_Speed(4U, -120.0f);
    HAL_Delay(60U);
    Shoot_Stop();
}

/* ---------------- 视觉 ---------------- */

static uint8_t Shoot_VisionStart(void);

/* 非阻塞取一帧；没有新行就返回 INVALID */
static uint8_t Shoot_Poll(int *dx, int *can_hit, uint8_t *red, int *red_dx)
{
    char line[32];

    if (Visual_ReadLatestLine(line) == 0U)
    {
        return VISUAL_COMMAND_INVALID;
    }

    return Visual_ParseLine(line, dx, can_hit, red, red_dx);
}

/* 等一帧有效数据，最多 ms 毫秒 */
static uint8_t Shoot_WaitFrame(int *dx, int *can_hit, uint8_t *red, int *red_dx,
                               uint32_t ms)
{
    uint32_t t0 = HAL_GetTick();
    uint8_t cmd;

    do
    {
        cmd = Shoot_Poll(dx, can_hit, red, red_dx);
        if (cmd != VISUAL_COMMAND_INVALID)
        {
            return cmd;
        }
        HAL_Delay(2U);
    } while ((uint32_t)(HAL_GetTick() - t0) < ms);

    return VISUAL_COMMAND_INVALID;
}

static uint8_t Shoot_VisionStart(void)
{
    uint32_t t0;
    int dx = 0;
    int can_hit = 0;
    int red_dx = 0;
    uint8_t red = 0U;

    Visual_Rx_Init();
    Visual_Rx_Clear();
    HAL_Delay(30U);
    Serial_SendString("START");

    /* 等首帧，最多 SHOOT_VISION_WAIT_MS；超时也继续，不能因为视觉慢就不动 */
    t0 = HAL_GetTick();
    while ((uint32_t)(HAL_GetTick() - t0) < SHOOT_VISION_WAIT_MS)
    {
        if (Shoot_WaitFrame(&dx, &can_hit, &red, &red_dx, 20U) != VISUAL_COMMAND_INVALID)
        {
            return 1U;
        }
    }

    return 0U;
}

/* 用 dx 微调云台 */
static void Shoot_Aim(float *servo, int dx, float gain, int deadband, float max_step)
{
    float corr = 0.0f;

    if (dx > deadband)
    {
        corr = (float)(dx - deadband) * gain;
    }
    else if (dx < -deadband)
    {
        corr = (float)(dx + deadband) * gain;
    }

    if (corr > max_step)
    {
        corr = max_step;
    }
    else if (corr < -max_step)
    {
        corr = -max_step;
    }

    *servo -= corr;

    if (*servo < 80.0f)
    {
        *servo = 80.0f;
    }
    else if (*servo > 258.0f)
    {
        *servo = 258.0f;
    }

    Servo_SetAngle(2, *servo);
}

/* ---------------- 位移 ---------------- */

/* 沿车体 (fwd, left) 方向走，航向锁 heading_deg */
static void Shoot_MoveBy(float fwd_mm, float left_mm, float speed_cms, float heading_deg)
{
    uint32_t t0 = HAL_GetTick();
    uint32_t last = t0;
    float len;
    float slow_mm;
    float dir_x;
    float dir_y;
    float travelled = 0.0f;
    float dt;
    float vx;
    float vy;
    float speed;

    len = sqrtf((fwd_mm * fwd_mm) + (left_mm * left_mm));
    if (len < 3.0f)
    {
        return;
    }

    dir_x = fwd_mm / len;
    dir_y = left_mm / len;

    /* 短距离时按比例缩短减速段，否则整段都在爬低速 */
    slow_mm = SHOOT_SLOW_MM;
    if (slow_mm > (len * 0.30f))
    {
        slow_mm = len * 0.30f;
    }

    while ((uint32_t)(HAL_GetTick() - t0) < SHOOT_TIMEOUT_MS)
    {
        uint32_t now = HAL_GetTick();

        dt = (float)(uint32_t)(now - last) / 1000.0f;
        last = now;
        if (dt > 0.05f)
        {
            dt = 0.05f;
        }

        Shoot_BodySpeed(&vx, &vy);
        travelled += sqrtf((vx * vx) + (vy * vy)) * dt * 10.0f;

        if (travelled >= (len - SHOOT_ARRIVE_MM))
        {
            break;
        }

        if ((len - travelled) > slow_mm)
        {
            speed = speed_cms;
        }
        else
        {
            speed = SHOOT_MIN_SPEED;
        }

        Car_Move(speed * dir_x, speed * dir_y, Shoot_HoldOmega(heading_deg));
        HAL_Delay(5U);
    }

    Shoot_Stop();
    Shoot_Brake();
}

/* ---------------- 戳 ---------------- */

static void Shoot_TryStab(float gimbal_deg, float heading_deg)
{
    int dx = 0;
    int can_hit = 0;
    int red_dx = 0;
    uint8_t red = 0U;
    uint8_t cmd;
    uint8_t target_cnt = 0U;
    uint8_t none_cnt = 0U;
    uint8_t lost_cnt = 0U;
    uint32_t t0;
    uint32_t last;
    float servo = gimbal_deg;
    float bearing;
    float bx;
    float by;
    float moved = 0.0f;
    float dt;
    float vx;
    float vy;
    float elapsed;
    uint8_t phase;
    uint8_t phase_prev;

    /* ---- 0) 到点先停稳，并丢掉移动途中积压的旧帧 ---- */
    Shoot_Stop();
    HAL_Delay(100U);
    Visual_Rx_Clear();
    HAL_Delay(30U);

    /* ---- 1) 判红 / 判目标：最多 SHOOT_SEEK_MS ---- */
    t0 = HAL_GetTick();
    while ((uint32_t)(HAL_GetTick() - t0) < SHOOT_SEEK_MS)
    {
        cmd = Shoot_Poll(&dx, &can_hit, &red, &red_dx);

        if (cmd == VISUAL_COMMAND_INVALID)
        {
            HAL_Delay(5U);
            continue;
        }

        /*
         * 红球一票否决：只要这一帧带了红球标记（none,red,x 或 dx,n,red,x），
         * 立刻放弃该点，不出针也不移动。
         */
        if (red != 0U)
        {
            return;
        }

        if (cmd == VISUAL_COMMAND_TARGET)
        {
            target_cnt++;
            break;      /* 已确认可戳，不用再等 */
        }

        if (cmd == VISUAL_COMMAND_NONE)
        {
            none_cnt++;
            if (none_cnt >= SHOOT_JUDGE_FRAMES)
            {
                return; /* 收够帧仍没有目标，原地不动 */
            }
        }

        HAL_Delay(5U);
    }

    if (target_cnt == 0U)
    {
        return;         /* 没看到目标：原地不动，绝不盲戳 */
    }

    /* ---- 2) 对准：最多 SHOOT_AIM_MAX_MS ---- */
    t0 = HAL_GetTick();
    while ((uint32_t)(HAL_GetTick() - t0) < SHOOT_AIM_MAX_MS)
    {
        cmd = Shoot_WaitFrame(&dx, &can_hit, &red, &red_dx, 20U);

        if ((cmd == VISUAL_COMMAND_TARGET) && (red == 0U))
        {
            Shoot_Aim(&servo, dx, SHOOT_AIM_GAIN, SHOOT_AIM_DEADBAND, SHOOT_AIM_MAX_STEP);

            if ((dx <= SHOOT_AIM_TOLERANCE) && (dx >= -SHOOT_AIM_TOLERANCE))
            {
                break;
            }
        }
    }

    /*
     * ---- 3) 出针 ----
     * 到点就已经在可戳距离内，所以不再做"等 can_hit 才靠近"，
     * 直接按参考工程 can_hit=1 之后的流程走：出针 -> 慢速直推 -> 反复扎。
     */
    bearing = (servo - SHOOT_SERVO_CENTER) * (PI / 180.0f);
    bx = cosf(bearing);
    by = sinf(bearing);

    Servo_SetAngle(1, SHOOT_NEEDLE_OUT);
    HAL_Delay(SHOOT_NEEDLE_OUT_MS);

    /* ---- 4) 先直推，再反复出针 + 持续前移，直到目标消失 ---- */
    t0 = HAL_GetTick();
    last = t0;
    phase_prev = 1U;
    lost_cnt = 0U;

    while ((uint32_t)(HAL_GetTick() - t0) < SHOOT_STAB_MAX_MS)
    {
        uint32_t now = HAL_GetTick();

        dt = (float)(uint32_t)(now - last) / 1000.0f;
        last = now;
        if (dt > 0.05f)
        {
            dt = 0.05f;
        }

        elapsed = (float)(uint32_t)(now - t0);

        /* 前 SHOOT_PUSH_MS 保持出针直推，之后进入反复扎节拍 */
        if (elapsed >= (float)SHOOT_PUSH_MS)
        {
            uint32_t jab_phase;

            jab_phase = ((uint32_t)elapsed - SHOOT_PUSH_MS) %
                        (SHOOT_JAB_IN_MS + SHOOT_JAB_OUT_MS);
            phase = (jab_phase >= SHOOT_JAB_IN_MS) ? 1U : 0U;

            if (phase != phase_prev)
            {
                phase_prev = phase;
                Servo_SetAngle(1, (phase != 0U) ? SHOOT_NEEDLE_OUT : SHOOT_NEEDLE_IN);
            }
        }

        Shoot_BodySpeed(&vx, &vy);
        moved += sqrtf((vx * vx) + (vy * vy)) * dt * 10.0f;

        cmd = Shoot_Poll(&dx, &can_hit, &red, &red_dx);

        if (cmd == VISUAL_COMMAND_TARGET)
        {
            lost_cnt = 0U;
            Shoot_Aim(&servo, dx, SHOOT_TRIM_GAIN, SHOOT_TRIM_DEADBAND,
                      SHOOT_TRIM_MAX_STEP);
        }
        else if (cmd == VISUAL_COMMAND_NONE)
        {
            /* 出针后至少 SHOOT_MIN_STAB_MS 才允许判破，避免刚出针就误判 */
            if ((red == 0U) && (elapsed >= (float)SHOOT_MIN_STAB_MS))
            {
                lost_cnt++;
                if (lost_cnt >= SHOOT_LOST_FRAMES)
                {
                    break; /* 目标消失 = 扎破了 */
                }
            }
        }
        else
        {
            /* 本拍没有新帧，保持上一拍状态 */
        }

        Car_Move((elapsed < (float)SHOOT_PUSH_MS) ? (SHOOT_PUSH_SPEED * bx) : (SHOOT_JAB_SPEED * bx),
                 (elapsed < (float)SHOOT_PUSH_MS) ? (SHOOT_PUSH_SPEED * by) : (SHOOT_JAB_SPEED * by),
                 Shoot_HoldOmega(heading_deg));
        HAL_Delay(5U);
    }

    /* ---- 7) 收针 ---- */
    Shoot_Stop();
    Servo_SetAngle(1, SHOOT_NEEDLE_IN);
    HAL_Delay(SHOOT_RETRACT_MS);

    /* ---- 8) 回位：沿同一方向反向走回去 ---- */
    if (moved > 3.0f)
    {
        Shoot_MoveBy(-(moved * bx), -(moved * by), SHOOT_RETURN_SPEED, heading_deg);
    }

    /* 云台与航向复位 */
    Servo_SetAngle(2, gimbal_deg);
    (void)Imu_TurnTo(heading_deg);
}

void Shoot_Run(void)
{
    uint8_t i;
    float expect = 0.0f; /* 期望航向：上电 0°，各段 turn_deg 累加 */

    (void)Shoot_VisionStart();

    for (i = 0U; i < (uint8_t)SHOOT_LEG_COUNT; i++)
    {
        /* 表里写的是增量，累加成这一段的绝对期望航向 */
        expect += SHOOT_LEGS[i].turn_deg;

        /* 需要转向的段：对齐到累计后的绝对角度，消掉单次转向残差 */
        if ((Imu_IsOnline() != 0U) &&
            ((SHOOT_LEGS[i].turn_deg > 2.0f) || (SHOOT_LEGS[i].turn_deg < -2.0f)))
        {
            (void)Imu_TurnTo(expect);
        }

        /*
         * 关键：整段平移/靠近/扎击/回位都用陀螺仪约束到 expect，
         * 不是锁在"转完那一刻的实际航向"，这样残差和途中被撞歪都会被拉回来。
         */
        Servo_SetAngle(2, SHOOT_LEGS[i].gimbal_deg);
        Servo_SetAngle(1, SHOOT_NEEDLE_IN);

        Shoot_MoveBy(SHOOT_LEGS[i].fwd_mm, SHOOT_LEGS[i].left_mm, SHOOT_SPEED, expect);

        /* 只有标记为"判可戳"的段，到点后才做判红/靠近/扎击 */
        if (SHOOT_LEGS[i].stab != 0U)
        {
            Shoot_TryStab(SHOOT_LEGS[i].gimbal_deg, expect);
        }
    }

    Shoot_Stop();
}
