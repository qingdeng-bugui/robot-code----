#include "Nav.h"
#include "main.h"
#include "Imu.h"
#include "Motor.h"
#include "PID.h"
#include "Servo.h"
#include "OLED.h"
#include <math.h>
#include <string.h>

Nav_State_t nav_state;

static uint32_t nav_last_tick = 0U;
static uint8_t  nav_inited    = 0U;

/* ==================================================================
 * 小工具
 * ================================================================== */

static float Nav_Wrap180(float a)
{
    a = fmodf(a + 180.0f, 360.0f);
    if (a < 0.0f)
    {
        a += 360.0f;
    }
    return a - 180.0f;
}

/* 世界速度 -> 车体速度：vx 前进为正，vy 左移为正 */
static void Nav_WorldToBody(float th_deg, float Vx, float Vy, float *vx, float *vy)
{
    float th = th_deg * 3.14159265f / 180.0f;
    *vx = -sinf(th) * Vx + cosf(th) * Vy;
    *vy = -cosf(th) * Vx - sinf(th) * Vy;
}

void Nav_Stop(void)
{
    Set_Target_Speed(1U, 0.0f);
    Set_Target_Speed(2U, 0.0f);
    Set_Target_Speed(3U, 0.0f);
    Set_Target_Speed(4U, 0.0f);
    Set_Speed(1, 0);
    Set_Speed(2, 0);
    Set_Speed(3, 0);
    Set_Speed(4, 0);
}

/* ==================================================================
 * 里程计
 * ================================================================== */

void Nav_Update(void)
{
    uint32_t now = HAL_GetTick();
    float dt;
    float lf, rf, lr, rr;
    float vx, vy, th;

    if (nav_inited == 0U)
    {
        nav_last_tick = now;
        return;
    }

    if (nav_last_tick == 0U)
    {
        nav_last_tick = now;
        return;
    }

    dt = (float)(uint32_t)(now - nav_last_tick) / 1000.0f;
    nav_last_tick = now;

    if (dt <= 0.0f)
    {
        return;
    }
    if (dt > 0.05f)
    {
        dt = 0.05f;     /* 长间隔不补积，避免一次冲很远 */
    }

    /* 轮速 °/s -> 轮周线速度 cm/s */
    lf = PID_Get_Current_Speed(1U) * (PI / 180.0f) * WHEEL_R;
    rf = PID_Get_Current_Speed(2U) * (PI / 180.0f) * WHEEL_R;
    lr = PID_Get_Current_Speed(3U) * (PI / 180.0f) * WHEEL_R;
    rr = PID_Get_Current_Speed(4U) * (PI / 180.0f) * WHEEL_R;

    /* 麦轮正解（Motor.c 逆解的逆） */
    vx = ( lf + rf + lr + rr) * 0.25f;
    vy = (-lf + rf + lr - rr) * 0.25f;

    th = Imu_GetYaw() * 3.14159265f / 180.0f;

    nav_state.x += (-vx * sinf(th) - vy * cosf(th)) * dt * 10.0f;
    nav_state.y += ( vx * cosf(th) - vy * sinf(th)) * dt * 10.0f;
    nav_state.theta = Imu_GetYaw();
}

/* ==================================================================
 * 运动原语
 * ================================================================== */

void Nav_MoveTo(float x_mm, float y_mm, float vmax_cms)
{
    uint32_t t0 = HAL_GetTick();
    float th_tgt = Imu_GetYaw();
    float dx, dy, dist, speed, Vx, Vy, vx, vy, werr, w;

    while ((uint32_t)(HAL_GetTick() - t0) < NAV_MOVE_TIMEOUT_MS)
    {
        Nav_Update();

        dx = x_mm - nav_state.x;
        dy = y_mm - nav_state.y;
        dist = sqrtf(dx * dx + dy * dy);

        if (dist < NAV_ARRIVE_MM)
        {
            break;
        }

        /* 远快近慢：短距离也要尽快吃到高速档 */
        if (dist > 120.0f)
        {
            speed = vmax_cms;
        }
        else if (dist > 50.0f)
        {
            speed = vmax_cms * 0.58f;
        }
        else if (dist > 20.0f)
        {
            speed = vmax_cms * 0.30f;
        }
        else
        {
            speed = 8.0f;
        }

        Vx = dx / dist * speed;
        Vy = dy / dist * speed;
        Nav_WorldToBody(Imu_GetYaw(), Vx, Vy, &vx, &vy);

        /* 航向保持：整段锁在出发时的朝向 */
        werr = Nav_Wrap180(th_tgt - Imu_GetYaw());
        w = 3.0f * werr;
        if (w > 100.0f)
        {
            w = 100.0f;
        }
        else if (w < -100.0f)
        {
            w = -100.0f;
        }

        Car_Move(vx, vy, w);
        HAL_Delay(5U);
    }

    Nav_Stop();
    Nav_Update();

    /*
     * 轻点一脚反向刹车，削掉惯性前冲。
     * 不刹的话 60cm/s 巡航的余速会让停车点往前多出好几厘米，
     * 这对"离气球还剩多少"影响很大。
     */
    Set_Target_Speed(1U, -120.0f);
    Set_Target_Speed(2U, -120.0f);
    Set_Target_Speed(3U, -120.0f);
    Set_Target_Speed(4U, -120.0f);
    HAL_Delay(60U);
    Nav_Stop();

    HAL_Delay(NAV_TURN_SETTLE_MS);
}

/*
 * 快速原地转向：直接给角速度闭环，比 Imu_TurnTo 的三遍收敛快得多。
 * 到位精度约 ±2°，对 25cm 的位移只对应 9mm 横向误差，够用。
 */
void Nav_TurnTo(float deg)
{
    uint32_t t0 = HAL_GetTick();
    float err;
    float w;

    while ((uint32_t)(HAL_GetTick() - t0) < 2000U)
    {
        err = Nav_Wrap180(deg - Imu_GetYaw());

        if (fabsf(err) < 2.0f)
        {
            break;
        }

        w = 5.0f * err;
        if (w > 260.0f)
        {
            w = 260.0f;
        }
        else if (w < -260.0f)
        {
            w = -260.0f;
        }

        Car_Move(0.0f, 0.0f, w);
        HAL_Delay(5U);
    }

    Nav_Stop();
    HAL_Delay(NAV_TURN_SETTLE_MS);
}

/* ==================================================================
 * 初始化与一步到位
 * ================================================================== */

void Nav_Init(void)
{
    memset(&nav_state, 0, sizeof(nav_state));

    Nav_Stop();

    /* 关掉上电时的 0° 航向保持，本模块自己做航向 */
    Imu_HoldStop();
    /* 以当前车头方向为 0° */
    Imu_ZeroYaw();

    nav_last_tick = HAL_GetTick();
    nav_inited    = 1U;
}

void Nav_GotoOnce(void)
{
    float bearing;

    Nav_Init();

    /* 钢针保持收回，云台先回正 */
    Servo_SetAngle(1, NAV_NEEDLE_IN);
    Servo_SetAngle(2, NAV_SERVO_CENTER);
    HAL_Delay(50U);

    /* 先转到目标方位：里程计直线前进比横移准 */
    bearing = atan2f(-NAV_TARGET_X_MM, NAV_TARGET_Y_MM) * 180.0f / 3.14159265f;
    Nav_TurnTo(bearing);

    /* 车头已经对着球，云台再按需要补一个角度，让针轴正对气球 */
    Servo_SetAngle(2, NAV_SERVO_CENTER + NAV_AIM_DEG);

    Nav_MoveTo(NAV_TARGET_X_MM, NAV_TARGET_Y_MM, NAV_VMAX);

    Nav_Stop();
    nav_inited = 0U;
}

#if (NAV_DEBUG_OLED != 0)
void Nav_DebugToOled(void)
{
    static uint32_t last = 0U;
    uint32_t now = HAL_GetTick();

    if ((uint32_t)(now - last) < 200U)
    {
        return;
    }
    last = now;

    OLED_ShowString(1, 1, "X");
    OLED_ShowSignedNum(1, 2, (int32_t)nav_state.x, 4);
    OLED_ShowString(1, 8, "Y");
    OLED_ShowSignedNum(1, 9, (int32_t)nav_state.y, 4);

    OLED_ShowString(2, 1, "T");
    OLED_ShowSignedNum(2, 2, (int32_t)nav_state.theta, 4);
}
#endif
