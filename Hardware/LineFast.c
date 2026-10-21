#include "LineFast.h"
#include "main.h"
#include "Function.h"
#include "Imu.h"
#include "Motor.h"
#include "PID.h"
#include "track.h"
#include <math.h>

/* 循迹模块黑线判定阈值，和 track.c 里解析出的模拟量单位一致 */
#define LF_BLACK_THRESHOLD   1300U

static void LF_Stop(void)
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

static float LF_Wrap180(float a)
{
    a = fmodf(a + 180.0f, 360.0f);
    if (a < 0.0f)
    {
        a += 360.0f;
    }
    return a - 180.0f;
}

/*
 * 把左右轮目标压到 PID 的上限以内，但保持两者的差值。
 * 直接分别限幅会让外侧轮被截断，高速时转向能力凭空消失。
 */
static void LF_LimitPair(float *l, float *r)
{
    float m = fabsf(*l);
    float k;

    if (fabsf(*r) > m)
    {
        m = fabsf(*r);
    }

    if (m > 1500.0f)
    {
        k = 1500.0f / m;
        *l *= k;
        *r *= k;
    }
}

void LineFast_Run(void)
{
    int i;
    int mask;
    int cnt;
    int line_err;
    int raw;
    int last_dir = -1;

    int turning = 0;
    int corner_ready = 1;
    int corners = 0;
    int release_cnt = 0;
    float turn_target = 0.0f;
    uint32_t turn_tick = 0U;
    uint32_t release_tick = 0U;

    int finish_armed = 0;
    int guard_passed = 0;
    int white_cnt = 0;
    uint32_t fourth_tick = 0U;

    uint32_t now;
    uint32_t lost_ms = 0U;
    float lost_heading = 0.0f;

    float last_err = 0.0f;
    float base, corr, left, right, err, omega;

    /* 导航/保持都让路，本模块自己做航向 */
    Imu_HoldStop();
    Track_Control(1);

    /* ---- 起步对准：沿用原来标好的动作 ---- */
    Car_Move(LF_START_FWD_SPEED, 0.0f, 0.0f);
    HAL_Delay(LF_START_FWD_MS);
    Car_Move(LF_START_TURN_SPEED, 0.0f, LF_START_TURN_OMEGA);
    HAL_Delay(LF_START_TURN_MS);

    while (1)
    {
        now = HAL_GetTick();

        /* ---- 采样 ---- */
        mask = 0;
        cnt  = 0;
        for (i = 0; i < 8; i++)
        {
            if (track_values[i] > LF_BLACK_THRESHOLD)
            {
                mask |= (1 << i);
                cnt++;
            }
        }

        /* ---- 终点确认后执行收尾动作 ---- */
        if (guard_passed != 0 && cnt == 0)
        {
            white_cnt++;
            if (white_cnt >= LF_FINISH_WHITE)
            {
                /* 原始收尾动作：急停 -> 边走边左转 -> 再走一小段 -> 停 */
                LF_Stop();
                HAL_Delay(30U);
                Car_Move(60.0f, 0.0f, 180.0f);
                HAL_Delay(365U);
                Car_Move(20.0f, 0.0f, 180.0f);
                HAL_Delay(304U);
                LF_Stop();

                /* 收尾把车头转回上电方向（0°），陀螺闭环 */
                (void)Imu_TurnTo(LF_FINISH_HEADING_DEG);
                break;
            }
        }
        else if (cnt != 0)
        {
            white_cnt = 0;
        }

        if ((finish_armed != 0) &&
            ((uint32_t)(now - fourth_tick) >= LF_FINISH_GUARD_MS))
        {
            guard_passed = 1;
        }

        /* ================= 弯道中：陀螺仪闭环转 90° ================= */
        if (turning != 0)
        {
            uint8_t done = 0U;

            if (Imu_IsOnline() == 0U)
            {
                /* 陀螺离线：退回定时转弯（90° / 280°/s ≈ 320ms），别让它一直转 */
                omega = 270.0f;
                if ((uint32_t)(now - turn_tick) > 320U)
                {
                    done = 1U;
                }
            }
            else
            {
                err = LF_Wrap180(turn_target - Imu_GetYaw());
                omega = 4.0f * err;

                if (omega > LF_TURN_OMEGA)
                {
                    omega = LF_TURN_OMEGA;
                }
                else if (omega < -LF_TURN_OMEGA)
                {
                    omega = -LF_TURN_OMEGA;
                }

                if ((fabsf(err) < LF_TURN_TOL) ||
                    ((uint32_t)(now - turn_tick) > LF_TURN_TIMEOUT_MS))
                {
                    done = 1U;
                }
            }

            if (done != 0U)
            {
                turning = 0;
                corner_ready = 0;
                release_cnt = 0;
                release_tick = HAL_GetTick();
                LF_Stop();
                HAL_Delay(20U);
            }
            else
            {
                Car_Move(LF_TURN_VX, 0.0f, omega);
                HAL_Delay(5U);
                continue;
            }
        }

        /* ================= 角点检测 ================= */
        if ((turning == 0) && (corner_ready != 0) &&
            ((mask & 0x07) == 0x07))
        {
            /* 先刹一下，削掉高速带来的前冲，转弯才不会甩过头 */
            LF_Stop();
            HAL_Delay(40U);

            turning = 1;
            turn_tick = HAL_GetTick();
            turn_target = Imu_GetYaw() + 90.0f;
            corners++;

            if (corners >= 4)
            {
                finish_armed = 1;
                guard_passed = 0;
                white_cnt = 0;
                fourth_tick = turn_tick;
            }

            continue;
        }

        /* 出弯后确认已经离开角点，才允许识别下一个 */
        if ((corner_ready == 0) && (cnt != 0))
        {
            if (((mask & 0x07) == 0) && ((mask & 0x18) != 0))
            {
                release_cnt++;
                if (release_cnt >= LF_RELEASE_CYCLES)
                {
                    corner_ready = 1;
                    release_cnt = 0;
                }
            }
            else if ((mask & 0x07) != 0)
            {
                release_cnt = 0;
            }

            /*
             * 兜底：转弯后 500ms 还没满足出弯条件也强制解锁。
             * 否则一旦某次没判准，后面所有角点都会被锁死、再也识别不到。
             */
            if ((uint32_t)(now - release_tick) > 500U)
            {
                corner_ready = 1;
                release_cnt = 0;
            }
        }

        /* ================= 正常循迹 ================= */

        /* 全白：丢线 */
        if (cnt == 0)
        {
            if (finish_armed != 0)
            {
                /* 收尾阶段短暂全白：保持原速直行，等终点确认 */
                Set_Target_Speed(1U, (float)LF_SPEED_FINISH);
                Set_Target_Speed(3U, (float)LF_SPEED_FINISH);
                Set_Target_Speed(2U, (float)LF_SPEED_FINISH);
                Set_Target_Speed(4U, (float)LF_SPEED_FINISH);
            }
            else
            {
                if (lost_ms == 0U)
                {
                    lost_heading = Imu_GetYaw();
                }
                lost_ms += 5U;

                if (lost_ms < LF_LOST_HOLD_MS)
                {
                    /* 先用陀螺仪保持朝向直行，绝大多数丢线在这里就找回来了 */
                    err = LF_Wrap180(lost_heading - Imu_GetYaw());
                    omega = 2.5f * err;
                    if (omega > 120.0f)
                    {
                        omega = 120.0f;
                    }
                    else if (omega < -120.0f)
                    {
                        omega = -120.0f;
                    }
                    Car_Move(LF_LOST_SPEED, 0.0f, omega);
                }
                else
                {
                    /* 还没找到：按最后一次偏差方向慢速搜索 */
                    if (last_dir > 0)
                    {
                        Car_Move(10.0f, 0.0f, LF_SEARCH_OMEGA);
                    }
                    else
                    {
                        Car_Move(10.0f, 0.0f, -LF_SEARCH_OMEGA);
                    }
                }
            }

            HAL_Delay(5U);
            continue;
        }

        lost_ms = 0U;

        /* 权重从左到右递增，黑线越靠右偏差越大 */
        line_err = -7 * ((mask >> 0) & 1) - 5 * ((mask >> 1) & 1)
                   - 3 * ((mask >> 2) & 1) - ((mask >> 3) & 1)
                   + ((mask >> 4) & 1) + 3 * ((mask >> 5) & 1)
                   + 5 * ((mask >> 6) & 1) + 7 * ((mask >> 7) & 1);

        raw = line_err / cnt;

        if (raw < -1)
        {
            last_dir = -1;
        }
        else if (raw > 1)
        {
            last_dir = 1;
        }

        /* PD：微分项抑制高速摆动 */
        corr = LF_KP * (float)raw + LF_KD * ((float)raw - last_err);
        last_err = (float)raw;

        if (corr > LF_CORR_MAX)
        {
            corr = LF_CORR_MAX;
        }
        else if (corr < -LF_CORR_MAX)
        {
            corr = -LF_CORR_MAX;
        }

        /* 基础速度：多路压线 / 偏差大 → 提前降速准备过弯 */
        if (finish_armed != 0)
        {
            base = (float)LF_SPEED_FINISH;
        }
        else if ((cnt >= 3) || (raw <= -5) || (raw >= 5))
        {
            base = (float)LF_SPEED_CORNER;
        }
        else
        {
            base = (float)LF_SPEED_MAX;
        }

        left  = base + corr;
        right = base - corr;
        LF_LimitPair(&left, &right);

        Set_Target_Speed(1U, left);
        Set_Target_Speed(3U, left);
        Set_Target_Speed(2U, right);
        Set_Target_Speed(4U, right);

        HAL_Delay(5U);
    }

    Track_Control(0);
}
