#ifndef __NAV_H
#define __NAV_H

#include <stdint.h>

/* ==================================================================
 * 单点定点移动（第一版：只负责"走到一个点然后停住"）
 *
 * 坐标系：以循迹结束、收尾动作完成后的停车位置为原点
 *   +Y = 停车时车头的正前方
 *   +X = 车头的右侧
 *   单位 mm，航向 θ 由陀螺仪给，右转（顺时针）为负
 *
 * 流程：原地转到目标方位 -> 直线走过去 -> 到位停车
 * 先转再走，是因为直线前进的里程计精度比横移好得多。
 * ================================================================== */

/* 1 = 在 OLED 上显示实时 x/y/θ */
#define NAV_DEBUG_OLED       0

/* ---------------- 目标点（7 号气球位） ---------------- */

/*
 * 这是相对停车位置的第一版估计值，实测后按偏差改这两个数即可：
 *   停得偏前  -> 减小 NAV_TARGET_Y_MM
 *   停得偏右  -> 减小 |NAV_TARGET_X_MM|（更负=更靠左）
 * 改完重新编译烧录，跑一次，再量。
 */
/*
 * 实测反推：黄球表面就在循迹终点前方约 30cm。
 * 上一次命令 257mm 加上惯性前冲正好撞上，所以收到 150mm，
 * 到位后再轻点一脚刹车，预计停在离球心 20~25cm。
 */
#define NAV_TARGET_X_MM      (-117.0f)   /* 负 = 停车时的左侧 */
#define NAV_TARGET_Y_MM      (  94.0f)   /* 正 = 停车时的前方 */

/* ---------------- 运动参数 ---------------- */

#define NAV_ARRIVE_MM        (  12.0f)   /* 到位判据 */
#define NAV_MOVE_TIMEOUT_MS  (5000U)     /* 单次移动超时 */
#define NAV_VMAX             (  60.0f)   /* 巡航速度 cm/s */
#define NAV_TURN_SETTLE_MS   (  60U)     /* 转完/走完的稳定等待 */

/* ---------------- 云台对准气球 ---------------- */

#define NAV_SERVO_CENTER     ( 168.0f)   /* 云台正前方 */
#define NAV_AIM_DEG          (  30.0f)   /* 到位后云台再往左偏多少度对准气球 */
#define NAV_NEEDLE_IN        (  87)      /* 钢针完全收回 */

/* ---------------- 状态 ---------------- */

typedef struct
{
    float x;        /* mm */
    float y;        /* mm */
    float theta;    /* 度，0 = 停车时的车头方向，顺时针为负 */
} Nav_State_t;

extern Nav_State_t nav_state;

/* ---------------- 接口 ---------------- */

/* 把当前位置设成原点、当前朝向置 0、关掉航向保持 */
void Nav_Init(void);

/* 主循环周期调用：里程计积分 */
void Nav_Update(void);

/* 阻塞走到 (x_mm, y_mm) 后停车 */
void Nav_MoveTo(float x_mm, float y_mm, float vmax_cms);

/* 阻塞原地转到 deg（度） */
void Nav_TurnTo(float deg);

/* 立即停车 */
void Nav_Stop(void);

/* 一步到位：以当前位姿为原点，走到 NAV_TARGET_X/Y 后停住 */
void Nav_GotoOnce(void);

#endif
