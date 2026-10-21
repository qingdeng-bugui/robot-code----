#ifndef __LINEFAST_H
#define __LINEFAST_H

#include <stdint.h>

/* ==================================================================
 * 陀螺仪辅助的高速循迹（本工程唯一的循迹实现）
 *
 *   1. 弯道用陀螺仪精确转 90°，不再"固定差速 + 定时补转"
 *   2. 直线段用 PD 控制（加微分抑制高速摆动）
 *   3. 丢线时用陀螺仪保持航向直行，不再原地慢速打转找线
 *   4. 左右轮目标做过归一化，高速时不会因为外侧饱和而失去转向能力
 * ================================================================== */
/* ---------------- 速度 ---------------- */

/* 轮速单位是 °/s，PID 里硬上限 1500（约 78.5 cm/s 车体线速度） */
#define LF_SPEED_MAX          1470   /* 直线基础速度 */
#define LF_SPEED_CORNER        980   /* 接近弯道时的基础速度 */
#define LF_SPEED_FINISH       1050   /* 第四弯之后的收尾速度 */

/* ---------------- 循迹 PD ---------------- */

#define LF_KP                  52.0f
#define LF_KD                  16.0f
#define LF_CORR_MAX           380.0f  /* 差速修正上限 */

/* ---------------- 弯道（陀螺仪） ---------------- */

#define LF_TURN_OMEGA         280.0f  /* 转弯角速度 °/s，上限约 328 */
#define LF_TURN_VX              0.0f  /* 转弯时的前进速度 cm/s，0 = 原地转 */
#define LF_TURN_TOL             3.0f  /* 角度到位判据 ±3° */
#define LF_TURN_TIMEOUT_MS    1500U   /* 转弯超时，防止卡死 */
#define LF_RELEASE_CYCLES        6    /* 出弯后连续几拍确认离开角点 */

/* ---------------- 丢线 ---------------- */

#define LF_LOST_SPEED          40.0f  /* 丢线时保持航向直行的速度 cm/s */
#define LF_LOST_HOLD_MS       200U    /* 保持直行这么久还没找回线才转为搜索 */
#define LF_SEARCH_OMEGA       120.0f

/* ---------------- 终点 ---------------- */

#define LF_FINISH_GUARD_MS    1900U
#define LF_FINISH_WHITE       12

/* 收尾结束时要归到的绝对方向：0 = 上电方向（车头朝北） */
#define LF_FINISH_HEADING_DEG   0.0f

/* 收尾动作就是原来的定时弧线（60cm/s 365ms + 20cm/s 304ms），
 * 写在 LineFast.c 的终点分支里，没有额外参数。 */

/* 启动对准：沿用原来标好的动作，改这里就能调 */
#define LF_START_FWD_SPEED     60.0f
#define LF_START_FWD_MS       480U
#define LF_START_TURN_SPEED    30.0f
#define LF_START_TURN_OMEGA  (-180.0f)
#define LF_START_TURN_MS      520U

void LineFast_Run(void);

#endif
