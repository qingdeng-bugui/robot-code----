#ifndef __SHOOT_H
#define __SHOOT_H

#include <stdint.h>

/* ==================================================================
 * 场内戳球
 *
 * 循迹结束后：开视觉 -> 按点位表位移到第一个点 -> 判红/判目标
 *   -> 能戳：对准、靠近、反复出针并持续前移直到目标消失 -> 退回点位
 *   -> 不能戳：原地不动，准备去下一个点
 * ================================================================== */

/* ---------------- 点位表 ---------------- */

typedef struct
{
    float turn_deg;   /* 走这一段之前先相对当前航向转多少度（正数=左转，负数=右转） */
    float fwd_mm;     /* 相对上一个点位：向前 */
    float left_mm;    /* 相对上一个点位：向左（负值 = 向右） */
    float gimbal_deg; /* 到点后云台（舵机2）的角度 */
    uint8_t stab;     /* 1 = 到点后判红/判可戳；0 = 只移动 */
} Shoot_Leg_t;

/* ---------------- 第一段位移 ---------------- */

#define SHOOT_LEFT_MM 320.0f /* 向左位移（原来 25cm 再多 7cm） */
#define SHOOT_FWD_MM  50.0f  /* 向前位移 */

/* ---------------- 第二段位移：7 号位 -> 6 号位 ---------------- */

#define SHOOT_FWD2_MM     210.0f /* 向前 21cm（23 − 2） */
#define SHOOT_RIGHT2_MM   30.0f  /* 向右 3cm（4 − 1） */
#define SHOOT_GIMBAL2_DEG 170.0f /* 6 号位的云台角度 */

/* ---------------- 第三、四段：探路，不判可戳 ---------------- */

#define SHOOT_RIGHT3_MM 250.0f /* 6 号位 -> 向右 25cm */
#define SHOOT_FWD3_MM   300.0f /* 再向前 30cm */

/* ---------------- 第五段：真正到 3 号位 ---------------- */

#define SHOOT_TURN3_DEG 40.0f /* 相对当前航向：左转 40°（-> 3 号位方向） */
/* 相对原落点：场地正西 3cm、正北 1cm。
 * 换算到 40° 车体系（前 = 北偏西 40°，左 = 西南偏 40°）：
 *   前进 117mm、左移 17mm  <->  场地 (-88, +79) mm  */
#define SHOOT_FWD4_MM     117.0f
#define SHOOT_LEFT4_MM    17.0f
#define SHOOT_GIMBAL3_DEG 135.0f /* 3 号位的云台角度 */

/* ---------------- 第六、七段：3 号位 -> 4 号位 ---------------- */

#define SHOOT_TURN4_DEG   (-40.0f) /* 相对当前航向：右转 40° 转回 0° */
#define SHOOT_BACK4_MM    50.0f    /* 再后退 5cm */
#define SHOOT_RIGHT4_MM   130.0f   /* 最后右移 13cm */
#define SHOOT_FWD4B_MM    150.0f   /* 右移的同时前进 15cm（原 10cm，4 号位北移 5cm） */
#define SHOOT_GIMBAL4_DEG 80.0f    /* 4 号位的云台角度 */

/* ---------------- 第八段：4 号位 -> 1 号位 ---------------- */

#define SHOOT_FWD_TO1_MM  200.0f /* 前进 20cm（原 25cm，补偿 4 号位的 5cm 北移） */
#define SHOOT_GIMBAL1_DEG 185.0f /* 1 号位的云台角度 */

/* ---------------- 第九~十一段：1 号位 -> 2 号位 ---------------- */

#define SHOOT_RIGHT1_MM      100.0f /* 第一段：右移 10cm */
#define SHOOT_FWD1_MM        150.0f /* 第二段：斜着走，前进 15cm */
#define SHOOT_RIGHT1B_MM     150.0f /* 第二段：同时右移 15cm（45° 斜移） */
#define SHOOT_TURN2_DEG      180.0f /* 相对当前航向：左转 180°（掉头） */
#define SHOOT_FWD2B_MM       30.0f  /* 掉头后再往车头方向 1cm */
#define SHOOT_LEFT2_MM       80.0f  /* 掉头后左移 8cm */
#define SHOOT_GIMBAL_TO2_DEG 248.0f /* 2 号位的云台角度 */

/* ---------------- 第十二~十四段：2 号位 -> 5 号位 ---------------- */

#define SHOOT_RIGHT2B_MM  150.0f   /* 右移 15cm（180° 朝向下 = 场地正西） */
#define SHOOT_TURN5A_DEG  (-30.0f) /* 相对当前航向：右转 30° */
#define SHOOT_FWD5_MM     520.0f   /* 再直行 52cm */
#define SHOOT_TURN5_DEG   120.0f   /* 相对当前航向：左转 120°，转到场地正东 */
#define SHOOT_FWD5B_MM    290.0f   /* 转向正东后前进 30cm */
#define SHOOT_GIMBAL5_DEG 168.0f   /* 5 号位的云台角度（正前方） */

/* ---------------- 第十五段：5 号位 -> 8 号位 ---------------- */

#define SHOOT_TURN8_DEG   (-60.0f) /* 相对当前航向：右转 60° */
#define SHOOT_FWD8_MM     278.0f   /* 再前进 27cm（沿车头方向） */
#define SHOOT_GIMBAL8_DEG 168.0f   /* 8 号位的云台角度 */

/* ---------------- 行走 ---------------- */

#define SHOOT_SPEED      60.0f /* 平移速度 cm/s */
#define SHOOT_MIN_SPEED  20.0f /* 收尾减速段速度 cm/s */
#define SHOOT_SLOW_MM    80.0f /* 剩余这么多就切低速 */
#define SHOOT_ARRIVE_MM  8.0f  /* 提前这么多停车，抵消惯性 */
#define SHOOT_TIMEOUT_MS 3000U /* 单段位移超时 */

/* ---------------- 方向保持 ---------------- */

#define SHOOT_HEADING_DEG 0.0f  /* 全程保持的方向（0 = 上电方向） */
#define SHOOT_HOLD_GAIN   3.0f  /* 航向修正比例 */
#define SHOOT_HOLD_OMEGA  60.0f /* 航向修正上限 °/s */

/* ---------------- 舵机 ---------------- */

#define SHOOT_GIMBAL_DEG   235.0f /* 云台（舵机2）固定角度 */
#define SHOOT_SERVO_CENTER 168.0f /* 云台正前方对应的逻辑角度 */
#define SHOOT_NEEDLE_IN    87     /* 钢针完全收回 */
#define SHOOT_NEEDLE_OUT   135    /* 钢针完全伸出 */

/* ---------------- 视觉 ---------------- */

#define SHOOT_VISION_WAIT_MS 400U  /* 等首帧上限 */
#define SHOOT_SEEK_MS        1500U /* 判红/判目标的采样窗口（帧率低，要给够时间） */
#define SHOOT_JUDGE_FRAMES   3U    /* 至少收到这么多"无目标"帧才下结论 */

/* ---------------- 对准 ---------------- */

#define SHOOT_AIM_GAIN      0.1f
#define SHOOT_AIM_DEADBAND  6
#define SHOOT_AIM_MAX_STEP  6.0f
#define SHOOT_AIM_TOLERANCE 30
#define SHOOT_AIM_MAX_MS    150U

/* ---------------- 扎击 ---------------- */

#define SHOOT_NEEDLE_OUT_MS 130U  /* 出针后等舵机走完 */
#define SHOOT_PUSH_SPEED    8.0f  /* 直推速度 cm/s */
#define SHOOT_PUSH_MS       400U  /* 直推时长 */
#define SHOOT_JAB_SPEED     3.0f  /* 反复扎时的前进速度 cm/s */
#define SHOOT_JAB_IN_MS     180U  /* 收针保持 */
#define SHOOT_JAB_OUT_MS    200U  /* 出针保持 */
#define SHOOT_STAB_MAX_MS   2500U /* 出针后总时限 */
#define SHOOT_MIN_STAB_MS   300U  /* 出针后至少这么长才允许判破，防误判 */
#define SHOOT_TRIM_GAIN     0.05f /* 扎击期间云台微调 */
#define SHOOT_TRIM_DEADBAND 20
#define SHOOT_TRIM_MAX_STEP 2.0f
#define SHOOT_RETRACT_MS    180U /* 收针后稳定 */

/* ---------------- 回位 ---------------- */

#define SHOOT_RETURN_SPEED 30.0f /* cm/s */
#define SHOOT_LOST_FRAMES  3U    /* 连续几帧没有目标就认为已扎破 */

void Shoot_Run(void);

#endif
