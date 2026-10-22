#ifndef __FUNCTION_H
#define __FUNCTION_H

#include <stdint.h>

/* 循迹已迁移到 LineFast.c 的 LineFast_Run()（陀螺仪辅助高速版） */
/* 场内动作已迁移到 Shoot.c 的 Shoot_Run() */

/* 视觉帧解析返回的命令类型（Function.c 与 Shoot.c 共用） */
#define VISUAL_COMMAND_INVALID 0U
#define VISUAL_COMMAND_TARGET  1U
#define VISUAL_COMMAND_NONE    2U
#define VISUAL_COMMAND_OK      3U

/*
 * 比赛锁：任意一种方式（红外 0x15 / 光敏）启动过一次比赛后置 1。
 * 置 1 之后红外按键与光敏都不再生效，必须重新上电才能再跑。
 */
extern volatile uint8_t race_locked;

/* USART1视觉接收：中断接收字节，场内循环解析完整帧。 */
void Visual_Rx_Init(void);
void Visual_Rx_Clear(void);
void Visual_RxCpltCallback(void);
void Visual_RxErrorCallback(void);

/* 视觉帧解析：不再设成 static，方便其它模块复用。
 * 返回值：0=无效帧 1=目标帧(dx,can_hit) 2=none 3=ok */
uint8_t Visual_ReadLatestLine(char *line);
uint8_t Visual_ParseLine(const char *line, int *dx, int *can_hit,
                         uint8_t *red_present, int *red_dx);

void All_functions(void);
#endif
