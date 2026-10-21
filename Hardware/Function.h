#ifndef __FUNCTION_H
#define __FUNCTION_H

#include <stdint.h>

/* 循迹已迁移到 LineFast.c 的 LineFast_Run()（陀螺仪辅助高速版） */

/*
 * 在运动控制循环中，每次正常运动指令后周期调用。白底时不发送运动指令；
 * 检测到边界时停车或向内平移，始终不旋转车身。
 */
void Keep_In(void);

/* USART1视觉接收：中断接收字节，场内循环解析完整帧。 */
void Visual_Rx_Init(void);
void Visual_Rx_Clear(void);
void Visual_RxCpltCallback(void);
void Visual_RxErrorCallback(void);

/* 视觉帧解析，Nav.c 也要用，所以不再设成 static。
 * 返回值：0=无效帧 1=目标帧(dx,can_hit) 2=none 3=ok */
uint8_t Visual_ReadLatestLine(char *line);
uint8_t Visual_ParseLine(const char *line, int *dx, int *can_hit,
                         uint8_t *red_present, int *red_dx);

void Hit_Balloons(void);
void All_functions(void);
#endif
