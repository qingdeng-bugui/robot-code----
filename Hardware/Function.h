#ifndef __FUNCTION_H
#define __FUNCTION_H

void Tack_line(void);

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
void Hit_Balloons(void);
void All_functions(void);
#endif
