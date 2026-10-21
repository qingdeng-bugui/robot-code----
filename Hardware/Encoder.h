#ifndef __encoder_H
#define __encoder_H

#include "main.h"

#define ENCODER_NUM 4 /* 编码器数量 */

void Encoder_Init(void);
float Get_Encoder_Value(int16_t model);

#endif
