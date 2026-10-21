#ifndef __IR_NEC_H
#define __IR_NEC_H

#include "main.h"
extern volatile uint8_t IR_buf[4];
void IR_NEC_Init(void);
void IR_NEC_DeInit(void);
void IR_work(void);

#endif