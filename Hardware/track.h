#ifndef __TRACK_H
#define __TRACK_H

#include "main.h"

extern volatile uint16_t track_values[8];

void Track_Control(uint8_t enable);
void Track_RxCpltCallback(void);
void Track_ErrorCallback(void);

#endif
