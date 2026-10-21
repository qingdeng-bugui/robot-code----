#ifndef __PHOTORESISTOR_H
#define __PHOTORESISTOR_H

#include "main.h"

void Photoresistor(void);

#define LIGHT_ADC_CHANNEL ADC_CHANNEL_5  //光敏电阻通道,PB14
#define LIGHT_OFFSET 800  //强光偏移值
extern uint16_t stronglight_threshold; //强光阈值
extern uint16_t EnvnironmentLight; //上电环境光强度
extern volatile uint16_t LightValue; //光敏电阻采集值
extern volatile uint8_t car_start_flag; //强光启动标志位

void Photoresistor_Init(void); //光敏电阻初始化
uint16_t Set_StrongLightThreshold(void);
uint16_t Set_EnvnironmentLight(void);

#endif
