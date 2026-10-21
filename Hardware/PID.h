#ifndef __PID_H
#define __PID_H

#include "main.h"
#include <stdint.h>
#define Motor_Num 4 

typedef struct {
    float Kp;
    float Ki;
    float Kd;
    float error; // 当前误差
    float Last_error; // 上一次误差
    float integral; // 误差累积
    float derivative; // 误差变化率
    float target; // 目标速度
    float Current_Speed; // 编码器反馈速度
    float Out; //PID输出
    float integral_Max; //积分限幅阈值
} PIDController;

void PID_All_Init(float Kp, float Ki, float Kd);
void Set_Target_Speed(uint16_t model,float Speed);
void PID_Cala(uint16_t model);
float PID_Get_Output(uint16_t model);
float PID_Get_Error(uint16_t model);
float PID_Get_Current_Speed(uint16_t model);
float PID_Get_Target(uint16_t model);
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim);

#endif
