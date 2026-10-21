#include "PID.h"
#include "main.h"
#include "tim.h"
#include "Motor.h"
#include "Encoder.h"
#include <math.h>

static PIDController g_pid[Motor_Num]; //存放pid

// 初始化单个PID控制器
static void PID_Init(PIDController *pid)
{
    pid->Kp = 0;
    pid->Ki = 0;
    pid->Kd = 0;
    pid->error = 0.0f;
    pid->Last_error = 0.0f;
    pid->integral = 0.0f;
    pid->derivative = 0.0f;
    pid->target = 0.0f;
    pid->Current_Speed = 0.0f;
    pid->Out = 0.0f;
}

// 设置目标速度并初始化PID,角速度最大为1500
void Set_Target_Speed(uint16_t model, float Speed)
{
    if ((model < 1) || (model > Motor_Num))
    {
        return;
    }
    g_pid[model - 1].target = Speed;



    if (Speed > 1500.0f)
    {
        g_pid[model - 1].target = 1500.0f;
    }
    else if (Speed < -1500.0f)
    {
        g_pid[model - 1].target = -1500.0f;
    }


    if(model==1)
    {
        g_pid[model - 1].Kp = 84.5645f;
        g_pid[model - 1].Ki = 1.3735f;
        g_pid[model - 1].Kd = 5.9654f;
        g_pid[model - 1].integral_Max = 7280.66982f;
    }
    else if(model==2)
    {
        g_pid[model - 1].Kp =60.5f;
        g_pid[model - 1].Ki =1.34f;
        g_pid[model - 1].Kd =0.560f;
        g_pid[model - 1].integral_Max =6666.667;
    }
    
    else if(model==3)
    {
        g_pid[model - 1].Kp = 65.5f;
        g_pid[model - 1].Ki = 1.75f;
        g_pid[model - 1].Kd = 15.0f;
        g_pid[model - 1].integral_Max =12000.66982f;
    }
    
    else if(model==4)
    {
        g_pid[model - 1].Kp =60.5f;
        g_pid[model - 1].Ki =1.34f;
        g_pid[model - 1].Kd =0.560f;
        g_pid[model - 1].integral_Max =6666.667;
    }
}

// 读取PID输出
float PID_Get_Output(uint16_t model)
{
    if ((model < 1) || (model > Motor_Num))
    {
        return 0.0f;
    }
    return g_pid[model - 1].Out;
}

// 读取当前误差
float PID_Get_Error(uint16_t model)
{
    if ((model < 1) || (model > Motor_Num))
    {
        return 0.0f;
    }
    return g_pid[model - 1].error;
}

// 读取编码器反馈速度
float PID_Get_Current_Speed(uint16_t model)
{
    if ((model < 1) || (model > Motor_Num))
    {
        return 0.0f;
    }
    return g_pid[model - 1].Current_Speed;
}

// 读取目标速度
float PID_Get_Target(uint16_t model)
{
    if ((model < 1) || (model > Motor_Num))
    {
        return 0.0f;
    }
    return g_pid[model - 1].target;
}

// 计算PID输出
void PID_Cala(uint16_t model)
{
    if ((model < 1) || (model > Motor_Num))
    {
        return;
    }

    PIDController *pid = &g_pid[model - 1];

    // 读取编码器的速度
    float current_speed = (float)Get_Encoder_Value(model);
    pid->Current_Speed = current_speed;

    // 计算误差
    pid->Last_error = pid->error;
    pid->error = pid->target - current_speed;

    // 计算积分项
    pid->integral += pid->error;

    // 积分项限幅，防止积分饱和
    if (pid->integral > pid->integral_Max)
    {
        pid->integral = pid->integral_Max;
    }
    else if (pid->integral < -pid->integral_Max)
    {
        pid->integral = -pid->integral_Max;
    }

    // 计算微分项
    pid->derivative = pid->error - pid->Last_error;

    // 计算PID输出
    pid->Out = pid->Kp * pid->error + pid->Ki * pid->integral + pid->Kd * pid->derivative;

    // 输出限幅
    if (pid->Out > 10000.0f)
    {
        pid->Out = 10000.0f;
    }
    else if (pid->Out < -10000.0f)
    {
        pid->Out = -10000.0f;
    }
}

//用回调函数完成PID
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim == &htim1)
    {
        for (uint16_t i = 0; i < Motor_Num; i++)
        {
            PID_Cala(i + 1);
            Set_Speed(i + 1, (int16_t)g_pid[i].Out);
        }
    }
}
