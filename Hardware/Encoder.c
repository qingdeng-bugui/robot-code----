#include "main.h"
#include "gpio.h"
#include "tim.h"
#include "math.h"
#include "Encoder.h"
#define PPR 4680.0f   // 输出轴每圈计数（13线 × 90减速比 × 4） 

// 上一次的计数
static int16_t last_count[ENCODER_NUM] = {0};

/**
 * @brief  初始化编码器
 * @retval None
 */
void Encoder_Init(void)
{
    HAL_TIM_Encoder_Start(&htim4,TIM_CHANNEL_ALL);  //PA11 CH1 / PA12 CH2

    HAL_TIM_Encoder_Start(&htim5,TIM_CHANNEL_ALL);  //PA0 CH1 / PC12 CH2

    HAL_TIM_Encoder_Start(&htim8,TIM_CHANNEL_ALL);  //PC6 CH1 / PC7 CH2

    HAL_TIM_Encoder_Start(&htim20,TIM_CHANNEL_ALL);  //PB2 CH1 / PC2 CH2
}

/**
 * @brief  把计数增量换算成角速度）
 * @retval 角速度，单位 °/s
 */
float Get_Encoder_Value(int16_t model)
{
    float Angle=0;
    float W=0;
    int16_t delta=0;
    int16_t now=0;
    if (model < 1 || model > ENCODER_NUM)
    {
        return 0.0f;
    }
    if (model==1)
    {
        now= (int16_t)__HAL_TIM_GET_COUNTER(&htim4);
        delta=(int16_t)(now - last_count[model-1]);
        last_count[model-1]=now;
        Angle=(float)(delta * 360.0f)/PPR;
        W=Angle/0.01f; 
        return W;
    }
       
    else if (model==2)
    {
        now= (int16_t)__HAL_TIM_GET_COUNTER(&htim5);
        delta=(int16_t)(now - last_count[model-1]);
        last_count[model-1]=now;
        Angle=(float)(delta * 360.0f) / PPR;
        W=Angle/0.01f;
        return W;
    }
    else if (model==3)
    {
        now= (int16_t)__HAL_TIM_GET_COUNTER(&htim8);
        delta=(int16_t)(now - last_count[model-1]);
        last_count[model-1]=now;
        Angle=(float)(delta * 360.0f) / PPR;
        W=Angle/0.01f;
        return W;
    }
    else if (model==4)
    {
        now= (int16_t)__HAL_TIM_GET_COUNTER(&htim20);
        delta=(int16_t)(now - last_count[model-1]);
        last_count[model-1]=now;
        Angle=(float)(delta * 360.0f) / PPR;
        W=Angle/0.01f;
        return W;
    }
    else
    {
        return 0.0f;
    }
 }


