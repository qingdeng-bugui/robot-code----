#include "Photoresistor.h"
#include "adc.h"

uint16_t stronglight_threshold = 1000;
uint16_t EnvnironmentLight = 0;
volatile uint16_t LightValue = 0;
uint8_t volatile car_start_flag=0;

/**
 * @brief 计算上电环境光强度
 * @retval 环境光强度
 */
uint16_t Set_EnvnironmentLight(void)
{
    uint32_t sum = 0;
    uint16_t sample_count = 0;
    for(uint8_t i=0;i<10;i++)
    {
        HAL_ADC_Start(&hadc1);
        if(HAL_ADC_PollForConversion(&hadc1, 10) == HAL_OK)
        {
            sum += HAL_ADC_GetValue(&hadc1);
            sample_count++;
        }
        HAL_ADC_Stop(&hadc1);
        HAL_Delay(10);
    }
    if(sample_count > 0)
    {
        EnvnironmentLight = sum / sample_count;
        return EnvnironmentLight;
    }
    else
    {
        return 0;
    }
    
}

/**
 * @brief 计算强光阈值，并开启ADC中断
 * @retval 强光阈值
 * @note 该函数根据环境光强度计算强光阈值，并启动ADC中断。
 */
uint16_t Set_StrongLightThreshold(void)
{
    if (EnvnironmentLight > LIGHT_OFFSET)
  {
      stronglight_threshold = EnvnironmentLight - LIGHT_OFFSET;
      HAL_ADC_Start_IT(&hadc1);
  }
    if(stronglight_threshold > 4095)
    {
        stronglight_threshold = 4095;
    }
    HAL_ADC_Start_IT(&hadc1);
    return stronglight_threshold;
}

/**
 * @brief 初始化光敏电阻
 * @note 该函数用于初始化光敏电阻，包括计算环境光强度和强光阈值。
 */
//光敏电阻初始化
void Photoresistor_Init(void)
{
    if (Set_EnvnironmentLight() != 0)
  {
      Set_StrongLightThreshold();
  }
}

//光敏电阻中断回调函数
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
    if(hadc->Instance == hadc1.Instance)
    {
        LightValue = HAL_ADC_GetValue(hadc);
        if(LightValue <= stronglight_threshold)
        {
            car_start_flag = 1; //强光启动标志位
            HAL_ADC_Stop_IT(&hadc1);
        }
    }
}
