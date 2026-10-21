#include "Servo.h"
#include "gpio.h"
#include "main.h"
#include "tim.h"

/**
 * @brief  用于将角度限制在指定范围内的辅助函数。
 * @param  angle: 要限制的角度。
 * @param  max_angle: 允许的最大角度。
 * @retval 限制后的角度值。
 */
static float clamp_angle(float angle, float max_angle)
{
    if (angle < 0)
    {
        return 0;
    }

    if (angle > max_angle)
    {
        return max_angle;
    }

    return angle;
}

/**
 * @brief  初始化舵机控制的PWM信号。
 * @retval None
 */
void Servo_Init(void)
{
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);
}

/**
 * @brief  设置指定舵机的角度。
 * @param  model: 舵机编号（1、2或3）其中3为预留舵机接口。
 *                其中舵机2的角度范围为0-360°，舵机1和3的角度范围为0-180°。
 *                如果输入的角度超出范围，将会被限制在允许的范围内。
 *                舵机1对应TIM3的通道3，舵机2对应TIM3的通道4，舵机3对应TIM3的通道2。
 *                例如：
 *                - Servo_SetAngle(1, 90); // 设置舵机1的角度为90°
 *                - Servo_SetAngle(2, 180); // 设置舵机2的角度为180°
 *                - Servo_SetAngle(3, 45); // 设置舵机3的角度为45°（预留接口，暂时用不到）
 * @param  angle: 要设置的角度值。
 * @retval None
 * 接线说明：
 * 舵机1的信号线连接到TIM3的通道3（PB0），舵机2的信号线连接到TIM3的通道4（PB1），舵机3的信号线连接到TIM3的通道2（PA4）。
 */
void Servo_SetAngle(int model, float angle)
{
    uint32_t channel;

    if (model == 1)
    {
        channel = TIM_CHANNEL_3;
        angle = clamp_angle(angle, 180);
    }
    else if (model == 2)
    {
        channel = TIM_CHANNEL_4;
        angle = clamp_angle(angle, 360) / 2;
    }
    else if (model == 3) // 暂时用不到为预留接口
    {
        channel = TIM_CHANNEL_2;
        angle = clamp_angle(angle, 180);
    }
    else
    {
        return;
    }

    __HAL_TIM_SET_COMPARE(&htim3, channel, angle * (htim3.Init.Period + 1) / 1800 + (htim3.Init.Period + 1) / 40);
}
