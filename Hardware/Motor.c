#include "Motor.h"
#include "PID.h"
#include "gpio.h"
#include "main.h"
#include "tim.h"

/**
 * @brief  Initializes the motor control.
 * @retval None
 */
void Motor_Init(void)
{
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);
}

/**
 * @brief  Sets the speed of the motor.
 * @param  model: Motor model (1 to 4),1对应左前轮，2对应右前轮，3对应左后轮，4对应右后轮
 * @param  speed: Speed value (-10000 to 10000)
 * @retval None
 */
void Set_Speed(int model, int speed)
{
    uint32_t duty;

    if ((model != 1) && (model != 2) && (model != 3) && (model != 4))
    {
        return;
    }

    if (speed > 10000)
    {
        speed = 10000;
    }
    else if (speed < -10000)
    {
        speed = -10000;
    }

    if (speed > 0)
    {

        if (model == 1)
        {
            HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, GPIO_PIN_SET);
            HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_RESET);
        }
        else if (model == 2)
        {
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_RESET);
        }
        else if (model == 3)
        {
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_SET);
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_RESET);
        }
        else if (model == 4)
        {
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_SET);
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_RESET);
        }

        duty = speed;
    }

    else if (speed < 0)
    {
        if (model == 1)
        {
            HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_SET);
        }
        else if (model == 2)
        {
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_SET);
        }
        else if (model == 3)
        {
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_SET);
        }
        else if (model == 4)
        {
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_SET);
        }

        duty = -speed;
    }

    else
    {
        if (model == 1)
        {
            HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, GPIO_PIN_SET);
            HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_SET);
        }
        else if (model == 2)
        {
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_SET);
        }
        else if (model == 3)
        {
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_SET);
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_SET);
        }
        else if (model == 4)
        {
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_SET);
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_SET);
        }

        duty = 0;
    }

    if (model == 1)
    {
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, duty); // PA5
    }

    else if (model == 2)
    {
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, duty); // PA1
    }

    else if (model == 3)
    {
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, duty); // PB10
    }

    else if (model == 4)
    {
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, duty); // PB11
    }
}

/**
 * @brief  麦轮逆运动学解算，输入车体速度输出四个轮子目标角速度
 * @note   轮子编号对应：1=左前，2=右前，3=左后，4=右后
 * @note   坐标系：X向前，Y向左，逆时针旋转为正
 */
void Car_Move(float vx, float vy, float omega)
{
    // 半轴距 + 半轮距 = 旋转力臂
    float half_sum = (WHEEL_Y + WHEEL_X) / 2.0f;

    // 旋转角速度单位转换：°/s → rad/s（保证线速度单位统一为cm/s）
    float omega_rad = omega * PI / 180.0f;

    // 1. 计算四个轮子的轮周线速度（cm/s）—— X型麦轮标准逆解公式
    float v_lf = vx - vy - omega_rad * half_sum; // 左前轮
    float v_rf = vx + vy + omega_rad * half_sum; // 右前轮
    float v_lr = vx + vy - omega_rad * half_sum; // 左后轮
    float v_rr = vx - vy + omega_rad * half_sum; // 右后轮

    // 2. 线速度 → 角速度（°/s），和PID目标单位完全匹配
    // 公式：ω(rad/s) = v / r  →  ω(°/s) = v/r * 180/π
    float w_lf = v_lf / WHEEL_R * 180.0f / PI;
    float w_rf = v_rf / WHEEL_R * 180.0f / PI;
    float w_lr = v_lr / WHEEL_R * 180.0f / PI;
    float w_rr = v_rr / WHEEL_R * 180.0f / PI;

    // 3. 直接设置PID目标速度（编码器反馈单位也是°/s，闭环匹配）
    Set_Target_Speed(1, w_lf);
    Set_Target_Speed(2, w_rf);
    Set_Target_Speed(3, w_lr);
    Set_Target_Speed(4, w_rr);
}
