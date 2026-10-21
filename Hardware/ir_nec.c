#include "ir_nec.h"
#include "Function.h"
#include "Motor.h"
#include "Serial.h"
#include "Servo.h"
#include "gpio.h"
#include "tim.h"
#include "track.h"

uint8_t frame[4];
volatile uint16_t time_us = 0;
volatile uint8_t cnt = 0;
volatile uint8_t IR_temp = 0;
volatile uint8_t IR_buf[4] = {0};
/* 中断中接收完成一帧后置 1，由主循环负责解析 */
static volatile uint8_t ir_frame_ready = 0;
static uint8_t function_flag = 0;
/**
 * @brief 初始化 NEC 红外接收
 *
 * TIM15_CH2 使用下降沿捕获。由于红外接收头输出的是解调后的数字信号，
 * 相邻下降沿之间的时间正好可以表示 NEC 协议的引导码和数据位宽度。
 */
void IR_NEC_Init(void)
{
    uint8_t i;

    /* 先配置捕获边沿，再启动输入捕获，避免启动瞬间使用旧配置 */
    __HAL_TIM_SET_CAPTUREPOLARITY(&htim15, TIM_CHANNEL_2,
                                  TIM_ICPOLARITY_FALLING);

    /* 清空上一次可能残留的接收状态 */
    time_us = 0;
    cnt = 0;
    IR_temp = 0;
    ir_frame_ready = 0;
    for (i = 0; i < 4; i++)
        IR_buf[i] = 0;

    /* TIM15 预分频为 169，当前配置下计数周期为 1 us */
    __HAL_TIM_SET_COUNTER(&htim15, 0);
    /* 清除启动前可能已经存在的捕获标志，避免误进回调 */
    __HAL_TIM_CLEAR_FLAG(&htim15, TIM_FLAG_CC2);
    if (HAL_TIM_IC_Start_IT(&htim15, TIM_CHANNEL_2) != HAL_OK)
        Error_Handler();
}

/**
 * @brief 关闭 NEC 红外接收
 *
 * TIM15 与 TIM1 共用 TIM1_BRK_TIM15_IRQn 中断入口，因此这里只停止
 * TIM15 的捕获通道，不能直接关闭整个共享 NVIC 中断。
 */
void IR_NEC_DeInit(void)
{
    uint8_t i;

    /* 停止捕获中断，同时关闭 TIM15 的输入捕获通道和计数器 */
    (void)HAL_TIM_IC_Stop_IT(&htim15, TIM_CHANNEL_2);
    /* 清除可能在停止前产生的捕获标志 */
    __HAL_TIM_CLEAR_FLAG(&htim15, TIM_FLAG_CC2);
    __HAL_TIM_SET_COUNTER(&htim15, 0);

    /* 清空软件接收状态，保证下次重新初始化时从完整帧开始 */
    time_us = 0;
    cnt = 0;
    IR_temp = 0;
    ir_frame_ready = 0;
    for (i = 0; i < 4; i++)
        IR_buf[i] = 0;
}

void IR_work(void)
{

    uint8_t i;

    /* 没有收到完整的 32 位数据帧时，不进行解析 */
    if (ir_frame_ready == 0U)
        return;

    /*
     * 在关中断的极短时间内复制完整帧。
     * 这样可以防止主循环读取 IR_buf 的过程中，中断同时改写缓冲区。
     */
    __disable_irq();
    for (i = 0; i < 4; i++)
        frame[i] = IR_buf[i];
    ir_frame_ready = 0;
    __enable_irq();

    /* NEC 协议要求地址和地址反码、命令和命令反码分别互为反码 */
    if ((frame[0] == (uint8_t)~frame[1]) &&
        (frame[2] == (uint8_t)~frame[3]))
    {
        /* 根据红外遥控器的命令码执行对应动作 */
        switch (frame[2])
        {
        case 0x45:
            Servo_SetAngle(1, 87);
            Car_Move(20, 0, 0);
            HAL_Delay(4000);
            Car_Move(0, 0, 0);
            HAL_Delay(10000);
            break;

        case 0x46:
            Serial_SendString("START");
            break;

        case 0x47:
            Tack_line();
            break;

        case 0x44:
            Hit_Balloons();
            break;

        case 0x43:
            Servo_SetAngle(2, 168);
            break;

        case 0x08:
            Car_Move(0.0f, 30.0f, 0.0f);
            break;

        case 0x5A:
            Car_Move(0.0f, -30.0f, 0.0f);
            break;

        case 0x18:
            Car_Move(30.0f, 0.0f, 0.0f);
            break;

        case 0x52:
            Car_Move(-30.0f, 0.0f, 0.0f);
            break;

        case 0x16:
            Car_Move(0.0f, 0.0f, 90.0f);
            break;

        case 0x0d:
            Car_Move(0.0f, 0.0f, -90.0f);
            break;

        case 0x1c:
            Car_Move(0.0f, 0.0f, 0.0f);
            break;

        case 0x40:
            function_flag = 1;
            break;

        case 0x15:
            if (function_flag == 1)
            {
                function_flag = 0;
                All_functions();
            }
            break;

        default:
            break;
        }
    }
}

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
    if (htim != &htim15)
        return;

    /* 读取本次下降沿距离上次下降沿的计数值，单位为 us */
    time_us = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_2);

    /*
     * 采用下降沿捕获时，相邻下降沿之间的时间约为：
     *   引导码：9 ms 低电平 + 4.5 ms 高电平，约 13.5 ms；
     *   数据 0：560 us 低电平 + 560 us 高电平，约 1.12 ms；
     *   数据 1：560 us 低电平 + 1.68 ms 高电平，约 2.24 ms。
     *
     * 设置一定的时间容差，用于抵抗遥控器晶振误差和中断响应抖动。
     */
    if (time_us > 12000U && time_us < 15000U)
    {
        /* 检测到引导码，开始接收一帧新的 32 位数据 */
        cnt = 0;
        IR_temp = 0;
        ir_frame_ready = 0;
    }
    else if (time_us > 900U && time_us < 1300U)
    {
        /* 逻辑 0：右移后补 0 */
        if (cnt < 32U)
        {
            cnt++;
            IR_temp >>= 1;
        }
    }
    else if (time_us > 2000U && time_us < 2500U)
    {
        /* 逻辑 1：右移后在最高位补 1 */
        if (cnt < 32U)
        {
            cnt++;
            IR_temp >>= 1;
            IR_temp |= 0x80U;
        }
    }
    else
    {
        /* 时间不符合 NEC 协议，丢弃当前帧并重新等待引导码 */
        cnt = 0;
        IR_temp = 0;
        ir_frame_ready = 0;
    }

    /* 每收到 8 位数据，就保存一个字节 */
    if (cnt == 8U)
        IR_buf[0] = IR_temp;
    else if (cnt == 16U)
        IR_buf[1] = IR_temp;
    else if (cnt == 24U)
        IR_buf[2] = IR_temp;
    else if (cnt == 32U)
    {
        /* 第 32 位到达，完整帧接收完成，通知主循环解析 */
        IR_buf[3] = IR_temp;
        ir_frame_ready = 1;
    }

    /* 清零计数器，让下一次下降沿重新开始计时 */
    __HAL_TIM_SET_COUNTER(htim, 0);
}
