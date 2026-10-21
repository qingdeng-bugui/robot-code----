#include "main.h"
#include "usart.h" //Tx:PC4:Rx:PC5

/*
 * USART1 专属视觉链路的发送封装。
 * 视觉协议是纯文本帧（START 无换行，dx/can_hit/none/ok 带换行），
 * 因此这里只保留最小必要的发送接口，不再重定向 printf。
 */

/**
 * 函    数：串口发送单个字节
 * 参    数：Byte 要发送的一字节数据
 * 返 回 值：无
 */
void Serial_SendByte(uint8_t Byte)
{
    (void)HAL_UART_Transmit(&huart1, &Byte, 1, 20);
}

/**
 * 函    数：串口发送一个字符串
 * 参    数：String 要发送字符串的首地址
 * 返 回 值：无
 */
void Serial_SendString(const char *String)
{
    uint8_t i;

    for (i = 0; String[i] != '\0'; i++) // 遍历字符数组（字符串），遇到字符串结束标志位后停止
    {
        Serial_SendByte(String[i]); // 依次调用Serial_SendByte发送每个字节数据
    }
}
