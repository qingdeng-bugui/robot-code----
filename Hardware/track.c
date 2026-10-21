#include "track.h"
#include "usart.h"

volatile uint16_t track_values[8]; // 用于存储循迹模块的8个传感器数值

static uint8_t receive_data[80];           // 用于存储接收到的数据帧
static volatile uint8_t track_enable;      // 用于标记循迹模块是否启用
static volatile uint8_t receive_byte;      // 用于存储接收到的字节数据
static volatile uint16_t receive_location; // 用于记录接收到的数据帧的索引位置
static volatile uint8_t receive_flag;      // 用于标记是否正在接收数据帧

/*
  功能：控制循迹模块的使能状态，启用后会初始化串口并开始接收数据，禁用后会停止接收数据并释放串口资源。
  参数：enable：1表示启用循迹模块，0表示禁用循迹模块。
  返回值：无
*/
static void Track_SendCommand(uint8_t enable)
{
    uint8_t command[] = "$0,0,0#";

    if (enable == 1)
    {
        command[3] = '1';
    }

    HAL_UART_Transmit(&huart2, command, sizeof(command) - 1, 20);
}

/*
  功能：解析接收到的循迹模块数据帧，将解析出的8个传感器数值存储到track_values数组中。
  数据帧格式：$A,x1:xxxx,x2:xxxx,x3:xxxx,x4:xxxx,x5:xxxx,x6:xxxx,x7:xxxx,x8:xxxx#
  返回值：无
*/
static void Track_ParseFrame(void)
{
    uint16_t values[8];
    uint16_t index = 3;
    uint8_t i;
    uint8_t digit;

    //  验证数据帧的起始符、结束符和长度是否符合预期
    if (receive_location < 27 ||
        receive_data[0] != '$' ||
        receive_data[1] != 'A' ||
        receive_data[2] != ',' ||
        receive_data[receive_location - 1] != '#')
    {
        return;
    }

    //  解析数据帧中的8个传感器数值
    for (i = 0; i < 8; i++)
    {
        if (receive_data[index++] != 'x' ||
            receive_data[index++] != (uint8_t)('1' + i) ||
            receive_data[index++] != ':')
        {
            return;
        }

        /*数据处理
        比如当前解析 `x1:4096`，进入循环前 values[0] = 0，`index` 指向字符 `'4'`：

            1. 第 1 次循环：字符 `'4'` → 数字 4 → `values[0] = 0*10 + 4 = 4`
            2. 第 2 次循环：字符 `'0'` → 数字 0 → `values[0] = 4*10 + 0 = 40`
            3. 第 3 次循环：字符 `'9'` → 数字 9 → `values[0] = 40*10 + 9 = 409`
            4. 第 4 次循环：字符 `'6'` → 数字 6 → `values[0] = 409*10 + 6 = 4096`
        */
        values[i] = 0;
        digit = 0;
        while (digit < 4 &&
               receive_data[index] >= '0' &&
               receive_data[index] <= '9')
        {
            //  验证每个数值的字符是否为数字，如果不是则返回
            if (receive_data[index] < '0' || receive_data[index] > '9')
            {
                return;
            }

            values[i] = values[i] * 10 + receive_data[index++] - '0';
            digit++;
        }

        if (digit == 0)
        {
            return;
        }

        if (i < 7 && receive_data[index++] != ',')
        {
            return;
        }
    }

    if (index != receive_location - 1)
    {
        return;
    }

    for (i = 0; i < 8; i++)
    {
        track_values[i] = values[i];
    }
}

/*
  功能：处理接收到的字节数据，根据数据帧的起始符和结束符来判断是否开始接收和解析数据帧。
  参数：byte：接收到的字节数据。
  返回值：无
*/
static void Track_ProcessByte(uint8_t byte)
{
    if (byte == '$')
    {
        receive_location = 0;
        receive_flag = 1;
    }

    if (receive_flag == 0)
    {
        return;
    }

    if (receive_location >= sizeof(receive_data))
    {
        receive_location = 0;
        receive_flag = 0;
        return;
    }

    receive_data[receive_location++] = byte;

    if (byte == '#')
    {
        Track_ParseFrame();
        receive_location = 0;
        receive_flag = 0;
    }
}

/*
  功能：控制循迹模块的使能状态，启用后会初始化串口并开始接收数据，禁用后会停止接收数据并释放串口资源。
  参数：enable：1表示启用循迹模块，0表示禁用循迹模块。
  返回值：无
*/
void Track_Control(uint8_t enable)
{
    if (enable == 1)
    {
        if (track_enable == 0)
        {
            MX_USART2_UART_Init();
            receive_location = 0;
            receive_flag = 0;
            track_enable = 1;
            HAL_UART_Receive_IT(&huart2, &receive_byte, 1);
            Track_SendCommand(1);
        }
        return;
    }

    if (track_enable == 1)
    {
        Track_SendCommand(0);
        track_enable = 0;
        HAL_UART_AbortReceive_IT(&huart2);
        HAL_UART_DeInit(&huart2);
        receive_location = 0;
        receive_flag = 0;
    }
}

void Track_RxCpltCallback(void)
{
    if (track_enable == 1)
    {
        Track_ProcessByte(receive_byte);                // 处理刚收到的这1个字节
        HAL_UART_Receive_IT(&huart2, &receive_byte, 1); // 提交下一个字节的接收请求
    }
}

void Track_ErrorCallback(void)
{
    if (track_enable == 1)
    {
        HAL_UART_Receive_IT(&huart2, &receive_byte, 1);
    }
}
