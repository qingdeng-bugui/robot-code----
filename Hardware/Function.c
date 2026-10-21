#include "Function.h"
#include "adc.h"
#include "LineFast.h"
#include "Motor.h"
#include "PID.h"
#include "Serial.h"
#include "Servo.h"
#include "Shoot.h"
#include "ir_nec.h"
#include "track.h"
#include "usart.h"
#include "Photoresistor.h"
#include <math.h>
#include <string.h>

/*
 * 视觉串口一行数据的最大长度（包含结尾的 '\0'）。
 * 当前协议最长的正常帧是“整数,0/1,red,整数”，同时还要容纳负号；
 * 预留余量可以让解析器在收到异常长帧时主动丢弃，而不是写越界。
 */
#define VISUAL_LINE_LENGTH 32U

/*
 * USART1 中断接收使用的环形字节缓存大小。
 * 中断只负责把字节写入缓存，主循环稍后再组装完整行，
 * 因此可以吸收树莓派短时间内连续发送的数据突发。
 */
#define VISUAL_RX_BUFFER_SIZE 512U


/* 视觉解析结果的内部命令类型，避免在调用处直接比较字符串。
 * 具体取值定义在 Function.h，Shoot.c 也要用。 */

/*
 * 下面四个变量由 USART1 接收中断和主循环共同访问，因此声明为 volatile。
 * head 指向下一个写入位置，tail 指向下一个待读取字节；两者相等表示为空。
 */
static volatile uint8_t visual_rx_buffer[VISUAL_RX_BUFFER_SIZE];
static volatile uint16_t visual_rx_head = 0U;
static volatile uint16_t visual_rx_tail = 0U;
static volatile uint8_t visual_rx_byte = 0U;
static volatile uint8_t visual_rx_overflow = 0U;

/* 主循环从环形缓存取字节后，在这里逐行拼接视觉协议帧。 */
static char visual_line_buffer[VISUAL_LINE_LENGTH];
static uint8_t visual_line_length = 0U;
static uint8_t visual_discard_line = 0U;

/*
 * USART1 使用单字节中断接收，主循环从环形缓冲区提取最新完整视觉帧。
 */
static uint16_t Visual_RxNext(uint16_t index)
{
    /* 环形缓冲区到达末尾后回到 0，避免访问数组范围之外。 */
    index++;
    if (index >= VISUAL_RX_BUFFER_SIZE)
    {
        index = 0U;
    }
    return index;
}

void Visual_Rx_Init(void)
{
    /*
     * 初始化前暂时关中断，确保主循环清零 head/tail 时不会与接收中断竞争。
     * 清零完成后立即挂起单字节中断接收，供后续 START 帧使用。
     */
    __disable_irq();
    visual_rx_head = 0U;
    visual_rx_tail = 0U;
    visual_rx_overflow = 0U;
    visual_line_length = 0U;
    visual_discard_line = 0U;
    __enable_irq();

    (void)HAL_UART_Receive_IT(&huart1, (uint8_t *)&visual_rx_byte, 1U);
}

void Visual_Rx_Clear(void)
{
    /* 丢弃当前缓存中的旧数据，只从当前 head 位置开始接收新的视觉命令。 */
    __disable_irq();
    visual_rx_tail = visual_rx_head;
    visual_rx_overflow = 0U;
    __enable_irq();

    visual_line_length = 0U;
    visual_discard_line = 0U;
}

void Visual_RxCpltCallback(void)
{
    /*
     * 每收到一个字节就进入一次回调：先写入当前 head，再推进 head。
     * 若推进后追上 tail，说明缓存已满，丢掉最旧字节以保留最新数据。
     */
    uint16_t next = Visual_RxNext(visual_rx_head);

    visual_rx_buffer[visual_rx_head] = visual_rx_byte;
    if (next == visual_rx_tail)
    {
        /* 缓冲区满时丢弃最旧字节，保证控制循环拿到最新视觉数据。 */
        visual_rx_tail = Visual_RxNext(visual_rx_tail);
        visual_rx_overflow = 1U;
    }
    visual_rx_head = next;

    (void)HAL_UART_Receive_IT(&huart1, (uint8_t *)&visual_rx_byte, 1U);
}

void Visual_RxErrorCallback(void)
{
    /*
     * 串口溢出等错误可能使 HAL 停止当前接收事务；记录错误并重新挂接
     * 单字节接收，让后续视觉帧仍能恢复进入环形缓存。
     */
    visual_rx_overflow = 1U;

    /*
     * ORE会让HAL结束当前接收事务。错误标志由HAL_UART_IRQHandler清除，
     * 这里负责立即重新挂上单字节接收，避免后续帧无人接收。
     */
    (void)HAL_UART_Receive_IT(&huart1, (uint8_t *)&visual_rx_byte, 1U);
}

static uint8_t Visual_RxPop(uint8_t *byte)
{
    /* 从 tail 取出一个字节；返回 0 表示空，返回 1 表示成功取出。 */
    uint16_t tail = visual_rx_tail;

    if (tail == visual_rx_head)
    {
        return 0U;
    }

    *byte = visual_rx_buffer[tail];
    visual_rx_tail = Visual_RxNext(tail);
    return 1U;
}

/*
 * 取出当前已经收到的所有完整行，只保留最后一行。
 * 视觉帧可能比控制周期更快，不能把旧dx逐条排队执行。
 */
uint8_t Visual_ReadLatestLine(char *line)
{
    /*
     * 将缓存中的字节按换行符组帧。一次调用可能读到多行，
     * 通过反复覆盖 line 只保留最后一行，避免控制循环执行过期 dx。
     */
    uint8_t byte;
    uint8_t has_line = 0U;

    if (visual_rx_overflow != 0U)
    {
        /* 发生覆盖后放弃当前半行，直到下一个换行符再恢复正常组帧。 */
        visual_rx_overflow = 0U;
        visual_line_length = 0U;
        visual_discard_line = 1U;
    }

    while (Visual_RxPop(&byte) != 0U)
    {
        if (byte == '\r')
        {
            continue;
        }

        if (byte == '\n')
        {
            if (visual_discard_line == 0U &&
                visual_line_length > 0U)
            {
                visual_line_buffer[visual_line_length] = '\0';
                strcpy(line, visual_line_buffer);
                has_line = 1U;
            }

            visual_line_length = 0U;
            visual_discard_line = 0U;
            continue;
        }

        if (visual_discard_line != 0U)
        {
            continue;
        }

        if (visual_line_length < VISUAL_LINE_LENGTH - 1U)
        {
            visual_line_buffer[visual_line_length++] = (char)byte;
        }
        else
        {
            /* 超过协议允许长度的帧直接丢弃，防止字符串缓冲区越界。 */
            visual_line_length = 0U;
            visual_discard_line = 1U;
        }
    }

    return has_line;
}

static uint8_t Visual_ParseSignedInt(const char *line, uint8_t *index, int *value)
{
    int sign = 1;
    int number = 0;
    uint8_t parsed = 0U;

    if (line[*index] == '-')
    {
        sign = -1;
        (*index)++;
    }
    else if (line[*index] == '+')
    {
        (*index)++;
    }

    while (line[*index] >= '0' && line[*index] <= '9')
    {
        parsed = 1U;
        number = number * 10 + line[*index] - '0';
        (*index)++;
    }

    if (parsed == 0U)
    {
        return 0U;
    }

    *value = sign * number;
    return 1U;
}

uint8_t Visual_ParseLine(const char *line, int *dx, int *can_hit,
                                uint8_t *red_present, int *red_dx)
{
    /*
     * 解析树莓派发送的单行协议：
     *   none       表示暂时没有目标；none,red,red_dx 表示只有红球；
     *   ok         表示视觉端已准备好或流程确认；
     *   dx,0/1     表示横向误差以及当前是否允许戳击；
     *   dx,0/1,red,red_dx 表示同时检测到红球。
     * 返回值只描述命令类型，具体数据通过输出参数提供。
     */
    uint8_t index = 0U;
    int target_dx = 0;
    int target_can_hit = 0;
    int detected_red_dx = 0;

    if (strcmp(line, "none") == 0)
    {
        *red_present = 0U;
        return VISUAL_COMMAND_NONE;
    }

    if (strncmp(line, "none,red,", 9U) == 0)
    {
        index = 9U;
        if (Visual_ParseSignedInt(line, &index, &detected_red_dx) == 0U ||
            line[index] != '\0')
        {
            return VISUAL_COMMAND_INVALID;
        }

        *red_present = 1U;
        *red_dx = detected_red_dx;
        return VISUAL_COMMAND_NONE;
    }

    if (strcmp(line, "ok") == 0)
    {
        *red_present = 0U;
        return VISUAL_COMMAND_OK;
    }

    if (Visual_ParseSignedInt(line, &index, &target_dx) == 0U ||
        line[index++] != ',')
    {
        return VISUAL_COMMAND_INVALID;
    }

    if (line[index] == '0')
    {
        target_can_hit = 0;
    }
    else if (line[index] == '1')
    {
        target_can_hit = 1;
    }
    else
    {
        return VISUAL_COMMAND_INVALID;
    }

    index++;
    if (line[index] == '\0')
    {
        *dx = target_dx;
        *can_hit = target_can_hit;
        *red_present = 0U;
        return VISUAL_COMMAND_TARGET;
    }

    if (line[index++] != ',' ||
        line[index++] != 'r' ||
        line[index++] != 'e' ||
        line[index++] != 'd' ||
        line[index++] != ',')
    {
        return VISUAL_COMMAND_INVALID;
    }

    if (Visual_ParseSignedInt(line, &index, &detected_red_dx) == 0U ||
        line[index] != '\0')
    {
        return VISUAL_COMMAND_INVALID;
    }

    *dx = target_dx;
    *can_hit = target_can_hit;
    *red_present = 1U;
    *red_dx = detected_red_dx;
    return VISUAL_COMMAND_TARGET;
}

void All_functions(void)
{
    /*
     * 比赛主流程入口：先完成赛道循迹，再进入场内。
     * 两个函数都是阻塞式任务，只有前一个流程返回后才会开始后一个流程。
     */
    /* 陀螺仪辅助的高速循迹 */
    LineFast_Run();

    /* 场内第一步：云台 120°，快速左移 25cm、前移 5cm */
    Shoot_Run();
}

/* *
 * @brief 处理光敏电阻逻辑
 * @note 该函数在主循环中被调用，用于根据光敏电阻的状态决定是否执行其他功能。
 */
void Photoresistor(void)
{
    // 处理光敏电阻的逻辑
    if (car_start_flag==1)  // 处理启动的情况
    {
        All_functions();
    }
    else    // 处理未启动的情况
    {
        if((HAL_ADC_GetState(&hadc1) & HAL_ADC_STATE_REG_BUSY) == 0U) //如果ADC不在转换中，则启动下一次转换
        {
            HAL_ADC_Start_IT(&hadc1);
        }
        HAL_Delay(10);
    }
}

