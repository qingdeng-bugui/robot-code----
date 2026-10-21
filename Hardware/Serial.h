#ifndef __SERIAL_H
#define __SERIAL_H

#include "main.h"
#include <stdint.h>

/*
 * USART1 只用于视觉链路：主控发出 START，其余都是接收。
 * 这里只保留发送字节/字符串两个接口，不再提供 printf 调试输出，
 * 避免调试文本混进视觉协议流。
 */
void Serial_SendByte(uint8_t Byte);
void Serial_SendString(const char *String);

#endif
