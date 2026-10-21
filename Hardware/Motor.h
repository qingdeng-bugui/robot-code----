#ifndef __MOTOR_H
#define __MOTOR_H

/********************* 麦轮物理参数 *********************/
#define WHEEL_R 3.0f        // 轮子半径，单位cm【请根据实际底盘测量修改】
#define WHEEL_Y 13.5f       // 前后轴距（前后轮中心距离），单位cm【请根据实际底盘测量修改】
#define WHEEL_X 13.9f       // 左右轮距（左右轮中心距离），单位cm【请根据实际底盘测量修改】
#define PI      3.14159265f // 圆周率

/********************* 函数声明 *********************/
void Motor_Init(void);
void Set_Speed(int model, int speed);

/**
 * @brief  麦轮小车全向运动控制
 * @param  vx: X方向速度（前进为正，后退为负），单位cm/s
 * @param  vy: Y方向速度（左移为正，右移为负），单位cm/s
 * @param  omega: 车体旋转角速度（逆时针为正，顺时针为负），单位°/s
 * @retval None
 */
void Car_Move(float vx, float vy, float omega);

#endif