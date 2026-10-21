#ifndef __SERVO_H
#define __SERVO_H

void Servo_Init(void);
/*
    舵机2操纵云台旋转，168°为正中央即云台朝向正前方，168°-80°为向右偏（80度为云台朝向正右边），168°-258°为向左偏（80°为在出发区的位置，完成循迹后回到168°位置）
    范围：80°-258°，最小角度为80°，最大角度为258°
    舵机1操纵钢针是否刺出，最小角度为87°（钢针完全收缩在管子中），最大角度为135°（钢针伸出刺破气球）
    舵机1的速度大于舵机2,如果要同时使用这两个需要让舵机2的代码在前面且增加500ms的延时
        例如：  Servo_SetAngle(2,168);
                HAL_Delay(500);
                Servo_SetAngle(1,135);

*/
void Servo_SetAngle(int model, float angle);

#endif
