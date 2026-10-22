#ifndef __SEARCH_H
#define __SEARCH_H

#include <stdint.h>

/*
 * 场内搜寻模式
 *
 * 从参考工程 robot-code 的 Hit_Balloons() 原样搬来：
 * 云台左右扫描 + 场地慢速巡场 + 发现可戳目标就靠近并反复扎，
 * 内部带红球规避和出界保护。函数不返回，一直跑到断电。
 */
void Search_Run(void);

#endif
