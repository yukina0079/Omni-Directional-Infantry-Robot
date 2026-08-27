#ifndef  __KEY_H__
#define __KEY_H__
#include "sys.h"

// 按键掩码宏定义（保持原8位掩码，若需16位可自行替换）
#define KEY1_MASK    0xFE  // 1111 1110
#define KEY2_MASK    0xFD  // 1111 1101
#define KEY3_MASK    0xFB  // 1111 1011
#define KEY4_MASK    0xF7  // 1111 0111
#define KEY5_MASK    0xEF  // 1110 1111
#define KEY6_MASK    0xDF  // 1101 1111
#define KEY7_MASK    0xBF  // 1011 1111
#define KEY8_MASK    0x7F  // 0111 1111
#define KEY9_MASK    0xF0  // 1111 0000
#define KEY10_MASK   0x0F  // 0000 1111


void key_init(void);
void key_scan(void);


#endif
