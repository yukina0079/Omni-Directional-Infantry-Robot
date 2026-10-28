#ifndef  __KEY_H__
#define __KEY_H__
#include "sys.h"

#define KEY1_MASK    0xFE
#define KEY2_MASK    0xFD
#define KEY3_MASK    0xFB
#define KEY4_MASK    0xF7
#define KEY5_MASK    0xEF
#define KEY6_MASK    0xDF
#define KEY7_MASK    0xBF
#define KEY8_MASK    0x7F
#define KEY9_MASK    0xF0
#define KEY10_MASK   0x0F

void key_init(void);
void key_scan(void);

#endif
