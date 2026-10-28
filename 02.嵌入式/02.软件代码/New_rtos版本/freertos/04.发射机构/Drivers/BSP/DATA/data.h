#ifndef  __DATA_H__
#define __DATA_H__
#include "sys.h"

#define LIMIT(val, min, max) (((val) < (min)) ? (min) : (((val) > (max)) ? (max) : (val)))

void data_print(void);
void data_change(void);
void light_contral(void);
void motor_contral(void);
void laser_contral(void);

#endif
