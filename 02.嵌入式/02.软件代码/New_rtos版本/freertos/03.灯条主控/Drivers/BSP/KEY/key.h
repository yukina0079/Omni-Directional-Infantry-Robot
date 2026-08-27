#ifndef  __KEY_H__
#define __KEY_H__
#include "sys.h"
#include "stdbool.h"
void key_init(void);
void key_scan(void);
bool key_edge_detect(void);
uint8_t key_edge_mask(void);

#endif
