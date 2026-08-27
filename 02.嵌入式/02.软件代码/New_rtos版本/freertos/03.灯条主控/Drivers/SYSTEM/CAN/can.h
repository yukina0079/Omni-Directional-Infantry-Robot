#ifndef  __CAN_H__
#define  __CAN_H__
#include "sys.h"
#include "stdio.h"
void can_init(void);
void can_send_data(uint32_t id,uint8_t *buf,uint8_t len);
uint8_t can_receive_data(uint8_t *buf);
#endif
