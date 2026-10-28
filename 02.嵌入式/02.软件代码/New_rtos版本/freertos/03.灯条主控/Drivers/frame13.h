#ifndef __FRAME13_H__
#define __FRAME13_H__

#include <stdint.h>

#define FRAME13_LEN      13u
#define FRAME13_HEAD     0x55u
#define FRAME13_TAIL     0xFFu
#define F13_TAIL_IDX     12u

#define F13_KEY          1u
#define F13_ENL          2u
#define F13_ENR          3u
#define F13_LX_H         4u
#define F13_LX_L         5u
#define F13_LY_H         6u
#define F13_LY_L         7u

#define F13_RX_H         8u
#define F13_RX_L         9u
#define F13_RY_H         10u
#define F13_RY_L         11u

#define F13_ATT_H        8u
#define F13_ATT_L        9u
#define F13_CMD_H        10u
#define F13_CMD_L        11u

#define F13_ANG_H        1u
#define F13_ANG_L        2u

#endif
