#ifndef __FRAME13_H__
#define __FRAME13_H__

/*
 * 13-byte control frame -- the ONLY frame format in this project.
 *
 * Every link carries 13 bytes with a 0x55 header in byte 0 and a 0xFF tail in
 * byte 12. There is no CRC and no sequence number by design: the radio already
 * runs a 16-bit CRC on the air, and the serial links are short and point to
 * point. Integrity on the wire is therefore "header + tail", nothing else.
 *
 * The same 13 bytes are used three ways, differing only in what occupies
 * bytes 8..11:
 *
 *   1) RADIO   remote -> chassis / gimbal / shooter / lightbar
 *      [8..9]   right stick X (Vx), +-127
 *      [10..11] right stick Y (Vy), +-127
 *
 *   2) FOC     chassis -> yaw board,  gimbal -> pitch board
 *      A FOC axis has no use for chassis translation, so those four bytes
 *      carry the sender's own state instead:
 *      [8..9]   sender attitude (chassis yaw / gimbal pitch), rad*100
 *      [10..11] axis command, rad*100 (absolute for pitch, incremental for yaw)
 *
 *   3) REPLY   yaw / pitch -> chassis / gimbal
 *      [1..2]   measured shaft angle, rad*100
 *
 * Byte 12 is always the tail, so a receiver can validate a candidate frame
 * without knowing which of the three variants it is looking at.
 */

#include <stdint.h>

#define FRAME13_LEN      13u
#define FRAME13_HEAD     0x55u
#define FRAME13_TAIL     0xFFu
#define F13_TAIL_IDX     12u

/* Common to all three variants */
#define F13_KEY          1u   /* key bitmask, active low           */
#define F13_ENL          2u   /* left knob, 0..127                 */
#define F13_ENR          3u   /* right knob, 0..127                */
#define F13_LX_H         4u   /* left stick X, rad*100             */
#define F13_LX_L         5u
#define F13_LY_H         6u   /* left stick Y, rad*1000            */
#define F13_LY_L         7u

/* RADIO variant */
#define F13_RX_H         8u   /* right stick X, +-127              */
#define F13_RX_L         9u
#define F13_RY_H         10u  /* right stick Y, +-127              */
#define F13_RY_L         11u

/* FOC variant */
#define F13_ATT_H        8u   /* sender attitude, rad*100          */
#define F13_ATT_L        9u
#define F13_CMD_H        10u  /* axis command, rad*100             */
#define F13_CMD_L        11u

/* REPLY variant */
#define F13_ANG_H        1u   /* shaft angle, rad*100              */
#define F13_ANG_L        2u

#endif /* __FRAME13_H__ */
