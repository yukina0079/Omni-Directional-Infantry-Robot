#ifndef  __DATA_H__
#define __DATA_H__
#include "sys.h"

/*
 * Integrator gain for the lx / ly sticks, in radians per (stick count * tick).
 *
 * data_get_task() runs at vTaskDelay(2) with a 1 kHz tick, so the integration
 * rate is 500 Hz and full stick deflection (128 counts) commands
 *
 *     128 * Joystick_Points_kp * 500  rad/s
 *
 * At the original 0.0002 that was 12.8 rad/s, or 733 deg/s. The yaw axis cannot
 * deliver that: its position loop is capped at 2.5 V, and the bench measurement
 * (0.43 V sustained 1.9 rad/s) puts the ceiling near 11 rad/s. Commanding more
 * than the axis can follow is not merely wasted -- the position error is wrapped
 * to [-PI, PI], so once the setpoint outruns the shaft by more than PI the wrap
 * flips the error's sign and the gimbal slams into reverse. Holding full stick
 * for ~2 s was enough to reach that.
 *
 * 0.0001 puts full stick at 6.4 rad/s (367 deg/s), which needs about 1.45 V
 * steady -- inside the thermal cap with room left for acceleration and for the
 * extra friction a fitted barrel will add. Following error at that rate is
 * ~0.5 rad, a factor of six short of the PI wrap point.
 */
#define Joystick_Points_kp 0.0001

/*
 * Stick dead zone, in counts of the +-128 mapped range.
 *
 * A dead zone is mandatory here because lx and ly feed an INTEGRATOR: any
 * non-zero reading at rest is not a small error, it is a constant drift rate
 * that never stops accumulating. A stick sitting one count off centre would
 * swing the gimbal at 0.05 rad/s forever.
 *
 * It was 20 counts, which is 15.6% of travel on each side -- nearly a third of
 * the stick did nothing. That alone is bad, but the real defect was the shape:
 * the old code zeroed the band and passed the rest through unchanged, so the
 * first live count commanded 2.1 rad/s. The stick had no slow region at all.
 *
 * 8 counts is 128 raw ADC counts of the 4096 range (3.1%), which still absorbs
 * pot noise and mechanical centring error, and map0_4096To128_128WithMaskOpt()
 * now rescales the live range so the response leaves zero continuously.
 */
#define JOY_DEADZONE 8

extern uint8_t key_num;
extern char key_num_buffer[20];
extern uint8_t key_prev_state[10];
extern uint8_t key_arr[10];
extern uint16_t adc_value[4];
extern int16_t Joystick_value[4];
extern int16_t encoder_value[2];
extern float Joystick_Points_value[4];
extern uint8_t Send_Out[32];

int16_t map0_4096To128_128WithMaskOpt(uint16_t raw_value);
void floatToTwoSint8(float num, uint8_t* high_byte, uint8_t* low_byte);
void floatToTwoSint8Milli(float num, uint8_t* high_byte, uint8_t* low_byte);
float twoSint8ToFloat(uint8_t high_byte, uint8_t low_byte);

uint8_t KeyArr_To_KeyNum(uint8_t key_arr[10]);
void KeyNum_To_BinStr(uint8_t key_num, char *bin_str);
void oled_show_keynum_bin(uint8_t x, uint8_t y, uint8_t key_num, uint8_t size);
float limit_float(float val, float min, float max);
void data_change(void);
void data_print(void);
void oled_show_data(void);

#endif
