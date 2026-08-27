#ifndef  __LVBO_H__
#define  __LVBO_H__
#include "sys.h"
/* ==============================
 * 均值滤波结构体及函数声明
 * ============================== 
 */

#define SAMPLE_SIZE 16      // 滤波采样点数
#define DEAD_ZONE 15       // 死区阈值

// 移动平均滤波器结构体
typedef struct {
    uint16_t buffer[SAMPLE_SIZE];
    uint8_t index;
    uint32_t sum;
    uint8_t is_buffer_full;
} moving_average_filter_t;

// 摇杆轴结构体
typedef struct {
    moving_average_filter_t filter;
    uint16_t last_stable_value;
    uint16_t raw_value;
    uint16_t filtered_value;
} joystick_axis_t;

/* ==============================
 * 函数声明
 * ============================== 
*/

// 滤波器初始化
void filter_init(moving_average_filter_t* filter);

// 移动平均滤波
uint16_t moving_average_filter(moving_average_filter_t* filter, uint16_t new_sample);

// 摇杆轴初始化
void joystick_axis_init(joystick_axis_t* axis);

// 带死区的摇杆滤波
uint16_t joystick_filter_with_deadzone(joystick_axis_t* axis, uint16_t new_sample);
void float_to_two_uint8_signed(float num, uint8_t* high_byte, uint8_t* low_byte);
float two_uint8_to_float_signed(uint8_t high_byte, uint8_t low_byte);
#endif
