#ifndef  __LVBO_H__
#define  __LVBO_H__
#include "sys.h"

#define SAMPLE_SIZE 16
#define DEAD_ZONE 15

typedef struct {
    uint16_t buffer[SAMPLE_SIZE];
    uint8_t index;
    uint32_t sum;
    uint8_t is_buffer_full;
} moving_average_filter_t;

typedef struct {
    moving_average_filter_t filter;
    uint16_t last_stable_value;
    uint16_t raw_value;
    uint16_t filtered_value;
} joystick_axis_t;

void filter_init(moving_average_filter_t* filter);

uint16_t moving_average_filter(moving_average_filter_t* filter, uint16_t new_sample);

void joystick_axis_init(joystick_axis_t* axis);

uint16_t joystick_filter_with_deadzone(joystick_axis_t* axis, uint16_t new_sample);
float sliding_avg_filter(float new_val);
#endif
