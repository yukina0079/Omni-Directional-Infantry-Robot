#include "lvbo.h"
#include "stdlib.h"
#include "math.h"

void filter_init(moving_average_filter_t* filter) {
    for(int i = 0; i < SAMPLE_SIZE; i++) {
        filter->buffer[i] = 0;
    }
    filter->index = 0;
    filter->sum = 0;
    filter->is_buffer_full = 0;
}

uint16_t moving_average_filter(moving_average_filter_t* filter, uint16_t new_sample) {
    if(filter->is_buffer_full) {
        filter->sum -= filter->buffer[filter->index];
    }

    filter->sum += new_sample;
    filter->buffer[filter->index] = new_sample;
    filter->index = (filter->index + 1) % SAMPLE_SIZE;

    if(filter->index == 0 && !filter->is_buffer_full) {
        filter->is_buffer_full = 1;
    }

    uint8_t count = filter->is_buffer_full ? SAMPLE_SIZE : filter->index;
    return (uint16_t)(filter->sum / count);
}

void joystick_axis_init(joystick_axis_t* axis) {
    filter_init(&axis->filter);
    axis->last_stable_value = 2048;
    axis->raw_value = 0;
    axis->filtered_value = 0;
}

uint16_t joystick_filter_with_deadzone(joystick_axis_t* axis, uint16_t new_sample) {
    axis->raw_value = new_sample;
    axis->filtered_value = moving_average_filter(&axis->filter, new_sample);

    if(abs(axis->filtered_value - axis->last_stable_value) < DEAD_ZONE) {
        return axis->last_stable_value;
    }

    axis->last_stable_value = axis->filtered_value;
    return axis->filtered_value;
}

void float_to_two_uint8_signed(float num, uint8_t* high_byte, uint8_t* low_byte)
{

    if(num > 327.67f) num = 327.67f;
    if(num < -327.68f) num = -327.68f;

    int16_t scaled = (int16_t)(round(num * 100.0f));

    *high_byte = (scaled >> 8) & 0xFF;
    *low_byte = scaled & 0xFF;
}

float two_uint8_to_float_signed(uint8_t high_byte, uint8_t low_byte)
{

    int16_t scaled = (int16_t)((high_byte << 8) | low_byte);

    return (float)scaled / 100.0f;
}
