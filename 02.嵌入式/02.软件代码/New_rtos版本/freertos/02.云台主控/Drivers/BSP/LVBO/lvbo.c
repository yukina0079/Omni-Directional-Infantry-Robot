#include "lvbo.h"
#include "stdlib.h"

#define FILTER_N 5  // 采样窗口大小
float buf[FILTER_N];
int idx = 0;

float sliding_avg_filter(float new_val) {
    buf[idx++] = new_val;
    if (idx >= FILTER_N) idx = 0;  // 循环覆盖
    
    float sum = 0.0f;
    for (int i = 0; i < FILTER_N; i++) sum += buf[i];
    return sum / FILTER_N;
}
/**
 * @brief 初始化滤波器
 * @param filter 滤波器结构体指针
 */
void filter_init(moving_average_filter_t* filter) {
    for(int i = 0; i < SAMPLE_SIZE; i++) {
        filter->buffer[i] = 0;
    }
    filter->index = 0;
    filter->sum = 0;
    filter->is_buffer_full = 0;
}

/**
 * @brief 移动平均滤波处理
 * @param filter 滤波器结构体指针
 * @param new_sample 新采样值
 * @return 滤波后的值
 */
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

/**
 * @brief 摇杆轴初始化
 * @param axis 摇杆轴结构体指针
 */
void joystick_axis_init(joystick_axis_t* axis) {
    filter_init(&axis->filter);
    axis->last_stable_value = 2048;  // 中间值
    axis->raw_value = 0;
    axis->filtered_value = 0;
}

/**
 * @brief 带死区的摇杆滤波
 * @param axis 摇杆轴结构体指针
 * @param new_sample 新采样值
 * @return 滤波后的稳定值
 */
uint16_t joystick_filter_with_deadzone(joystick_axis_t* axis, uint16_t new_sample) {
    axis->raw_value = new_sample;
    axis->filtered_value = moving_average_filter(&axis->filter, new_sample);
    
    if(abs(axis->filtered_value - axis->last_stable_value) < DEAD_ZONE) {
        return axis->last_stable_value;
    }
    
    axis->last_stable_value = axis->filtered_value;
    return axis->filtered_value;
}

/* ==============================
 * 使用方法
 * ============================== */

/*
// 1. 定义摇杆轴变量
joystick_axis_t joy_x, joy_y;

// 2. 在初始化函数中初始化
void System_Init(void) {
    joystick_axis_init(&joy_x);
    joystick_axis_init(&joy_y);
}

// 3. 在主循环中读取和滤波
while(1) {
    // 读取X轴原始ADC值
    uint16_t raw_adc_x = read_adc_channel(ADC_CHANNEL_0);
    
    // 滤波处理
    uint16_t filtered_x = joystick_filter_with_deadzone(&joy_x, raw_adc_x);
    
    // 读取Y轴原始ADC值  
    uint16_t raw_adc_y = read_adc_channel(ADC_CHANNEL_1);
    
    // 滤波处理
    uint16_t filtered_y = joystick_filter_with_deadzone(&joy_y, raw_adc_y);
    
    // 使用滤波后的值
    float voltage_x = (float)filtered_x / 4095.0f * 3.3f;
    float voltage_y = (float)filtered_y / 4095.0f * 3.3f;
    
    // 控制逻辑...
    HAL_Delay(50);
}

// 4. 调试输出（可选）
void debug_output(void) {
    printf("X: raw=%d, filtered=%d\n", joy_x.raw_value, joy_x.filtered_value);
    printf("Y: raw=%d, filtered=%d\n", joy_y.raw_value, joy_y.filtered_value);
}

// 根据应用调整延时
HAL_Delay(20);   // 50Hz - 快速响应
HAL_Delay(50);   // 20Hz - 平衡
HAL_Delay(100);  // 10Hz - 平滑

采样点数选择：
轻度滤波：4-8个样本，响应快
中度滤波：8-16个样本，平衡响应和平滑
重度滤波：16-32个样本，非常平滑但响应慢

死区大小：
高精度应用：5-10（消除微小噪声）
游戏控制：15-30（防止操作抖动）
机械控制：30-50（避免频繁调整）
*/

