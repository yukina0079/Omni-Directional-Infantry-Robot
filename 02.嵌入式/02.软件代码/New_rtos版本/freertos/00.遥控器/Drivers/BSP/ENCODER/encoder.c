#include "encoder.h"
#include "usart.h"

TIM_HandleTypeDef motor1_encoder = {0};
TIM_HandleTypeDef motor2_encoder = {0};

void encoder_init(void)
{
    TIM_Encoder_InitTypeDef config = {0};

    motor1_encoder.Instance = TIM4;
    motor1_encoder.Init.Period = 127;
    motor1_encoder.Init.Prescaler = 0;
    motor1_encoder.Init.CounterMode = TIM_COUNTERMODE_UP;
    motor1_encoder.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;

    motor2_encoder.Instance = TIM3;
    motor2_encoder.Init.Period = 127;
    motor2_encoder.Init.Prescaler = 0;
    motor2_encoder.Init.CounterMode = TIM_COUNTERMODE_UP;
    motor2_encoder.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;

    config.EncoderMode = TIM_ENCODERMODE_TI12;
    config.IC1Polarity = TIM_ICPOLARITY_RISING;
    config.IC1Selection = TIM_ICSELECTION_DIRECTTI;
    config.IC1Prescaler = TIM_ICPSC_DIV1;
    config.IC1Filter = 0xF;

    config.IC2Polarity = TIM_ICPOLARITY_RISING;
    config.IC2Selection = TIM_ICSELECTION_DIRECTTI;
    config.IC2Prescaler = TIM_ICPSC_DIV1;
    config.IC2Filter = 0xF;

    HAL_TIM_Encoder_Init(&motor1_encoder, &config);
    HAL_TIM_Encoder_Init(&motor2_encoder, &config);
    HAL_TIM_Encoder_Start(&motor1_encoder, TIM_CHANNEL_ALL);
    HAL_TIM_Encoder_Start(&motor2_encoder, TIM_CHANNEL_ALL);
}

void HAL_TIM_Encoder_MspInit(TIM_HandleTypeDef *htim)
{
    if(htim->Instance == TIM4)
    {
        __HAL_RCC_TIM4_CLK_ENABLE();
        __HAL_RCC_GPIOB_CLK_ENABLE();

        GPIO_InitTypeDef gpio_initstruct;
        gpio_initstruct.Pin = GPIO_PIN_6 | GPIO_PIN_7;
        gpio_initstruct.Mode = GPIO_MODE_INPUT;
        gpio_initstruct.Pull = GPIO_PULLUP;
        gpio_initstruct.Speed = GPIO_SPEED_FREQ_HIGH;
        HAL_GPIO_Init(GPIOB, &gpio_initstruct);
    }
    if(htim->Instance == TIM3)
    {
        __HAL_RCC_TIM3_CLK_ENABLE();
        __HAL_RCC_GPIOA_CLK_ENABLE();

        GPIO_InitTypeDef gpio_initstruct;
        gpio_initstruct.Pin = GPIO_PIN_6 | GPIO_PIN_7;
        gpio_initstruct.Mode = GPIO_MODE_INPUT;
        gpio_initstruct.Pull = GPIO_PULLUP;
        gpio_initstruct.Speed = GPIO_SPEED_FREQ_HIGH;
        HAL_GPIO_Init(GPIOA, &gpio_initstruct);
    }
}

uint8_t encoder_r(void)
{
    static uint8_t prev_cnt = 0;
    static uint8_t output_val = 0;
    uint8_t curr_cnt = __HAL_TIM_GET_COUNTER(&motor1_encoder);
    int16_t diff = (int16_t)curr_cnt - prev_cnt;

    if (diff > 64) diff -= 128;
    else if (diff < -64) diff += 128;

    diff = -diff;

    if (diff > 0) output_val = (output_val + diff > 127) ? 127 : (output_val + diff);
    else if (diff < 0) output_val = (output_val + diff < 0) ? 0 : (output_val + diff);

    prev_cnt = curr_cnt;
    return output_val;
}

uint8_t encoder_l(void)
{
    static uint8_t prev_cnt = 0;
    static uint8_t output_val = 0;
    uint8_t curr_cnt = __HAL_TIM_GET_COUNTER(&motor2_encoder);
    int16_t diff = (int16_t)curr_cnt - prev_cnt;

    if (diff > 64) diff -= 128;
    else if (diff < -64) diff += 128;

    if (diff > 0) output_val = (output_val + diff > 127) ? 127 : (output_val + diff);
    else if (diff < 0) output_val = (output_val + diff < 0) ? 0 : (output_val + diff);

    prev_cnt = curr_cnt;
    return output_val;

}
