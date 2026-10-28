#include "adc.h"
#include "Data.h"
#include "Lowpass.h"
#include "FOC.h"
#include <math.h>
#include "delay.h"
#include "uart1.h"
#include "pwm.h"

ADC_HandleTypeDef adc_handle = {0};
DMA_HandleTypeDef adc_dma_handle = {0};

#define ADC_VREF        3.3f
#define ADC_FULL_SCALE  4096.0f

void adc_config(void)
{
    adc_handle.Instance = ADC1;
    adc_handle.Init.DataAlign = ADC_DATAALIGN_RIGHT;
    adc_handle.Init.ScanConvMode = ADC_SCAN_ENABLE;
    adc_handle.Init.ContinuousConvMode = ENABLE;
    adc_handle.Init.NbrOfConversion = 2;
    adc_handle.Init.DiscontinuousConvMode = DISABLE;
    adc_handle.Init.NbrOfDiscConversion = 0;
    adc_handle.Init.ExternalTrigConv = ADC_SOFTWARE_START;
    HAL_ADC_Init(&adc_handle);

    HAL_ADCEx_Calibration_Start(&adc_handle);
}

void HAL_ADC_MspInit(ADC_HandleTypeDef* hadc)
{
    if(hadc->Instance == ADC1)
    {
        RCC_PeriphCLKInitTypeDef adc_clk_init = {0};
        GPIO_InitTypeDef gpio_init_struct = {0};

        __HAL_RCC_ADC1_CLK_ENABLE();
        __HAL_RCC_GPIOA_CLK_ENABLE();

        gpio_init_struct.Pin =  GPIO_PIN_0 | GPIO_PIN_1;
        gpio_init_struct.Mode = GPIO_MODE_ANALOG;
        HAL_GPIO_Init(GPIOA, &gpio_init_struct);

        adc_clk_init.PeriphClockSelection = RCC_PERIPHCLK_ADC;
        adc_clk_init.AdcClockSelection = RCC_ADCPCLK2_DIV6;
        HAL_RCCEx_PeriphCLKConfig(&adc_clk_init);
    }
}

void dma_config(void)
{
    __HAL_RCC_DMA1_CLK_ENABLE();
    adc_dma_handle.Instance = DMA1_Channel1;
    adc_dma_handle.Init.Direction = DMA_PERIPH_TO_MEMORY;

    adc_dma_handle.Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;
    adc_dma_handle.Init.MemInc = DMA_MINC_ENABLE;

    adc_dma_handle.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
    adc_dma_handle.Init.PeriphInc = DMA_PINC_DISABLE;

    adc_dma_handle.Init.Priority = DMA_PRIORITY_MEDIUM;
    adc_dma_handle.Init.Mode = DMA_CIRCULAR;
    HAL_DMA_Init(&adc_dma_handle);

    __HAL_LINKDMA(&adc_handle, DMA_Handle, adc_dma_handle);
}

void adc_channel_config(ADC_HandleTypeDef* hadc, uint32_t ch, uint32_t rank, uint32_t stime)
{
    ADC_ChannelConfTypeDef adc_ch_config = {0};

    adc_ch_config.Channel = ch;
    adc_ch_config.Rank = rank;
    adc_ch_config.SamplingTime = stime;
    HAL_ADC_ConfigChannel(hadc, &adc_ch_config);
}

void adc_dma_init(uint32_t *mar)
{
    adc_config();
    adc_channel_config(&adc_handle, ADC_CHANNEL_0, ADC_REGULAR_RANK_1, ADC_SAMPLETIME_239CYCLES_5);
    adc_channel_config(&adc_handle, ADC_CHANNEL_1, ADC_REGULAR_RANK_2, ADC_SAMPLETIME_239CYCLES_5);
    dma_config();

    HAL_ADC_Start_DMA(&adc_handle, mar, 2);
}

float _readADCVoltage(uint16_t ch)
{
    return (float)ch * ADC_VREF / ADC_FULL_SCALE;
}

float cal_Iq_Id(float i_a, float i_c, float angle_el)
{
    float I_alpha = i_a;
    float I_beta  = -_1_SQRT3 * i_a - _2_SQRT3 * i_c;

    return I_beta * _cos(angle_el) - I_alpha * _sin(angle_el);
}

void Current_calibrateOffsets(void)
{
    offset_ia = 0;
    offset_ic = 0;

    for(int i = 0; i < 1000; i++)
    {
        offset_ia += _readADCVoltage(adc_result[0]);
        offset_ic += _readADCVoltage(adc_result[1]);
        delay_ms(1);
    }

    offset_ia = offset_ia / 1000.0f;
    offset_ic = offset_ic / 1000.0f;
}

void CurrSense_init(void)
{
    volts_to_amps_ratio = 1.0f / shunt_resistor / amp_gain;

    gain_a =  volts_to_amps_ratio;
    gain_c = -volts_to_amps_ratio;

    adc_dma_init((uint32_t *)&adc_result);

    Current_calibrateOffsets();
}

void CurrSense_getPhaseCurrents(void)
{
    current_a = (_readADCVoltage(adc_result[0]) - offset_ia) * gain_a;
    current_c = (_readADCVoltage(adc_result[1]) - offset_ic) * gain_c;
}

float Get_Current(void)
{
    CurrSense_getPhaseCurrents();
    float Iq_ori = cal_Iq_Id(current_a, current_c, _electricalAngle());
    return LowPassFilter_operator(&LPF_current, Iq_ori);
}

#define POLARITY_SETTLE_MS   1000u
#define POLARITY_SAMPLES     200u
#define POLARITY_FLOOR_A     0.020f

#define ADC_OFFSET_MIN_V     1.30f
#define ADC_OFFSET_MAX_V     2.00f

static void polarity_measure(float u_test, float angle_el,
                             float *avg_a, float *avg_c)
{
    float sum_a = 0.0f;
    float sum_c = 0.0f;
    uint32_t i;

    setPhaseVoltage(0.0f, u_test, angle_el);
    delay_ms(POLARITY_SETTLE_MS);

    for (i = 0; i < POLARITY_SAMPLES; i++) {
        CurrSense_getPhaseCurrents();
        sum_a += current_a;
        sum_c += current_c;
        delay_ms(1);
    }

    *avg_a = sum_a / (float)POLARITY_SAMPLES;
    *avg_c = sum_c / (float)POLARITY_SAMPLES;
}

static void polarity_report(const char *axis, float driven, float other,
                            const char *driven_name, const char *other_name)
{
    float ratio;

    printf("  %s axis: i_%s = %.1f mA, i_%s = %.1f mA\r\n",
           axis, driven_name, driven * 1000.0f,
           other_name, other * 1000.0f);

    if (driven < POLARITY_FLOOR_A && driven > -POLARITY_FLOOR_A) {
        printf("    -> i_%s too small to judge, raise u_test\r\n", driven_name);
        return;
    }

    printf("    -> i_%s %s\r\n", driven_name,
           (driven > 0.0f) ? "positive, as expected"
                           : "NEGATIVE: this channel is inverted");

    ratio = other / driven;
    if (ratio > -0.65f && ratio < -0.35f) {
        printf("    -> ratio %.3f, correct (expect -0.5)\r\n", ratio);
    } else if (ratio > 0.35f && ratio < 0.65f) {
        printf("    -> ratio %+.3f: magnitude correct but SIGN FLIPPED.\r\n",
               ratio);
        printf("       i_%s and i_%s read with the same sign; one of the two\r\n",
               driven_name, other_name);
        printf("       INA240 channels is wired opposite to the other.\r\n");
    } else {
        printf("    -> ratio %+.3f: magnitude wrong (expect +-0.5). Check\r\n",
               ratio);
        printf("       gains, offsets and that both shunts are populated.\r\n");
    }
}

static void clarke_check(float i_a, float i_c, float angle_cmd)
{
    float alpha = i_a;
    float beta  = -_1_SQRT3 * i_a - _2_SQRT3 * i_c;
    float mag   = sqrtf(alpha * alpha + beta * beta);
    float meas  = atan2f(beta, alpha);
    float err   = meas - angle_cmd;

    while (err >  _PI)  { err -= _2PI; }
    while (err < -_PI)  { err += _2PI; }

    printf("    -> Clarke: |I| = %.0f mA, vector at %+.1f deg, commanded %+.1f,"
           " error %+.1f deg\r\n",
           mag * 1000.0f,
           meas * 180.0f / _PI,
           angle_cmd * 180.0f / _PI,
           err * 180.0f / _PI);
}

void current_polarity_test(float u_test)
{
    float a0, c0;
    float a1, c1;

    printf("\r\n=== current sense polarity test ===\r\n");
    printf("u_test = %.2f V, assumed Vbus = %.2f V\r\n",
           u_test, voltage_power_supply);
    printf("offsets: ia = %.4f V, ic = %.4f V, gain = %.2f A/V\r\n",
           offset_ia, offset_ic, volts_to_amps_ratio);

    if (offset_ia < ADC_OFFSET_MIN_V || offset_ia > ADC_OFFSET_MAX_V ||
        offset_ic < ADC_OFFSET_MIN_V || offset_ic > ADC_OFFSET_MAX_V) {
        printf("ABORT: offsets outside %.2f-%.2f V, expected ~1.65 V.\r\n",
               ADC_OFFSET_MIN_V, ADC_OFFSET_MAX_V);
        printf("Check INA240 supply, the 1.65 V reference, and PA0/PA1.\r\n");
        printf("Motor NOT energised.\r\n");
        return;
    }

    motor1_pwm_start();
    polarity_measure(u_test, 0.0f, &a0, &c0);
    setPhaseVoltage(0.0f, 0.0f, 0.0f);
    motor1_pwm_stop();
    polarity_report("A", a0, c0, "a", "c");
    clarke_check(a0, c0, 0.0f);

    motor1_pwm_start();
    polarity_measure(u_test, 2.0f * _2PI / 3.0f, &a1, &c1);
    setPhaseVoltage(0.0f, 0.0f, 0.0f);
    motor1_pwm_stop();
    polarity_report("C", c1, a1, "c", "a");
    clarke_check(a1, c1, 2.0f * _2PI / 3.0f);

    printf("=== polarity test complete ===\r\n");
}
