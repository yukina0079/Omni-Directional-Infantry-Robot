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

/*
 * Current sensing on this board (new_foc, DRV8313):
 *   U6 (INA240A2) measures phase A  via R80 (10 mOhm), output -> PA0 / ADC_IN0
 *   U7 (INA240A2) measures phase C  via R81 (10 mOhm), output -> PA1 / ADC_IN1
 *
 * Note it is phases A and C, not A and B. PA4/PA5 are unconnected on the
 * schematic -- the old code scanned four channels and sampled two floating
 * pins, then treated the second reading as phase B.
 *
 * The INA240A2 has a fixed gain of 50 V/V and its REF pins are tied to the
 * 1.65 V rail (U1, ME432AXG), so a zero-current output sits at mid-supply and
 * the usable swing is bipolar around it. That offset is measured at startup by
 * Current_calibrateOffsets() and subtracted, so it must NOT also be baked into
 * the volts-per-count constant.
 */

#define ADC_VREF        3.3f     /* VDDA, and therefore ADC full scale */
#define ADC_FULL_SCALE  4096.0f  /* 12-bit */

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

        /* Only the two INA240 outputs are wired. */
        gpio_init_struct.Pin =  GPIO_PIN_0 | GPIO_PIN_1;
        gpio_init_struct.Mode = GPIO_MODE_ANALOG;
        HAL_GPIO_Init(GPIOA, &gpio_init_struct);

        /* ADC clock max is 14 MHz; 72/6 = 12 MHz. */
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

/*
 * Converts a raw ADC count to volts.
 *
 * This used to read `ch * 1.65 / 4096`, which halved every reading. 1.65 V is
 * the INA240 reference level, not the converter's full scale -- the ADC still
 * spans 0..VDDA (3.3 V). The reference offset is handled separately by
 * Current_calibrateOffsets().
 */
float _readADCVoltage(uint16_t ch)
{
    return (float)ch * ADC_VREF / ADC_FULL_SCALE;
}

/*
 * Clarke transform to the stationary alpha/beta frame, then project onto the
 * rotor's q axis.
 *
 * This board measures phases A and C (INA240 U6 on M0_OUT1, U7 on M0_OUT3),
 * not A and B. Starting from the usual A/B form and substituting the
 * Kirchhoff constraint Ib = -Ia - Ic:
 *
 *   I_beta = (Ia + 2*Ib) / sqrt(3)
 *          = (Ia + 2*(-Ia - Ic)) / sqrt(3)
 *          = -(Ia + 2*Ic) / sqrt(3)
 *
 * so both terms are negative. Getting this sign wrong flips the q-axis
 * current and turns the current loop into positive feedback.
 */
float cal_Iq_Id(float i_a, float i_c, float angle_el)
{
    float I_alpha = i_a;
    float I_beta  = -_1_SQRT3 * i_a - _2_SQRT3 * i_c;

    /* Project onto q: Iq = I_beta*cos(theta) - I_alpha*sin(theta). */
    return I_beta * _cos(angle_el) - I_alpha * _sin(angle_el);
}

void Current_calibrateOffsets(void)
{
    offset_ia = 0;
    offset_ic = 0;

    /* Motor must be de-energised here, so whatever the INA240s report is the
     * zero-current reference level. */
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
    volts_to_amps_ratio = 1.0f / shunt_resistor / amp_gain; // volts to amps

    /*
     * Phase C is negated. MEASURED, not inferred.
     *
     * current_polarity_test() parks a DC voltage vector on the phase A axis and
     * then on the phase C axis, and reads both channels each time. Ground truth
     * is fixed by the modulator: the driven phase must carry positive current
     * and the other measured phase must carry half of it, negative.
     *
     *                       A-axis test          C-axis test
     *   expected  i_a       +365 mA              -178 mA
     *   measured  i_a       +365 mA  ok          -175 mA  ok
     *   expected  i_c       -182 mA              +355 mA
     *   measured  i_c       +184 mA  INVERTED    -356 mA  INVERTED
     *
     * Channel A tracks in both tests; channel C is inverted in both. The
     * magnitudes are right either way (ratios 0.505 and 0.492 against an
     * expected 0.5), so this is a wiring polarity difference between the two
     * INA240 channels, not a gain or offset error.
     *
     * An earlier revision of this file removed the negation, reasoning from the
     * schematic that both amplifiers were wired the same way round. The bench
     * measurement above says otherwise, so the negation is restored. Getting
     * this wrong flips Iq and turns the current loop into positive feedback.
     */
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

/*
 * ---------------------------------------------------------------------------
 * DC injection polarity test
 * ---------------------------------------------------------------------------
 *
 * Determines the sign convention of the two INA240 channels without a scope
 * and without spinning the motor.
 *
 * The Clarke algebra in cal_Iq_Id() is pure maths and needs no hardware to
 * verify. What DOES need hardware is which way round the shunt amplifiers are
 * wired, because getting that wrong inverts Iq and turns the current loop into
 * positive feedback. This test settles it with two static measurements.
 *
 * Method: park a DC voltage vector on a known phase axis and check the sign of
 * the measured current.
 *
 *   setPhaseVoltage(0, U, 0)      -> Ud only, atan2f(0,U)=0, so the vector sits
 *                                   at electrical 0 = the phase A axis.
 *                                   Hand-checking the modulator: sector 1,
 *                                   T1 = 1.5*Uout, T2 = 0, so case 1 gives
 *                                   Ta > Tb = Tc. Phase A is driven high and
 *                                   B/C low.  =>  i_a > 0, i_c = -i_a/2
 *
 *   setPhaseVoltage(0, U, 4PI/3)  -> sector 5, case 5 gives Tc > Ta = Tb.
 *                                   =>  i_c > 0, i_a = -i_c/2
 *
 * Why the ratio is exactly -0.5: with the vector on the A axis the winding sees
 * R_a in series with R_b||R_c, so the phase A current splits evenly between B
 * and C. This depends only on the duty pattern and the three phase resistances
 * being equal -- NOT on where the rotor happens to be, because a stationary
 * rotor generates no back-EMF. So the test needs no knowledge of rotor position
 * and no calibrated zero. It only needs the shaft to stop moving before
 * sampling, hence the settle delays below.
 *
 * The ratio is a free cross-check on the two channels' gains: if it comes back
 * at something other than -0.5, the amplifier gains or the offsets disagree
 * even though both signs may be right.
 *
 * Safety: this is a static test, nothing spins. The rotor is pulled to the
 * commanded angle and held. Use a current-limited supply (1 A) and start with
 * u_test = 2.0 V. Run it with the barrel off -- the rotor will snap to the
 * commanded angle, which can be a sudden movement.
 */

#define POLARITY_SETTLE_MS   1000u   /* let the rotor stop before sampling */
#define POLARITY_SAMPLES     200u    /* averaged, at 1 ms spacing */
#define POLARITY_FLOOR_A     0.020f  /* below this the sign is just noise */

/*
 * Plausible window for the zero-current output of an INA240A2 whose REF is on
 * the 1.65 V rail. Generous, because it only needs to catch a dead or floating
 * channel, not to validate accuracy.
 */
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

    /*
     * The driven phase carries current INTO the motor, so its reading must be
     * positive. The other measured phase carries half of it back out, so its
     * reading must be negative and half the magnitude.
     *
     * Splitting the magnitude check from the sign check matters: a correct
     * magnitude with a flipped sign means one amplifier is wired backwards,
     * whereas a wrong magnitude would mean mismatched gains or a bad offset.
     */
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

/*
 * End-to-end check of the Clarke transform.
 *
 * Reconstructs the current-space vector from the two measured phase currents
 * and compares its direction against the electrical angle that was commanded.
 * The applied angle is ground truth -- the modulator put the voltage vector
 * there -- so a small angle error validates the ENTIRE chain at once: ADC
 * scaling, both gain signs, and the A/C-phase Clarke algebra in cal_Iq_Id().
 *
 * This is the part no amount of schematic reading can settle, because it
 * depends on the real polarity of the shunt amplifiers.
 */
static void clarke_check(float i_a, float i_c, float angle_cmd)
{
    float alpha = i_a;
    float beta  = -_1_SQRT3 * i_a - _2_SQRT3 * i_c;
    float mag   = sqrtf(alpha * alpha + beta * beta);
    float meas  = atan2f(beta, alpha);
    float err   = meas - angle_cmd;

    /* atan2f returns [-PI, PI]. Fold the difference into the same range so a
     * commanded 240 deg compares cleanly against a measured -120 deg. */
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
    float a0, c0;       /* vector on the phase A axis   */
    float a1, c1;       /* vector on the phase C axis   */

    printf("\r\n=== current sense polarity test ===\r\n");
    printf("u_test = %.2f V, assumed Vbus = %.2f V\r\n",
           u_test, voltage_power_supply);
    printf("offsets: ia = %.4f V, ic = %.4f V, gain = %.2f A/V\r\n",
           offset_ia, offset_ic, volts_to_amps_ratio);

    /*
     * Refuse to energise if the zero-current level is implausible.
     *
     * Both INA240A2 REF pins are tied to the 1.65 V rail, so with no current
     * flowing their outputs must sit near mid-supply. A reading far from that
     * means the amplifier is unpowered, REF is missing, or the ADC channel is
     * floating -- and none of those can be diagnosed by pushing current through
     * the winding. Checking first also avoids reporting a confident "INVERTED"
     * verdict that is really just a dead analog channel.
     */
    if (offset_ia < ADC_OFFSET_MIN_V || offset_ia > ADC_OFFSET_MAX_V ||
        offset_ic < ADC_OFFSET_MIN_V || offset_ic > ADC_OFFSET_MAX_V) {
        printf("ABORT: offsets outside %.2f-%.2f V, expected ~1.65 V.\r\n",
               ADC_OFFSET_MIN_V, ADC_OFFSET_MAX_V);
        printf("Check INA240 supply, the 1.65 V reference, and PA0/PA1.\r\n");
        printf("Motor NOT energised.\r\n");
        return;
    }

    /*
     * Each axis is measured, de-energised, and reported before moving on.
     *
     * Reporting immediately rather than batching both at the end means a
     * failure during the second axis cannot cost us the first axis's result,
     * and the winding is never left loaded during the milliseconds that a
     * printf at 115200 baud takes.
     */
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
