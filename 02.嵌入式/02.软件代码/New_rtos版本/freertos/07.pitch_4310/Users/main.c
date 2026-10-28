#include "sys.h"
#include "uart1.h"
#include "delay.h"
#include "systime.h"
#include "LED.h"
#include "iic.h"
#include "key.h"
#include "as5600.h"
#include "Data.h"
#include "FOC.h"
#include "pwm.h"
#include "Lowpass.h"
#include "Motor.h"
#include "Pid.h"
#include "adc.h"
#include "dma.h"

#define SELFTEST_ENCODER       0
#define SELFTEST_POLARITY      0
#define COMMISSION_DIRECTION   0
#define SELFTEST_CLOSEDLOOP    0

#define SKIP_MOTOR_INIT     0

int main(void)
{
    HAL_Init();
    stm32_clock_init(RCC_PLL_MUL9);

    systime_init();

    lde_init();
    i2c_init();
    key_init();
    uart1_init(115200);

    pwm_init(PWM_PERIOD, 0);
    as5600_init();
    dma_init();

    lde1_open();

    PID_Pos_Set(0.20f, 0.0f, 0.0005f, PITCH_TORQUE_LIMIT_V);
    PID_Vel_Set(0.01f, 0.0f, 0.0f, 2.5f);
    PID_Cur_Set(0.5f, 0.0f, 0.0f, 2.5f);

    PID_Pos.D_lpf_Tf = 0.003f;

    PID_Pos.output_ramp = 80.0f;
    PID_Vel.output_ramp = 50.0f;
    PID_Cur.output_ramp = 50.0f;

    data_init();

    CurrSense_init();
    LOWPass_Init();

#if SELFTEST_ENCODER
    as5600_selftest();
#endif
#if SELFTEST_POLARITY
    current_polarity_test(2.0f);
#endif
#if COMMISSION_DIRECTION
    motor_direction_test();
#endif

#if SKIP_MOTOR_INIT
    printf("\r\nMotor_init() SKIPPED - rotor never energised, PWM outputs off.\r\n"
           "zero_electric_angle is NOT calibrated in this build.\r\n");
#else
    Motor_init();
#endif

#if SELFTEST_CLOSEDLOOP
    closedloop_bench_test();
#endif

#if SELFTEST_ENCODER || SELFTEST_POLARITY || COMMISSION_DIRECTION || SELFTEST_CLOSEDLOOP || COMMISSION_DIRECTION

    printf("telemetry DMA disabled (self-test build)\r\n");
#else
    HAL_UART_Transmit_DMA(&uart1_handle, send_angal, sizeof(send_angal));
#endif

    while(1)
    {
        data_change();
    }
}
