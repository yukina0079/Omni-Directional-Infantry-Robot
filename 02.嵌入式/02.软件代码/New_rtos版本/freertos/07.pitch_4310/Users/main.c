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

/*
 * Bench self-tests. Both print to USART2 and must run BEFORE the TX DMA is
 * started below: printf() goes through fputc(), which writes USART2->DR
 * directly, and the transmit DMA is configured circular. Running them
 * concurrently means the two fight over the same register and the output comes
 * out shredded. Ordering them ahead of HAL_UART_Transmit_DMA() sidesteps that
 * entirely -- no need to stop and restart the DMA.
 *
 * SELFTEST_ENCODER  safe: motor never energised. Needs only 3.3 V and the
 *                   encoder connected. Measures the real I2C bit period, the
 *                   real cost of one encoder read, and scans the clock rate
 *                   down to this board's actual failure point. ~15 s.
 *
 * SELFTEST_POLARITY energises the winding with DC. Nothing spins, but the
 *                   rotor snaps to the commanded angle -- run it with the
 *                   barrel off and the supply limited to ~1 A. Determines the
 *                   INA240 sign convention, which decides whether the current
 *                   loop is negative or positive feedback. ~3 s.
 *
 * COMMISSION_DIRECTION  physically turns the rotor ~0.3 rev to determine
 *                   sensor_direction (encoder polarity) and validate pole_pairs.
 *                   Getting sensor_direction wrong makes the position loop
 *                   positive feedback — the classic runaway. This test MUST run
 *                   before Position_VelocityCloseloop is ever enabled. ~3 s.
 *
 * SELFTEST_CLOSEDLOOP  closes the position loop against an internal setpoint,
 *                   so the control chain can be validated with no chassis MCU
 *                   attached (the comms interlock would otherwise never
 *                   energise the axis on a bench). Needs Motor_init() to have
 *                   run, so SKIP_MOTOR_INIT must be 0. ~14 s.
 *
 * Set all four to 0 for a normal build.
 */
#define SELFTEST_ENCODER       0
#define SELFTEST_POLARITY      0
#define COMMISSION_DIRECTION   0
#define SELFTEST_CLOSEDLOOP    0

/*
 * Skip the power-up electrical-zero calibration.
 *
 * Motor_init() drives the rotor with 2 V for about 2.4 s, which on an assembled
 * gimbal swings the barrel. Neither self-test needs zero_electric_angle -- the
 * encoder test only reads the bus, and the polarity test commands raw
 * electrical angles directly -- so while characterising the board this can be
 * skipped, leaving the PWM outputs disabled from reset to main loop.
 *
 * Set back to 0 for a normal build, otherwise commutation has no zero.
 */
#define SKIP_MOTOR_INIT     0

int main(void)
{
    HAL_Init();
    stm32_clock_init(RCC_PLL_MUL9);     /* 8 MHz HSE * 9 = 72 MHz */

    /*
     * Must come before anything that calls delay_ms/delay_us -- those are now
     * backed by the TIM2 microsecond counter rather than by hijacking SysTick.
     */
    systime_init();

    lde_init();
    i2c_init();
    key_init();
    uart1_init(115200);

    pwm_init(PWM_PERIOD, 0);
    as5600_init();
    dma_init();

    lde1_open();

    /*
     * PID limits, and what their units actually are.
     *
     * The output of a loop is interpreted by whoever consumes it:
     *   PositionCloseloop()        -> setPhaseVoltage()  => PID_Pos is VOLTS
     *   Position_VelocityCloseloop -> VelocityCloseloop() => PID_Pos is rad/s
     *   VelocityCloseloop()        -> setPhaseVoltage()  => PID_Vel is VOLTS
     *   CurrentCloseloop()         -> setPhaseVoltage()  => PID_Cur is VOLTS
     *
     * Any limit expressed in volts must not exceed FOC_voltage_limit(), which
     * is 0.577 * Vbus = the most SVPWM can actually synthesise (~6.75 V at
     * 11.7 V bus). The old PID_Vel limit of 60 V was ~9x beyond reach, so the
     * velocity loop saturated almost immediately and behaved as bang-bang.
     *
     * PID_Pos keeps 62.8 because the intended topology is the cascade, where
     * that number is a velocity slew limit (62.8 rad/s = 10 rev/s), not a
     * voltage. If you run PositionCloseloop() standalone, change it to
     * FOC_voltage_limit().
     *
     * NOTE: all P/I/D gains below are inherited from the old firmware, which
     * was tuned against a timestep permanently stuck at 1 ms. They are almost
     * certainly wrong now that Ts is real, and must be re-tuned on a bench.
     *
     * CONSERVATIVE CURRENT LIMITING (2026-08-09): 用户要求"电流不需要太激进，
     * 太激进发热高"。实测相电阻约 5.5Ω，360 mA → 0.72 W/相 → 2.2 W 铜耗。
     * 限 4.0 V（相电压幅值，2026-08-11 从 2.5 V 上调）时峰值电流约
     * 4.0/5.5 = 730 mA。持续全速时铜耗约为 2.5 V 时的 (4/2.5)² = 2.6 倍，
     * 动态裕量仍充足；过流兜底线是 Data.c 的 OC_LIMIT_A = 1.2 A。
     *
     * SINGLE-LOOP POSITION CONTROL (2026-08-10): the cascade was dropped in
     * favour of PositionCloseloop(), so PID_Pos is now in VOLTS per degree and
     * PID_Vel is unused by the main control path.
     *
     * Why: velocity is derived by differentiating the AS5600, and one 12-bit
     * LSB (0.088 deg) over the ~470 us loop period is 3.26 rad/s of quantisation
     * noise. Hand-slewing the gimbal sits at 0.5-2 rad/s -- below one quantum --
     * so the velocity feedback flickered between 0 and 3.26 rad/s instead of
     * reporting a slow crawl, and the loop chattered. That was the reported
     * low-speed judder. Position feedback has no such problem: its quantisation
     * is 0.088 deg, far finer than anything the eye or the mechanism resolves.
     *
     * Dropping the inner loop also removes its 10 ms LPF_velocity and one stage
     * of PID phase lag from the command path, which matters for the planned
     * chassis-spin mode: at 10 rad/s of chassis rotation, 10 ms of lag alone is
     * 0.1 rad (5.7 deg) of pointing error.
     *
     * Gains: P = 0.05 V/deg gives 4.0 V at 80 deg of error (limit raised
     * 2.5 -> 4.0 V on 2026-08-11: at 2.5 V the loop saturated past 50 deg of
     * error, so full-stick yaw commands outran the axis and the top of the
     * stick range went flat -- the reported "dead zone at full left"),
     * so the axis reaches full authority on a large step but stays
     * proportional across the small errors it actually lives in.
     * D = 0.0008 V/(deg/s) supplies
     * the damping the velocity loop used to, filtered at 1.5 ms -- long enough to
     * reject encoder quantisation, short enough not to reintroduce the lag just
     * removed. I stays 0: the axis is not gravity-loaded in yaw, so there is no
     * steady-state torque for an integrator to cancel, and integrating a
     * wrapped error invites windup at the branch cut.
     */
    /* Match yaw's loop parameters. PositionCloseloop() retains the smooth
     * breakaway assist, while PITCH_TORQUE_LIMIT_V remains the hard cap. */
    PID_Pos_Set(0.20f, 0.0f, 0.0005f, PITCH_TORQUE_LIMIT_V);
    PID_Vel_Set(0.01f, 0.0f, 0.0f, 2.5f);
    PID_Cur_Set(0.5f, 0.0f, 0.0f, 2.5f);

    PID_Pos.D_lpf_Tf = 0.003f;      /* 3 ms */

    /* Faster voltage slew: 0..6.5 V in ~80 ms instead of 160 ms. */
    PID_Pos.output_ramp = 80.0f;
    PID_Vel.output_ramp = 50.0f;
    PID_Cur.output_ramp = 50.0f;

    data_init();

    CurrSense_init();
    LOWPass_Init();

    /* Must sit after LOWPass_Init(): as5600_update() feeds LPF_velocity, and
     * the self-test calls it thousands of times. */
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
    Motor_init();                 /* stationary electrical-zero alignment */
#endif

    /*
     * Must come AFTER Motor_init(): closing the position loop commutates from
     * _electricalAngle(), which is only meaningful once zero_electric_angle has
     * been calibrated. Running it on an uncalibrated zero would command torque
     * at an arbitrary angle relative to the rotor.
     */
#if SELFTEST_CLOSEDLOOP
    closedloop_bench_test();
#endif

#if SELFTEST_ENCODER || SELFTEST_POLARITY || COMMISSION_DIRECTION || SELFTEST_CLOSEDLOOP || COMMISSION_DIRECTION
    /*
     * Telemetry stays off in a self-test build. send_angal is a binary frame
     * pushed by a circular DMA with no gap between repetitions, so leaving it
     * running would bury the self-test's text output in 20-byte binary frames
     * on the same wire.
     */
    printf("telemetry DMA disabled (self-test build)\r\n");
#else
    HAL_UART_Transmit_DMA(&uart1_handle, send_angal, sizeof(send_angal));
#endif

    while(1)
    {
        data_change();
    }
}
