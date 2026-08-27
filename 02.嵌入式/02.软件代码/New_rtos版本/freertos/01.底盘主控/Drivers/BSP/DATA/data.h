#ifndef __DATA_H
#define __DATA_H
#include "sys.h"

// 机器人参数
/*
 * Chassis geometry -- but NOT SI units, despite what the comments below say.
 *
 * The original comments describe LX/LY as metres and MAX_SPEED as m/s. Follow
 * the values through OmniKinematics() and that reading does not hold up: Vx, Vy
 * and omega all arrive as ints in raw stick / PID-output counts, the wheel sum
 * is clamped against MAX_SPEED in those same counts, and the result is
 * multiplied by data_kp (120, in data.c) to become a TIM8 compare value. No
 * conversion from metres or from rad/s happens anywhere.
 *
 * So what these three really are:
 *
 *   (LX + LY) = 0.3   a pure gain on the rotation term, setting how much
 *                     authority omega has relative to translation. It happens to
 *                     equal the correct geometric factor for a 150 mm-by-150 mm
 *                     wheel offset, which is presumably where the numbers came
 *                     from, but nothing downstream treats it as a length.
 *   MAX_SPEED = 100    per-wheel saturation, in the same abstract counts. With
 *                     data_kp = 120 this caps duty at 12000/16799 = 71.4%.
 *
 * Practical consequences of the pseudo-unit scheme:
 *   - Retuning translation-versus-rotation balance means editing LX/LY, which
 *     reads like changing the robot's dimensions. It is not.
 *   - Raising MAX_SPEED to 140 unlocks the remaining 29% of duty. Above that the
 *     compare value exceeds ARR and the channel pins at 100%; motorN_speed()
 *     does not clamp.
 *   - Because the clamp is applied per wheel AFTER the mix, a saturating command
 *     distorts the commanded direction rather than scaling it down: the robot
 *     drifts off the intended heading at full stick instead of simply going as
 *     fast as it can. Fixing that needs a vector-wide scale factor, not a
 *     per-element clamp.
 */
#define LX 0.15f   // 轮子到中心的 X 方向距离（米）
#define LY 0.15f   // 轮子到中心的 Y 方向距离（米）
#define MAX_SPEED 100  // 最大速度（m/s 或 PWM 最大值）
// 核心常量（精简定义）

#define PI          3.1415926f
#define PI_2        (2 * PI)
/*
 * Wrap-detection threshold for process_continuous_angle().
 *
 * The AHRS reports Euler angles folded into [-PI, PI], so a shaft crossing the
 * branch cut shows up as a near-2*PI jump between consecutive samples. Anything
 * larger than THRESHOLD is taken to be that wrap rather than real motion, and
 * 2*PI is added to or subtracted from the running offset.
 *
 * PI - 0.1 is the discriminator between "wrapped" and "moved fast". Sampling at
 * 1 ms, a genuine rotation would have to exceed (PI - 0.1) rad per millisecond
 * -- about 3040 rad/s, or 29000 rpm -- to be misread as a wrap, so the margin is
 * enormous in the direction that matters.
 *
 * The failure mode runs the other way: if a sample is ever MISSED (a dropped I2C
 * read, a long preemption) so that two real samples straddle the cut with less
 * than PI of apparent jump, the wrap goes undetected and the continuous angle
 * silently gains a full turn of error that never washes out. That is why the IMU
 * read is the highest-priority task on the board.
 */
#define THRESHOLD   (PI - 0.1f)
/*
 * Coefficient of the first-order IIR in low_pass_filter():
 *     y[n] = alpha*x[n] + (1-alpha)*y[n-1]
 *
 * Applied to all three continuous Euler angles in imu.c (imu_updata), which runs
 * every 1 ms, so alpha = 0.15 gives
 *
 *     tau = -T / ln(1 - alpha) = -1 ms / ln(0.85) = 6.2 ms
 *     f_c = 1 / (2*pi*tau)     = about 26 Hz
 *
 * i.e. roughly 6 ms of added lag on the yaw angle that data.c both feeds to
 * yaw_gimble_pid and transmits to the yaw board as bytes 12/13. That lag is the
 * cost of the smoothing and it is not free: at a chassis spin rate of 10 rad/s,
 * 6 ms is 0.06 rad (3.4 degrees) of pointing error in small-gyro mode.
 *
 * Raise alpha for less lag and more noise, lower it for the reverse. The
 * original comment's 0.05..0.3 band corresponds to about 8..56 Hz here.
 */
#define LPF_ALPHA 0.15f  // 滤波系数（0.05~0.3之间调，越小越平滑）

/*
 * Unused, and would not compile if they were used: M_PI is not a standard C
 * macro and math.h is not even included here. Left in place because deleting
 * declarations from a published reference design costs more confusion than it
 * saves. Use the PI macro above instead.
 */
#define DEG2RAD(deg) ((deg) * M_PI / 180.0)
#define RAD2DEG(rad) ((rad) * 180.0 / M_PI)


/*
 * Raw nRF24L01 receive buffer, written by DATA_COMM_task in my_task.c.
 *
 * 32 bytes because that is the radio's payload size; only bytes 0..19 carry the
 * remote's frame (0x55 header, 0xFF tail at [19]) and the remaining 12 are
 * whatever the module returned. data_change() copies all 20 into
 * Uart_sand_byte and then overwrites [12]..[15] before forwarding to the yaw
 * board, so the chassis is a relay for the remote's frame as much as a consumer
 * of it.
 *
 * Not volatile and not double-buffered: DATA_COMM_task (priority 8) writes it
 * while data_exchange_task (priority 7) reads it, so a decode can in principle
 * straddle two payloads. Benign in practice because each field is read once and
 * the fields are independent, but it is the reason the header/tail check lives
 * in the receive task rather than here -- a torn buffer must not be able to
 * refresh the failsafe timer.
 */
extern uint8_t C_rx_buf[32];
extern uint8_t Uart_recv_byte[20];

extern float INS_angle_filtered[3];
extern float angle_lpf[3];
extern float last_angle[3];
extern float angle_offset[3];
extern float INS_continuous_angle[3];

extern float gyro[3];
extern float accel[3];
extern float mag[3];
extern float temp[2];
extern float INS_quat[4];
extern float INS_angle[3];


void gimbal_to_chassis_speed_compute(float gimbal_vx, float gimbal_vy, float yaw_rad,float *chassis_vx, float *chassis_vy);
void MecanumKinematics(int Vx, int Vy, int omega);
void OmniKinematics(int Vx, int Vy, int omega);
void float_to_two_uint8_signed(float num, uint8_t* high_byte, uint8_t* low_byte);
float two_uint8_to_float_signed(uint8_t high_byte, uint8_t low_byte);

void data_change(void);
void pid_calculate(void);
void data_print(void);

float process_continuous_angle(uint8_t axis, float curr);
float low_pass_filter(float input, float *prev_output) ;
uint8_t frame_sync(uint8_t byte, uint8_t *frame_out);
float normalizeAngleRad(float angle);
void data_init(void);
void uart_sand(void);

/* Radio link supervision. Call nrf_mark_rx() from the receive task for every
 * payload that passes its header/tail check; data_change() brings the wheels to
 * a stop once CHASSIS_NRF_HOLD_MS has elapsed with no such payload. */
void nrf_mark_rx(void);

/* Link frame counters. Deliberately not static so they can be watched in the
 * debugger or read over SWD: g_nrf_frames stalling means the remote is gone,
 * g_yaw_frames stalling means the FOC board is gone. Telling those two apart is
 * otherwise guesswork on a board whose only console shares the FOC wire. */
extern volatile uint32_t g_nrf_frames;
extern volatile uint32_t g_yaw_frames;
#endif


