#include "data.h"
#include "math.h"
#include "string.h"
#include "sys.h"
#include "usart.h"
#include "delay.h"
#include "led.h"
#include "key.h"
#include "spi.h"
#include "laser.h"
#include "oled.h"
#include "nrf24l01.h"
#include "bmi055.h"
#include "ist8310.h"
#include "imu.h"
#include "pid.h"
#include "lvbo.h"
#include "timer.h"
#include "iic_hw.h"
#include "my_task.h"

#define PI          3.1415926f
#define PI_2        (2 * PI)
#define THRESHOLD   (PI - 0.1f)

#define LPF_TAU             0.025f
#define PID_OUT_LPF_TAU     0.030f

#define PITCH_ERR_DEADZONE  0.015f

#define PID_OUT_SLEW        1.5f
#define NRF_HOLD_MS         200u
#define FOC_FRAME_HEAD      0x55u
#define FOC_FRAME_TAIL      0xFFu
#define LIMIT(val, min, max) (((val) < (min)) ? (min) : (((val) > (max)) ? (max) : (val)))

float last_angle[3] = {0.0f, 0.0f, 0.0f};
float INS_angle_filtered[3] = {0.0f, 0.0f, 0.0f};
float angle_offset[3] = {0.0f, 0.0f, 0.0f};
float INS_continuous_angle[3] = {0.0f, 0.0f, 0.0f};
float angle_lpf[3] = {0.0f, 0.0f, 0.0f};
float gyro[3];
float accel[3];
float mag[3];
float temp[2];
float INS_quat[4] = {0.0f, 0.0f, 0.0f, 0.0f};
float INS_angle[3] = {0.0f, 0.0f, 0.0f};

int8_t joy[5] = {0};
uint8_t B_rx_A_buf[FRAME13_LEN] = {0};
uint8_t Uart_sand_byte[FRAME13_LEN] = {0};
uint8_t Uart_recv_byte[FRAME13_LEN] = {0};
pid_type_def pitch_pos_pid;

float pitch_position[3] = {0.3f, 0.0f, 0.01f};
float pitch_val[3] = {0.0f,0.0f,0.0f};
volatile uint32_t nrf_last_ms = 0;

static float pid_out_lpf = 0.0f;
static uint8_t foc_dma_buf[FRAME13_LEN];

void nrf_mark_rx(void)
{
	nrf_last_ms = HAL_GetTick();
}

static uint32_t nrf_age_ms(void)
{
	uint32_t now = HAL_GetTick();
	uint32_t last = nrf_last_ms;

	if (last == 0u)
	{
		return 0xFFFFFFFFu;
	}
	return now - last;
}

static float slew_limit(float prev, float want, float dt)
{
	float max_step;

	if (dt < 0.0001f)
	{
		dt = 0.0001f;
	}
	max_step = PID_OUT_SLEW * dt;
	if ((want - prev) > max_step)
	{
		return prev + max_step;
	}
	if ((want - prev) < -max_step)
	{
		return prev - max_step;
	}
	return want;
}

void data_init(void)
{

	PID_init(&pitch_pos_pid, PID_POSITION, pitch_position, 1.2f, 0.4f);
	Uart_sand_byte[0] = FOC_FRAME_HEAD;
	Uart_sand_byte[F13_TAIL_IDX] = FOC_FRAME_TAIL;
}

void pid_calculate(void)
{
	float raw;
	float err;
	float target;

	if (imu_is_healthy() == 0)
	{
		PID_clear(&pitch_pos_pid);
		pid_out_lpf = slew_limit(pid_out_lpf, 0.0f, imu_dt);
		pitch_val[1] = pid_out_lpf;
		return;
	}

	target = -pitch_val[0];
	err = INS_angle_filtered[1] - target;

	if (fabsf(err) < PITCH_ERR_DEADZONE)
	{
		pitch_pos_pid.Iout = 0.0f;
		pitch_pos_pid.error[0] = 0.0f;
		raw = pid_out_lpf;
	}
	else
	{
		raw = PID_calc(&pitch_pos_pid, target, INS_angle_filtered[1]);
	}

	raw = slew_limit(pid_out_lpf, raw, imu_dt);
	pid_out_lpf = low_pass_filter(raw, &pid_out_lpf, imu_dt);
	pitch_val[1] = pid_out_lpf;
}

void data_change(void)
{
	usart_poll();

	Uart_sand_byte[0] = FOC_FRAME_HEAD;
	if (nrf_age_ms() <= NRF_HOLD_MS)
	{
		for (uint8_t i = F13_KEY; i <= F13_LY_L; i++) {
			Uart_sand_byte[i] = B_rx_A_buf[i];
		}
		pitch_val[0] = two_uint8_to_float_signed(B_rx_A_buf[F13_LY_H], B_rx_A_buf[F13_LY_L]);
	}
	else
	{
		for (uint8_t i = F13_KEY; i <= F13_LY_L; i++) {
			Uart_sand_byte[i] = 0;
		}
		pitch_val[0] = 0.0f;
	}

	float_to_two_uint8_signed(INS_angle_filtered[1], &Uart_sand_byte[F13_ATT_H], &Uart_sand_byte[F13_ATT_L]);
#if GIMBAL_IMU_ENABLE
	float_to_two_uint8_signed(pitch_val[1], &Uart_sand_byte[F13_CMD_H], &Uart_sand_byte[F13_CMD_L]);
#else

	float_to_two_uint8_signed(pitch_val[0], &Uart_sand_byte[F13_CMD_H], &Uart_sand_byte[F13_CMD_L]);
#endif
	Uart_sand_byte[F13_TAIL_IDX] = FOC_FRAME_TAIL;

	joy[0] = (int8_t)Uart_sand_byte[1];
	joy[1] = (int8_t)Uart_sand_byte[2];
	joy[2] = (int8_t)Uart_sand_byte[3];
	pitch_val[2] = (float)normalizeAngleRad(two_uint8_to_float_signed(Uart_recv_byte[1], Uart_recv_byte[2]));
}

void uart_sand(void)
{

	if (DMA1_Stream3->CR & DMA_SxCR_EN)
	{
		return;
	}
	memcpy(foc_dma_buf, Uart_sand_byte, sizeof(foc_dma_buf));
	usart3_dma_send(foc_dma_buf, (uint16_t)sizeof(foc_dma_buf));
}

void data_print(void)
{
	uint32_t age = nrf_age_ms();

	printf("MOTOR p=%.2f tgt=%.2f pid=%.3f enc=%.3f nrf=%lu h=%u dt=%.4f\r\n",
	       INS_angle_filtered[1] * 57.2957795f,
	       (-pitch_val[0]) * 57.2957795f,
	       pitch_val[1],
	       pitch_val[2],
	       (unsigned long)age,
	       (unsigned)imu_is_healthy(),
	       imu_dt);
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

	float process_continuous_angle(uint8_t axis, float curr) {
    float delta = curr - last_angle[axis];
    if(delta < -THRESHOLD) angle_offset[axis] += PI_2;
    else if(delta > THRESHOLD) angle_offset[axis] -= PI_2;
    last_angle[axis] = curr;
    return curr + angle_offset[axis];
}

float low_pass_filter(float input, float *prev_output, float dt)
{
    float alpha;

    if (dt < 0.0001f)
    {
        dt = 0.0001f;
    }
    alpha = dt / (LPF_TAU + dt);
    *prev_output = alpha * input + (1.0f - alpha) * (*prev_output);
    return *prev_output;
}

uint8_t frame_sync(uint8_t byte, uint8_t *frame_out)
{
    static uint8_t ring[FRAME13_LEN];
    static uint8_t w_idx = 0;
    static uint8_t count = 0;

    ring[w_idx] = byte;
    w_idx = (w_idx + 1) % FRAME13_LEN;
    if (count < FRAME13_LEN) count++;

    if (count >= FRAME13_LEN) {
        uint8_t start = w_idx;
        if (ring[start] == FRAME13_HEAD && ring[(start + F13_TAIL_IDX) % FRAME13_LEN] == FRAME13_TAIL) {
            for (uint8_t i = 0; i < FRAME13_LEN; i++) {
                frame_out[i] = ring[(start + i) % FRAME13_LEN];
            }
            count = 0;
            return 1;
        }
    }
    return 0;
}

double normalizeAngleRad(double angle)
{
    const double two_pi = 2 * PI;
    return fmod(angle, two_pi);
}
