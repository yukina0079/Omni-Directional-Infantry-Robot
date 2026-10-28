#include "data.h"
#include "math.h"
#include "sys.h"
#include "usart.h"
#include "delay.h"
#include "led.h"
#include "key.h"
#include "nrf24l01.h"
#include "bmi088.h"
#include "ist8310.h"
#include "imu.h"
#include "pid.h"
#include "iic.h"
#include "motor.h"
#include "pid.h"
#include "my_task.h"
#include "string.h"
#include "FreeRTOS.h"
#include "task.h"

#define data_kp 120

int8_t joy[5] = {0};
float joy_yaw_num = 0.0f;
int wheel_speeds[4];

uint8_t C_rx_buf[FRAME13_LEN] = {0};
uint8_t Uart_sand_byte[FRAME13_LEN] = {0};
uint8_t Uart_recv_byte[FRAME13_LEN] = {0};

pid_type_def yaw_gimble_pid;
pid_type_def yaw_chassis_pid;

float yaw_gimble_pos[3]   	  = {1.0f, 0.0f, 0.01f};
float yaw_chassis_pos[3] 	  = {200.0f, 0.0f, 0.0f};
float yaw_output[2] 		  = {0.0f,0.0f};
float gimble_chassis[2] 	  = {0.0f,0.0f};

float gimble_yaw = 0.0f;
float gimble_angal = 0.5f;
float normal_yaw = 0.0f;

float last_angle[3]           = {0.0f, 0.0f, 0.0f};
float INS_angle_filtered[3]   = {0.0f, 0.0f, 0.0f};
float angle_offset[3]         = {0.0f, 0.0f, 0.0f};
float INS_continuous_angle[3] = {0.0f, 0.0f, 0.0f};
float angle_lpf[3]            = {0.0f, 0.0f, 0.0f};
float gyro[3];
float accel[3];
float mag[3];
float temp[2];
float INS_quat[4]             = {0.0f, 0.0f, 0.0f, 0.0f};
float INS_angle[3] 			  = {0.0f, 0.0f, 0.0f};

volatile uint32_t g_nrf_frames = 0u;
volatile uint32_t g_yaw_frames = 0u;

static volatile uint32_t s_nrf_last_ms = 0u;
static volatile uint32_t s_yaw_last_ms = 0u;
static uint8_t s_yaw_valid = 0u;

void nrf_mark_rx(void)
{
	s_nrf_last_ms = HAL_GetTick();
	g_nrf_frames++;
}

static uint32_t link_age_ms(uint32_t last_ms)
{
	if (last_ms == 0u) {
		return 0xFFFFFFFFu;
	}
	return HAL_GetTick() - last_ms;
}

static uint8_t foc_dma_buf[FRAME13_LEN];

void data_init(void)
{
	PID_init(&yaw_gimble_pid, PID_POSITION, yaw_gimble_pos,   300.0f, 300.0f);
	PID_init(&yaw_chassis_pid, PID_POSITION, yaw_chassis_pos, 300.0f, 300.0f);

}

void pid_calculate(void)
{
		yaw_output[0] = PID_calc(&yaw_gimble_pid, joy_yaw_num, INS_angle_filtered[0]);
		yaw_output[1] = PID_calc(&yaw_chassis_pid, gimble_angal ,normal_yaw);
}

void data_change(void)
{

		usart_poll();

		if (link_age_ms(s_nrf_last_ms) > CHASSIS_NRF_HOLD_MS) {

			OmniKinematics(0, 0, 0);
			return;
		}

		Uart_sand_byte[0] = FRAME13_HEAD;
		for (uint8_t i = F13_KEY; i <= F13_LY_L; i++) {
			Uart_sand_byte[i] = C_rx_buf[i];
		}

		joy[0] = C_rx_buf[1];
		joy[1] = C_rx_buf[2];
		joy[2] = C_rx_buf[3];
		joy[3] = two_uint8_to_float_signed(C_rx_buf[8],C_rx_buf[9]);
		joy[4] = two_uint8_to_float_signed(C_rx_buf[10],C_rx_buf[11]);
		joy_yaw_num = -two_uint8_to_float_signed(C_rx_buf[4],C_rx_buf[5]);

		float_to_two_uint8_signed(INS_angle_filtered[0], &Uart_sand_byte[F13_ATT_H], &Uart_sand_byte[F13_ATT_L]);
#if CHASSIS_IMU_ENABLE
		float_to_two_uint8_signed(yaw_output[0], &Uart_sand_byte[F13_CMD_H], &Uart_sand_byte[F13_CMD_L]);
#else

		float_to_two_uint8_signed(joy_yaw_num, &Uart_sand_byte[F13_CMD_H], &Uart_sand_byte[F13_CMD_L]);
#endif
		Uart_sand_byte[F13_TAIL_IDX] = FRAME13_TAIL;

		if (link_age_ms(s_yaw_last_ms) <= YAW_REPLY_HOLD_MS) {
			gimble_yaw = two_uint8_to_float_signed(Uart_recv_byte[1],Uart_recv_byte[2]);
			normal_yaw = normalizeAngleRad(gimble_yaw);
			s_yaw_valid = 1u;
		} else {
			s_yaw_valid = 0u;
		}

		gimbal_to_chassis_speed_compute(joy[3],joy[4],
		                                s_yaw_valid ? (normal_yaw - gimble_angal) : 0.0f,
		                                &gimble_chassis[0],&gimble_chassis[1]);

#if CHASSIS_MOTOR_OUTPUT_ENABLE

		if((joy[0] & 0x01) != 0x01){
			OmniKinematics((int)gimble_chassis[0],(int)gimble_chassis[1],joy[1]*3);
		}else{

			OmniKinematics(joy[3],joy[4],s_yaw_valid ? (int)yaw_output[1] : 0);
		}
#else

		OmniKinematics(0,0,0);
#endif

}
void uart_sand(void)
{

	if (DMA1_Stream3->CR & DMA_SxCR_EN) {
		return;
	}

	taskENTER_CRITICAL();
	memcpy(foc_dma_buf, Uart_sand_byte, sizeof(foc_dma_buf));
	taskEXIT_CRITICAL();

	usart3_dma_send(foc_dma_buf, (uint16_t)sizeof(foc_dma_buf));
}
void data_print(void)
{

#if CHASSIS_UART3_DEBUG_ENABLE
		printf("%.2f,%.2f,%.2f\n",INS_angle[0],INS_angle[1],INS_angle[2]);
#endif

}

void MecanumKinematics(int Vx, int Vy, int omega)
{
    wheel_speeds[0] = Vx - Vy - omega * (LX + LY);
    wheel_speeds[1] = Vx + Vy + omega * (LX + LY);
    wheel_speeds[2] = Vx + Vy - omega * (LX + LY);
    wheel_speeds[3] = Vx - Vy + omega * (LX + LY);

    for (int i = 0; i < 4; i++) {
        if (wheel_speeds[i] > MAX_SPEED) {
            wheel_speeds[i] = MAX_SPEED;
        } else if (wheel_speeds[i] < -MAX_SPEED) {
            wheel_speeds[i] = -MAX_SPEED;
        }
    }
		MOTOR_PWM_UPDATE(data_kp*wheel_speeds[0],data_kp*wheel_speeds[1],data_kp*wheel_speeds[2],data_kp*wheel_speeds[3]);
}

void OmniKinematics(int Vx, int Vy, int omega)
{
    wheel_speeds[0] =  - Vx + Vy - omega * (LX + LY);
    wheel_speeds[1] =    Vx + Vy - omega * (LX + LY);
    wheel_speeds[2] =    Vx - Vy - omega * (LX + LY);
    wheel_speeds[3] =  - Vx - Vy - omega * (LX + LY);

    for (int i = 0; i < 4; i++) {
        if (wheel_speeds[i] > MAX_SPEED) wheel_speeds[i] = MAX_SPEED;
        else if (wheel_speeds[i] < -MAX_SPEED) wheel_speeds[i] = -MAX_SPEED;
    }

    MOTOR_PWM_UPDATE(data_kp*wheel_speeds[0], data_kp*wheel_speeds[1],
                     data_kp*wheel_speeds[2], data_kp*wheel_speeds[3]);
}

 void gimbal_to_chassis_speed_compute(float gimbal_vx, float gimbal_vy,
                                     float yaw_rad,
                                     float *chassis_vx, float *chassis_vy) {

    float cos_yaw = cosf(yaw_rad);
    float sin_yaw = sinf(yaw_rad);

    *chassis_vx = gimbal_vx * cos_yaw - gimbal_vy * sin_yaw;
    *chassis_vy = gimbal_vx * sin_yaw + gimbal_vy * cos_yaw;
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

float process_continuous_angle(uint8_t axis, float curr)
{
    float delta = curr - last_angle[axis];
    if(delta < -THRESHOLD) angle_offset[axis] += PI_2;
    else if(delta > THRESHOLD) angle_offset[axis] -= PI_2;
    last_angle[axis] = curr;
    return curr + angle_offset[axis];
}

float low_pass_filter(float input, float *prev_output)
{
    *prev_output = LPF_ALPHA * input + (1.0f - LPF_ALPHA) * (*prev_output);
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

            s_yaw_last_ms = HAL_GetTick();
            g_yaw_frames++;
            return 1;
        }
    }
    return 0;
}
float normalizeAngleRad(float angle)
{
    const float two_pi = 2 * PI;
    const float pi = PI;

    angle = fmodf(angle, two_pi);
    if (angle > pi)
        angle -= two_pi;
    else if (angle < -pi)
        angle += two_pi;

    return angle;
}
