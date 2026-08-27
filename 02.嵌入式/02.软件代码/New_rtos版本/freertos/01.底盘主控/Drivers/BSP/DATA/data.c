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
#include "my_task.h"   /* CHASSIS_IMU_ENABLE */
#include "string.h"    /* memcpy(), for the FOC transmit snapshot */
#include "FreeRTOS.h"  /* taskENTER_CRITICAL() */
#include "task.h"

#define data_kp 120

int8_t joy[5] = {0};
float joy_yaw_num = 0.0f;
int wheel_speeds[4];

uint8_t C_rx_buf[32] = {0};
uint8_t Uart_sand_byte[20] = {0};
uint8_t Uart_recv_byte[20] = {0};

pid_type_def yaw_gimble_pid;
pid_type_def yaw_chassis_pid;

float yaw_gimble_pos[3]   	  = {1.0f, 0.0f, 0.01f};  /* spin mode: Kp=1.0, Ki=0, Kd=0.01 */
float yaw_chassis_pos[3] 	  = {200.0f, 0.0f, 0.0f};  /* gimbal-follow: Kp=200, no Ki/Kd */
float yaw_output[2] 		  = {0.0f,0.0f};  /* [0]=spin loop out, [1]=follow loop out (chassis omega) */
float gimble_chassis[2] 	  = {0.0f,0.0f};  /* stick velocity rotated into the chassis frame: [0]=vx, [1]=vy */

float gimble_yaw = 0.0f;/* yaw shaft angle as reported by the FOC board, raw (rad) */
float gimble_angal = 0.5f;/* mechanical zero: gimbal-to-chassis offset the follow loop drives towards (rad). NOT calibrated -- see data_change() */
float normal_yaw = 0.0f;/* gimble_yaw folded into [-PI, PI] */
/*
 * Attitude pipeline. Every array below is written by imu_updata() in imu.c, and
 * the chain runs:
 *
 *   gyro/accel/mag  --Mahony AHRS-->  INS_quat  --get_angle-->  INS_angle
 *     (wrapped to +-PI)  --process_continuous_angle-->  INS_continuous_angle
 *     (unwrapped)        --low_pass_filter-->           INS_angle_filtered
 *
 * last_angle[] and angle_offset[] are the unwrapper's per-axis state; angle_lpf[]
 * is the filter's. Index 0 = yaw, 1 = pitch, 2 = roll throughout.
 *
 * INS_angle_filtered[0] is the one value this file actually uses: it is both
 * transmitted as frame bytes 12/13 and closed on by yaw_gimble_pid.
 *
 * None are volatile despite being written in one task and read in another. Safe
 * here only because each is a 4-byte aligned float accessed by a single
 * instruction on Cortex-M4, so a reader never sees a half-written value -- but a
 * reader can still see a set of three angles from two different samples.
 */
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


/* ---------------------------------------------------------------------------
 * Link supervision
 * ---------------------------------------------------------------------------
 *
 * Two independent links feed this board and neither used to be supervised:
 *
 *   remote --NRF--> chassis      A dropout left joy[] frozen at the last stick
 *                                position, so the robot kept driving at the
 *                                last commanded speed indefinitely. The three
 *                                other receivers on this radio all had a
 *                                200 ms hold; the chassis did not.
 *
 *   yaw board --UART--> chassis  Absence left Uart_recv_byte all zero, which
 *                                decodes to a perfectly plausible 0.00 rad.
 *                                Against gimble_angal = 0.5f that is a
 *                                standing 0.5 rad error, and yaw_chassis_pid's
 *                                Kp of 200 turns it into omega = -100, i.e.
 *                                about 21% duty on all four wheels. A chassis
 *                                with no FOC board attached span in place by
 *                                construction, not by chance.
 *
 * HAL_GetTick() is a valid time base here even under the scheduler: delay.c:21
 * installs a SysTick_Handler that calls HAL_IncTick() before handing the tick
 * to xPortSysTickHandler(), so uwTick advances at 1 kHz both before and after
 * vTaskStartScheduler().
 */
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

/*
 * Age of a link in milliseconds, or 0xFFFFFFFF if it has never been seen.
 *
 * The never-seen case must report expired rather than zero. Both stamps are 0
 * at reset, so a plain subtraction would answer "0 ms ago" and a board that had
 * never heard the remote would consider the link perfectly healthy and drive.
 * Returning the maximum makes silence at power-up indistinguishable from
 * silence after a dropout, which is exactly the intent.
 */
static uint32_t link_age_ms(uint32_t last_ms)
{
	if (last_ms == 0u) {
		return 0xFFFFFFFFu;
	}
	return HAL_GetTick() - last_ms;
}

/* Snapshot of Uart_sand_byte handed to the TX DMA. See uart_sand(). */
static uint8_t foc_dma_buf[20];

void data_init(void)
{
	PID_init(&yaw_gimble_pid, PID_POSITION, yaw_gimble_pos,   300.0f, 300.0f);  /* spin-mode heading loop */
	PID_init(&yaw_chassis_pid, PID_POSITION, yaw_chassis_pos, 300.0f, 300.0f);  /* gimbal-follow loop */

}

/*
 * Evaluate both yaw loops. Called once per millisecond from data_get_task
 * (my_task.c), immediately after a successful IMU read -- so the loops only
 * advance when there is fresh attitude, and a dead sensor freezes them instead
 * of integrating against stale data.
 *
 * The two loops answer different questions, and only one drives the wheels:
 *
 *   yaw_gimble_pid   "where should the GIMBAL point?"  Compares the operator's
 *     integrated stick angle against the chassis IMU heading. Its output leaves
 *     the board as frame bytes 14/15 for the yaw FOC board to act on; nothing
 *     local consumes it. Live only when CHASSIS_IMU_ENABLE is 1.
 *
 *   yaw_chassis_pid  "how fast should the CHASSIS rotate?"  Compares the
 *     gimbal's measured angle relative to the chassis against gimble_angal. Its
 *     output becomes the omega term of OmniKinematics() in follow mode -- this
 *     is what swings the chassis round to line up behind the barrel.
 *
 * ARGUMENT ORDER. PID_calc(pid, ref, set) computes error = set - ref, so the
 * first argument is the FEEDBACK and the second the TARGET (see pid.h). Both
 * calls below pass them the other way round, so what each loop really computes
 * is
 *
 *     error = measurement - target
 *
 * the negative of the textbook definition. That is a project-wide convention
 * rather than a slip -- the gimbal board's pid_calculate() computes the same
 * error by hand and arrives at the identical sign -- and the compensating
 * inversion lives downstream: for the follow loop, the global -omega in
 * OmniKinematics() together with the motor wiring.
 *
 * So do not flip this in isolation, and do not flip it on one board only. What
 * IS genuinely unverified is the end-to-end polarity. Settle that once with the
 * robot on blocks and the wheels free: push the gimbal off-centre by hand and
 * check the chassis rotates to follow rather than away.
 *
 * Note too that Kp = 200 acts on an error in RADIANS. 0.1 rad (5.7 deg) gives
 * omega = 20, which after the (LX+LY) = 0.3 factor is 6 counts per wheel; but
 * 0.5 rad gives omega = 100 and saturates every wheel. Beyond about half a
 * radian of error the follow loop is effectively bang-bang.
 */
void pid_calculate(void)
{
		yaw_output[0] = PID_calc(&yaw_gimble_pid, joy_yaw_num, INS_angle_filtered[0]);  /* spin loop: arg1=stick angle, arg2=IMU yaw -> error = IMU - stick */
		yaw_output[1] = PID_calc(&yaw_chassis_pid, gimble_angal ,normal_yaw);  /* follow loop: arg1=target offset, arg2=measured -> error = measured - target */
}
/*
 * FRAME LAYOUT -- 20 bytes, the same on both links.
 *
 * The remote builds it (00.yaokongqi Drivers/BSP/DATA/data.c, data_change) and
 * broadcasts over nRF24L01 every 3 ms. This board receives it into C_rx_buf,
 * consumes some fields, overwrites four bytes, and relays the whole thing to the
 * yaw FOC board over USART3 every 1 ms.
 *
 *   byte   field      meaning                            written by
 *   -----  ---------  ---------------------------------  --------------
 *    0     0x55       header                             remote
 *    1     key_num    8 key bits, idle 0xFF, active low  remote
 *    2     enl        left knob, 0..127                  remote
 *    3     enr        right knob, 0..127                 remote
 *   4-5    lxh lxl    left stick X, INTEGRATED, rad*100  remote
 *   6-7    lyh lyl    left stick Y, INTEGRATED, rad*1000 remote
 *   8-9    rxh rxl    right stick X, +-128, *100         remote
 *  10-11   ryh ryl    right stick Y, +-128, *100         remote
 *  12-13   yaw        chassis IMU yaw, rad*100           THIS BOARD
 *  14-15   pid        yaw command, rad*100               THIS BOARD
 *  16-18   0          unused                             remote
 *   19     0xFF       tail                               remote
 *
 * All 16-bit fields are big-endian signed, scaled by 100 (bytes 6/7 by 1000) --
 * see float_to_two_uint8_signed() below. Header and tail are the ONLY integrity
 * check anywhere on either link: no CRC, no sequence number. Any payload that
 * starts 0x55 and ends 0xFF is trusted.
 *
 * Two properties of this layout drive most of the code below:
 *
 *   - The left stick arrives ALREADY INTEGRATED, as an absolute angle. The
 *     remote does the integration (Joystick_Points_kp in its data.h), so the
 *     gimbal command is a position and this board integrates nothing to steer
 *     the turret.
 *   - The right stick arrives as instantaneous deflection, i.e. a velocity.
 *     That is what feeds the chassis kinematics.
 *
 *   left stick -> gimbal (position)     right stick -> chassis (velocity)
 *   left knob  -> spin rate             right knob  -> friction wheels
 *   key bit 0  -> spin / follow mode
 */
void data_change(void)
{
		/* Drain the USART3 RX ring into frame_sync(). Must also run on the
		 * failsafe path below -- see the note there. */
		usart_poll();


		if (link_age_ms(s_nrf_last_ms) > CHASSIS_NRF_HOLD_MS) {
			/*
			 * Remote is gone. Wheels to a hard stop, and Uart_sand_byte is
			 * deliberately left untouched so the TX DMA keeps re-sending the
			 * last complete frame.
			 *
			 * Freezing the frame rather than zeroing it is the correct failsafe
			 * for this protocol: bytes 14/15 are an INCREMENTAL command on the
			 * yaw board (08.yaw_4310/Drivers/BSP/Foc/Data.c:478 does
			 * s_cmd += dcmd), so repeating the same value yields dcmd == 0 and
			 * the axis holds position while staying energised. Zeroing the bytes
			 * would instead look like one large negative increment and slew the
			 * gimbal; stopping transmission altogether would trip the yaw board's
			 * 100 ms interlock and let the barrel go limp.
			 *
			 * usart_poll() above still runs on this path, and must: it has to keep
			 * draining the circular RX ring, or last_dma_pos falls behind the DMA
			 * write pointer and the byte-stream position is lost for good.
			 */
			OmniKinematics(0, 0, 0);
			return;
		}
	
		/* Relay the remote's frame verbatim; [12]..[15] get overwritten below. */

		
		/* Relay the remote's frame verbatim into the outgoing buffer; bytes
		 * [12]..[15] are overwritten further down. Copying all 20 rather than
		 * just the fields this board cares about is what makes the chassis a
		 * relay: the yaw board needs the sticks and keys too, and it has no
		 * radio of its own. */
		for(int i=0;i<20;i++){Uart_sand_byte[i] = C_rx_buf[i];};

		/*
		 * Decode. joy[] is int8_t, which is worth pausing on: joy[3] and joy[4]
		 * are assigned from a float that legitimately reaches +-128.0, while
		 * int8_t tops out at +127. Full deflection in the positive direction
		 * therefore wraps to -128 -- the stick pushed hard one way commands full
		 * speed the OTHER way, for exactly that extreme value.
		 *
		 * Reachable rather than theoretical: map0_4096To128_128WithMaskOpt() on
		 * the remote rescales the live stick range so the mechanical stop maps to
		 * precisely 128. Left as-is because the fix belongs on one side only --
		 * clamp to 127 on the remote, or widen joy[] to int16_t here -- and the
		 * gimbal and shooter boards decode the same bytes, so changing their wire
		 * meaning is not a local decision.
		 */
		joy[0] = C_rx_buf[1];  /* key bitmask, idle 0xFF; bit0 selects spin vs follow */
		joy[1] = C_rx_buf[2];  /* left knob 0..127 -> spin rate */
		joy[2] = C_rx_buf[3];  /* right knob 0..127 -> friction wheels; relayed only, unused here */
		joy[3] = two_uint8_to_float_signed(C_rx_buf[8],C_rx_buf[9]);  /* right stick X, instantaneous */
		joy[4] = two_uint8_to_float_signed(C_rx_buf[10],C_rx_buf[11]);  /* right stick Y, instantaneous */
		joy_yaw_num = -two_uint8_to_float_signed(C_rx_buf[4],C_rx_buf[5]);  /* left stick X: already INTEGRATED by the remote, so an angle in rad, not a rate. Negated to match the yaw axis sense. */

		/*
		 * Overwrite the two chassis-authored fields. Everything else in
		 * Uart_sand_byte is still the remote's own bytes, copied above, so the
		 * yaw board receives the operator's inputs and this board's attitude in
		 * a single frame.
		 */
		float_to_two_uint8_signed(INS_angle_filtered[0],&Uart_sand_byte[12],&Uart_sand_byte[13]);  /* bytes 12/13: chassis IMU yaw */
#if CHASSIS_IMU_ENABLE
		float_to_two_uint8_signed(yaw_output[0],&Uart_sand_byte[14],&Uart_sand_byte[15]);				//yaw_pid
#else
		/*
		 * IMU not fitted yet: pass the stick's own angle command straight to the
		 * yaw board instead of the (never-computed) gimbal PID output.
		 *
		 * Why this works without the IMU. The remote already integrates the lx
		 * stick into an angular setpoint -- 00.遥控器/Drivers/BSP/DATA/data.c:58,
		 * Joystick_Points_value[0] += lx * 0.0002f every millisecond -- so
		 * joy_yaw_num is a position command in radians, not a rate. The yaw board
		 * closes its own position loop on its AS5600 encoder, so it can follow
		 * that command with no attitude reference of any kind.
		 *
		 * What is LOST versus the IMU path: this makes the gimbal follow the
		 * STICK, not hold a heading in the world frame. Rotate the chassis and the
		 * gimbal rotates with it, because nothing tells it that it was carried
		 * away. The IMU stage is what turns "point where I push" into "stay
		 * pointed there" -- a refinement, not a prerequisite for motion. Setting
		 * CHASSIS_IMU_ENABLE to 1 restores the original path unchanged.
		 *
		 * Sent unclamped. The yaw axis rotates a full 360 degrees, so there is
		 * no excursion to limit, and clamping here only hid how far the stick
		 * had actually been pushed -- during bench testing a clamped value sat
		 * at exactly the limit whether the operator was steering or the remote
		 * had dropped its link, which made the two indistinguishable.
		 *
		 * The remote's lx integrator is unbounded (its limit_float() call is
		 * commented out at 00.遥控器/data.c:62, unlike ly on the line above), so
		 * this value grows without limit while the stick is held over. On a
		 * continuous-rotation axis that is the correct behaviour: the gimbal
		 * keeps turning as long as the operator keeps pushing.
		 */
		float_to_two_uint8_signed(joy_yaw_num,
		                          &Uart_sand_byte[14], &Uart_sand_byte[15]);
#endif

		/*
		 * Decode the yaw board's reported shaft angle, but only while its reply is
		 * fresh. An absent board leaves Uart_recv_byte all zero, which decodes to
		 * a valid-looking 0.00 rad -- absence of data must not be able to
		 * masquerade as data. On expiry the last known angle is kept (it remains
		 * the best estimate available) and only the validity flag drops, so each
		 * consumer can pick its own fallback.
		 */
		if (link_age_ms(s_yaw_last_ms) <= YAW_REPLY_HOLD_MS) {
			gimble_yaw = two_uint8_to_float_signed(Uart_recv_byte[1],Uart_recv_byte[2]);
			normal_yaw = normalizeAngleRad(gimble_yaw);
			s_yaw_valid = 1u;
		} else {
			s_yaw_valid = 0u;
		}

		/*
		 * Chassis-frame velocity, rotated by the gimbal's offset from the chassis.
		 *
		 * Uses normal_yaw rather than the raw gimble_yaw. Note this is defensive,
		 * not a fix: the rotation below is built from cosf/sinf, which are 2*PI
		 * periodic, so a reply that is a whole turn out would give bit-identical
		 * results either way. What the fold buys is independence from the yaw
		 * board's own normalisation -- today it folds its reply before sending
		 * (08.yaw_4310/Drivers/BSP/Foc/Data.c, reply_publish), but the wire format
		 * saturates at +-327.67 rad, and if that fold were ever removed a
		 * multi-turn value would first go large and then pin at the saturation
		 * limit, at which point the angle fed here would be meaningless. Folding
		 * on the consuming side keeps this computation correct on its own terms.
		 *
		 * It also makes the two consumers agree: yaw_output[1] above is already
		 * computed from normal_yaw, so using the raw value here meant the follow
		 * PID and the velocity rotation were reading the gimbal angle through two
		 * different conventions.
		 */
		gimbal_to_chassis_speed_compute(joy[3],joy[4],
		                                s_yaw_valid ? (normal_yaw - gimble_angal) : 0.0f,
		                                &gimble_chassis[0],&gimble_chassis[1]);

#if CHASSIS_MOTOR_OUTPUT_ENABLE
		/*
		 * Wheel output. Gated here rather than around the whole task, because
		 * everything above -- frame decode, the yaw command, the reply parse --
		 * must keep running for the gimbal link to work even when the chassis
		 * itself is meant to stay still on the bench.
		 */
		if((joy[0] & 0x01) != 0x01){
			OmniKinematics((int)gimble_chassis[0],(int)gimble_chassis[1],joy[1]*3);  /* spin mode */
		}else{
			/* Without a reply the follow loop cannot close at all, and the only
			 * safe rotation rate is zero. Translation still works. */
			OmniKinematics(joy[3],joy[4],s_yaw_valid ? (int)yaw_output[1] : 0);  /* gimbal-follow mode */
		}
#else
		/* Bench mode: keep the wheels commanded to a hard stop. */
		OmniKinematics(0,0,0);
#endif
//		OmniKinematics(0,0,50);
//		MecanumKinematics(joy[3],joy[4],0);
}
void uart_sand(void)
{
	/*
	 * 20 bytes at 115200 baud is 1.74 ms. The old blocking HAL_UART_Transmit()
	 * therefore demanded 174% of a 1 ms task period: the wire was saturated with
	 * no idle gap, the real frame rate collapsed to roughly 250 Hz instead of the
	 * intended 1 kHz, and the idle task never ran at all.
	 *
	 * It also tore frames. HAL walks Uart_sand_byte byte by byte across those
	 * 1.74 ms while data_exchange_task -- which runs at a HIGHER priority and
	 * rewrites the same array every millisecond -- preempts it. A frame on the
	 * wire could carry bytes 0-7 from one control cycle and bytes 8-19 from the
	 * next; split across the 14/15 pair that is a position command the shaft was
	 * never asked for. The yaw board guards its own reply against exactly this
	 * (reply_publish()); this direction had no guard.
	 *
	 * Skip rather than wait when the DMA is still busy: spinning here would stall
	 * the task for up to 1.7 ms, while skipping costs one cycle of staleness and
	 * the next cycle almost always succeeds.
	 */
	if (DMA1_Stream3->CR & DMA_SxCR_EN) {
		return;
	}

	/*
	 * Snapshot under a critical section. The copy is what makes the frame atomic
	 * with respect to data_exchange_task; without the guard the memcpy itself can
	 * be preempted and mix two cycles, which is the same defect in a smaller
	 * window. 20 bytes is a few tens of cycles at 168 MHz, so the added interrupt
	 * latency is negligible. (The gimbal board copies without this guard and
	 * still has that window.)
	 */
	taskENTER_CRITICAL();
	memcpy(foc_dma_buf, Uart_sand_byte, sizeof(foc_dma_buf));
	taskEXIT_CRITICAL();

	usart3_dma_send(foc_dma_buf, (uint16_t)sizeof(foc_dma_buf));
}
void data_print(void)
{
//		printf("%x,%d,%d,%d,%d,%d,%d\r\n",key_num,rx,ry,lx,ly,en1,en2);	
//		for(int i=0;i<20;i++){
//			printf("%2x,",C_rx_buf[i]);
//		}printf("\r\n");
//		for(int i=0;i<5;i++){
//			printf("%d,",joy[i]);
//		}printf("\r\n");
//		printf("%.2f,%.2f,%.2f\n",INS_angle_filtered[0],INS_angle_filtered[1],INS_angle_filtered[2]);	
#if CHASSIS_UART3_DEBUG_ENABLE
		printf("%.2f,%.2f,%.2f\n",INS_angle[0],INS_angle[1],INS_angle[2]);
#endif
//		printf("%.2f,%.2f,%.2f,%.2f,%.2f,%.2f\r\n",accel[0], accel[1], accel[2],gyro[0], gyro[1], gyro[2]);
//		printf("%.2f,%.2f,%.2f\n", accel[0], accel[1], accel[2]);
//		printf("%.2f\r\n",gimble_yaw);
//		printf("%.2f\r\n",yaw_output[1]);
	

}

/**
 * @brief Mecanum inverse kinematics -- UNUSED on this robot.
 *
 * Kept for reference; the chassis calls OmniKinematics() instead. Retained
 * verbatim because it is the textbook form and makes a useful comparison with
 * the omni version below:
 *
 *     w_FL = Vx - Vy - k*omega        k = LX + LY
 *     w_FR = Vx + Vy + k*omega
 *     w_RL = Vx + Vy - k*omega
 *     w_RR = Vx - Vy + k*omega
 *
 * The tell that the two functions describe genuinely different geometries, and
 * not a typo in one of them, is the omega column. Here the signs alternate
 * (-,+,-,+) because a mecanum chassis rotates by driving its left pair against
 * its right pair. In OmniKinematics() every omega term has the SAME sign,
 * because omni wheels mounted at 45 degrees all point tangentially around the
 * centre and so all drive the same way during a spin.
 *
 * Note Vx appears with the same sign on all four wheels here -- a mecanum
 * chassis drives straight ahead by turning all wheels forward -- whereas the
 * omni form has mixed Vx signs. Same reason: the wheels face different
 * directions.
 *
 * @param Vx     forward velocity, in the abstract counts described in data.h
 * @param Vy     lateral velocity, same units
 * @param omega  rotation rate, same units
 */
void MecanumKinematics(int Vx, int Vy, int omega) 
{
    wheel_speeds[0] = Vx - Vy - omega * (LX + LY);  // Wheel1
    wheel_speeds[1] = Vx + Vy + omega * (LX + LY);  // Wheel2
    wheel_speeds[2] = Vx + Vy - omega * (LX + LY);  // Wheel3
    wheel_speeds[3] = Vx - Vy + omega * (LX + LY);  // Wheel4

    /* Per-wheel clamp. See the saturation note in OmniKinematics(). */
    for (int i = 0; i < 4; i++) {
        if (wheel_speeds[i] > MAX_SPEED) {
            wheel_speeds[i] = MAX_SPEED;
        } else if (wheel_speeds[i] < -MAX_SPEED) {
            wheel_speeds[i] = -MAX_SPEED;
        }
    }
		MOTOR_PWM_UPDATE(data_kp*wheel_speeds[0],data_kp*wheel_speeds[1],data_kp*wheel_speeds[2],data_kp*wheel_speeds[3]);
}
/**
 * @brief Four-wheel omni inverse kinematics, X-CONFIGURATION. This is the one
 *        the chassis actually uses.
 *
 *     w0 = -Vx + Vy - k*omega          k = LX + LY = 0.3
 *     w1 =  Vx + Vy - k*omega
 *     w2 =  Vx - Vy - k*omega
 *     w3 = -Vx - Vy - k*omega
 *
 * Read as a matrix, each row is the dot product of the commanded body velocity
 * with that wheel's drive direction, plus its tangential contribution to
 * rotation:
 *
 *     [w0]   [-1  +1  -k] [Vx   ]
 *     [w1] = [+1  +1  -k] [Vy   ]
 *     [w2]   [+1  -1  -k] [omega]
 *     [w3]   [-1  -1  -k]
 *
 * The +-1 pattern is what a 45-degree mount produces: the 1/sqrt(2) that belongs
 * in a properly normalised projection is folded into the overall gain instead,
 * which is harmless because nothing downstream treats these numbers as physical
 * speeds (see data.h). The omega column is uniform, which is the signature of
 * true rotation about the centre.
 *
 * WHICH INDEX IS WHICH WHEEL is not determinable from this code. w0..w3 become
 * motors 1..4 through MOTOR_PWM_UPDATE(), and the physical arrangement depends
 * on how the four motors are plugged in. Establish it empirically -- command one
 * index at a time with the robot on blocks -- before trusting any sign here. A
 * single swapped pair turns forward into a spin.
 *
 * SATURATION IS PER WHEEL, AND THAT IS A REAL LIMITATION. Each w is clamped to
 * +-MAX_SPEED independently, so once any wheel saturates the four no longer form
 * the commanded velocity vector: the robot veers off the intended heading rather
 * than simply travelling as fast as it can in the right direction. Full stick
 * plus a spin command is exactly where this bites, because the omega term alone
 * can reach 114 against a limit of 100. The correct fix is to scale all four by
 * a common factor when the largest exceeds the limit; it is deliberately not
 * done here, because changing the feel of the controls is a driving decision
 * rather than a code-correctness one.
 *
 * @param Vx     forward velocity, abstract counts (see data.h)
 * @param Vy     lateral velocity, same units
 * @param omega  rotation rate, same units; from the left knob in spin mode or
 *               from yaw_chassis_pid in follow mode
 */
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
/**
 * @brief Rotate a velocity from the GIMBAL frame into the CHASSIS frame.
 *
 *     [chassis_vx]   [cos(yaw)  -sin(yaw)] [gimbal_vx]
 *     [chassis_vy] = [sin(yaw)   cos(yaw)] [gimbal_vy]
 *
 * a plain 2D rotation by the gimbal's angular offset from the chassis.
 *
 * WHY THIS EXISTS. The operator steers in the frame they can see, which is where
 * the barrel points -- push the stick forward and the robot should move the way
 * the turret is facing. In spin mode ("small gyro") the chassis is rotating
 * continuously underneath the turret, so the same stick input has to mean a
 * different set of wheel speeds from one millisecond to the next. This rotation
 * is what decouples the two: the stick commands a velocity in the turret's
 * frame, and this converts it into the frame the wheels live in.
 *
 * Called only on the spin path in data_change(). Follow mode passes joy[3]/joy[4]
 * to OmniKinematics() unrotated, which is correct by construction: the whole
 * point of follow mode is that the chassis is already aligned with the gimbal,
 * so the rotation is approximately the identity and applying it would only add
 * the follow loop's tracking error into the translation command.
 *
 * No failure mode and no return value: cosf/sinf are defined for every finite
 * input, and a yaw_rad that is a whole number of turns out gives bit-identical
 * results because both are 2*PI periodic. (The original signature documented a
 * bool return; there has never been one.)
 *
 * @param gimbal_vx   velocity along the gimbal's X axis
 * @param gimbal_vy   velocity along the gimbal's Y axis
 * @param yaw_rad     gimbal angle relative to the chassis, radians, already
 *                    offset by gimble_angal at the call site
 * @param chassis_vx  out: velocity along the chassis X axis
 * @param chassis_vy  out: velocity along the chassis Y axis
 */
 void gimbal_to_chassis_speed_compute(float gimbal_vx, float gimbal_vy, 
                                     float yaw_rad,
                                     float *chassis_vx, float *chassis_vy) {

    float cos_yaw = cosf(yaw_rad);  
    float sin_yaw = sinf(yaw_rad);

    *chassis_vx = gimbal_vx * cos_yaw - gimbal_vy * sin_yaw;
    *chassis_vy = gimbal_vx * sin_yaw + gimbal_vy * cos_yaw;
}


/**
 * @brief Pack a float into two big-endian bytes, scaled by 100.
 *
 * The wire format for every 16-bit field in the frame except bytes 6/7. Range is
 * therefore +-327.67 with a resolution of 0.01, and the input is CLAMPED to that
 * range rather than allowed to wrap -- which matters, because a wrapped angle
 * would decode to a plausible value pointing somewhere else entirely.
 *
 * 0.01 rad of resolution is 0.57 degrees. Coarse for a turret: it is the reason
 * the remote uses a *1000 variant (floatToTwoSint8Milli) for the pitch field,
 * where the whole travel is only about 50 degrees and 0.57-degree steps are
 * visible as judder. Yaw gets away with *100 because it rotates continuously.
 *
 * round() rather than truncation, so the error is +-0.005 rather than 0..0.01
 * biased towards zero -- worth having on a value that is differenced downstream.
 *
 * @param num        value to pack
 * @param high_byte  out: most significant byte
 * @param low_byte   out: least significant byte
 */
void float_to_two_uint8_signed(float num, uint8_t* high_byte, uint8_t* low_byte) 
{

    if(num > 327.67f) num = 327.67f;
    if(num < -327.68f) num = -327.68f;
    
    int16_t scaled = (int16_t)(round(num * 100.0f));
    
    *high_byte = (scaled >> 8) & 0xFF;
    *low_byte = scaled & 0xFF;
}

/**
 * @brief Unpack two big-endian bytes into a float, undoing the *100 scale.
 *
 * The cast to int16_t is what makes negative values work: (high << 8) | low is
 * an int in the range 0..65535, and the cast reinterprets the top bit as a sign.
 * Leave it out and -1 decodes as +655.35.
 *
 * @param high_byte  most significant byte
 * @param low_byte   least significant byte
 * @retval the value, range +-327.68
 */
float two_uint8_to_float_signed(uint8_t high_byte, uint8_t low_byte) 
{
    int16_t scaled = (int16_t)((high_byte << 8) | low_byte);
    
    return (float)scaled / 100.0f;
}

/**
 * @brief Unwrap a wrapped angle into a continuous one.
 *
 * The AHRS reports Euler angles folded into [-PI, PI]. That fold is fatal to a
 * control loop: a shaft creeping past PI jumps to -PI, and a PID sees a 2*PI
 * error step and slams the output over. This function detects the jump (any
 * delta larger than THRESHOLD, see data.h) and maintains a running multiple of
 * 2*PI in angle_offset[] so the returned value moves smoothly through the cut.
 *
 * State is PER AXIS, held in last_angle[] and angle_offset[], so the axis
 * argument selects which history to use -- 0 = yaw, 1 = pitch, 2 = roll,
 * matching INS_angle[]. Passing the wrong index corrupts two axes at once and
 * the symptom (an angle that occasionally leaps by a full turn and stays there)
 * looks nothing like an indexing bug.
 *
 * The offset never resets and is unbounded by design: after enough turns in one
 * direction the value grows large enough to lose float resolution, and the
 * transmit path saturates at +-327.67 rad (about 52 turns) long before that.
 * Nothing here guards against it, which is why data_change() folds the angle
 * back with normalizeAngleRad() before using it.
 *
 * @param axis  0 = yaw, 1 = pitch, 2 = roll
 * @param curr  the freshly wrapped angle for that axis
 * @retval the continuous (unwrapped) angle
 */
float process_continuous_angle(uint8_t axis, float curr) 
{
    float delta = curr - last_angle[axis];
    if(delta < -THRESHOLD) angle_offset[axis] += PI_2;
    else if(delta > THRESHOLD) angle_offset[axis] -= PI_2;
    last_angle[axis] = curr;
    return curr + angle_offset[axis];
}


/**
 * @brief First-order IIR low-pass, in place.
 *
 *     y[n] = LPF_ALPHA*x[n] + (1 - LPF_ALPHA)*y[n-1]
 *
 * The caller owns the state, passed by pointer, so one function serves all three
 * axes with independent histories (angle_lpf[] in imu.c). At the 1 ms IMU rate
 * and LPF_ALPHA = 0.15 this is roughly a 26 Hz corner with about 6 ms of lag --
 * see the derivation at LPF_ALPHA in data.h.
 *
 * Apply this only AFTER unwrapping. Filtering a wrapped angle averages values
 * either side of the branch cut and produces an output that sweeps through zero
 * while the shaft has not moved at all.
 *
 * @param input        the new sample
 * @param prev_output  in/out: filter state, one per signal
 * @retval the filtered value (also stored through prev_output)
 */
float low_pass_filter(float input, float *prev_output) 
{
    *prev_output = LPF_ALPHA * input + (1.0f - LPF_ALPHA) * (*prev_output);
    return *prev_output;
}
uint8_t frame_sync(uint8_t byte, uint8_t *frame_out)
{
    static uint8_t ring[20];
    static uint8_t w_idx = 0;
    static uint8_t count = 0;

    ring[w_idx] = byte;
    w_idx = (w_idx + 1) % 20;
    if (count < 20) count++;

    if (count >= 20) {
        uint8_t start = w_idx;
        if (ring[start] == 0x55 && ring[(start + 19) % 20] == 0xFF) {
            for (uint8_t i = 0; i < 20; i++) {
                frame_out[i] = ring[(start + i) % 20];
            }
            count = 0;   /* consume the window so one frame is never delivered twice */
            /* The only place a reply frame is confirmed well-formed, so the
             * only honest place to stamp the link. Mirrors the yaw board's own
             * frame_sync() (08.yaw_4310/Drivers/BSP/Foc/Data.c:692). */
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
