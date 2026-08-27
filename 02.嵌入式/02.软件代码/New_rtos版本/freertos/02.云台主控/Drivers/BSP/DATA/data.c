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
#include "my_task.h"   /* GIMBAL_IMU_ENABLE */

/*
 * Gimbal main control board -- the PITCH half of the turret.
 *
 * WHAT THIS BOARD DOES, and what it deliberately does not.
 *
 * It is one of four independent receivers on the remote's nRF24L01 broadcast
 * (chassis, gimbal, shooter, referee/lightbar all share one address). It takes
 * the operator frame off the radio, keeps the pitch-relevant part, and relays a
 * frame to the PITCH FOC board over USART3. It never touches yaw -- that link is
 * the chassis board's job -- and it drives no motor itself. Every actuator
 * decision is made downstream on the 4310 board.
 *
 * Mirror-image of 01.底盘主控/Drivers/BSP/DATA/data.c: same 20-byte frame, same
 * PID module, same task skeleton. Three differences are worth knowing because
 * they are the reason the two files are not interchangeable:
 *
 *   1. FRAME CONSTRUCTION. The chassis memcpy's all 20 received bytes and then
 *      overwrites four. This board builds the outgoing frame FIELD BY FIELD and
 *      never copies the radio buffer -- see the note in data_change() for the
 *      failure that motivated it.
 *   2. printf. The chassis has only USART3, which IS its FOC link, so its debug
 *      output must stay off. This board logs to USART2 on separate pins
 *      (usart.c, fputc), so data_print() runs continuously at 10 Hz with no
 *      effect on the FOC frames.
 *   3. IMU ROLE. The chassis IMU measures the body it controls. This board's
 *      BMI055 does NOT ride the pitch axis (see GIMBAL_IMU_ENABLE in my_task.h),
 *      so the attitude PID below is built and maintained but is not in the
 *      command path today.
 *
 * There is no watchdog on this board, unlike the chassis. A hang here leaves the
 * FOC frames stopped, which the 4310 board catches with its own 100 ms comms
 * interlock -- so the failure is contained downstream rather than locally.
 */
#define PI          3.1415926f
#define PI_2        (2 * PI)
#define THRESHOLD   (PI - 0.1f)
/*
 * Tuning constants for the pitch chain. Every one of these exists to keep the
 * FOC board from being handed a setpoint that is really just noise.
 *
 * The chain, in order, is:
 *
 *     BMI055 -> AHRS -> INS_angle_filtered[1]     (measurement)
 *          |
 *          +-> PITCH_ERR_DEADZONE   ignore error below the sensor noise floor
 *          +-> PID_calc             the position loop itself
 *          +-> PID_OUT_SLEW         bound how fast the command may change
 *          +-> PID_OUT_LPF_TAU      smooth what is left
 *          -> Uart_sand_byte[14..15] (command sent to the FOC board)
 *
 * Three filters in series looks excessive until you consider what is on the
 * other end: the FOC board closes a POSITION loop on this value, so any jitter
 * here becomes shaft motion, and the shaft carries a barrel whose inertia turns
 * small commanded steps into visible shake. Each stage removes a different
 * defect -- the dead zone removes standing noise, the slew removes single-sample
 * outliers, the LPF removes the remaining high-frequency content.
 *
 * LIMIT() evaluates its arguments up to three times, so no side effects. It is
 * currently unused; kept because it is referenced by commented-out code.
 *//*
 * Control-angle LPF. 25 ms is slow enough to reject BMI055/motor vibration
 * (~40 Hz and up) but still lets a human stick move through. The old 6 ms
 * tau tracked IMU noise and the FOC board treated that chatter as a position
 * setpoint, which is the twitch you feel.
 */
#define LPF_TAU             0.025f
#define PID_OUT_LPF_TAU     0.030f
/* ~0.9 deg. Still-test peak-peak was 0.78 deg; below this the motor must sit. */
#define PITCH_ERR_DEADZONE  0.015f
/* rad/s. Stops a single bad IMU/NRF sample from kicking the shaft. */
#define PID_OUT_SLEW        1.5f
#define NRF_HOLD_MS         200u
#define FOC_FRAME_HEAD      0x55u
#define FOC_FRAME_TAIL      0xFFu
#define LIMIT(val, min, max) (((val) < (min)) ? (min) : (((val) > (max)) ? (max) : (val)))

/*
 * Attitude pipeline, written by imu_updata() in imu.c:
 *
 *   BMI055 gyro/accel --Mahony--> INS_quat --get_angle--> INS_angle (wrapped)
 *     --process_continuous_angle--> INS_continuous_angle (unwrapped)
 *     --low_pass_filter--> INS_angle_filtered
 *
 * Index 0 = yaw, 1 = pitch, 2 = roll. Only INS_angle_filtered[1] (pitch) is used
 * for control here; the rest are telemetry. mag[] stays zero unless
 * IMU_HAS_MAGNETOMETER is set, and the IST8310 is not fitted on this build.
 */
float last_angle[3] = {0.0f, 0.0f, 0.0f};
float INS_angle_filtered[3] = {0.0f, 0.0f, 0.0f};
float angle_offset[3] = {0.0f, 0.0f, 0.0f};
float INS_continuous_angle[3] = {0.0f, 0.0f, 0.0f};
float angle_lpf[3] = {0.0f, 0.0f, 0.0f};   /* filter state for the three angles */
float gyro[3];
float accel[3];
float mag[3];
float temp[2];
float INS_quat[4] = {0.0f, 0.0f, 0.0f, 0.0f};
float INS_angle[3] = {0.0f, 0.0f, 0.0f};    
/*************************/
int8_t joy[5] = {0};
uint8_t B_rx_A_buf[32] = {0};
uint8_t Uart_sand_byte[20] = {0};
uint8_t Uart_recv_byte[20] = {0};
pid_type_def pitch_pos_pid;

float pitch_position[3] = {0.3f, 0.0f, 0.01f};
float pitch_val[3] = {0.0f,0.0f,0.0f};
volatile uint32_t nrf_last_ms = 0;

static float pid_out_lpf = 0.0f;
static uint8_t foc_dma_buf[20];

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

/*
 * Rate limiter: move `prev` towards `want` by at most PID_OUT_SLEW per second.
 *
 * Returns the new value rather than mutating, so the caller controls when the
 * state advances -- which matters in pid_calculate(), where the same helper is
 * used both to slew towards a real PID output and to slew towards zero on the
 * sensor-fault path.
 *
 * The dt floor of 0.1 ms is not cosmetic. dt comes from imu_dt, which is
 * measured, and a measured interval can legitimately read as zero if two IMU
 * updates land inside one timer tick. max_step would then be 0 and the output
 * would freeze permanently at its current value -- a stuck barrel that looks
 * like a mechanical jam. Clamping dt low instead makes the worst case "moves
 * slower than intended for one cycle".
 */
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

/*
 * One loop on this board: pitch position.
 *
 * max_out 1.2 and max_iout 0.4 are in RADIANS, because that is how the FOC board
 * interprets bytes 14/15 -- an angle command, not a voltage or a rate. 1.2 rad is
 * about 69 degrees, comfortably outside the roughly 50-degree mechanical window,
 * so this limit is a sanity bound rather than the travel limit; the real ends are
 * enforced on the 4310 board from its AS5600 absolute reading. max_iout at a
 * third of max_out keeps the integrator from being able to command full travel on
 * its own.
 *
 * Header and tail are stamped once here rather than every cycle. data_change()
 * re-stamps them anyway, which is deliberate belt-and-braces: if this board ever
 * transmits before its first control cycle, the frame is still well-formed and
 * the FOC board stays locked instead of dropping to its failsafe.
 */
void data_init(void)
{
	/* Soft limits: FOC treats this output as radians around 0.2 rad centre. */
	PID_init(&pitch_pos_pid, PID_POSITION, pitch_position, 1.2f, 0.4f);
	Uart_sand_byte[0] = FOC_FRAME_HEAD;
	Uart_sand_byte[19] = FOC_FRAME_TAIL;
}

/*
 * Pitch position loop, run every 1 ms from data_get_task.
 *
 * SIGN CONVENTION. err = measurement - target, i.e. the NEGATIVE of the textbook
 * definition, and PID_calc() computes the same thing (its signature is
 * PID_calc(pid, ref, set) with error = set - ref, so passing target first and
 * measurement second yields measurement - target). This is consistent across
 * every board in the project -- the chassis does exactly the same in its
 * pid_calculate() -- so it is a house convention, not a slip. The compensating
 * sign lives in the actuator direction downstream. Do not "correct" it on one
 * board only.
 *
 * target = -pitch_val[0] because the remote's ly stick and the pitch axis have
 * opposite senses; pushing the stick forward should lower the barrel.
 *
 * THE SENSOR-FAULT PATH comes first for a reason. On a dead IMU the loop is
 * cleared and the output is SLEWED to zero rather than snapped there. Snapping
 * would hand the FOC board a step change of up to 1.2 rad, which it would
 * faithfully execute as a fast barrel swing at the exact moment the system has
 * just lost its attitude reference. Ramping down at PID_OUT_SLEW takes about
 * 0.8 s to cover the full range instead.
 *
 * THE DEAD ZONE branch does something subtler than skipping the calculation: it
 * also zeroes Iout and error[0]. Without that, an error sitting just under the
 * threshold would keep accumulating in the integrator while the output was held
 * frozen, and the moment the error crossed the threshold the loop would dump the
 * whole accumulated integral at once. Holding `raw` at the previous output makes
 * the dead zone a true hold rather than a gap.
 */
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
	/*
	 * Hold the last command while the error is inside the IMU noise floor.
	 * Recalculating every 1 ms on a 0.15 deg wobble makes the FOC shaft
	 * chase a moving target that is not a real motion request.
	 */
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

/*
 * Rebuild the outgoing FOC frame from the latest radio data. Runs every 1 ms.
 *
 * Bytes 1..11 are the operator's own fields, relayed unchanged: keys, both knobs,
 * both sticks. The pitch board needs them because it has no radio of its own.
 * Bytes 12/13 carry this board's measured pitch, 14/15 the pitch command.
 *
 * ZEROING versus FREEZING on radio loss -- this board and the chassis do OPPOSITE
 * things, and both are right for their own link:
 *
 *   here (gimbal)  the fields are ZEROED. Byte 14/15 is an ABSOLUTE angle for the
 *                  pitch axis, so zero means "return to the neutral angle", which
 *                  is a safe, defined pose for a barrel.
 *   chassis        the frame is left FROZEN. Its byte 14/15 is an INCREMENTAL yaw
 *                  command, so repeating the last value yields a delta of zero and
 *                  the yaw axis holds position. Zeroing there would read as one
 *                  large negative increment and slew the turret.
 *
 * Same two bytes, opposite failsafe, because the axis downstream interprets them
 * differently. Getting this backwards on either board produces a turret that
 * lunges the moment the radio drops.
 *
 * Note the frame keeps being transmitted either way. Stopping transmission would
 * trip the FOC board's 100 ms interlock and de-energise the axis, letting the
 * barrel fall under gravity -- worse than holding it at a known angle.
 */
void data_change(void)
{
	usart_poll();

	/*
	 * Build a FOC frame from scratch. Never memcpy the NRF buffer.
	 *
	 * FOC frame_sync() only accepts 0x55 ... 0xFF. A missed NRF packet
	 * leaves B_rx_A_buf at 0, the copy loses the header, the joint board
	 * drops lock, and after 100 ms it de-energises. The next good frame
	 * slams the motor back on — that is the twitch.
	 */
	Uart_sand_byte[0] = FOC_FRAME_HEAD;
	if (nrf_age_ms() <= NRF_HOLD_MS)
	{
		Uart_sand_byte[1] = B_rx_A_buf[1];
		Uart_sand_byte[2] = B_rx_A_buf[2];
		Uart_sand_byte[3] = B_rx_A_buf[3];
		Uart_sand_byte[4] = B_rx_A_buf[4];
		Uart_sand_byte[5] = B_rx_A_buf[5];
		Uart_sand_byte[6] = B_rx_A_buf[6];
		Uart_sand_byte[7] = B_rx_A_buf[7];
		Uart_sand_byte[8] = B_rx_A_buf[8];
		Uart_sand_byte[9] = B_rx_A_buf[9];
		Uart_sand_byte[10] = B_rx_A_buf[10];
		Uart_sand_byte[11] = B_rx_A_buf[11];
		pitch_val[0] = two_uint8_to_float_signed(B_rx_A_buf[6], B_rx_A_buf[7]);
	}
	else
	{
		Uart_sand_byte[1] = 0;
		Uart_sand_byte[2] = 0;
		Uart_sand_byte[3] = 0;
		Uart_sand_byte[4] = 0;
		Uart_sand_byte[5] = 0;
		Uart_sand_byte[6] = 0;
		Uart_sand_byte[7] = 0;
		Uart_sand_byte[8] = 0;
		Uart_sand_byte[9] = 0;
		Uart_sand_byte[10] = 0;
		Uart_sand_byte[11] = 0;
		pitch_val[0] = 0.0f;
	}

	float_to_two_uint8_signed(INS_angle_filtered[1], &Uart_sand_byte[12], &Uart_sand_byte[13]);
#if GIMBAL_IMU_ENABLE
	float_to_two_uint8_signed(pitch_val[1], &Uart_sand_byte[14], &Uart_sand_byte[15]);
#else
	/*
	 * Pass ly bytes through unchanged. The remote now uses 0.001 rad
	 * LSBs; a decode/re-encode at 0.01 rad would throw that away.
	 */
	Uart_sand_byte[14] = B_rx_A_buf[6];
	Uart_sand_byte[15] = B_rx_A_buf[7];
#endif
	Uart_sand_byte[16] = 0;
	Uart_sand_byte[17] = 0;
	Uart_sand_byte[18] = 0;
	Uart_sand_byte[19] = FOC_FRAME_TAIL;

	joy[0] = (int8_t)Uart_sand_byte[1];
	joy[1] = (int8_t)Uart_sand_byte[2];
	joy[2] = (int8_t)Uart_sand_byte[3];
	pitch_val[2] = (float)normalizeAngleRad(two_uint8_to_float_signed(Uart_recv_byte[1], Uart_recv_byte[2]));
}

/*
 * Hand the frame to the TX DMA, skipping the cycle if the previous transfer is
 * still in flight.
 *
 * NOTE, and this is a genuine difference from the chassis: the memcpy below is
 * NOT protected by a critical section. data_change() and uart_sand() are called
 * back to back from the same task here (data_exchange_task), so in the current
 * arrangement nothing can preempt between them and the copy is atomic by
 * construction. That safety is a property of the task layout, not of this code --
 * move uart_sand() into its own task, as the chassis does, and this becomes a
 * torn-frame bug: the copy could take bytes 0..7 from one control cycle and 8..19
 * from the next, which across the 14/15 pair is a pitch command the axis was
 * never asked for. The chassis version wraps the same memcpy in
 * taskENTER_CRITICAL() for exactly that reason.
 */
void uart_sand(void)
{
	/* 20 bytes @ 115200 takes ~1.74 ms. Skip rather than tear a live DMA. */
	if (DMA1_Stream3->CR & DMA_SxCR_EN)
	{
		return;
	}
	memcpy(foc_dma_buf, Uart_sand_byte, sizeof(foc_dma_buf));
	usart3_dma_send(foc_dma_buf, (uint16_t)sizeof(foc_dma_buf));
}

/*
 * Telemetry, 10 Hz, from data_get_task.
 *
 * Safe to leave permanently enabled on this board because printf goes to USART2
 * on its own pins (usart.c, fputc), not to the USART3 FOC link. The chassis has
 * no such luxury -- one port, shared with the FOC board -- which is why its
 * equivalent is gated behind CHASSIS_UART3_DEBUG_ENABLE and left off.
 *
 * The fields, and what each one tells you when the turret misbehaves:
 *   p    measured pitch, degrees        -- is the AHRS producing sane attitude?
 *   tgt  commanded pitch, degrees       -- is the operator's stick arriving?
 *   pid  loop output, radians           -- is the loop responding to the error?
 *   enc  the FOC board's reported angle -- is the axis actually following?
 *   nrf  ms since the last radio frame  -- is the link alive?
 *   h    IMU health flag
 *   dt   measured loop period, seconds  -- is the 1 ms schedule being met?
 *
 * Together they separate "the operator's command never arrived" from "the loop
 * ignored it" from "the axis refused to move", which are otherwise the same
 * symptom: a barrel that does not point where the stick says.
 */
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

/**
 * @brief Pack a float into two big-endian bytes, scaled by 100 (0.01 LSB).
 *
 * Wire format for bytes 12/13 of the frame. Clamped, not wrapped: a wrapped
 * angle would decode to a plausible value pointing somewhere else entirely.
 *
 * @param num        value to pack, clamped to +-327.67
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
 * The int16_t cast is what makes negatives work -- (high << 8) | low is a
 * non-negative int, and the cast reinterprets the top bit as a sign. Without it
 * -1 decodes as +655.35.
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
 * @brief Unwrap a wrapped angle into a continuous one, per axis.
 *
 * Identical to the chassis version. The AHRS folds Euler angles into [-PI, PI];
 * a shaft crossing the cut jumps by nearly 2*PI, which a PID would read as a
 * huge error step. Any delta beyond THRESHOLD is treated as that wrap and
 * compensated in angle_offset[].
 *
 * State is per axis in last_angle[]/angle_offset[], so `axis` selects the
 * history: 0 = yaw, 1 = pitch, 2 = roll, matching INS_angle[].
 *
 * NOT USED IN THE COMMAND PATH on this board -- pid_calculate() closes on
 * INS_angle_filtered[1], which imu.c derives from the wrapped INS_angle. That is
 * acceptable for pitch specifically, because pitch is mechanically limited to
 * about 50 degrees and can never approach the +-PI branch cut. It would be wrong
 * for yaw, which is exactly why the chassis (which does own yaw) routes its
 * angle through here first.
 *
 * @param axis  0 = yaw, 1 = pitch, 2 = roll
 * @param curr  freshly wrapped angle for that axis
 * @retval the continuous angle
 */
	float process_continuous_angle(uint8_t axis, float curr) {
    float delta = curr - last_angle[axis];
    if(delta < -THRESHOLD) angle_offset[axis] += PI_2;
    else if(delta > THRESHOLD) angle_offset[axis] -= PI_2;
    last_angle[axis] = curr;
    return curr + angle_offset[axis];
}

/**
 * @brief First-order low-pass with a TIME-CONSTANT parameterisation.
 *
 *     alpha = dt / (LPF_TAU + dt);   y += alpha * (x - y)
 *
 * Different from the chassis version, which hard-codes a fixed alpha (LPF_ALPHA
 * 0.15) and so silently changes its cutoff if the loop rate ever moves. Deriving
 * alpha from the MEASURED dt each call fixes the corner frequency at
 * 1/(2*pi*LPF_TAU) = 6.4 Hz regardless of scheduling jitter, which is what makes
 * it safe to feed a position loop downstream.
 *
 * The dt floor of 0.1 ms guards the same failure as slew_limit(): a measured dt
 * of zero would give alpha = 0 and freeze the filter output forever.
 *
 * @param input        new sample
 * @param prev_output  in/out: filter state, one per signal
 * @param dt           measured interval since the last call, seconds
 * @retval the filtered value (also stored through prev_output)
 */
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
/*
 * Byte-at-a-time frame synchroniser for the FOC board's reply. Fed from
 * usart_poll(), which drains the circular RX DMA.
 *
 * A 20-byte sliding window: after the write index advances it points at the
 * OLDEST byte, so ring[w_idx] is the candidate header and ring[w_idx+19] the
 * candidate tail. Clearing count on a hit consumes the window, so one frame is
 * never delivered twice.
 *
 * The header/tail pair is the only validation on this link -- no CRC, no
 * sequence -- so any 20 bytes that happen to start 0x55 and end 0xFF are
 * accepted. With a fixed 20-byte frame arriving continuously that is adequate;
 * it would not be on a link carrying variable-length traffic.
 *
 * DIFFERENCE FROM THE CHASSIS: that board stamps a timestamp here
 * (s_yaw_last_ms) so it can tell whether the reply is still fresh, and refuses
 * to steer on a stale angle. This board does not, so pitch_val[2] -- the encoder
 * angle decoded in data_change() -- silently keeps its last value if the pitch
 * board goes quiet, and an absent board reads as a valid 0.00 rad. Harmless
 * today because pitch_val[2] is only printed, never acted on; it would need the
 * same freshness guard before anything closed a loop on it.
 */
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
            return 1;
        }
    }
    return 0;
}
/*
 * Fold an angle by one revolution.
 *
 * WARNING -- this DOES NOT FOLD to [-PI, PI], despite the name and despite the
 * chassis function of the same name doing exactly that. fmod() alone returns a
 * value in (-2*PI, 2*PI) that keeps the sign of its input: feed it 4.0 rad and
 * you get 4.0 rad back, not the equivalent -2.28 rad.
 *
 * Also note it is `double` here and `float` on the chassis, so every call drags
 * in the double-precision library routines on a single-precision FPU -- fmod on
 * an M4F is a software call, not an instruction.
 *
 * Currently harmless: the only caller is data_change(), where the result lands in
 * pitch_val[2] and is printed but never used for control, and a pitch axis
 * limited to ~50 degrees never wraps in the first place. It becomes a real defect
 * the moment anything compares this value against a target, since the same
 * physical angle can come back as two different numbers.
 */
double normalizeAngleRad(double angle)
{
    const double two_pi = 2 * PI;
    return fmod(angle, two_pi);
}
