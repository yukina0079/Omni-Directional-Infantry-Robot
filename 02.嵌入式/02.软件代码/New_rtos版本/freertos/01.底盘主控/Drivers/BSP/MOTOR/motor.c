#include "motor.h"
#include "stdlib.h"
/*
	MOTOR1;PB8, PB9, PC9-TIM8_CH4 右上
	MOTOR2;PE0, PE1, PC8-TIM8_CH3 右下
	MOTOR3;PE12,PE13,PC7-TIM8_CH2 左上
	MOTOR4;PE14,PE15,PC6-TIM8_CH1 左下
	Tout=((arr+1)*(psc+1))/168Mhz
*/

/*
 * Chassis actuator layer, in full. Everything below drives two TB6612FNG dual
 * H-bridges (U39 and U40 on the chassis schematic) which in turn drive four
 * BRUSHED geared motors -- not CAN-bus smart motors. There is no current loop,
 * no velocity loop and no position loop anywhere in the chassis firmware: the
 * value handed to MOTOR_PWM_UPDATE() becomes a duty cycle directly, so the
 * wheels run OPEN LOOP.
 *
 * That last point is worth stating plainly because the hardware suggests
 * otherwise: the schematic carries four quadrature encoder headers (U58..U61,
 * wired to TIM1/TIM2/TIM3/TIM4), and nothing in this firmware reads any of
 * them. Wheel speed is commanded, never measured. A wheel that stalls, slips or
 * meets a slope simply does not do what it was told, and no layer notices.
 *
 * Per-channel wiring, verified against the schematic net list:
 *
 *   motor  IN1/IN2 (direction)   PWM pin   TIM8 channel   set by
 *   -----  --------------------  --------  -------------  ------------------
 *     1    PB8 / PB9             PC9       CH4            motor1_speed()
 *     2    PE0 / PE1             PC8       CH3            motor2_speed()
 *     3    PE12 / PE13           PC7       CH2            motor3_speed()
 *     4    PE14 / PE15           PC6       CH1            motor4_speed()
 *
 * Note the channel order runs BACKWARDS against the motor numbering: motor 1 is
 * on CH4 and motor 4 is on CH1. That is not a mistake to be tidied up -- it is
 * what the PCB routes -- but it means the TIM8 capture/compare register you
 * watch in a debugger is never the one whose number matches the motor.
 *
 * Each TB6612 half-bridge takes IN1, IN2 and PWM. The IN pair selects
 * direction (or brake/coast) and the PWM pin gates the output stage, which is
 * why direction is plain GPIO here and only speed goes through the timer.
 */
#define MOTOR_GPIO1_PORT GPIOE
#define MOTOR_GPIO2_PORT GPIOB

#define MOTOR1_GPIO_P1 GPIO_PIN_8
#define MOTOR1_GPIO_P2 GPIO_PIN_9

#define MOTOR2_GPIO_P1 GPIO_PIN_0
#define MOTOR2_GPIO_P2 GPIO_PIN_1

#define MOTOR3_GPIO_P1 GPIO_PIN_12
#define MOTOR3_GPIO_P2 GPIO_PIN_13

#define MOTOR4_GPIO_P1 GPIO_PIN_14
#define MOTOR4_GPIO_P2 GPIO_PIN_15

TIM_HandleTypeDef pwm_handle = {0};			//定义定时器参数结构体

//初始化GPIO口
/*
 * Direction pins only -- the PWM pins are handled by HAL_TIM_PWM_MspInit().
 *
 * Note the port split, which follows the PCB rather than any logic: motor 1's
 * pair is on GPIOB (PB8/PB9) while motors 2..4 are all on GPIOE. Hence the two
 * separate HAL_GPIO_Init() calls and the two *_PORT macros.
 *
 * GPIO_PULLUP on an output is harmless but meaningless -- the push-pull driver
 * wins over the weak pull the instant it is enabled. What it does buy is a
 * defined level in the window between the clock enable and the first
 * HAL_GPIO_WritePin(): both IN pins float high, which on a TB6612 is
 * IN1=IN2=H, i.e. short brake. Given the PWM channels are still gated off at
 * this point (pwm_init sets Pulse = 0) the bridges are inert either way, but
 * the ordering in main() -- motor_init() before pwm_init() -- means the
 * direction pins are never left genuinely undriven.
 */
void motor_init(void)
{
		GPIO_InitTypeDef gpio_initstruct;
	
		__HAL_RCC_GPIOE_CLK_ENABLE();
		gpio_initstruct.Pin = MOTOR3_GPIO_P1|MOTOR3_GPIO_P2|MOTOR2_GPIO_P1|MOTOR2_GPIO_P2|MOTOR4_GPIO_P1|MOTOR4_GPIO_P2;
		gpio_initstruct.Mode = GPIO_MODE_OUTPUT_PP;
		gpio_initstruct.Pull = GPIO_PULLUP;
		gpio_initstruct.Speed = GPIO_SPEED_FREQ_HIGH;
		HAL_GPIO_Init(MOTOR_GPIO1_PORT,&gpio_initstruct);
	
		__HAL_RCC_GPIOB_CLK_ENABLE();
		gpio_initstruct.Pin = MOTOR1_GPIO_P1|MOTOR1_GPIO_P2;
		gpio_initstruct.Mode = GPIO_MODE_OUTPUT_PP;
		gpio_initstruct.Pull = GPIO_PULLUP;
		gpio_initstruct.Speed = GPIO_SPEED_FREQ_HIGH;
		HAL_GPIO_Init(MOTOR_GPIO2_PORT,&gpio_initstruct);
}

/*
 * Configure TIM8 as a 4-channel edge-aligned PWM generator and start all four.
 *
 * Called from main() as pwm_init(16799, 0). Working that through:
 *
 *   TIM8 sits on APB2. With sys_stm32_clock_init(336, 8, 2, 7) the SYSCLK is
 *   8 MHz / 8 * 336 / 2 = 168 MHz, APB2 is SYSCLK/2 = 84 MHz, and the APB2
 *   TIMER clock is doubled back to 168 MHz. So
 *
 *       f_pwm = 168 MHz / ((arr + 1) * (psc + 1)) = 168e6 / 16800 = 10 kHz
 *
 *   which is the frequency the file header's Tout formula describes. 10 kHz is
 *   a deliberate choice for brushed motors: above the ~8 kHz audible ceiling so
 *   the chassis does not whine, and still far below the TB6612's 100 kHz limit.
 *
 *   Full scale is therefore 16799. data.c never commands more than
 *   data_kp * MAX_SPEED = 120 * 100 = 12000, i.e. 71.4% duty, so roughly a
 *   quarter of the available output is unreachable by design. If the chassis
 *   ever needs the rest, raise MAX_SPEED in data.h to 140 -- do NOT raise
 *   data_kp past 168, because MOTOR_PWM_UPDATE() does not clamp and a compare
 *   value above ARR means the channel never matches and the output pins high
 *   for the whole period.
 *
 * TIM_OCMODE_PWM1 with TIM_OCPOLARITY_HIGH gives the usual sense: output is
 * active while CNT < CCR, so a larger compare value is a longer on-time.
 *
 * Pulse starts at 0 on every channel, which matters for safety: between this
 * call and the first MOTOR_PWM_UPDATE() the bridges are gated off regardless of
 * whatever state the direction pins happen to be in.
 */
void pwm_init(uint16_t arr,uint16_t psc)
{
		TIM_OC_InitTypeDef pwm_config = {0};
	
		pwm_handle.Instance = TIM8;					//定时器通道
		pwm_handle.Init.Prescaler = psc;
		pwm_handle.Init.Period = arr;
		pwm_handle.Init.CounterMode = TIM_COUNTERMODE_UP;//向上计数
		
		HAL_TIM_PWM_Init(&pwm_handle);
		
		pwm_config.OCMode = TIM_OCMODE_PWM1; //通用定时器
		pwm_config.Pulse  = 0;
		pwm_config.OCPolarity = TIM_OCPOLARITY_HIGH;//计数方式
		
		HAL_TIM_PWM_ConfigChannel(&pwm_handle,&pwm_config,TIM_CHANNEL_1);
		HAL_TIM_PWM_ConfigChannel(&pwm_handle,&pwm_config,TIM_CHANNEL_2);
		HAL_TIM_PWM_ConfigChannel(&pwm_handle,&pwm_config,TIM_CHANNEL_3);
		HAL_TIM_PWM_ConfigChannel(&pwm_handle,&pwm_config,TIM_CHANNEL_4);
		
		HAL_TIM_PWM_Start(&pwm_handle,TIM_CHANNEL_1); 
		HAL_TIM_PWM_Start(&pwm_handle,TIM_CHANNEL_2); 
		HAL_TIM_PWM_Start(&pwm_handle,TIM_CHANNEL_3); 
		HAL_TIM_PWM_Start(&pwm_handle,TIM_CHANNEL_4); 
		
}
/*
 * HAL calls this from inside HAL_TIM_PWM_Init(); it is the weak hook where the
 * peripheral's clocks and pins get set up. Two things it implies:
 *
 *   - pwm_init() must run after the system clock is configured, because the
 *     GPIO and TIM8 clock enables here derive from it.
 *   - This is a SHARED hook. Any other TIM used for PWM on this board would
 *     land in the same function, which is why the body is guarded on
 *     htim->Instance. (The chassis has exactly one: dma.c configures TIM3_CH4
 *     for a DMA-fed waveform and provides its own MspInit for it.)
 *
 * GPIO_AF3_TIM8 is the alternate-function number that maps TIM8's channels onto
 * PC6..PC9 on an F407. Getting this wrong is silent: the pins configure as
 * alternate-function push-pull either way, they just carry a different
 * peripheral's signal (AF2 would give TIM3/TIM4), so the motors would sit dead
 * with no error anywhere. Worth remembering when reading the schematic, which
 * labels these nets TIM8_CH1..TIM8_CH4 by function rather than by AF number.
 */
void HAL_TIM_PWM_MspInit(TIM_HandleTypeDef *htim)
{
    if(htim->Instance == TIM8){
        GPIO_InitTypeDef gpio_initstruct;

        __HAL_RCC_TIM8_CLK_ENABLE();
        __HAL_RCC_GPIOC_CLK_ENABLE();

        gpio_initstruct.Pin = GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_9;
        gpio_initstruct.Mode = GPIO_MODE_AF_PP;
        gpio_initstruct.Pull = GPIO_PULLUP;
        gpio_initstruct.Speed = GPIO_SPEED_FREQ_HIGH;
        gpio_initstruct.Alternate = GPIO_AF3_TIM8; // 【关键修复】设置复用功能为TIM8

        HAL_GPIO_Init(GPIOC, &gpio_initstruct);
    }
}

/*
 * Direction select for each channel. i == 1 and i == 0 are the two directions;
 * any other argument is silently ignored, because these are two independent
 * `if`s rather than an if/else.
 *
 * TB6612FNG truth table for one half-bridge, with PWM held high:
 *
 *   IN1  IN2   output
 *   ---  ---   ---------------------------
 *    L    H    reverse  (this is i == 1)
 *    H    L    forward  (this is i == 0)
 *    L    L    coast (both transistors off, motor free-wheels)
 *    H    H    short brake (both low sides on, winding shorted)
 *
 * and with PWM held LOW the output is short brake regardless of IN1/IN2.
 *
 * Which physical rotation "forward" corresponds to is a property of the motor
 * wiring and the gearbox, not of this code, and it differs per wheel: the
 * per-wheel signs are baked into OmniKinematics() in data.c. Do not try to fix
 * a wheel that spins the wrong way by editing this function -- it is shared by
 * both directions of the same wheel, so inverting it here just moves the
 * problem. Swap the sign in the kinematics, or swap the motor's two power
 * leads.
 *
 * These functions are intentionally not declared in motor.h: nothing outside
 * this file should command a single wheel's direction without also setting its
 * speed. Use motorN_move() or MOTOR_PWM_UPDATE().
 */
void motor1(int i)
{
	if(i == 1){
		HAL_GPIO_WritePin(MOTOR_GPIO2_PORT,MOTOR1_GPIO_P1,GPIO_PIN_RESET);
		HAL_GPIO_WritePin(MOTOR_GPIO2_PORT,MOTOR1_GPIO_P2,GPIO_PIN_SET);
	}	
	if(i == 0){
		HAL_GPIO_WritePin(MOTOR_GPIO2_PORT,MOTOR1_GPIO_P1,GPIO_PIN_SET);
		HAL_GPIO_WritePin(MOTOR_GPIO2_PORT,MOTOR1_GPIO_P2,GPIO_PIN_RESET);	
	}
}
void motor2(int i)
{
	if(i == 1){
		HAL_GPIO_WritePin(MOTOR_GPIO1_PORT,MOTOR2_GPIO_P1,GPIO_PIN_RESET);
		HAL_GPIO_WritePin(MOTOR_GPIO1_PORT,MOTOR2_GPIO_P2,GPIO_PIN_SET);
	}	
	if(i == 0){
		HAL_GPIO_WritePin(MOTOR_GPIO1_PORT,MOTOR2_GPIO_P1,GPIO_PIN_SET);
		HAL_GPIO_WritePin(MOTOR_GPIO1_PORT,MOTOR2_GPIO_P2,GPIO_PIN_RESET);	
	}
}
void motor3(int i)
{
	if(i == 1){
		HAL_GPIO_WritePin(MOTOR_GPIO1_PORT,MOTOR3_GPIO_P1,GPIO_PIN_RESET);
		HAL_GPIO_WritePin(MOTOR_GPIO1_PORT,MOTOR3_GPIO_P2,GPIO_PIN_SET);
	}	
	if(i == 0){
		HAL_GPIO_WritePin(MOTOR_GPIO1_PORT,MOTOR3_GPIO_P1,GPIO_PIN_SET);
		HAL_GPIO_WritePin(MOTOR_GPIO1_PORT,MOTOR3_GPIO_P2,GPIO_PIN_RESET);	
	}
}
void motor4(int i)
{
	if(i == 1){
		HAL_GPIO_WritePin(MOTOR_GPIO1_PORT,MOTOR4_GPIO_P1,GPIO_PIN_RESET);
		HAL_GPIO_WritePin(MOTOR_GPIO1_PORT,MOTOR4_GPIO_P2,GPIO_PIN_SET);
	}	
	if(i == 0){
		HAL_GPIO_WritePin(MOTOR_GPIO1_PORT,MOTOR4_GPIO_P1,GPIO_PIN_SET);
		HAL_GPIO_WritePin(MOTOR_GPIO1_PORT,MOTOR4_GPIO_P2,GPIO_PIN_RESET);	
	}
}


/*
 * Magnitude only, written straight into the channel's compare register.
 *
 * NOT clamped. val must stay <= ARR (16799 as configured in main()); a larger
 * value means CNT never reaches CCR, the compare never fires, and the channel
 * sits at 100% instead of the intended duty. The bound is enforced upstream, by
 * MAX_SPEED and data_kp in data.c -- there is nothing here that would catch a
 * mistake made there.
 *
 * __HAL_TIM_SET_COMPARE writes CCRx directly with no preload, so a new value
 * takes effect the moment it lands. Mid-period that shortens or lengthens the
 * pulse currently being generated, which is a one-cycle glitch of at most 100 us
 * at 10 kHz. Harmless on a geared brushed motor whose mechanical time constant
 * is three orders of magnitude longer; it would matter on a three-phase bridge,
 * which is why the FOC boards use preloaded compare registers instead.
 *
 * Remember the channel numbering is reversed with respect to the motor
 * numbering -- see the table in the file header.
 */
void motor1_speed(uint16_t val)
{
			__HAL_TIM_SET_COMPARE(&pwm_handle,TIM_CHANNEL_4,val);
}
void motor2_speed(uint16_t val)
{
			__HAL_TIM_SET_COMPARE(&pwm_handle,TIM_CHANNEL_3,val);
}
void motor3_speed(uint16_t val)
{
			__HAL_TIM_SET_COMPARE(&pwm_handle,TIM_CHANNEL_2,val);
}
void motor4_speed(uint16_t val)
{
			__HAL_TIM_SET_COMPARE(&pwm_handle,TIM_CHANNEL_1,val);
}


/*
 * Signed command for one wheel: sign selects direction, magnitude becomes duty.
 *
 * val == 0 is the case worth understanding. Neither `if` fires, so the
 * direction pins KEEP their previous state, and the duty is set to 0. Per the
 * TB6612 table, PWM low is short brake -- so a commanded zero is an actively
 * braked wheel, not a coasting one. That is the right default for a combat
 * chassis (it resists being pushed) and it is what makes the failsafe path in
 * data.c effective: OmniKinematics(0,0,0) does not merely stop driving, it
 * clamps the wheels.
 *
 * Two rough edges, both deliberate and both left alone:
 *
 *   - No dead time on reversal. Direction is written BEFORE the new compare
 *     value, so for the handful of cycles in between, the OLD duty is applied in
 *     the NEW direction. A full-scale reversal in one 1 ms control tick
 *     therefore drives the motor hard against its own rotation, and the current
 *     is limited only by winding resistance and the TB6612's own protection.
 *     Reaching that state needs the stick slammed from one stop to the other, so
 *     it is rare rather than impossible. The fix, if it is ever wanted, is a
 *     slew limit in data.c -- not a delay here, which would block the task.
 *   - abs() on an int is undefined for INT_MIN. Unreachable in practice: every
 *     value arriving here has passed through the +/-MAX_SPEED clamp in
 *     OmniKinematics() and then a multiply by data_kp, giving +/-12000.
 */
void motor1_move(int val)
{
		if(val>0){motor1(1);}
		if(val<0){motor1(0);}
		motor1_speed(abs(val));
}
void motor2_move(int val)
{
		if(val>0){motor2(1);}
		if(val<0){motor2(0);}
		motor2_speed(abs(val));
}
void motor3_move(int val)
{
		if(val>0){motor3(1);}
		if(val<0){motor3(0);}
		motor3_speed(abs(val));
}
void motor4_move(int val)
{
		if(val>0){motor4(1);}
		if(val<0){motor4(0);}
		motor4_speed(abs(val));
}

/*
 * Set all four wheels. The only entry point this module exposes for motion, and
 * the only one motor.h declares.
 *
 * Two callers, for two opposite purposes:
 *   - OmniKinematics()/MecanumKinematics() in data.c, every control cycle;
 *   - the FreeRTOS fault hooks in my_task.c, with (0,0,0,0), to kill the output
 *     stage before parking the CPU. That path is why this function must stay
 *     free of anything that can block or fail -- it runs with interrupts
 *     already disabled and the scheduler in an unknown state.
 *
 * The four updates are NOT atomic with respect to each other. This runs at
 * priority 7 and can be preempted by data_get_task (priority 9), so a control
 * cycle can in principle land on the wheels split across two ticks. The
 * consequence is a sub-millisecond skew between wheel commands -- invisible on a
 * chassis with metal-geared brushed motors, and not worth a critical section on
 * a path that executes every millisecond.
 *
 * Also note the parameter order maps to motors 1..4 and therefore to
 * wheel_speeds[0..3] in data.c, NOT to TIM8 channels 1..4.
 */
void MOTOR_PWM_UPDATE(int val1,int val2,int val3,int val4)
{
	motor1_move(val1);
	
	motor2_move(val2);
	
	motor3_move(val3);
	
	motor4_move(val4);
}






