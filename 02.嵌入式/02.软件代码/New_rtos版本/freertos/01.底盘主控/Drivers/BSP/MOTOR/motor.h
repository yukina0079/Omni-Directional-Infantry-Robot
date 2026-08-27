#ifndef  __MOTOR_H__
#define  __MOTOR_H__
#include "sys.h"
/*
 * Public surface of the chassis actuator layer. See motor.c for the wiring
 * table, the PWM frequency derivation and the TB6612FNG truth table.
 *
 * Deliberately narrow: the per-wheel helpers (motor1..motor4, motorN_speed,
 * motorN_move) are defined in motor.c but not declared here, so nothing outside
 * that file can set a direction without also setting a duty.
 */
void motor_init(void);
void pwm_init(uint16_t arr,uint16_t psc);
/*
 * DECLARED BUT NEVER DEFINED -- there is no body for any of these three
 * anywhere in the project. They are harmless as long as nobody calls them
 * (a declaration alone emits no code); the first call site turns into an
 * "undefined symbol" at link time, not a compile error, which is an annoying
 * place to discover it.
 *
 * What they were evidently meant for: the TB6612FNG's STBY pin, which must be
 * high for either half-bridge to produce any output at all. No code on this
 * board writes an STBY pin, so on this PCB it has to be strapped active in
 * hardware -- worth confirming against the schematic before concluding that a
 * chassis which refuses to move has a firmware problem.
 *
 * If a software output enable is ever wanted -- and it would be a genuinely
 * useful thing to have, since it kills all four bridges with one write rather
 * than four compare registers -- this is the right place for it. Note the
 * FreeRTOS fault hooks in my_task.c currently have to call
 * MOTOR_PWM_UPDATE(0,0,0,0) instead, which is four function calls deep in a
 * context where interrupts are already off.
 */
void EN_init(void);
void EN_close(void);
void EN_open(void);
void MOTOR_PWM_UPDATE(int val1,int val2,int val3,int val4);

#endif
