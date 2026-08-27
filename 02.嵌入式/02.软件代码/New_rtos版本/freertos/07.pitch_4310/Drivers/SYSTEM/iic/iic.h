#ifndef __IIC_H__
#define __IIC_H__
#include "uart1.h"
#include "delay.h"

#define I2C_SCL_CLK()      __HAL_RCC_GPIOB_CLK_ENABLE();
#define I2C_SCL_PORT       GPIOB
#define I2C_SCL2_PIN        GPIO_PIN_10

#define I2C_SDA_CLK()      __HAL_RCC_GPIOB_CLK_ENABLE();
#define I2C_SDA_PORT       GPIOB
#define I2C_SDA2_PIN        GPIO_PIN_11

/*
 * Direct BSRR/IDR access instead of HAL_GPIO_WritePin/ReadPin.
 * HAL_GPIO_WritePin costs ~10 cycles of branching per edge; at the bit-bang
 * rates used here that dominated the transfer time. BSRR is a single atomic
 * store, so the timing below is dictated by the explicit delays, not by
 * incidental HAL overhead.
 */
#define I2C_SCL_RESET()        (I2C_SCL_PORT->BSRR = (uint32_t)I2C_SCL2_PIN << 16u)
#define I2C_SCL_SET()          (I2C_SCL_PORT->BSRR = (uint32_t)I2C_SCL2_PIN)

#define I2C_SDA_RESET()        (I2C_SDA_PORT->BSRR = (uint32_t)I2C_SDA2_PIN << 16u)
#define I2C_SDA_SET()          (I2C_SDA_PORT->BSRR = (uint32_t)I2C_SDA2_PIN)

#define I2C_SDA_VALUE()        ((I2C_SDA_PORT->IDR & I2C_SDA2_PIN) ? 1u : 0u)

/*
 * Both pins stay in open-drain output mode permanently. In open-drain mode
 * writing a 1 releases the line to the external pull-up (R6/R7 = 4.7k on the
 * DRV8313 board) and IDR still reflects the actual bus level, so the slave can
 * drive it low and we read that back. The old SDA_IN()/SDA_OUT() macros called
 * HAL_GPIO_Init() on every single byte to flip between input and output --
 * hundreds of cycles of register read-modify-write per byte, for no benefit.
 */

/* Return codes for the transfer helpers below. */
#define I2C_OK      0u
#define I2C_ERR     1u

/*
 * Iteration count for the half-bit delay.
 *
 * MEASURED on this board (STM32F103 @ 72 MHz, -O2, 4.7k pull-ups):
 *
 *   delay_ns ~= 514 + 111.3 * ticks
 *
 * i.e. ~111 ns (8 CPU cycles) per iteration, plus ~514 ns of fixed call and
 * volatile-stack overhead. The fixed term dominates at low counts, so halving
 * the ticks does NOT halve the bit period.
 *
 *   ticks 24 -> 3182 ns half-bit -> 439 us per encoder read -> 2.3 kHz ceiling
 *   ticks 10 -> 1626 ns half-bit -> 236 us per encoder read -> 4.2 kHz ceiling
 *
 * A margin scan from 24 ticks down to 1 produced zero NACKs and zero data
 * glitches over 500 reads at every step -- this bus stayed healthy even at a
 * 625 ns half-bit (~800 kHz), twice the AS5600's rated 400 kHz fast mode.
 *
 * 10 is chosen rather than the fastest working value: a 1626 ns half-bit is
 * ~307 kHz, still inside the rated fast mode, so the setting rests on the
 * datasheet rather than only on one sample of one board at room temperature.
 * Against the measured no-error point it carries roughly 2.6x of margin.
 */
#define I2C_DELAY_TICKS_DEFAULT   10u

void i2c_init(void);
void i2c_start(void);
void i2c_stop(void);
void i2c_send_byte(uint8_t data);
void i2c_send_ack(uint8_t ack);
uint8_t i2c_receive_byte(void);
uint8_t i2c_receive_ack(void);

/*
 * Bus timing controls, used by the self-test. Adjusting the tick count changes
 * the clock rate of every subsequent transfer, so the margin scan can find the
 * point where this particular board stops acknowledging.
 */
void     i2c_set_delay_ticks(uint32_t ticks);
uint32_t i2c_get_delay_ticks(void);

/* Measured duration of one i2c_delay(), in nanoseconds. Averaged over 10000
 * calls against the TIM2 microsecond counter. */
uint32_t i2c_measure_delay_ns(void);

/*
 * Nine clocks with SDA released, then a STOP. Frees a slave that is stuck
 * holding SDA low after an aborted transfer -- which is what a too-fast clock
 * produces, so the margin scan needs this between attempts.
 */
void i2c_bus_recover(void);

/* All of these return I2C_OK or I2C_ERR; check them. */
uint8_t i2c_write_register(uint8_t i2c_address, uint8_t address, uint8_t data);
uint8_t i2c_read_register(uint8_t i2c_address, uint8_t address, uint8_t *out);
uint8_t i2c_read_len(uint8_t i2c_address, uint8_t reg_addr, uint8_t len, uint8_t *data);

#endif
