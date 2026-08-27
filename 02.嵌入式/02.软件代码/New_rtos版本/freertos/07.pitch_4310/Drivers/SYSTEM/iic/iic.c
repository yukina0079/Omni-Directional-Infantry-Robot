#include "iic.h"
#include "systime.h"

/*
 * Software I2C master for the AS5600 magnetic encoder.
 *
 * Timing: the AS5600 supports fast mode (400 kHz), which requires a minimum
 * SCL high time of 0.6 us and a minimum low time of 1.3 us. The previous code
 * had no delays at all -- SCL toggled in consecutive instructions, giving a
 * high time of tens of nanoseconds at 72 MHz. It only worked because the 4.7k
 * pull-ups plus board capacitance dragged the rising edge out far enough to
 * accidentally satisfy the slave. Changing the pull-up value or lengthening the
 * cable would have broken it.
 *
 * Note that t_LOW is NOT the same in both directions, which is what sets the
 * floor on the tick count:
 *
 *   i2c_send_byte()    SCL low for TWO delays (data setup + setup margin),
 *                      high for one.
 *   i2c_receive_byte() SCL low for ONE delay, high for one.
 *
 * So the receive path is the binding constraint: one i2c_delay() must exceed
 * the 1.3 us minimum low time on its own. At the default tick count it is
 * ~1.63 us. See I2C_DELAY_TICKS_DEFAULT in iic.h for the measured calibration.
 */

/*
 * Half-bit-period delay. The iteration count is a variable (see below) rather
 * than a hardcoded number so that i2c_measure_delay_ns() can report its real
 * duration and the margin scan can walk it down to the failure point.
 *
 * The exact figure does not matter much: I2C specifies MINIMUM high/low times,
 * not maximum, so erring slow only costs throughput. What matters is that there
 * is a delay at all -- the old code toggled SCL in back-to-back instructions
 * and relied on the 4.7k pull-ups' RC slope to accidentally stretch the edge.
 *
 * volatile so -O2 cannot delete the loop as dead code. This is exactly the
 * kind of thing that silently breaks when optimisation is turned on.
 */
/*
 * Iteration count for i2c_delay(). A variable rather than a constant so the
 * margin scan in as5600_selftest() can walk it down and find where this bus
 * actually stops working. With no scope available that empirical failure point
 * is better evidence than a computed edge time: it accounts for this board's
 * pull-ups, this cable and this encoder instead of the datasheet worst case.
 */
static volatile uint32_t s_delay_ticks = I2C_DELAY_TICKS_DEFAULT;

static void i2c_delay(void)
{
    volatile uint32_t i = s_delay_ticks;
    while (i--) {
        /* burn cycles */
    }
}

void i2c_set_delay_ticks(uint32_t ticks)
{
    /*
     * Clamp to 1. Zero does NOT mean "no iterations": the loop above tests the
     * pre-decrement value, so `i = 0` makes `i--` wrap to 0xFFFFFFFF and the
     * delay hangs for minutes.
     */
    s_delay_ticks = (ticks < 1u) ? 1u : ticks;
}

uint32_t i2c_get_delay_ticks(void)
{
    return s_delay_ticks;
}

/*
 * Times i2c_delay() by running it 10000 times against the TIM2 microsecond
 * counter. The counter has 1 us granularity, so averaging over 10000 calls
 * resolves to 0.1 ns -- finer than reading a cursor off a scope screen.
 *
 * Includes the for-loop overhead (a few cycles per iteration), which is
 * negligible next to a delay of ~2 us.
 */
uint32_t i2c_measure_delay_ns(void)
{
    const uint32_t reps = 10000u;
    uint32_t i, t0, t1;

    t0 = micros();
    for (i = 0; i < reps; i++) {
        i2c_delay();
    }
    t1 = micros();

    return ((t1 - t0) * 1000u) / reps;
}

/*
 * Standard I2C bus recovery.
 *
 * If a transfer is aborted part way through a byte -- exactly what a too-fast
 * clock causes during the margin scan -- the slave can be left holding SDA low
 * while it waits to finish shifting out its data. The master cannot then issue
 * a valid START, and the bus stays wedged. Nine clock pulses with SDA released
 * let the slave clock out whatever it had left, and the following STOP resets
 * its state machine.
 */
void i2c_bus_recover(void)
{
    uint8_t i;

    I2C_SDA_SET();
    for (i = 0; i < 9u; i++) {
        I2C_SCL_SET();
        i2c_delay();
        I2C_SCL_RESET();
        i2c_delay();
    }
    i2c_stop();
}

void i2c_init(void)
{
    GPIO_InitTypeDef gpio_initstruct;

    I2C_SCL_CLK();
    I2C_SDA_CLK();

    /*
     * Open-drain output for both lines, permanently. See the comment in iic.h
     * for why SDA is never switched to input mode.
     */
    gpio_initstruct.Pin   = I2C_SCL2_PIN | I2C_SDA2_PIN;
    gpio_initstruct.Mode  = GPIO_MODE_OUTPUT_OD;
    gpio_initstruct.Pull  = GPIO_NOPULL;
    gpio_initstruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOB, &gpio_initstruct);

    /* Idle state: both lines released high. */
    I2C_SCL_SET();
    I2C_SDA_SET();
    i2c_delay();
}

void i2c_start(void)
{
    /* SDA falls while SCL is high. */
    I2C_SDA_SET();
    I2C_SCL_SET();
    i2c_delay();
    I2C_SDA_RESET();
    i2c_delay();
    I2C_SCL_RESET();
    i2c_delay();
}

void i2c_stop(void)
{
    /* SDA rises while SCL is high. */
    I2C_SCL_RESET();
    I2C_SDA_RESET();
    i2c_delay();
    I2C_SCL_SET();
    i2c_delay();
    I2C_SDA_SET();
    i2c_delay();
}

void i2c_send_byte(uint8_t data)
{
    uint8_t i;

    for (i = 0; i < 8; i++) {
        /* Set up the data bit while SCL is low (MSB first). */
        if (data & 0x80u) {
            I2C_SDA_SET();
        } else {
            I2C_SDA_RESET();
        }
        data <<= 1;

        i2c_delay();
        I2C_SCL_SET();      /* slave samples on this rising edge */
        i2c_delay();
        I2C_SCL_RESET();
        i2c_delay();
    }

    /* Release SDA so the slave can drive the ACK bit. */
    I2C_SDA_SET();
}

uint8_t i2c_receive_byte(void)
{
    uint8_t i;
    uint8_t data = 0;

    /* Release SDA -- the slave drives it from here. */
    I2C_SDA_SET();

    for (i = 0; i < 8; i++) {
        data <<= 1;
        I2C_SCL_RESET();
        i2c_delay();
        I2C_SCL_SET();
        i2c_delay();
        if (I2C_SDA_VALUE()) {
            data |= 0x01u;
        }
    }

    I2C_SCL_RESET();
    i2c_delay();
    return data;
}

/*
 * Reads the ACK bit the slave places on SDA after a byte.
 * Returns I2C_OK when the slave acknowledged (SDA pulled low), I2C_ERR on NACK.
 *
 * Every caller MUST check this. Previously the return value was discarded
 * everywhere, so a disconnected or wedged encoder was indistinguishable from a
 * healthy one and the FOC loop kept driving PWM from a stale angle.
 */
uint8_t i2c_receive_ack(void)
{
    uint8_t ack;

    I2C_SDA_SET();          /* release SDA for the slave */
    i2c_delay();
    I2C_SCL_SET();
    i2c_delay();

    ack = I2C_SDA_VALUE() ? I2C_ERR : I2C_OK;

    I2C_SCL_RESET();
    i2c_delay();
    return ack;
}

/* ack = 0 -> ACK (continue reading), ack = 1 -> NACK (last byte). */
void i2c_send_ack(uint8_t ack)
{
    if (ack) {
        I2C_SDA_SET();
    } else {
        I2C_SDA_RESET();
    }
    i2c_delay();
    I2C_SCL_SET();
    i2c_delay();
    I2C_SCL_RESET();
    i2c_delay();
    I2C_SDA_SET();
}

uint8_t i2c_write_register(uint8_t i2c_address, uint8_t address, uint8_t data)
{
    i2c_start();

    i2c_send_byte(i2c_address << 1);            /* write */
    if (i2c_receive_ack() != I2C_OK) { i2c_stop(); return I2C_ERR; }

    i2c_send_byte(address);
    if (i2c_receive_ack() != I2C_OK) { i2c_stop(); return I2C_ERR; }

    i2c_send_byte(data);
    if (i2c_receive_ack() != I2C_OK) { i2c_stop(); return I2C_ERR; }

    i2c_stop();
    return I2C_OK;
}

uint8_t i2c_read_register(uint8_t i2c_address, uint8_t address, uint8_t *out)
{
    return i2c_read_len(i2c_address, address, 1, out);
}

/*
 * Reads len consecutive registers starting at reg_addr in a single bus
 * transaction, using the slave's address auto-increment. Reading the AS5600
 * angle this way costs one transaction instead of two, which halves the I2C
 * time in the control loop AND removes the possibility of the high and low
 * bytes coming from two different samples (a torn read that shows up as an
 * angle spike).
 */
uint8_t i2c_read_len(uint8_t i2c_address, uint8_t reg_addr, uint8_t len, uint8_t *data)
{
    uint8_t i;

    if (len == 0 || data == 0) {
        return I2C_ERR;
    }

    i2c_start();

    i2c_send_byte(i2c_address << 1);            /* write: set register pointer */
    if (i2c_receive_ack() != I2C_OK) { i2c_stop(); return I2C_ERR; }

    i2c_send_byte(reg_addr);
    if (i2c_receive_ack() != I2C_OK) { i2c_stop(); return I2C_ERR; }

    i2c_start();                                /* repeated start */

    i2c_send_byte((i2c_address << 1) | 0x01u);  /* read */
    if (i2c_receive_ack() != I2C_OK) { i2c_stop(); return I2C_ERR; }

    for (i = 0; i < len; i++) {
        data[i] = i2c_receive_byte();
        /* ACK every byte except the last one. */
        i2c_send_ack((i == len - 1) ? 1u : 0u);
    }

    i2c_stop();
    return I2C_OK;
}
