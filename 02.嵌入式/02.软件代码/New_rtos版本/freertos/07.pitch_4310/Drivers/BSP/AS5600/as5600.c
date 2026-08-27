#include "as5600.h"
#include "iic.h"
#include "math.h"
#include "Lowpass.h"
#include "Data.h"
#include "systime.h"
#include "uart1.h"

#define AS5600_ADDR         0x36u
#define AS5600_REG_RAWANGLE 0x0Cu   /* 0x0C = high byte, 0x0D = low byte */
#define AS5600_CPR          4096

volatile uint8_t encoder_fault = 0;

static ENCODER_TypeDef E1;

/*
 * Cached sample, refreshed once per control cycle by as5600_update().
 * Everything downstream reads these instead of touching the I2C bus.
 */
static uint16_t s_raw          = 0;      /* 0..4095                      */
static float    s_angle_track  = 0.0f;   /* continuous, radians          */
static float    s_angle_single = 0.0f;   /* 0..2PI, radians              */
static float    s_velocity     = 0.0f;   /* filtered, rad/s              */

void as5600_init(void)
{
    /* CONF register (0x07/0x08): all defaults. Failure here means the sensor
     * is not responding at all, so flag it immediately rather than waiting for
     * the first control cycle to notice. */
    if (i2c_write_register(AS5600_ADDR, 0x07, 0x00) != I2C_OK ||
        i2c_write_register(AS5600_ADDR, 0x08, 0x00) != I2C_OK) {
        encoder_fault = 1;
    }

    E1.cpr                  = AS5600_CPR;
    E1.angle_data_prev      = 0;
    E1.angle_prev           = 0.0f;
    E1.full_rotation_offset = 0.0f;
    E1.velocity_calc_timestamp = micros();
}

uint8_t as5600_update(void)
{
    uint8_t  buf[2];
    uint16_t raw;
    float    d_angle;
    float    Ts;
    float    vel_raw;

    /*
     * Both bytes in one transaction via the AS5600's address auto-increment.
     * Two separate reads (the old approach) doubled the bus time and allowed
     * the high and low bytes to come from different samples, producing angle
     * spikes right at the 0x0FFF -> 0x000 wrap.
     */
    if (i2c_read_len(AS5600_ADDR, AS5600_REG_RAWANGLE, 2, buf) != I2C_OK) {
        encoder_fault = 1;
        return 0;               /* cache left untouched, deliberately stale */
    }

    raw = (uint16_t)(((buf[0] & 0x0Fu) << 8) | buf[1]);
    s_raw = raw;

    /* --- multi-turn tracking (SimpleFOC algorithm) --- */
    d_angle = (float)raw - (float)E1.angle_data_prev;
    /* A jump larger than 80% of a full revolution can only be a wrap. */
    if (fabsf(d_angle) > (0.8f * (float)E1.cpr)) {
        E1.full_rotation_offset += (d_angle > 0.0f) ? -_2PI : _2PI;
    }
    E1.angle_data_prev = (int32_t)raw;

    s_angle_single = ((float)raw / (float)AS5600_CPR) * _2PI;
    s_angle_track  = E1.full_rotation_offset + s_angle_single;

    /* --- velocity from the continuous angle --- */
    Ts = systime_delta_s(&E1.velocity_calc_timestamp);
    /*
     * No wrap correction here: s_angle_track is already continuous thanks to
     * full_rotation_offset. The old code applied a +/-180 correction to a value
     * expressed in radians, which could never trigger and only obscured the
     * units.
     */
    vel_raw = (s_angle_track - E1.angle_prev) / Ts;
    E1.angle_prev = s_angle_track;

    s_velocity = LowPassFilter_operator(&LPF_velocity, vel_raw);

    encoder_fault = 0;
    return 1;
}

float as5600_read_angal(void)
{
    return (float)s_raw;
}

float Get_Angel(void)
{
    return s_angle_track;
}

float Get_Angel_Notrack(void)
{
    return s_angle_single;
}

float Get_Velocity(void)
{
    return s_velocity;
}

/*
 * ---------------------------------------------------------------------------
 * Self-test
 * ---------------------------------------------------------------------------
 *
 * Answers, without any external instrument, the questions a scope would have
 * been used for:
 *
 *   1. How long is one i2c_delay() really? (the half-bit period)
 *   2. How long does one encoder sample cost? (the control loop's I2C budget,
 *      and therefore the real Ts that the old firmware faked as 1 ms)
 *   3. How much timing margin does this bus have?
 *
 * Question 3 is the important one, and the scan below is stronger evidence
 * than a scope trace. A scope tells you "SCL high = 2.1 us", which you then
 * compare against a datasheet worst case. The scan tells you the clock rate at
 * which THIS board, with THIS cable and THIS encoder, actually starts failing.
 * Running four times slower than the measured failure point is a margin claim
 * backed by measurement rather than by a spec sheet.
 *
 * Two independent failure modes are counted per step:
 *   NACK    - protocol-level failure, the slave did not acknowledge
 *   glitch  - the transfer completed but the data is wrong. Detected as an
 *             implausible jump in the raw count between consecutive reads.
 *             Requires the shaft to be STATIONARY, which it is here: this runs
 *             before the motor is ever energised.
 *
 * Motor must be off and the shaft still. Takes a few seconds.
 */
void as5600_selftest(void)
{
    const uint32_t default_ticks = I2C_DELAY_TICKS_DEFAULT;
    const uint32_t reads_per_step = 500u;
    const int32_t  glitch_threshold = 40;   /* counts; 40/4096 = 3.5 degrees */

    uint32_t ticks, i, t0, t1;
    uint32_t nacks, glitches, per_read_us;
    uint16_t prev_raw;
    int32_t  d;

    printf("\r\n=== I2C / AS5600 self-test ===\r\n");

    i2c_set_delay_ticks(default_ticks);
    i2c_bus_recover();

    /* --- 1. real duration of the half-bit delay --- */
    printf("half-bit delay @ %u ticks : %u ns\r\n",
           (unsigned)default_ticks, (unsigned)i2c_measure_delay_ns());

    /* --- 2. cost of one encoder sample --- */
    encoder_fault = 0;
    t0 = micros();
    for (i = 0; i < 200u; i++) {
        as5600_update();
    }
    t1 = micros();
    per_read_us = (t1 - t0) / 200u;
    if (per_read_us == 0u) {
        per_read_us = 1u;               /* guard the division below */
    }
    printf("as5600_update()          : %u us  (I2C caps the loop at %u Hz)\r\n",
           (unsigned)per_read_us, (unsigned)(1000000u / per_read_us));

    /* --- 3. margin scan --- */
    printf("\r\nticks  half-bit  read     NACK   glitch\r\n");
    printf(  "        (ns)     (us)    /%u    /%u\r\n",
           (unsigned)reads_per_step, (unsigned)reads_per_step);

    for (ticks = default_ticks; ticks >= 1u; ticks--) {
        uint32_t ns;

        i2c_set_delay_ticks(ticks);
        i2c_bus_recover();
        ns = i2c_measure_delay_ns();

        nacks    = 0;
        glitches = 0;
        prev_raw = 0xFFFFu;             /* sentinel: no previous sample yet */
        encoder_fault = 0;

        t0 = micros();
        for (i = 0; i < reads_per_step; i++) {
            if (!as5600_update()) {
                nacks++;
                i2c_bus_recover();      /* a failed transfer can wedge the bus */
                prev_raw = 0xFFFFu;     /* cache is stale, restart comparison */
                continue;
            }

            if (prev_raw != 0xFFFFu) {
                /* Circular difference, so a shaft parked right on the 4095->0
                 * boundary does not read as a 4095-count jump. */
                d = (int32_t)s_raw - (int32_t)prev_raw;
                if (d >  2048) d -= 4096;
                if (d < -2048) d += 4096;
                if (d < 0)     d  = -d;
                if (d > glitch_threshold) {
                    glitches++;
                }
            }
            prev_raw = s_raw;
        }
        t1 = micros();

        printf("%4u   %6u   %5u   %5u   %5u%s\r\n",
               (unsigned)ticks,
               (unsigned)ns,
               (unsigned)((t1 - t0) / reads_per_step),
               (unsigned)nacks,
               (unsigned)glitches,
               (nacks || glitches) ? "   <-- FAIL" : "");
    }

    /*
     * Restore the working clock rate and wipe the tracking state. The scan
     * deliberately provoked bad reads, and a glitch large enough to look like a
     * revolution will have pushed a bogus +/-2PI into full_rotation_offset.
     * Leaving that in place would offset every subsequent Get_Angel().
     */
    i2c_set_delay_ticks(default_ticks);
    i2c_bus_recover();
    encoder_fault = 0;

    as5600_update();                            /* reload from a good sample */
    E1.angle_data_prev         = (int32_t)s_raw;
    E1.full_rotation_offset    = 0.0f;
    E1.angle_prev              = s_angle_single;
    E1.velocity_calc_timestamp = micros();
    s_angle_track              = s_angle_single;

    printf("\r\nrestored to %u ticks, raw = %u, fault = %u\r\n",
           (unsigned)i2c_get_delay_ticks(), (unsigned)s_raw,
           (unsigned)encoder_fault);
    printf("Rule of thumb: keep at least 3-4x the ticks of the first FAIL row.\r\n");
}
