#include "as5600.h"
#include "iic.h"
#include "math.h"
#include "Lowpass.h"
#include "Data.h"
#include "systime.h"
#include "uart1.h"

#define AS5600_ADDR         0x36u
#define AS5600_REG_RAWANGLE 0x0Cu
#define AS5600_CPR          4096

volatile uint8_t encoder_fault = 0;

static ENCODER_TypeDef E1;

static uint16_t s_raw          = 0;
static float    s_angle_track  = 0.0f;
static float    s_angle_single = 0.0f;
static float    s_velocity     = 0.0f;

void as5600_init(void)
{

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

    if (i2c_read_len(AS5600_ADDR, AS5600_REG_RAWANGLE, 2, buf) != I2C_OK) {
        encoder_fault = 1;
        return 0;
    }

    raw = (uint16_t)(((buf[0] & 0x0Fu) << 8) | buf[1]);
    s_raw = raw;

    d_angle = (float)raw - (float)E1.angle_data_prev;

    if (fabsf(d_angle) > (0.8f * (float)E1.cpr)) {
        E1.full_rotation_offset += (d_angle > 0.0f) ? -_2PI : _2PI;
    }
    E1.angle_data_prev = (int32_t)raw;

    s_angle_single = ((float)raw / (float)AS5600_CPR) * _2PI;
    s_angle_track  = E1.full_rotation_offset + s_angle_single;

    Ts = systime_delta_s(&E1.velocity_calc_timestamp);

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

void as5600_selftest(void)
{
    const uint32_t default_ticks = I2C_DELAY_TICKS_DEFAULT;
    const uint32_t reads_per_step = 500u;
    const int32_t  glitch_threshold = 40;

    uint32_t ticks, i, t0, t1;
    uint32_t nacks, glitches, per_read_us;
    uint16_t prev_raw;
    int32_t  d;

    printf("\r\n=== I2C / AS5600 self-test ===\r\n");

    i2c_set_delay_ticks(default_ticks);
    i2c_bus_recover();

    printf("half-bit delay @ %u ticks : %u ns\r\n",
           (unsigned)default_ticks, (unsigned)i2c_measure_delay_ns());

    encoder_fault = 0;
    t0 = micros();
    for (i = 0; i < 200u; i++) {
        as5600_update();
    }
    t1 = micros();
    per_read_us = (t1 - t0) / 200u;
    if (per_read_us == 0u) {
        per_read_us = 1u;
    }
    printf("as5600_update()          : %u us  (I2C caps the loop at %u Hz)\r\n",
           (unsigned)per_read_us, (unsigned)(1000000u / per_read_us));

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
        prev_raw = 0xFFFFu;
        encoder_fault = 0;

        t0 = micros();
        for (i = 0; i < reads_per_step; i++) {
            if (!as5600_update()) {
                nacks++;
                i2c_bus_recover();
                prev_raw = 0xFFFFu;
                continue;
            }

            if (prev_raw != 0xFFFFu) {

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

    i2c_set_delay_ticks(default_ticks);
    i2c_bus_recover();
    encoder_fault = 0;

    as5600_update();
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
