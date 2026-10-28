#ifndef __IST8310_H__
#define __IST8310_H__

#include "sys.h"

#define IST8310_ADDRESS 0x0E
#define IST8310_WHO_AM_I 0x00
#define IST8310_WHO_AM_I_VALUE 0x10

static const uint8_t ist8310_write_reg_data_error[][3] ={
        {0x0B, 0x08, 0x01},
        {0x41, 0x09, 0x02},
        {0x42, 0xC0, 0x03},
        {0x0A, 0x0B, 0x04}};

void ist8310_init(void);
void ist8310_updata(float *ax,float *ay,float *az,float *tmp);
void ist8310_get_data(int16_t *AccX, int16_t *AccY, int16_t *AccZ, int16_t *TEMP);
void ist8310_get_angal(void);
#endif
