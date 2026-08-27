#ifndef __IST8310_H__
#define __IST8310_H__

#include "sys.h"

#define IST8310_ADDRESS 0x0E
#define IST8310_WHO_AM_I 0x00       //ist8310 "who am I " 
#define IST8310_WHO_AM_I_VALUE 0x10 //device ID
//the first column:the registers of IST8310. 第一列:IST8310的寄存器
//the second column: the value to be writed to the registers.第二列:需要写入的寄存器值
//the third column: return error value.第三列:返回的错误码
static const uint8_t ist8310_write_reg_data_error[][3] ={
        {0x0B, 0x08, 0x01},     //enalbe interrupt  and low pin polarity.开启中断，并且设置低电平
        {0x41, 0x09, 0x02},     //average 2 times.平均采样两次
        {0x42, 0xC0, 0x03},     //must be 0xC0. 必须是0xC0
        {0x0A, 0x0B, 0x04}};    //200Hz output rate.200Hz输出频率

void ist8310_init(void);
void ist8310_updata(float *ax,float *ay,float *az,float *tmp);
void ist8310_get_data(int16_t *AccX, int16_t *AccY, int16_t *AccZ, int16_t *TEMP);
void ist8310_get_angal(void);
#endif

