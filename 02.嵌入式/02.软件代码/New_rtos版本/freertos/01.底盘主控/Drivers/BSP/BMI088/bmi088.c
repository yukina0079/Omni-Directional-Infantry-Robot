#include "bmi088.h"
#include "usart.h"
#include "delay.h"
#include "iic.h"
#include "math.h"

#define BMI088_ACCEL_3G_SEN 0.0008974358974f
#define BMI088_ACCEL_6G_SEN 0.00179443359375f
#define BMI088_ACCEL_12G_SEN 0.0035888671875f
#define BMI088_ACCEL_24G_SEN 0.007177734375f


#define BMI088_GYRO_2000_SEN 0.00106526443603169529841533860381f
#define BMI088_GYRO_1000_SEN 0.00053263221801584764920766930190693f
#define BMI088_GYRO_500_SEN 0.00026631610900792382460383465095346f
#define BMI088_GYRO_250_SEN 0.00013315805450396191230191732547673f
#define BMI088_GYRO_125_SEN 0.000066579027251980956150958662738366f

float AX, AY, AZ;
float GX, GY, GZ;
float Temperature;

void BMI088_get_angle(void)
{
	BMI088_update(&AX,&AY,&AZ,&GX,&GY,&GZ,&Temperature);
	
}
void BMI088_update(float *AX,float *AY,float *AZ,float *GX,float *GY,float *GZ,float *Temperature)
{
		int16_t ax, ay, az;
		int16_t gx, gy, gz;
		int16_t temp;
	
		BMI088_get_data(&ax, &ay, &az, &gx, &gy, &gz, &temp);
	
		*AX = (ax * BMI088_ACCEL_3G_SEN);
		*AY = (ay * BMI088_ACCEL_3G_SEN);
		*AZ = (az * BMI088_ACCEL_3G_SEN);
		*GX = gx * BMI088_GYRO_2000_SEN;
		*GY = gy * BMI088_GYRO_2000_SEN;
		*GZ = gz * BMI088_GYRO_2000_SEN;
		
//		uint16_t TEMP_LSB = iic_read_register(BMI088_ACCEL_ADDRESS,0x23);
//		uint16_t TEMP_MSB = iic_read_register(BMI088_ACCEL_ADDRESS,0x22);
//		
//		uint16_t Temp_uint11 = (TEMP_MSB * 8) + (TEMP_LSB / 32);
//		
//		int16_t Temp_int11 = (Temp_uint11 > 1023) ? (Temp_uint11 - 2048) : Temp_uint11;
//		
//		*Temperature = Temp_int11 * 0.125f + 23.0f;
		
}	


void BMI088_get_data(int16_t *AccX, int16_t *AccY, int16_t *AccZ,
                      int16_t *GyroX, int16_t *GyroY, int16_t *GyroZ, int16_t *Tmp) 
{
    uint8_t buf[6];

    // 1. 连续读取加速度计数据 (从 0x12 开始读取 6 字节)
    // 寄存器顺序: 0x12(XL), 0x13(XH), 0x14(YL), 0x15(YH), 0x16(ZL), 0x17(ZH)
    if (iic_read_len(BMI088_ACCEL_ADDRESS, 0x12, buf, 6) == 0)
    {
        *AccX = (int16_t)((buf[1] << 8) | buf[0]);
        *AccY = (int16_t)((buf[3] << 8) | buf[2]);
        *AccZ = (int16_t)((buf[5] << 8) | buf[4]);
    }

    // 2. 连续读取陀螺仪数据 (从 0x02 开始读取 6 字节)
    // 寄存器顺序: 0x02(XL), 0x03(XH), 0x04(YL), 0x05(YH), 0x06(ZL), 0x07(ZH)
    if (iic_read_len(BMI088_GYRO_ADDRESS, 0x02, buf, 6) == 0)
    {
        *GyroX = (int16_t)((buf[1] << 8) | buf[0]);
        *GyroY = (int16_t)((buf[3] << 8) | buf[2]);
        *GyroZ = (int16_t)((buf[5] << 8) | buf[4]);
    }

    // 3. 读取加速度计中的温度数据 (0x22, 0x23)
    // 根据 BMI088 手册，温度传感器通常在加速度计芯片上
    if (iic_read_len(BMI088_ACCEL_ADDRESS, 0x22, buf, 2) == 0)
    {
        *Tmp = (int16_t)((buf[0] << 3) | (buf[1] >> 5)); // 11位温度数据处理
    }
}

void BMI088_init(void)
{   


    iic_write_register(BMI088_ACCEL_ADDRESS,0x7E, 0xB6); 
	delay_ms(2);	
	iic_write_register(BMI088_ACCEL_ADDRESS,0x7D, 0x04); 
	iic_write_register(BMI088_ACCEL_ADDRESS,0x7C, 0x00);   
    iic_write_register(BMI088_ACCEL_ADDRESS,0x6D, 0x00); 
    iic_write_register(BMI088_ACCEL_ADDRESS,0x41, 0x00); 
	iic_write_register(BMI088_ACCEL_ADDRESS,0x40, 0x0A); 
	
	
    iic_write_register(BMI088_GYRO_ADDRESS,0x14, 0xB6);
	delay_ms(40);
	iic_write_register(BMI088_GYRO_ADDRESS,0x11, 0x00);
    iic_write_register(BMI088_GYRO_ADDRESS,0x0F, 0x00);
    iic_write_register(BMI088_GYRO_ADDRESS,0x10, 0x00);
    iic_write_register(BMI088_GYRO_ADDRESS,0x15, 0x00);

	
	
	uint8_t DataA = iic_read_register(BMI088_ACCEL_ADDRESS,BMI088_WHO_AM_I);
	uint8_t DataB = iic_read_register(BMI088_GYRO_ADDRESS,BMI088_WHO_AM_I);
	
	uint8_t DataC = iic_read_register(BMI088_ACCEL_ADDRESS,0x7E);
	uint8_t DataD = iic_read_register(BMI088_ACCEL_ADDRESS,0x7D);
	uint8_t DataE = iic_read_register(BMI088_ACCEL_ADDRESS,0x7C);
	uint8_t DataF = iic_read_register(BMI088_ACCEL_ADDRESS,0x6D);
	uint8_t DataG = iic_read_register(BMI088_ACCEL_ADDRESS,0x41);
	uint8_t DataH = iic_read_register(BMI088_ACCEL_ADDRESS,0x40);
	
	uint8_t DataI = iic_read_register(BMI088_GYRO_ADDRESS,0x14);
	uint8_t DataJ = iic_read_register(BMI088_GYRO_ADDRESS,0x11);	
	uint8_t DataK = iic_read_register(BMI088_GYRO_ADDRESS,0x0F);
	uint8_t DataL = iic_read_register(BMI088_GYRO_ADDRESS,0x15);

	uint8_t DataM = iic_read_register(BMI088_GYRO_ADDRESS,0x15);	
	
//	printf("0x%X 0x%X\r\n",DataA,DataB);
//	printf("0x%X 0x%X 0x%X 0x%X 0x%X 0x%X\r\n",DataC,DataD,DataE,DataF,DataG,DataH);
//	printf("0x%X 0x%X 0x%X 0x%X 0x%X 0x%X\r\n",DataI,DataJ,DataK,DataL,DataM,DataM);
}








