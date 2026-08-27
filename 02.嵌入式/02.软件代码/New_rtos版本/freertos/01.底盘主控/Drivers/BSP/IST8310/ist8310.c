#include "ist8310.h"
#include "usart.h"
#include "delay.h"
#include "iic.h"

#define MAG_SEN 0.3f //×ª»»³É uT

float ac_x,ac_y,ac_z,ac_tmp;

void ist8310_get_angal(void)
{
	ist8310_updata(&ac_x,&ac_y,&ac_z,&ac_tmp);
	
	//printf("%.2f,%.2f,%.2f,%.2f\n",ac_x,ac_y,ac_z,0.00);
}

void ist8310_updata(float *ax,float *ay,float *az,float *tmp)
{
	int16_t ax_mg,ay_mg,az_mg,tmp_mg;
	
	ist8310_get_data(&ax_mg,&ay_mg,&az_mg,&tmp_mg);
	
	*ax = ax_mg * MAG_SEN;
	*ay = ay_mg * MAG_SEN;
	*az = az_mg * MAG_SEN;
	
	//printf("%.2f,%.2f,%.2f,%.2f\n",ax,ay,az,0.00);
}

void ist8310_get_data(int16_t *AccX, int16_t *AccY, int16_t *AccZ, int16_t *TEMP)
{
    uint8_t DataH, DataL;
	
    DataL = iic_read_register(IST8310_ADDRESS,0x03);   
    DataH = iic_read_register(IST8310_ADDRESS,0x04);   
    *AccX = (DataH << 8) | DataL;                       
    
    DataL = iic_read_register(IST8310_ADDRESS,0x05);   
    DataH = iic_read_register(IST8310_ADDRESS,0x06);   
   *AccY = (DataH << 8) | DataL;                       
    
    DataL = iic_read_register(IST8310_ADDRESS,0x07);   
    DataH = iic_read_register(IST8310_ADDRESS,0x08);   
    *AccZ = (DataH << 8) | DataL;

    DataL = iic_read_register(IST8310_ADDRESS,0x1C);   
    DataH = iic_read_register(IST8310_ADDRESS,0x1D);   
    *TEMP = (DataH << 8) | DataL;	
}

void ist8310_init(void)
{   
    iic_write_register(IST8310_ADDRESS,0x0B, 0x08);           	
    iic_write_register(IST8310_ADDRESS,0x41, 0x09);           
    iic_write_register(IST8310_ADDRESS,0x42, 0xC0);           
    iic_write_register(IST8310_ADDRESS,0x0A, 0x0B);
       
	uint8_t DataA = iic_read_register(IST8310_ADDRESS,IST8310_WHO_AM_I);
	
	uint8_t DataB = iic_read_register(IST8310_ADDRESS,0x0B);
	uint8_t DataC = iic_read_register(IST8310_ADDRESS,0x0A);
	uint8_t DataD = iic_read_register(IST8310_ADDRESS,0x41);
	uint8_t DataE = iic_read_register(IST8310_ADDRESS,0x42);
	
	printf("%x %x %x %x %x\r\n",DataA,DataB,DataC,DataD,DataE);
	printf("%x\r\n",iic_read_register(0x76,0xD0));
}

	








