
//#define F_CPU 8000000UL
#include <avr/io.h>
#include <util/delay.h>
#include <stdio.h>
#include <stdbool.h>
#include <inttypes.h>
#include "ili9341.h"
#include "ili9341gfx.h"
#include <avr/pgmspace.h>
#include "grn_TWI.h"

#define DPS310_W 0xee
#define DPS310_R 0xef
#define PRS_B2	0x00
#define PRS_B1	0x01
#define PRS_B0	0x02
#define TMP_B2 	0x03
#define TMP_B1	0x04
#define TMP_B0	0x05
#define PRS_CFG	0x06
#define TMP_CFG	0x07
#define MEAS_CFG	0x08
#define CFG_REG		0x09
#define INT_STS		0x0A
#define FIFO_STS	0x0B
#define RESET		0x0C
#define PRODUCT_ID	0x0D
#define COEF_SRCE	0x28

#define COEF_RDY 1
#define SENSOR_RDY 2
#define TMP_RDY 3
#define PRS_RDY 4
#define PROD_ID 5

//Compensation Scale Factors (Oversampling)
#define Scal_1 524288 //sinlge
#define Scal_2 1572864
#define Scal_4 3670016
#define Scal_8 7864320
#define Scal_16 253952
#define Scal_32 516096
#define Scal_64 1040384
#define Scal_128 2088960



#define SENS_OP_STATUS 0x08

#define MODE_STBY	0x00
#define MODE_COMMAND_PRES 0x01
#define MODE_COMMAND_TEMP 0x02
#define MODE_COMMAND_PRES_AND_TEMP 0x03
#define MODE_BACKGROUND_PRES 0x05
#define MODE_BACKGROUND_TEMP 0x06
#define MODE_BACKGROUND_PRES_AND_TEMP 0x07
#define POINTCOLOUR PINK



#define SENSOR_READY_CHECK (DPS310_read(MEAS_CFG) & (1<<6)) != 0
#define COEFF_READY_CHECK (DPS310_read(MEAS_CFG) & (1<<7)) != 0
#define TEMP_READY_CHECK (DPS310_read(MEAS_CFG) & (1<<5)) != 0
#define PRES_READY_CHECK (DPS310_read(MEAS_CFG) & (1<<4)) != 0

extern uint16_t vsetx,vsety,vactualx,vactualy,isetx,isety,iactualx,iactualy;
static FILE mydata = FDEV_SETUP_STREAM(ili9341_putchar_printf, NULL, _FDEV_SETUP_WRITE);
uint8_t result,  xpos, error, x, value, rdy;
uint16_t xx, yy, zell, COLOR, var_x,color;

//compensation coefficients
int16_t m_C0;
int16_t m_C1;
int32_t m_C00;
int32_t m_C10;
int16_t m_C01;
int16_t m_C11;
int16_t m_C20;
int16_t m_C21;
int16_t m_C30;

uint8_t buffer[3] = {0};
uint8_t meas=0;
uint8_t id=0;



float ttt=0;
long ccc=0;

uint8_t buff[6]= {0};
	
	


uint32_t Pressure;
uint32_t Temperature;

uint8_t bit;

// String für Zahlenausgabe
char string[30] = "";

uint8_t DPS310_read(uint8_t reg)
{
		uint8_t result=0;
		
		TWIStart();
		if(TWIGetStatus() != 0x08)return 123;
		TWIWrite(DPS310_W);
		if(TWIGetStatus() != 0x18)return 2;
		TWIWrite(reg);
		if(TWIGetStatus() != 0x28)return 3;
		TWIStart();
		if(TWIGetStatus() != 0x10)return 4; //repetet Start sent?
		TWIWrite(DPS310_R);
		if(TWIGetStatus() != 0x40)return 5;
		result=TWIReadNACK();
		TWIStop();
		_delay_ms(2);
	return result;	
//Daten zurueckgeben
}
uint8_t DPS310_write(uint8_t reg, uint8_t data)
{
		TWIStart();
		if(TWIGetStatus() != 0x08)return 11;
		TWIWrite(DPS310_W);
		if(TWIGetStatus() != 0x18)return 22;
		TWIWrite(reg);
		if(TWIGetStatus() != 0x28)return 33;
		TWIWrite(data);
		if(TWIGetStatus() != 0x28)return 44;
		TWIStop();
		
		_delay_ms(2);
	return 0;	
	
	//Daten zurueckgeben
}

int16_t DPS310_readCoeffs(void)
{
	uint16_t buffer[19];//coeffizienten
	uint8_t coeff_start;
	coeff_start=0x10;
	
	//coeffizienten einlesen und in buffer-Array speichern
	//Addressen 0x10 - 0x21
	for(x=0;x<18;x++)
	{
		buffer[x]=DPS310_read(coeff_start);
		_delay_ms(10);
		coeff_start++;
	}
	 
    m_C0=(((int)buffer[0]<<8)|buffer[1])>>4;
    m_C0=m_C0/2;
   
    //m_C1=((((int)buffer[1]<<8)|buffer[2])<<4)>>4;
    
    m_C1 = (((uint32_t)buffer[1] & 0x0F) << 8) | (uint32_t)buffer[2];
	if(m_C1 & ((uint32_t)1 << 11))
	{
		m_C1 -= (uint32_t)1 << 12;
	}
      
    m_C00= ((((long)buffer[3]<<8)|buffer[4])<<8)|buffer[5];
    m_C00=(m_C00<<8)>>12;

    m_C10=((((long)buffer[5]<<8)|buffer[6])<<8)|buffer[7];
    m_C10=(m_C10<<12)>>12;

    m_C01=((int)buffer[8]<<8)|buffer[9];

    m_C11=((int)buffer[10]<<8)|buffer[11];

    m_C20=((int)buffer[12]<<8)|buffer[13];

    m_C21=((int)buffer[14]<<8)|buffer[15];

    m_C30=((int)buffer[16]<<8)|buffer[17];
       
    return 0;
}
void printcoeffs(void)
{
	
	uint8_t xxx=150;
	ili9341_setcursor(xxx,0);
	printf("1= %d", m_C0);
	ili9341_setcursor(xxx,20);
	printf("2= %03d", m_C1);
	ili9341_setcursor(xxx,40);
	printf("3= %ld", m_C00);
	ili9341_setcursor(xxx,60);
	printf("4= %ld", m_C10);
	ili9341_setcursor(xxx,80);
	printf("5= %d", m_C01);
	ili9341_setcursor(xxx,100);
	printf("6= %d", m_C11);
	ili9341_setcursor(xxx,120);
	printf("7= %d", m_C20);
	ili9341_setcursor(xxx,140);
	printf("8= %d", m_C21);
	ili9341_setcursor(xxx,160);
	printf("9= %d", m_C30);
}

void DPS310_sreset(void)
{
	// softreset of DPS310 sensor
	DPS310_write(0x0c, 0x99);
	_delay_ms(50);
}

void init_ili9341(void)
{
	stdout = & mydata;
	ili9341_init();//initial driver setup to drive ili9341
	ili9341_clear(BLACK);//fill screen with black colour
	_delay_ms(100);
	ili9341_setRotation(3);//rotate screen
	_delay_ms(2);
	ili9341_settextcolour(YELLOW,BLACK);
	ili9341_setcursor(0,0);
	ili9341_settextsize(2);
}
void DPS310_init(void)
{
	uint8_t bit=0;
	
	while(bit==0)// go on if Sensor ready flag is set
	{
		if(COEFF_READY_CHECK)bit=1;
		DPS310_readCoeffs();
		DPS310_write(PRS_CFG, 0x00);//eight times low power
		DPS310_write(TMP_CFG, 0x80);// 1 measurement
		DPS310_write(CFG_REG, 0x00);
		DPS310_write(MEAS_CFG, 0x07);
		DPS310_write(0x0E, 0xA5);
		DPS310_write(0x0F, 0x96);
		DPS310_write(0x62, 0x02);
		DPS310_write(0x0E, 0x00);
		DPS310_write(0x0F, 0x00);
	}
}
uint32_t DPS310_get_sc_temp(uint8_t oversampling)
{
	long temp_raw=0;

	buff[0] = DPS310_read(TMP_B2);
	buff[1] = DPS310_read(TMP_B1);
	buff[2] = DPS310_read(TMP_B0);
			
	temp_raw=((((long)buff[0]<<8)|buff[1])<<8)|buff[2];
	temp_raw=(temp_raw<<8)>>8;
	
	
			
	return temp_raw; 
}

uint32_t DPS310_get_temp(uint8_t oversampling)
{
	long temp_raw=0;
	double temp_sc=0;
	double temp_comp=0;
	long scalfactor=0;

			buff[0] = DPS310_read(TMP_B2);
			buff[1] = DPS310_read(TMP_B1);
			buff[2] = DPS310_read(TMP_B0);
			
			temp_raw=DPS310_get_sc_temp(oversampling);
			
			switch(oversampling)
			{
				case 1:	scalfactor = 524288;break;
				case 2:	scalfactor = 1572864;break;
				case 4:	scalfactor = 3670016;break;
				case 8:	scalfactor = 7864320;break;
				case 16:	scalfactor = 253952;break;
				case 32:	scalfactor = 516096;break;
				case 64:	scalfactor = 1040384;break;
				case 128:	scalfactor = 2088960;break;
				
			}
			temp_sc = (float)temp_raw/scalfactor;
			temp_comp=m_C0+m_C1*temp_sc;
			
			
			return temp_comp*100; //2505 entspricht 25,5 Grad
}


uint32_t DPS310_get_pres(void)
{
	long temp_raw;
	double temp_sc;
	
	long prs_raw;
	double prs_sc;
	double prs_comp;
	
		buff[0] = DPS310_read(TMP_B2);
		buff[1] = DPS310_read(TMP_B1);
		buff[2] = DPS310_read(TMP_B0);
		
		temp_raw=((((long)buff[0]<<8)|buff[1])<<8)|buff[2];
		temp_raw=(temp_raw<<8)>>8;
		
		temp_sc = (float)temp_raw/524288;
		
		buff[0] = DPS310_read(PRS_B2);
		buff[1] = DPS310_read(PRS_B1);
		buff[2] = DPS310_read(PRS_B0);
		
		prs_raw=((((long)buff[0]<<8)|buff[1])<<8)|buff[2];
		prs_raw=(prs_raw<<8)>>8;
		
		prs_sc = (float)prs_raw/524288;
		prs_comp=m_C00+prs_sc*(m_C10+prs_sc*(m_C20+(prs_sc*m_C30)))+temp_sc*m_C01+temp_sc*prs_sc*(m_C11+(prs_sc*m_C21));
		return prs_comp; //2505 entspricht 25,5 Grad
}
int main(void)
{
	init_ili9341();
	//display_init();//display initial data
	yy=240;
	xx=0;
	zell=0;
	color=123;
	var_x=0x01;
	
	value=0;
	rdy=0;
	TWIInit();
	
	
	DPS310_init();
	
	

	while(1)
	{
				
		id = DPS310_read(PRODUCT_ID);
		meas = DPS310_read(MEAS_CFG);
		
		if(TEMP_READY_CHECK)Temperature=DPS310_get_temp(1);
		if(PRES_READY_CHECK)Pressure=DPS310_get_pres();
		ili9341_setcursor(0,00);
		printf("temp_raw = %03d", id);
	
		ili9341_setcursor(0,200);
		printf("Temperature = %03ld",Temperature);
		ili9341_setcursor(0,220);
		printf("Pressure = %ld",Pressure);
		

		
		//printcoeffs();
	
		
	

	_delay_ms(10);
		
	
	}//end of while

}//end of main
