/*
***************************************************************************************************
*                                         APPLICATION CODE
*
*                      (c) Copyright 2016; Guangdong ENECO Science And Technology Co.,Ltd
*
*               All rights reserved.  Protected by international copyright laws.
*               Knowledge of the source code may NOT be used without authorization.
***************************************************************************************************
*/
/*
***************************************************************************************************
* Filename      : bsp_atmel_25256b_sshl.c
* Version       : V1.00
* Programmer(s) : Fanjun
*
***************************************************************************************************
*/
/*
***************************************************************************************************
*                                             INCLUDE FILES
***************************************************************************************************
*/
#include "bsp_atmel_25256b_sshl.h"
#include "bsp_spi.h"
/*
***************************************************************************************************
*                                           MACRO DEFINES
***************************************************************************************************
*/


/*
***************************************************************************************************
*                                         OS-RELATED    VARIABLES
***************************************************************************************************
*/


/*
***************************************************************************************************
*                                       LOCAL VARIABLES
***************************************************************************************************
*/

/*
***************************************************************************************************
*                                         FUNCTION PROTOTYPES
***************************************************************************************************
*/
/*
***************************************************************************************************
*                                    AT25256B_Init()
*
* Description : Atmel 25256b eeprom init.
*
* Arguments   : none.
*
* Returns     : none.
*
* Notes       : the speed grade whole number is 2000.
***************************************************************************************************
*/
////4KbytesΪһ��Sector����
////16������Ϊ1��Block
////����Ϊ16M�ֽ�,����128��Block,4096��Sector 												 
void AT25256B_Init(void)
{	
    GPIO_InitTypeDef GPIO_InitStructure;
	RCC_APB2PeriphClockCmd(	RCC_APB2Periph_GPIOD, ENABLE );//PORTdʱ��ʹ�� 

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3 | GPIO_Pin_4;// PB12 ���� 
 	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;  //�������
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
 	GPIO_Init(GPIOB, &GPIO_InitStructure);
 	GPIO_SetBits(GPIOB,GPIO_Pin_12);
 
    AT25256B_NCS;//SPI FLASH��ѡ��
	SPI3_Init();//��ʼ��SPI3

}  

/*
***************************************************************************************************
*                                 AT25256B_ReadSR()
*
* Description:  Read atmel 25256b status register.
*
* Arguments  :  none
*
* Returns    :  none
*
* Notes      :  BIT7  6   5   4   3    2    1    0.
*               WPEN  X   X   X  BP1  BP2  WEN  ~RDY 
*               WEN:дʹ������;RDY:æ���λ(1,æ;0,����)
*               WPEN��WEN��BP1��BP0:FLASH����д��������;BP1��BP0������������
*               Bit 0~7��Ϊ1��ʾ�������ڲ�д����;Bit4~6��Ϊ0��ʾû�д����ڲ�д����
***************************************************************************************************
*/
uint16_t AT25256B_ReadSR(void)   
{  
	uint16_t byte=0;   
	AT25256B_CS;                                 //ѡ������   
	SPI3_ReadWriteByte(AT_READ_STATUS_REGISTER); //���Ͷ�ȡ״̬�Ĵ�������    
	byte=SPI3_ReadWriteByte(0xFFFF);             //��ȡһ���ֽ�  
	AT25256B_NCS;                                //ȡ��Ƭѡ      
	return byte;   
} 
/*
***************************************************************************************************
*                                    AT25256B_Init()
*
* Description : Write atmel 25256b status register.
*
* Arguments   : none.
*
* Returns     : none.
*
* Notes       : ֻ��SPR,TB,BP2,BP1,BP0(bit 7,5,4,3,2)����д.
***************************************************************************************************
*/
void AT25256B_Write_SR(u8 sr)   
{   
	AT25256B_CS;                            //ʹ������   
	SPI3_ReadWriteByte(AT_WRITE_STATUS_REGISTER);//����дȡ״̬�Ĵ�������    
	SPI3_ReadWriteByte(sr);               	//д��һ���ֽ�  
	AT25256B_NCS;                             //ȡ��Ƭѡ     	      
}   
/*
***************************************************************************************************
*                                    AT25256B_Write_Enable()
*
* Description : Write atmel 25256b status register.
*
* Arguments   : none.
*
* Returns     : none.
*
* Notes       : ֻ��SPR,TB,BP2,BP1,BP0(bit 7,5,4,3,2)����д.
***************************************************************************************************
*/  
void AT25256B_Write_Enable(void)   
{
	AT25256B_CS;                          	
    SPI3_ReadWriteByte(AT_WRITE_ENABLE); 	
	AT25256B_NCS;                            		      
} 
/*
***************************************************************************************************
*                                    AT25256B_Init()
*
* Description : Write atmel 25256b status register.
*
* Arguments   : none.
*
* Returns     : none.
*
* Notes       : ֻ��SPR,TB,BP2,BP1,BP0(bit 7,5,4,3,2)����д.
***************************************************************************************************
*/ 
void AT25256B_Write_Disable(void)   
{  
	AT25256B_CS;                            
    SPI3_ReadWriteByte(AT_RESET_WRITE_ENABLE_REGISTER);   
	AT25256B_CS;                            
} 
/*
***************************************************************************************************
*                                    AT25256B_Read()
*
* Description : ��ȡSPI FLASH����ָ����ַ��ʼ��ȡָ�����ȵ�����.
*
* Arguments   : pBuffer:���ݴ洢����ReadAddr:��ʼ��ȡ�ĵ�ַ(24bit)��NumByteToRead:Ҫ��ȡ���ֽ���(���65535).
*
* Returns     : none.
*
* Notes       : none.
***************************************************************************************************
*/   		    
void AT25256B_Read(u8* pBuffer,u32 ReadAddr,u16 NumByteToRead)   
{ 
 	u16 i;   										    
	AT25256B_CS;                            	//ʹ������   
    SPI3_ReadWriteByte(AT_READ_DATA);         	//���Ͷ�ȡ����   
    SPI3_ReadWriteByte((u8)((ReadAddr)>>16));  	//����24bit��ַ    
    SPI3_ReadWriteByte((u8)((ReadAddr)>>8));   
    SPI3_ReadWriteByte((u8)ReadAddr);   
    for(i=0;i<NumByteToRead;i++)
	{ 
        pBuffer[i]=SPI3_ReadWriteByte(0xFF);   	//ѭ������  
    }
	AT25256B_CS;  				    	      
}  
  
   
