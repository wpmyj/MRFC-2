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
												 
void AT25256B_Init(void)
{	
    GPIO_InitTypeDef GPIO_InitStructure;
	RCC_APB2PeriphClockCmd(	RCC_APB2Periph_GPIOD, ENABLE);

	GPIO_InitStructure.GPIO_Pin = BSP_GPIOD_FLASH_HOLD_PORT_NMB | BSP_GPIOD_FLASH_WP_PORT_NMB;
 	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;  //�������
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
 	GPIO_Init(GPIOD, &GPIO_InitStructure);
    
    GPIO_ResetBits(GPIOD,BSP_GPIOD_FLASH_HOLD_PORT_NMB | BSP_GPIOD_FLASH_WP_PORT_NMB);//���ͣ��������ݴ����д����
    
    AT25256B_NCS;//SPI FLASH��ѡ��
	SPI3_Init();//��ʼ��SPI3

}  

/*
***************************************************************************************************
*                                    AT25256B_Write_Enable()
*
* Description : Atmel 25256b write enable.
*
* Arguments   : none.
*
* Returns     : none.
*
* Notes       : none.
***************************************************************************************************
*/  
static void AT25256B_Write_Enable(void)   
{
    GPIO_SetBits(GPIOD,BSP_GPIOD_FLASH_WP_PORT_NMB);//дʹ��ʱ�����WP����
    SPI3_ReadWriteByte(WREN);
    GPIO_ResetBits(GPIOD,BSP_GPIOD_FLASH_WP_PORT_NMB); 	                            		      
} 
 
static void AT25256B_Write_Disable(void)   
{  
//	AT25256B_CS;                            
    SPI3_ReadWriteByte(WRDI);   
//	AT25256B_NCS;                            
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
uint8_t AT25256B_ReadSR(void)   
{  
	uint8_t byte=0; 
    
	AT25256B_CS;                                 //ѡ������   
	SPI3_ReadWriteByte(RDSR);                    //���Ͷ�ȡ״̬�Ĵ�������    
	byte=SPI3_ReadWriteByte(0xFF);               //��ȡһ���ֽ�,���ͳ�ָ����������������� 
	AT25256B_NCS;                                //ȡ��Ƭѡ
    
	return byte;   
} 
/*
***************************************************************************************************
*                                    AT25256B_Write_SR()
*
* Description : Write atmel 25256b status register.
*
* Arguments   : 0x82:10000010-����д����,BP1��BP0Ҳ����д.
*
* Returns     : none.
*
* Notes       : none.
***************************************************************************************************
*/
void AT25256B_Write_SR(uint8_t sr)   
{   
	AT25256B_CS;                            
	SPI3_ReadWriteByte(WRSR);//����дȡ״̬�Ĵ�������    
	SPI3_ReadWriteByte(sr);  //д��һ���ֽ�  
	AT25256B_NCS;                            	      
} 

/*
***************************************************************************************************
*                                    AT25256B_Wait_Ready()
*
* Description : Wait the AT25256 is ready.
*
* Arguments   : none.
*
* Returns     : none.
*
* Notes       : none.
***************************************************************************************************
*/
static void AT25256B_Wait_Ready(void)   
{   
	while((AT25256B_ReadSR()&0x01)==0);  		// �ȴ�RDYλΪ0
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
void AT25256B_Read(u8* pBuffer,u16 ReadAddr,u16 NumByteToRead)   
{   
 	u16 i;   										    
	AT25256B_CS;                        //ʹ������   
    SPI3_ReadWriteByte(READ);         	//���Ͷ�ȡ����    	
    SPI3_ReadWriteByte((u8)((ReadAddr)>>8)); //����16bit��ַ��λ      
    SPI3_ReadWriteByte((u8)ReadAddr);        //����16bit��ַ��λ  
    for(i=0;i<NumByteToRead;i++)
	{ 
        pBuffer[i]=SPI3_ReadWriteByte(0xFF);   	//ѭ������  
    }
	AT25256B_NCS;  				    	      
}  
  
/*
***************************************************************************************************
*                                    AT25256B_Write()
*
* Description : ��ָ����ַ��ʼдָ�����ȵ�����.
*
* Arguments   : pBuffer:���ݴ洢��,WriteAddr:��ʼд��ĵ�ַ,NumByteToWrite:Ҫд����ֽ��� (���65535).
*
* Returns     : none.
*
* Notes       : none.
***************************************************************************************************
*/   
void AT25256B_Write(u8* pBuffer,u16 WriteAddr,u16 NumByteToWrite)
{
 	u16 i;  
    
    AT25256B_CS;
    AT25256B_Write_Enable();                  	                          	
    SPI3_ReadWriteByte(WRITE);      	        //����д����     
    SPI3_ReadWriteByte((u8)((WriteAddr)>>8));   //����16bit��ַ��λ 
    SPI3_ReadWriteByte((u8)WriteAddr);          //����16bit��ַ��λ  
    
    for(i=0;i<NumByteToWrite;i++)SPI3_ReadWriteByte(pBuffer[i]);//ѭ��д��  
    
	AT25256B_CS;                            	//ȡ��Ƭѡ 
	AT25256B_Wait_Ready();					   	//�ȴ�д�����
    AT25256B_Write_Disable();                   //д����
}

