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
////4Kbytes为一个Sector扇区
////16个扇区为1个Block
////容量为16M字节,共有128个Block,4096个Sector 												 
void AT25256B_Init(void)
{	
    GPIO_InitTypeDef GPIO_InitStructure;
	RCC_APB2PeriphClockCmd(	RCC_APB2Periph_GPIOD, ENABLE );//PORTd时钟使能 

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3 | GPIO_Pin_4;// PB12 推挽 
 	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;  //推挽输出
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
 	GPIO_Init(GPIOB, &GPIO_InitStructure);
 	GPIO_SetBits(GPIOB,GPIO_Pin_12);
 
    AT25256B_NCS;//SPI FLASH不选中
	SPI3_Init();//初始化SPI3

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
*               WEN:写使能锁定;RDY:忙标记位(1,忙;0,空闲)
*               WPEN、WEN、BP1、BP0:FLASH区域写保护设置;BP1、BP0决定保护区域
*               Bit 0~7都为1表示正处于内部写周期;Bit4~6都为0表示没有处于内部写周期
***************************************************************************************************
*/
uint16_t AT25256B_ReadSR(void)   
{  
	uint16_t byte=0;   
	AT25256B_CS;                                 //选中器件   
	SPI3_ReadWriteByte(AT_READ_STATUS_REGISTER); //发送读取状态寄存器命令    
	byte=SPI3_ReadWriteByte(0xFFFF);             //读取一个字节  
	AT25256B_NCS;                                //取消片选      
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
* Notes       : 只有SPR,TB,BP2,BP1,BP0(bit 7,5,4,3,2)可以写.
***************************************************************************************************
*/
void AT25256B_Write_SR(u8 sr)   
{   
	AT25256B_CS;                            //使能器件   
	SPI3_ReadWriteByte(AT_WRITE_STATUS_REGISTER);//发送写取状态寄存器命令    
	SPI3_ReadWriteByte(sr);               	//写入一个字节  
	AT25256B_NCS;                             //取消片选     	      
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
* Notes       : 只有SPR,TB,BP2,BP1,BP0(bit 7,5,4,3,2)可以写.
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
* Notes       : 只有SPR,TB,BP2,BP1,BP0(bit 7,5,4,3,2)可以写.
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
* Description : 读取SPI FLASH，在指定地址开始读取指定长度的数据.
*
* Arguments   : pBuffer:数据存储区，ReadAddr:开始读取的地址(24bit)，NumByteToRead:要读取的字节数(最大65535).
*
* Returns     : none.
*
* Notes       : none.
***************************************************************************************************
*/   		    
void AT25256B_Read(u8* pBuffer,u32 ReadAddr,u16 NumByteToRead)   
{ 
 	u16 i;   										    
	AT25256B_CS;                            	//使能器件   
    SPI3_ReadWriteByte(AT_READ_DATA);         	//发送读取命令   
    SPI3_ReadWriteByte((u8)((ReadAddr)>>16));  	//发送24bit地址    
    SPI3_ReadWriteByte((u8)((ReadAddr)>>8));   
    SPI3_ReadWriteByte((u8)ReadAddr);   
    for(i=0;i<NumByteToRead;i++)
	{ 
        pBuffer[i]=SPI3_ReadWriteByte(0xFF);   	//循环读数  
    }
	AT25256B_CS;  				    	      
}  
  
   
