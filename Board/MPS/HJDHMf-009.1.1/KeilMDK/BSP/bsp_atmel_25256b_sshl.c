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
 	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;  //推挽输出
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
 	GPIO_Init(GPIOD, &GPIO_InitStructure);
    
    GPIO_ResetBits(GPIOD,BSP_GPIOD_FLASH_HOLD_PORT_NMB | BSP_GPIOD_FLASH_WP_PORT_NMB);//拉低，保持数据传输和写保护
    
    AT25256B_NCS;//SPI FLASH不选中
	SPI3_Init();//初始化SPI3

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
    GPIO_SetBits(GPIOD,BSP_GPIOD_FLASH_WP_PORT_NMB);//写使能时必须把WP拉高
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
*               WEN:写使能锁定;RDY:忙标记位(1,忙;0,空闲)
*               WPEN、WEN、BP1、BP0:FLASH区域写保护设置;BP1、BP0决定保护区域
*               Bit 0~7都为1表示正处于内部写周期;Bit4~6都为0表示没有处于内部写周期
***************************************************************************************************
*/
uint8_t AT25256B_ReadSR(void)   
{  
	uint8_t byte=0; 
    
	AT25256B_CS;                                 //选中器件   
	SPI3_ReadWriteByte(RDSR);                    //发送读取状态寄存器命令    
	byte=SPI3_ReadWriteByte(0xFF);               //读取一个字节,发送除指令数据外的任意数据 
	AT25256B_NCS;                                //取消片选
    
	return byte;   
} 
/*
***************************************************************************************************
*                                    AT25256B_Write_SR()
*
* Description : Write atmel 25256b status register.
*
* Arguments   : 0x82:10000010-开启写保护,BP1和BP0也可以写.
*
* Returns     : none.
*
* Notes       : none.
***************************************************************************************************
*/
void AT25256B_Write_SR(uint8_t sr)   
{   
	AT25256B_CS;                            
	SPI3_ReadWriteByte(WRSR);//发送写取状态寄存器命令    
	SPI3_ReadWriteByte(sr);  //写入一个字节  
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
	while((AT25256B_ReadSR()&0x01)==0);  		// 等待RDY位为0
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
void AT25256B_Read(u8* pBuffer,u16 ReadAddr,u16 NumByteToRead)   
{   
 	u16 i;   										    
	AT25256B_CS;                        //使能器件   
    SPI3_ReadWriteByte(READ);         	//发送读取命令    	
    SPI3_ReadWriteByte((u8)((ReadAddr)>>8)); //发送16bit地址高位      
    SPI3_ReadWriteByte((u8)ReadAddr);        //发送16bit地址低位  
    for(i=0;i<NumByteToRead;i++)
	{ 
        pBuffer[i]=SPI3_ReadWriteByte(0xFF);   	//循环读数  
    }
	AT25256B_NCS;  				    	      
}  
  
/*
***************************************************************************************************
*                                    AT25256B_Write()
*
* Description : 从指定地址开始写指定长度的数据.
*
* Arguments   : pBuffer:数据存储区,WriteAddr:开始写入的地址,NumByteToWrite:要写入的字节数 (最大65535).
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
    SPI3_ReadWriteByte(WRITE);      	        //发送写命令     
    SPI3_ReadWriteByte((u8)((WriteAddr)>>8));   //发送16bit地址高位 
    SPI3_ReadWriteByte((u8)WriteAddr);          //发送16bit地址低位  
    
    for(i=0;i<NumByteToWrite;i++)SPI3_ReadWriteByte(pBuffer[i]);//循环写数  
    
	AT25256B_CS;                            	//取消片选 
	AT25256B_Wait_Ready();					   	//等待写入结束
    AT25256B_Write_Disable();                   //写禁能
}

