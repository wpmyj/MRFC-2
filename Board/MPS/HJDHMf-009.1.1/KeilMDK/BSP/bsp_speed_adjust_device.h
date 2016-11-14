#ifndef __APP_SPEED_ADJUST_DEVICE_H
#define __APP_SPEED_ADJUST_DEVICE_H


/*
*********************************************************************************************************
*                                             INCLUDE FILES
*********************************************************************************************************
*/

#include "stm32f10x.h"

/*
*********************************************************************************************************
*                                               DEFINES
*********************************************************************************************************
*/





/*
*********************************************************************************************************
*                                            FUNCTION PROTOTYPES
*********************************************************************************************************
*/

void        PumpSpdInc(void);
void        PumpSpdDec(void);
uint16_t    GetPumpCtlSpd(void);
uint16_t    GetPumpFeedBackSpd(void);
void        SetPumpCtlSpd(uint16_t);


void        HydrgFanSpdInc(void);
void        HydrgFanSpdDec(void);
uint16_t    GetHydrgFanCtlSpd(void);
uint16_t    GetHydrgFanFeedBackSpd(void);
void        SetHydrgFanCtlSpd(uint16_t);


void        StackFanSpdInc(void);
void        StackFanSpdDec(void);
uint16_t    GetStackFan1SpdFeedBack(void);
uint16_t    GetStackFan2SpdFeedBack(void);
void        SetStackFanCtlSpd(uint16_t);
uint16_t    GetStackFanCtlSpd(void);




#endif

