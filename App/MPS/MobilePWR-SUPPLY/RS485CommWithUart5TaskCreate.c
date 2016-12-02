#include <includes.h>
#include "RS485CommWithUart5TaskCreate.h"

#if 1
#define RS485_COMMU_WITH_UART5_TASK_SIZE 200


OS_TCB RS485CommWithUart5TaskTCB ;
static CPU_STK RS485_COMMU_WITH_UART5_TASK_STK[RS485_COMMU_WITH_UART5_TASK_SIZE];


void RS485CommWithUart5TaskCreate(void)
{
    OS_ERR  err;   
    //串口5的初始化
    BSP_SerToRS485_Init(9600);     
    OSTaskCreate((OS_TCB *)&RS485CommWithUart5TaskTCB,                    // Create the start task
                 (CPU_CHAR *)"RS485 Communicate Task Start",
                 (OS_TASK_PTR) RS485CommWithUart5Task,
                 (void *) 0,
                 (OS_PRIO) RS485_COMMUNICATE_WITH_UART5_TASK_PRIO,
                 (CPU_STK *)&RS485_COMMU_WITH_UART5_TASK_STK[0],
                 (CPU_STK_SIZE) RS485_COMMU_WITH_UART5_TASK_SIZE / 10,
                 (CPU_STK_SIZE) RS485_COMMU_WITH_UART5_TASK_SIZE,
                 (OS_MSG_QTY) 5u,
                 (OS_TICK) 0u,
                 (void *) 0,
                 (OS_OPT)(OS_OPT_TASK_STK_CHK | OS_OPT_TASK_STK_CLR),
                 (OS_ERR *)&err);
    APP_TRACE_INFO(("Created RS485 communication Task, and err code is %d...\n\r", err));  
}


void RS485CommWithUart5Task(void *p_arg)
{
    OS_ERR  err;
//    OSTaskSuspend(NULL, &err);
    APP_TRACE_INFO(("RS485 Communicate Task Start...\n\r"));
    u16 j, sum = 0;
    u16 GetAdress = 0x1c7;
    u16 SetVvalue[11] = {0x00, 0x08, 0xC8, 0x42, 0x55, 0x55, 0x00, 0x00, 0x00, 0x00, 0x00};
    u16 Value_temp = Vvalue;
    u16 Ivalue_temp = Ivalue;
   
    SetVvalue[6] = (Value_temp & 0xFF00) >> 8;
    SetVvalue[7] = (Value_temp & 0x00FF);
    SetVvalue[8] = (Ivalue_temp & 0xFF00) >> 8;
    SetVvalue[9] = (Ivalue_temp & 0x00FF);   
    
    for(j = 0; j < 10; j++)
    {
        sum = sum + SetVvalue[j];
    }
    SetVvalue[10] = sum & 0xFF;
    
    while(DEF_TRUE)
    {
        APP_TRACE_INFO(("RS485 Communicate Task Start...\n\r"));
               
        RS485_Send_Data1( &GetAdress,1 );
        
        OSTimeDlyHMSM(0, 0,2, 000,
            OS_OPT_TIME_HMSM_STRICT,
            &err);
        
        RS485_Send_Data1( &SetVvalue[0],11 );    
        OSTimeDlyHMSM(0, 0, 2, 000,
              OS_OPT_TIME_HMSM_STRICT,
              &err);

     }   
}

#endif

