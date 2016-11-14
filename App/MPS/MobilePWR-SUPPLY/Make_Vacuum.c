
#include "Make_Vacuum.h"
#include <includes.h>

#define MAKE_VACCUUM_TASK_STK_SIZE 200                                      

OS_TCB Make_Vaccuum_FunctionTaskTCB;                                            
static CPU_STK    MAKE_VACCUUM_TASK_STK[MAKE_VACCUUM_TASK_STK_SIZE];              

void Make_Vacuum_FunctionTaskCreate(void)
{
    OS_ERR  err;   

    
    OSTaskCreate((OS_TCB *)&Make_Vaccuum_FunctionTaskTCB,                 
                 (CPU_CHAR *)"Make Vaccuum Function Task Start",
                 (OS_TASK_PTR)Make_Vacuum_FunctionTask,
                 (void *) 0,
                 (OS_PRIO) Make_Vaccuum_Task_PRIO,
                 (CPU_STK *)&MAKE_VACCUUM_TASK_STK[0],
                 (CPU_STK_SIZE) MAKE_VACCUUM_TASK_STK_SIZE / 10,
                 (CPU_STK_SIZE) MAKE_VACCUUM_TASK_STK_SIZE,
                 (OS_MSG_QTY) 5u,
                 (OS_TICK) 0u,
                 (void *) 0,
                 (OS_OPT)(OS_OPT_TASK_STK_CHK | OS_OPT_TASK_STK_CLR),
                 (OS_ERR *)&err);
    APP_TRACE_INFO(("Created Make Vaccuum  Task, and err code is %d...\n\r", err));  
}

u16 g_Make_Vaccum_Flag = 0;

void  Make_Vacuum_FunctionTask(void *p_arg)                   //����պ�������Ĵ
{
   
//    OSTaskSuspend(NULL, &err);
    APP_TRACE_INFO(("MAKE VACCUUM Task Start...\n\r"));
    while(DEF_TRUE)
    {
        if( g_Make_Vaccum_Flag )
        {
            //Ԥ����5/6/7/8ȫ���ر�
            



            
 
        }
        else
        {
            //Ԥ����5/6ͬʱ�򿪣���һ���Ӻ���Ԥ����5�ر�
            //Ԥ����6/7/8ͬʱ��


            

  
        } 
    }
}





