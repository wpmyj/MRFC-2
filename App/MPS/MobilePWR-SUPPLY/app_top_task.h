#ifndef __APP_TOP_TASK_H
#define __APP_TOP_TASK_H

//自检结果类型变量
typedef enum
{
    EN_THROUGH,
    EN_NOT_THROUGH = !EN_THROUGH,
} VERIFY_RESULT_TYPE_VARIABLE_Typedef;

typedef enum
{
    EN_PASS,
    EN_NOT_PASS = !EN_PASS,
} IGNITE_CHECK_STATU_Typedef;

//调速设备关机动作响应标识
typedef enum
{
    EN_STOP_ALL_DIRECT,     //直接关机
    EN_DELAY_STOP_PART_ONE, //延时5分钟关闭半机1(制氢机)部分
    EN_DELAY_STOP_PART_TWO, //延时3分钟关闭半机2(电堆)部分
    EN_DELAY_STOP_BOTH_PARTS,   //延时3分钟关闭两部分
} SYSTEM_SHUT_DOWN_ACTION_FLAG_Typedef;




VERIFY_RESULT_TYPE_VARIABLE_Typedef     CheckAuthorization(void);
VERIFY_RESULT_TYPE_VARIABLE_Typedef     WaittingCommand(void);
void                                    Starting(void);
void                                    Running(void);
void                                    KeepingWarm(void);
//static    void                        ShutDown(void);//Be static, declare in the c file
void                                    DeviceFaultAlarm(void);
void                                    ResetDeviceAlarmStatu(void);

void    SetShutDownActionFlag(SYSTEM_SHUT_DOWN_ACTION_FLAG_Typedef);

void    UpdateBuzzerStatuInCruise(void);


u16 GetStartRemainSencond(void);
#endif

