#ifndef __APP_TOP_TASK_H
#define __APP_TOP_TASK_H

//�Լ������ͱ���
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

//�����豸�ػ�������Ӧ��ʶ
typedef enum
{
    EN_STOP_ALL_DIRECT,     //ֱ�ӹػ�
    EN_DELAY_STOP_PART_ONE, //��ʱ5���ӹرհ��1(�����)����
    EN_DELAY_STOP_PART_TWO, //��ʱ3���ӹرհ��2(���)����
    EN_DELAY_STOP_BOTH_PARTS,   //��ʱ3���ӹر�������
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

