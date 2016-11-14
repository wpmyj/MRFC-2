#ifndef __APP_SPEED_CONTROL_DEVICE_MONITOR_TASK_H__
#define __APP_SPEED_CONTROL_DEVICE_MONITOR_TASK_H__


#include "stm32f10x.h"


typedef enum
{
    PUMP_SPD_MONITOR_SWITCH = 0,
    HYDROGEN_FAN_SPD_MONITOR_SWITCH,
    STACK_FAN1_SPD_MONITOR_SWITCH,
    STACK_FAN2_SPD_MONITOR_SWITCH,

} SPEED_MONITOR_SWITCH_STATUS_Typedef;


void SpdCtlDevMonitorTaskCreate(void);


void SetHydrgProducerPumpRunningSpeedMonitorHookSwitch(u8 i_NewStatu);
void SetHydrgProducerFanRunningSpeedMonitorHookSwitch(u8 i_NewStatu);
//void SetStackFan1RunningSpeedMonitorHookSwitch(u8 i_NewStatu);
//void SetStackFan2RunningSpeedMonitorHookSwitch(u8 i_NewStatu);


SWITCH_TYPE_VARIABLE_Typedef GetHydrgProducerPumpRunningSpeedMonitorSwitchStatus(void);
SWITCH_TYPE_VARIABLE_Typedef GetHydrgProducerFanRunningSpeedMonitorSwitchStatus(void);
SWITCH_TYPE_VARIABLE_Typedef GetStackFan1RunningSpeedMonitorSwitchStatus(void);
SWITCH_TYPE_VARIABLE_Typedef GetStackFan2RunningSpeedMonitorSwitchStatus(void);

#endif

