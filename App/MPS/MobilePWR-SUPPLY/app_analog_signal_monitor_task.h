#ifndef __APP_ANALOG_SIGNAL_MONITOR_TASK_H__
#define __APP_ANALOG_SIGNAL_MONITOR_TASK_H__


void    AnaSigMonitorTaskCreate(void);

void    SetHydrgProducerDigSigIgniteFirstTimeBehindMonitorHookSwitch(unsigned char);
void    SetHydrgProducerDigSigAlarmRunningMonitorHookSwitch(unsigned char);
void    SetHydrgProducerAnaSigAlarmRunningMonitorHookSwitch(unsigned char);
void    SetHydrgProducerAnaSigRunningStartAutoAdjHookSwitch(unsigned char);
#endif
