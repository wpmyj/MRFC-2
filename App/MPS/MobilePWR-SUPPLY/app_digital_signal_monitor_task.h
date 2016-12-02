#ifndef __APP_DIGITAL_SIGNAL_MONITOR_TASK_H__
    #define __APP_DIGITAL_SIGNAL_MONITOR_TASK_H__

    float   GetReformerTemp(void);
    float   GetFireOrRodTemp(void);
    void    SetHydrgProducerDigSigIgniteFirstTimeBehindMonitorHookSwitch(uint8_t i_NewStatu);
    void    SetHydrgProducerDigSigAlarmRunningMonitorHookSwitch(uint8_t i_NewStatu);
    void    SetStackExhaustTimesCountPerMinutesMonitorHookSwitch(uint8_t i_NewStatu);
    void    DigSigMonitorTaskCreate(void);
#endif


