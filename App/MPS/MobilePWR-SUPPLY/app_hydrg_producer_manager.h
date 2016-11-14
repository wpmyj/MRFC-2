#ifndef __APP_HYDRG_PRODUCER_MANAGER_H__
#define __APP_HYDRG_PRODUCER_MANAGER_H__

IGNITE_CHECK_STATU_Typedef      IgniteFirstTime(float, float, uint8_t , uint8_t);
IGNITE_CHECK_STATU_Typedef      IgniteSecondTime(float, float, uint8_t , uint8_t);

void                            SetAheadRunningFlag(WHETHER_TYPE_VARIABLE_Typedef m_NES_STATU);
WHETHER_TYPE_VARIABLE_Typedef   GetAheadRunningFlag(void);

void    IgniterWorkForSeconds(uint16_t);
void    HydrgProducerManagerTaskCreate(void);
void    HydrgProducerDlyStopTaskCreate(void);
void    IgniterWorkTaskCreate(void);

u8      GetHydrgProducerStopDlyStatu(void);

#endif
