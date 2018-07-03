#ifndef _NTC_RESISTANCE_H_
#define _NTC_RESISTANCE_H_

#include "stdint.h"


#define     PAREL_TEST      0
#define     BALLARD_TEST    1  


typedef struct {
    int8_t temp;
    uint32_t resistance;
} NTC_PARAMETER_TABLE_Typedef;


int8_t GetSrcTemp(double Rt_Value);


#if  PAREL_TEST
int8_t GetSourceTemp(double Rt_Value);
#endif


#if  BALLARD_TEST
uint8_t GetSourceTemp(double Rt_Value);
#endif






#endif



