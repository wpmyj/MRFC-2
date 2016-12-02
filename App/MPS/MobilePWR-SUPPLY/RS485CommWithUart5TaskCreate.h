
#ifndef __RS485COMMWITHWART5TASKCREATE_H
#define __RS485COMMWITHWART5TASKCREATE_H

#define Vvalue  5350
#define Ivalue  3300

#include "includes.h"

extern OS_TCB RS485CommWithUart5TaskTCB ;

void RS485CommWithUart5TaskCreate(void);
void RS485CommWithUart5Task(void *p_arg);









#endif


