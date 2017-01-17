/*
***************************************************************************************************
*                                         APPLICATION CODE
*
*                      (c) Copyright 2016; Guangdong ENECO Science And Technology Co.,Ltd
*
*               All rights reserved.  Protected by international copyright laws.
*               Knowledge of the source code may NOT be used without authorization.
***************************************************************************************************
*/
/*
***************************************************************************************************
* Filename      : bsp_MF210.c
* Version       : V1.00
* Programmer(s) : JiaCai.He
***************************************************************************************************
*/
/*
***************************************************************************************************
*                                             INCLUDE FILES
***************************************************************************************************
*/
#include <string.h>
#include "bsp_MF210.h"
#include "bsp_ser.h"
#include "stm32f10x.h"
#include "os.h"
#include <bsp.h>

/*
***************************************************************************************************
*                                       MICRO DEFINE
***************************************************************************************************
*/
#define STRCMP(str1, str2) strncmp(str1, str2, strlen(str2))
#define SERVER_IP_PORT   "120.25.205.146,40304"

#define DEBUG_3G 1
#define GSM 1
#define WCDMA 2
#define NETWORK GSM

#if(DEBUG_3G)
    #define DEBUG_PRINTF BSP_Ser_Printf
#else
#define DEBUG_PRINTF(...)
#endif


typedef struct
{
    char *cmd;
    char *ask;
} cmd_ask_list_t;

typedef enum
{
    CLOSE_ECHO,
    CHECK_FUNCTIONALITY,
    CHECK_SIM_CARD,
    SET_NETWORK,
    CHECK_NETWORK,
    SET_ACCESS_POINT,
    CHECK_PSCALL,
    START_PS_CALL,
    CHECK_SOCKET_STATUS,
    CONNECT_TO_SERVER,
    DISCONNECT_TO_SERVER,
} module_status;

module_status current_status = CLOSE_ECHO;

const cmd_ask_list_t init_list[] =
{
    {"ATE0\r\n", "\r\nOK\r\n"},
    {"AT+CFUN?\r\n", "\r\n+CFUN: 1\r\n\r\nOK\r\n"},
    {"AT+CPIN?\r\n", "\r\n+CPIN: READY\r\n\r\nOK\r\n"},
#if(NETWORK == GSM)
    {"AT+ZSNT=1,0,0\r\n", "\r\nOK\r\n"},
    {"AT+ZPAS?\r\n", "\r\n+ZPAS: \"EDGE\",\"CS_PS\"\r\n\r\nOK\r\n"},
#else
    {"AT+ZSNT=2,0,0\r\n", "\r\nOK\r\n"},
    {"AT+ZPAS?\r\n", "\r\n+ZPAS: \"UMTS\",\"CS_PS\"\r\n\r\nOK\r\n"},
#endif
    {"AT+ZIPCFG=UNINET\r\n", "\r\nOK\r\n"},
    {"AT+ZIPCALL?\r\n", "\r\n+ZIPCALL: %d\r\n\r\nOK\r\n"},
    {"AT+ZIPCALL=1\r\n", "\r\nOK\r\n"},
    {"AT+ZIPSTAT=1\r\n", "\r\n+ZIPSTAT: 1,%d\r\n\r\nOK\r\n"},
    {"AT+ZIPOPEN=1,0,"SERVER_IP_PORT"\r\n", "\r\nOK\r\n"},
    {"AT+ZIPCLOSE=1\r\n", "\r\nOK\r\n"},
    {0, 0},
};

static char send_str[1024] = {0};
const char index[] = "0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZ";
static uint8_t reconnect_server_flag = 5;

bool module_control(const char *send, const char *ask, uint8_t *ret)
{
    char index_string[50] = {0};
    char *find_ret = NULL;
    uint8_t count = 0;
    OS_ERR  err;

    sscanf(ask, "%[^%]", index_string);
    CleanUsartRecvBuf(USART2);
    SendString(USART2, send);

    do
    {
        OSTimeDlyHMSM(0, 0, 0, 10, OS_OPT_TIME_HMSM_STRICT, &err);
        find_ret = strstr((const char *)usart2_recv_buff, index_string);

        if(find_ret != NULL)
        {
            if(strlen(ask) != strlen(index_string))
            {
                find_ret += strlen(index_string);

                if(*find_ret >= '0' && *find_ret <= '9')
                {
                    *ret = *find_ret - '0';
                    return true;
                }
            }
            else
            {
                return true;
            }
        }

        count++;
    }
    while(count < 10);

    return false;
}
#if(DEBUG_3G)
const char *status_str[] =
{
    "CLOSE_ECHO",
    "CHECK_FUNCTIONALITY",
    "CHECK_SIM_CARD",
    "SET_NETWORK",
    "CHECK_NETWORK",
    "SET_ACCESS_POINT",
    "CHECK_PSCALL",
    "START_PS_CALL",
    "CHECK_SOCKET_STATUS",
    "CONNECT_TO_SERVER",
    "DISCONNECT_TO_SERVER",
};
#endif

/*
***************************************************************************************************
*                                 MF210v2_ctrl()
*
* Description:  MF210 当前状态显示.
*
* Arguments  :  none
*
* Returns    :  返回指令状态
***************************************************************************************************
*/
bool MF210v2_ctrl(void)
{
    bool success = false;
    uint8_t ret = 0;
    static uint8_t retry_time = 0;
    DEBUG_PRINTF("current status : %s\r\n", status_str[current_status]);
    success = module_control(init_list[current_status].cmd, init_list[current_status].ask, &ret);

    switch(current_status)
    {
        case CLOSE_ECHO:
        case CHECK_FUNCTIONALITY:
        case CHECK_SIM_CARD:
        case SET_NETWORK:
        case CHECK_NETWORK:
        case SET_ACCESS_POINT:
        {

            if(success)
            {
                retry_time = 0;
                current_status++;
            }
            else if(current_status != CLOSE_ECHO)
            {
                current_status--;
            }

        }
        break;

        case CHECK_PSCALL:
        case CHECK_SOCKET_STATUS:
        {

            if(success)
            {
                if(ret == 1)
                {
                    retry_time = 0;

                    if(current_status == CHECK_PSCALL)
                    {
                        current_status = CHECK_SOCKET_STATUS;
                        return false;
                    }
                    else
                    {
                        return true;
                    }
                }
                else if(ret == 0)
                {
                    current_status++;
                }
            }

            if(retry_time++ > 5)
            {
                retry_time = 0;
                /* do something */
                current_status = CLOSE_ECHO;
            }
        }
        break;

        case START_PS_CALL:
        case CONNECT_TO_SERVER:
        {

            if(success)
            {
                current_status--;
            }
            else
            {
                current_status = SET_ACCESS_POINT;
            }
        }
        break;

        case DISCONNECT_TO_SERVER:
        {
            current_status = success ? CONNECT_TO_SERVER : SET_ACCESS_POINT;
        }
        break;

        default:
        {
            current_status = SET_ACCESS_POINT;
            retry_time = 0;
        }
        break;
    }

    return false;
}


bool SocketSend(uint8_t socket_id, uint8_t *send, uint16_t send_number)
{
    char *p_send = send_str + 13;

    if(reconnect_server_flag == 0)
    {
        current_status = DISCONNECT_TO_SERVER;
        reconnect_server_flag = 10;
    }

    if(MF210v2_ctrl())
    {
        sprintf(send_str, "AT+ZIPSEND=%d,", socket_id);

        for(int i = 0; i < send_number; i++)
        {
            *p_send++ = index[*send / 16];
            *p_send++ = index[*send++ % 16];
        }

        *p_send = 0;

        strcat(send_str, "\r\n");

        if(module_control(send_str, "\r\nOK\r\n", NULL))
        {
            reconnect_server_flag--;
            DEBUG_PRINTF("reconnect_server_flag = %d\r\n", reconnect_server_flag);
            return true;
        }
        else
        {
            current_status = CLOSE_ECHO;
        }
    }

    return false;
}

static char recv_str[1024] = {0};

uint16_t SocketRecv(uint8_t socket_id, uint8_t *recv)
{
    u16 i = 0;
    char *tmp = strstr((const char *)usart2_recv_buff, "+ZIPRECV:");

    if(tmp != NULL)
    {
        int length = 0;

        if(strchr(tmp, 0x0a))
        {
            sscanf(tmp, "+ZIPRECV: 1,"SERVER_IP_PORT",%d,%s", &length, recv_str);

           DEBUG_PRINTF("recv_str(%d) = %s \r\n", recv_cursor, recv_str);

            for(int i = 0; i < length; i++)
            {
                sscanf(recv_str + i * 2, "%2x", (uint32_t *)(recv + i));
            }

            recv[length] = 0;
            CleanUsartRecvBuf(USART2);

            if(recv[0] == 'O' && recv[1] == 'K')
            {
//                APP_TRACE_INFO(("receive server answer\r\n"));
                
                reconnect_server_flag = 10;
                return 0;
//                APP_TRACE_INFO(("receive server answer\r\n"));
//                reconnect_server_flag = 10;
//                return 0;
            }
            else
            {
                for( i = 0; i < length; i++)
                {
                    APP_TRACE_INFO(("recv: 0x%x\r\n",recv[i]));
                }

                return length;
            }
        }

        return 0;
    }

    return 0;
}

