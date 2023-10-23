#define USE_EVENT 0

#include <api_os.h>
#include <api_hal_uart.h>
#include <api_debug.h>
 #include <string.h>
 #include <stdio.h>
#include "pn532.h"


#if USE_EVENT
#include "api_os.h"
#include "api_event.h"
#endif


#define MAIN_TASK_STACK_SIZE    (1024 * 2)
#define MAIN_TASK_PRIORITY      0 
#define MAIN_TASK_NAME         "UART Test Task"

static HANDLE uartTaskHandle = NULL;

static void OnUart1ReceivedData(UART_Callback_Param_t param)
{
    char buffer[256];
    int offset = 0;

    //UART_Write(UART1,param.buf,param.length);
    // Trace(1,"uart1 interrupt received data,length:%d,read:,data:%s",param.length,param.buf);
    // for (int i = 0; i < param.length; i++) {
    //     Trace(1,"%02x ",(uint8_t)param.buf[i]);
    // }
    
    for (int i = 0; i < param.length; i++) {
        offset += snprintf(buffer + offset, sizeof(buffer) - offset, "%02x ",(uint8_t)param.buf[i]);
        //Trace(1,"%02x ",(uint8_t)param.buf[i]);
        uartBuffer[uartBufferIndex] = (uint8_t)param.buf[i];
        uartBufferIndex++;
    }
    fd = true;
    Trace(1,"Received %s ",buffer);
}

#if USE_EVENT
static void EventDispatch(API_Event_t* pEvent)
{
    switch(pEvent->id)
    {
        case API_EVENT_ID_UART_RECEIVED:
            Trace(1,"uart received API_EVENT_ID_UART_RECEIVED:%d",pEvent->param1);
            if(pEvent->param1 == UART1)
            {
                uint8_t data[pEvent->param2+1];
                data[pEvent->param2] = 0;
                memcpy(data,pEvent->pParam1,pEvent->param2);
                //UART_Write(UART1,data,pEvent->param2);
                Trace(1,"uart received data,length:%d,data:%s",pEvent->param2,data);
            }
            break;
        default:
            break;
    }
}
#endif

static void uart_MainTask()
{
    uint8_t buff[255];
    //uint8_t uid[MIFARE_UID_MAX_LENGTH];
    //int32_t uid_len = 0;
    PN532 pn532;

    UART_Config_t config = {
        .baudRate = UART_BAUD_RATE_115200,
        .dataBits = UART_DATA_BITS_8,
        .stopBits = UART_STOP_BITS_1,
        .parity   = UART_PARITY_NONE,
        .rxCallback = OnUart1ReceivedData,
        .useEvent = false,
    };
   // uint32_t times = 0;
#if USE_EVENT
    API_Event_t* event=NULL;
    config.useEvent = true;
    config.rxCallback = NULL;
#endif
    UART_Init(UART1,config);
    config.rxCallback = NULL;
    UART_Init(UART2,config);
    
    PN532_UART_Init(&pn532);
    OS_Sleep(50);
    PN532_Reset();
    
    Trace(1,"Build 22: A");


    if (PN532_GetFirmwareVersion(&pn532, buff) == PN532_STATUS_OK) {
        Trace(1,"First Found PN532 with firmware version: %d.%d\r\n", buff[1], buff[2]);
    }else{
        Trace(1,"GetFirmwareVersion Failed!");
    }

    while(1)
    {
        if(config.useEvent == false)
        {
            // uint8_t temp[20];
            // static int times = 0;
            // uint8_t buffer[50];

            PN532_UART_Wakeup();
            PN532_Reset();

            if (PN532_GetFirmwareVersion(&pn532, buff) == PN532_STATUS_OK) {
                Trace(1,"Found PN532 with firmware version: %d.%d\r\n", buff[1], buff[2]);
            }else{
                Trace(1,"GetFirmwareVersion Failed!");
            }
//             uint8_t data[] = {0x55, 0x55, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0x03, 0xFD, 0xD4, 0x14, 0x01, 0x17, 0x00};
    
//             snprintf(temp,20,"hello:%d\n",++times);
// //            UART_Write(UART1,temp,strlen(temp)+1);
//             UART_Write(UART1,data,sizeof(data));
            Trace(1,"Build 22: B");

            OS_Sleep(10);

            //memset(buffer,0,sizeof(buffer));
            // uint32_t readLen = UART_Read(UART1,buffer,10,3000);
            // Trace(1,"UART_Read uart2,readLen:%d,data:%s",readLen,buffer);
            OS_Sleep(10000);
            PN532_Reset();
        }
        else{
#if USE_EVENT
            if(OS_WaitEvent(uartTaskHandle, &event, OS_TIME_OUT_WAIT_FOREVER))
            {
                EventDispatch(event);
                OS_Free(event->pParam1);
                OS_Free(event->pParam2);
                OS_Free(event);
            }
#endif
        }
    }
}

void uart_Main(void)
{
    uartTaskHandle = OS_CreateTask(uart_MainTask ,
        NULL, NULL, MAIN_TASK_STACK_SIZE, MAIN_TASK_PRIORITY, 0, 0, MAIN_TASK_NAME);
    OS_SetUserMainHandle(&uartTaskHandle);
}

