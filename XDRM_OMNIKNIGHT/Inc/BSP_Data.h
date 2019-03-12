#ifndef __BSP_DATA_H
#define __BSP_DATA_H

#define BUFFER_MAX  500      //缓冲区大小

#include "stm32f4xx_hal.h"
#include "config.h"

#include "BSP_CAN.h"//这个.h本不应该写在这里的，但是can的那些结构体想不出放的地方了，就放这里

typedef struct _circle_buffer{
    uint16_t head_pos;             //缓冲区头部位置
    uint16_t tail_pos;             //缓冲区尾部位置   
    unsigned char circle_buffer[BUFFER_MAX];    //缓冲区数组 
}CIRCLE_BUFF_t;

void bufferPush(CIRCLE_BUFF_t* buffer, const unsigned char _buf);
int bufferPop(CIRCLE_BUFF_t* buffer, unsigned char* _buf);
uint8_t buffer_multiPop(CIRCLE_BUFF_t* buffer, unsigned char* rx_buf, uint16_t num);
uint16_t bufferlen(CIRCLE_BUFF_t* buffer);


extern CIRCLE_BUFF_t Que_JudgeFrame;
extern CIRCLE_BUFF_t Que_USART3_Tx;
extern CIRCLE_BUFF_t Que_MiniPC;

//------------------------CAN结构体类型队列-------------------------




void CAN_bufferPush(CAN_CIRCLE_BUFF_t* buffer, CanTxMsgTypeDef _buf);
int CAN_bufferPop(CAN_CIRCLE_BUFF_t* buffer, CanTxMsgTypeDef* _buf);
int CAN_buffer_multiPop(CAN_CIRCLE_BUFF_t* buffer, CanTxMsgTypeDef* rx_buf, uint16_t num);
uint16_t CAN_bufferlen(CAN_CIRCLE_BUFF_t* buffer);
extern CAN_CIRCLE_BUFF_t Que_CAN1_Tx;
extern CAN_CIRCLE_BUFF_t Que_CAN2_Tx;

#endif



