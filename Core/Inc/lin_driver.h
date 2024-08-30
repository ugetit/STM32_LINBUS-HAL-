#ifndef __LINDRIVER_H
#define __LINDRIVER_H
#include "main.h"    
#include "usart.h" 
 
#define  LIN_BUFFER_SIZE             64
 
 
//LIN接收状态
typedef enum{ 
	BREAK_GET = 0,
	SYNCH_GET,
	PID_GET,
	MSG_GET,
	CHECKSUM_GET, 
}LIN_RxState; 
 
 
//LIN错误代码
typedef enum{ 
	LIN_OK = 0, 
	FIFO_VOID,        //无数据
	SYNC_ERR,         //同步段错误
	PID_ERR,          //PID错误	
	NO_RESPONES,      //无响应	
	CHECK_ERR,        //数据长度错误	
	FORMAT_ERR        //校验和错误
}LIN_ERROR_Code;
 
 
//LIN消息结构
typedef struct{
  uint8_t Sync;         //同步段：固定值0x55
	uint8_t FrameID;      //帧ID
  uint8_t PID;          //PID
  uint8_t DataLen;      //LIN数据段有效字节数
  uint8_t Data[8];      //数据段(LIN规定数据长最多8字节)
  uint8_t Checksum;     //校验和
}LIN_MSG;
 
 
void LIN_MasterRxMsg(uint8_t Data);
uint8_t LIN_GetPID(uint8_t FrameID);
void LIN_SendBreak(void);
void LIN_TimOutCmd(TIM_TypeDef* TIMx, FunctionalState NewState);
void LIN_SendBytes(USART_TypeDef* USARTx,uint8_t* pData,uint8_t DataLen);
uint8_t LIN_GetChecksum(uint8_t PID, uint8_t* pData,uint8_t DataLen,uint8_t flag);
 
 
#endif
