#ifndef __LINDRIVER_H
#define __LINDRIVER_H
#include "main.h"    
#include "usart.h" 
 
#define  LIN_BUFFER_SIZE             64
 
 
//LIN����״̬
typedef enum{ 
	BREAK_GET = 0,
	SYNCH_GET,
	PID_GET,
	MSG_GET,
	CHECKSUM_GET, 
}LIN_RxState; 
 
 
//LIN�������
typedef enum{ 
	LIN_OK = 0, 
	FIFO_VOID,        //������
	SYNC_ERR,         //ͬ���δ���
	PID_ERR,          //PID����	
	NO_RESPONES,      //����Ӧ	
	CHECK_ERR,        //���ݳ��ȴ���	
	FORMAT_ERR        //У��ʹ���
}LIN_ERROR_Code;
 
 
//LIN��Ϣ�ṹ
typedef struct{
  uint8_t Sync;         //ͬ���Σ��̶�ֵ0x55
	uint8_t FrameID;      //֡ID
  uint8_t PID;          //PID
  uint8_t DataLen;      //LIN���ݶ���Ч�ֽ���
  uint8_t Data[8];      //���ݶ�(LIN�涨���ݳ����8�ֽ�)
  uint8_t Checksum;     //У���
}LIN_MSG;
 
 
void LIN_MasterRxMsg(uint8_t Data);
uint8_t LIN_GetPID(uint8_t FrameID);
void LIN_SendBreak(void);
void LIN_TimOutCmd(TIM_TypeDef* TIMx, FunctionalState NewState);
void LIN_SendBytes(USART_TypeDef* USARTx,uint8_t* pData,uint8_t DataLen);
uint8_t LIN_GetChecksum(uint8_t PID, uint8_t* pData,uint8_t DataLen,uint8_t flag);
 
 
#endif
