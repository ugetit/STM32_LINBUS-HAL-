#ifndef __LINBUFFER_H
#define __LINBUFFER_H
#include "main.h"                 
#include "lin_driver.h"
 
#define  LIN_RINGBUF_LEN  64             //缓冲区长度
 
typedef struct{
	uint16_t Head;						//队列头
	uint16_t Tail;						//队列尾
	uint16_t Length; 					//保存的数据长度
	uint16_t BufOutcnt;                	//溢出计数
	LIN_MSG  LINBuffer[LIN_RINGBUF_LEN]; 	//缓冲区	
}LIN_BUFFER;
 
 
uint16_t LIN_RingBUF_ReadLen(LIN_BUFFER* pLINBuff);
void LIN_RingBUF_Init(LIN_BUFFER* pLINBuff);
void LIN_RingBUF_Write(LIN_BUFFER* pLINBuff);
void LIN_RingBUF_ClearRxMsg(LIN_BUFFER* pLINBuf);
void LIN_Rx_data(uint8_t PID, uint8_t* pData,uint8_t DataLen);
LIN_MSG* LIN_RingBUF_GetMsg(LIN_BUFFER* pLINBuff);
int LIN_RingBUF_Read(LIN_BUFFER* pLINBuff, LIN_MSG** pLINMsg);
 
#endif  

