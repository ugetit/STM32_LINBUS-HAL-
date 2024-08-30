#include "linbuffer.h"
#include <string.h>
 
 
//缓冲区初始化
void LIN_RingBUF_Init(LIN_BUFFER* pLINBuff)
{
	pLINBuff->Head = 0;
	pLINBuff->Tail = 0;
	pLINBuff->Length = 0;
	pLINBuff->BufOutcnt = 0;
}
 
 
//获取LIN消息
LIN_MSG* LIN_RingBUF_GetMsg(LIN_BUFFER* pLINBuff)
{
	return  &pLINBuff->LINBuffer[pLINBuff->Tail];    //取出缓冲区内的数据
}
 
 
//写缓冲区
void LIN_RingBUF_Write(LIN_BUFFER* pLINBuff)
{
	pLINBuff->Tail = (pLINBuff->Tail + 1) % LIN_RINGBUF_LEN; //防止越界非法访问
	if (pLINBuff->Length < LIN_RINGBUF_LEN) {                //判断缓冲区是否已满
		pLINBuff->Length++;
	}else{
		pLINBuff->BufOutcnt++;
		if (pLINBuff->BufOutcnt == LIN_RINGBUF_LEN) {
			pLINBuff->BufOutcnt = 0;			
		}
		pLINBuff->Head = pLINBuff->BufOutcnt;
	}
}
 
//清空当前缓冲区
void LIN_RingBUF_ClearRxMsg(LIN_BUFFER* pLINBuf)
{
	LIN_MSG* pMsg = LIN_RingBUF_GetMsg(pLINBuf);
	pMsg->Sync = 0;
	pMsg->PID = 0;
	pMsg->DataLen = 0;
	pMsg->Checksum = 0;
}
 
 
//获取缓冲区消息长度
uint16_t LIN_RingBUF_ReadLen(LIN_BUFFER* pLINBuff)
{
	return pLINBuff->Length <= 0 ? 0 : pLINBuff->Length;
}
 
 
//读缓冲区
int LIN_RingBUF_Read(LIN_BUFFER* pLINBuff, LIN_MSG** pLINMsg)
{
	 __set_PRIMASK(1);
	if (pLINBuff->Length <= 0) {  //判断缓冲区是否为空	
		pLINBuff->BufOutcnt = 0;
		 __set_PRIMASK(0);
		return 1;
	}else {
		*pLINMsg = &pLINBuff->LINBuffer[pLINBuff->Head];   	      //先进先出FIFO，从缓冲区头出
		pLINBuff->Head = (pLINBuff->Head + 1) % LIN_RINGBUF_LEN;  //防止越界非法访问
		pLINBuff->Length--;
		 __set_PRIMASK(0);
		return 0;
	}
}
 



