#include "linbuffer.h"
#include <string.h>
 
 
//��������ʼ��
void LIN_RingBUF_Init(LIN_BUFFER* pLINBuff)
{
	pLINBuff->Head = 0;
	pLINBuff->Tail = 0;
	pLINBuff->Length = 0;
	pLINBuff->BufOutcnt = 0;
}
 
 
//��ȡLIN��Ϣ
LIN_MSG* LIN_RingBUF_GetMsg(LIN_BUFFER* pLINBuff)
{
	return  &pLINBuff->LINBuffer[pLINBuff->Tail];    //ȡ���������ڵ�����
}
 
 
//д������
void LIN_RingBUF_Write(LIN_BUFFER* pLINBuff)
{
	pLINBuff->Tail = (pLINBuff->Tail + 1) % LIN_RINGBUF_LEN; //��ֹԽ��Ƿ�����
	if (pLINBuff->Length < LIN_RINGBUF_LEN) {                //�жϻ������Ƿ�����
		pLINBuff->Length++;
	}else{
		pLINBuff->BufOutcnt++;
		if (pLINBuff->BufOutcnt == LIN_RINGBUF_LEN) {
			pLINBuff->BufOutcnt = 0;			
		}
		pLINBuff->Head = pLINBuff->BufOutcnt;
	}
}
 
//��յ�ǰ������
void LIN_RingBUF_ClearRxMsg(LIN_BUFFER* pLINBuf)
{
	LIN_MSG* pMsg = LIN_RingBUF_GetMsg(pLINBuf);
	pMsg->Sync = 0;
	pMsg->PID = 0;
	pMsg->DataLen = 0;
	pMsg->Checksum = 0;
}
 
 
//��ȡ��������Ϣ����
uint16_t LIN_RingBUF_ReadLen(LIN_BUFFER* pLINBuff)
{
	return pLINBuff->Length <= 0 ? 0 : pLINBuff->Length;
}
 
 
//��������
int LIN_RingBUF_Read(LIN_BUFFER* pLINBuff, LIN_MSG** pLINMsg)
{
	 __set_PRIMASK(1);
	if (pLINBuff->Length <= 0) {  //�жϻ������Ƿ�Ϊ��	
		pLINBuff->BufOutcnt = 0;
		 __set_PRIMASK(0);
		return 1;
	}else {
		*pLINMsg = &pLINBuff->LINBuffer[pLINBuff->Head];   	      //�Ƚ��ȳ�FIFO���ӻ�����ͷ��
		pLINBuff->Head = (pLINBuff->Head + 1) % LIN_RINGBUF_LEN;  //��ֹԽ��Ƿ�����
		pLINBuff->Length--;
		 __set_PRIMASK(0);
		return 0;
	}
}
 



