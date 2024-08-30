#include "lin_driver.h"  
#include "stdio.h"
#include "linbuffer.h"
 
LIN_RxState       LIN_RxStateGet = BREAK_GET; 
LIN_ERROR_Code    ErrCode = FIFO_VOID;
uint8_t           data[8];
 
 
//LIN���ջ�����
LIN_BUFFER  LIN_RxDataBuff = {
	.Head = 0,
	.Tail = 0,
	.Length = 0,
	.BufOutcnt = 0	
};
 
 
 
/**
  * @brief  LIN�ӻ����մ�����
	* @param  Data: ������Ϣ
  * @retval None
  */
void LIN_MasterRxMsg(uint8_t Data)
{
	LIN_MSG* pLINMsg = LIN_RingBUF_GetMsg(&LIN_RxDataBuff);
	
	switch (LIN_RxStateGet){
		case BREAK_GET:      //ͬ�������
		{
		}
		break;
		
		case SYNCH_GET:      //ͬ����
		{
			if(Data != 0x55){  //�ж��ǲ���ͬ����
				ErrCode = SYNC_ERR;
				LIN_RxStateGet = BREAK_GET;	
			}else{
				pLINMsg->Sync = Data;
				LIN_RxStateGet = PID_GET;	
			}	
		}
		break;	
		
		case PID_GET:        //PID��
		{
			pLINMsg->FrameID = Data&0x3F;
			pLINMsg->PID = Data;
			uint8_t PID = LIN_GetPID(pLINMsg->FrameID);  //����ID��ȡ��żУ��λ �õ�У���PID
			
			if(PID == pLINMsg->PID){                     //�ж�PID�Ƿ���ȷ  ��������LDF��
				//�����ж���ִ�л��Ƿ��� �ı��־λ
				if(pLINMsg->FrameID == 0x2D){              // 1 -- ִ��  ��������������
					LIN_RxStateGet = MSG_GET;	
				}else if(pLINMsg->FrameID == 0x2E){        // 2 -- ����  ����LIN���߷�����Ϣ
					LIN_Rx_data(pLINMsg->PID,data,sizeof(data)/sizeof(data[0]));   //������Ϣ
					LIN_RxStateGet = BREAK_GET;
				}else{                                     // 3 -- ����  ����ִ��Ҳ������                       
					LIN_RingBUF_ClearRxMsg(&LIN_RxDataBuff); //��յ�ǰ������
					LIN_RxStateGet = BREAK_GET;
				}
			}else{   //PIDУ�鲻��ȷ
				ErrCode = PID_ERR;
				LIN_RingBUF_ClearRxMsg(&LIN_RxDataBuff);   //��յ�ǰ������
				LIN_RxStateGet = BREAK_GET;				
			}												
		}
		break;	
		
		case MSG_GET:       //���ݶ�
		{
			pLINMsg->Data[pLINMsg->DataLen++] = Data;
			pLINMsg->Checksum = Data;
			LIN_RxStateGet = (pLINMsg->DataLen>=8)?CHECKSUM_GET:MSG_GET;	
		}
		break;	
		
		case CHECKSUM_GET:  //У��Ͷ�
		{
			pLINMsg->Checksum = Data;
			uint8_t Checksum = LIN_GetChecksum(pLINMsg->PID,pLINMsg->Data,pLINMsg->DataLen,1);     //��ȡУ��Ͷ�
			if((Checksum+pLINMsg->Checksum) == 0xFF){            //�ж�У����Ƿ���ȷ
				LIN_RingBUF_Write(&LIN_RxDataBuff);
			}else{
				ErrCode = FORMAT_ERR;
				LIN_RingBUF_ClearRxMsg(&LIN_RxDataBuff);           //��յ�ǰ������
			} 
			LIN_RxStateGet = BREAK_GET;		
		}
		break;
		
		default:       			//����
			LIN_RxStateGet = BREAK_GET;	
		break;
	}
}
 
 
/**
  * @brief  ������Ϣ������	
  * @param  PID��У��ID��pData������ָ�룬Length�����ݳ���
  * @retval ��
  */
void LIN_Rx_data(uint8_t PID, uint8_t* pData,uint8_t DataLen)
{
	uint8_t Linbuffer[DataLen+1]; 	                             //���巢������(����+У���)
	uint8_t Checksum = LIN_GetChecksum(PID,pData,DataLen,0);     //��ȡУ��Ͷ�
	for (uint8_t i = 0; i < DataLen; i++)                        //��DataLen���ֽ����ݶ�
	{     
		Linbuffer[i] = *(pData + i);		
	}
	Linbuffer[DataLen] = Checksum;                   //У���
	LIN_SendBytes(USART3, Linbuffer ,DataLen+1);     //���ʹӻ�����
}
 
 
/**
  * @brief  LINͬ������η���		
  * @param  USARTx�����ں�
  * @retval ��
  */
void LIN_SendBreak(void)
{
	HAL_LIN_SendBreak(&huart3);
}
 
 
/**
  * @brief  LIN�����ֽ�	
  * @param  USARTx�����ںţ�pData������ָ�룬Length�����ݳ���
  * @retval ��
  */
void LIN_SendBytes(USART_TypeDef* USARTx,uint8_t* pData,uint8_t DataLen)
{
	HAL_UART_Transmit(&huart3,pData,DataLen,0xffff);
}
 
 
/**
  * @brief  LINЭ��涨У��ͳ���Ϊ1���ֽڣ���ȡУ���	
	* @param  PID��У��ID��pData������ָ�룬DataLen�����ݳ��ȣ�flag ������0 ����1 
  * @retval �ۼ�У���
  */
uint8_t LIN_GetChecksum(uint8_t PID, uint8_t* pData,uint8_t DataLen,uint8_t flag) 			 
{  
	  uint16_t CheckSum = 0;  
    //FrameIDΪ3C 3D��PIDΪ3C 7D
	  if((PID!=0x3C)&&(PID!=0x7D)){    //���ֻ֡��ʹ�ñ�׼У��ͣ���׼У��Ͳ�����PID ֻУ�����ݶ�              	  
			CheckSum = PID;     		
	  }
	  for(uint8_t i = 0; i < DataLen; i++){
			CheckSum += pData[i];		  
			if (CheckSum > 0xFF){
				CheckSum -= 0xFF;  
			}
	  }
		
		if(flag == 0){
			return (~CheckSum) & 0xFF;  //���ͷ���Ҫȡ��
		}else{
			return CheckSum & 0xFF; 		//���շ�����Ҫ
		}
}
 
 
/**
  * @brief LIN_PIDУ�麯��		
  * @param ID(FrameID)��֡ID(0 ~ 63) 
  * P0(bit6) =   ID0 ^ ID1 ^ ID2 ^ ID4   <==>  (żУ�飺��������1�ĸ���Ϊż����У��λΪ0��1�ĸ���Ϊ����У��λΪ1)
  * P1(bit7) = ~(ID1 ^ ID3 ^ ID4 ^ ID5)  <==>  (��У�飺��������1�ĸ���Ϊ������У��λΪ0��1�ĸ���Ϊż��У��λΪ1)
  * @retval ����PID
  */
uint8_t LIN_GetPID(uint8_t ID)  
{
	uint8_t PID = 0,P0 = 0,P1 = 0;	
	P0 = (((ID>>0)^(ID>>1)^(ID>>2)^(ID>>4))&0x01)<<6; //żУ��λ           			
	P1 = ((~((ID>>1)^(ID>>3)^(ID>>4)^(ID>>5)))&0x01)<<7; //��У��λ	
	PID = (ID|P0|P1);	
	return PID;   
}
