#include "lin_driver.h"  
#include "stdio.h"
#include "linbuffer.h"
 
LIN_RxState       LIN_RxStateGet = BREAK_GET; 
LIN_ERROR_Code    ErrCode = FIFO_VOID;
uint8_t           data[8];
 
 
//LIN接收缓冲区
LIN_BUFFER  LIN_RxDataBuff = {
	.Head = 0,
	.Tail = 0,
	.Length = 0,
	.BufOutcnt = 0	
};
 
 
 
/**
  * @brief  LIN从机接收处理函数
	* @param  Data: 串口消息
  * @retval None
  */
void LIN_MasterRxMsg(uint8_t Data)
{
	LIN_MSG* pLINMsg = LIN_RingBUF_GetMsg(&LIN_RxDataBuff);
	
	switch (LIN_RxStateGet){
		case BREAK_GET:      //同步间隔段
		{
		}
		break;
		
		case SYNCH_GET:      //同步段
		{
			if(Data != 0x55){  //判断是不是同步段
				ErrCode = SYNC_ERR;
				LIN_RxStateGet = BREAK_GET;	
			}else{
				pLINMsg->Sync = Data;
				LIN_RxStateGet = PID_GET;	
			}	
		}
		break;	
		
		case PID_GET:        //PID段
		{
			pLINMsg->FrameID = Data&0x3F;
			pLINMsg->PID = Data;
			uint8_t PID = LIN_GetPID(pLINMsg->FrameID);  //根据ID获取奇偶校验位 得到校验的PID
			
			if(PID == pLINMsg->PID){                     //判断PID是否正确  后续根据LDF定
				//根据判断是执行还是反馈 改变标志位
				if(pLINMsg->FrameID == 0x2D){              // 1 -- 执行  即继续接收数据
					LIN_RxStateGet = MSG_GET;	
				}else if(pLINMsg->FrameID == 0x2E){        // 2 -- 反馈  即向LIN总线发送消息
					LIN_Rx_data(pLINMsg->PID,data,sizeof(data)/sizeof(data[0]));   //反馈消息
					LIN_RxStateGet = BREAK_GET;
				}else{                                     // 3 -- 其他  即不执行也不反馈                       
					LIN_RingBUF_ClearRxMsg(&LIN_RxDataBuff); //清空当前缓冲区
					LIN_RxStateGet = BREAK_GET;
				}
			}else{   //PID校验不正确
				ErrCode = PID_ERR;
				LIN_RingBUF_ClearRxMsg(&LIN_RxDataBuff);   //清空当前缓冲区
				LIN_RxStateGet = BREAK_GET;				
			}												
		}
		break;	
		
		case MSG_GET:       //数据段
		{
			pLINMsg->Data[pLINMsg->DataLen++] = Data;
			pLINMsg->Checksum = Data;
			LIN_RxStateGet = (pLINMsg->DataLen>=8)?CHECKSUM_GET:MSG_GET;	
		}
		break;	
		
		case CHECKSUM_GET:  //校验和段
		{
			pLINMsg->Checksum = Data;
			uint8_t Checksum = LIN_GetChecksum(pLINMsg->PID,pLINMsg->Data,pLINMsg->DataLen,1);     //获取校验和段
			if((Checksum+pLINMsg->Checksum) == 0xFF){            //判断校验和是否正确
				LIN_RingBUF_Write(&LIN_RxDataBuff);
			}else{
				ErrCode = FORMAT_ERR;
				LIN_RingBUF_ClearRxMsg(&LIN_RxDataBuff);           //清空当前缓冲区
			} 
			LIN_RxStateGet = BREAK_GET;		
		}
		break;
		
		default:       			//其他
			LIN_RxStateGet = BREAK_GET;	
		break;
	}
}
 
 
/**
  * @brief  反馈消息给总线	
  * @param  PID：校验ID，pData：数据指针，Length：数据长度
  * @retval 无
  */
void LIN_Rx_data(uint8_t PID, uint8_t* pData,uint8_t DataLen)
{
	uint8_t Linbuffer[DataLen+1]; 	                             //定义发送数组(数据+校验和)
	uint8_t Checksum = LIN_GetChecksum(PID,pData,DataLen,0);     //获取校验和段
	for (uint8_t i = 0; i < DataLen; i++)                        //存DataLen个字节数据段
	{     
		Linbuffer[i] = *(pData + i);		
	}
	Linbuffer[DataLen] = Checksum;                   //校验和
	LIN_SendBytes(USART3, Linbuffer ,DataLen+1);     //发送从机数据
}
 
 
/**
  * @brief  LIN同步间隔段发送		
  * @param  USARTx：串口号
  * @retval 无
  */
void LIN_SendBreak(void)
{
	HAL_LIN_SendBreak(&huart3);
}
 
 
/**
  * @brief  LIN发送字节	
  * @param  USARTx：串口号，pData：数据指针，Length：数据长度
  * @retval 无
  */
void LIN_SendBytes(USART_TypeDef* USARTx,uint8_t* pData,uint8_t DataLen)
{
	HAL_UART_Transmit(&huart3,pData,DataLen,0xffff);
}
 
 
/**
  * @brief  LIN协议规定校验和长度为1个字节，获取校验和	
	* @param  PID：校验ID，pData：数据指针，DataLen：数据长度，flag ：发送0 接收1 
  * @retval 累加校验和
  */
uint8_t LIN_GetChecksum(uint8_t PID, uint8_t* pData,uint8_t DataLen,uint8_t flag) 			 
{  
	  uint16_t CheckSum = 0;  
    //FrameID为3C 3D的PID为3C 7D
	  if((PID!=0x3C)&&(PID!=0x7D)){    //诊断帧只能使用标准校验和，标准校验和不包含PID 只校验数据段              	  
			CheckSum = PID;     		
	  }
	  for(uint8_t i = 0; i < DataLen; i++){
			CheckSum += pData[i];		  
			if (CheckSum > 0xFF){
				CheckSum -= 0xFF;  
			}
	  }
		
		if(flag == 0){
			return (~CheckSum) & 0xFF;  //发送方需要取反
		}else{
			return CheckSum & 0xFF; 		//接收方不需要
		}
}
 
 
/**
  * @brief LIN_PID校验函数		
  * @param ID(FrameID)：帧ID(0 ~ 63) 
  * P0(bit6) =   ID0 ^ ID1 ^ ID2 ^ ID4   <==>  (偶校验：操作数中1的个数为偶数，校验位为0，1的个数为奇数校验位为1)
  * P1(bit7) = ~(ID1 ^ ID3 ^ ID4 ^ ID5)  <==>  (奇校验：操作数中1的个数为奇数，校验位为0，1的个数为偶数校验位为1)
  * @retval 返回PID
  */
uint8_t LIN_GetPID(uint8_t ID)  
{
	uint8_t PID = 0,P0 = 0,P1 = 0;	
	P0 = (((ID>>0)^(ID>>1)^(ID>>2)^(ID>>4))&0x01)<<6; //偶校验位           			
	P1 = ((~((ID>>1)^(ID>>3)^(ID>>4)^(ID>>5)))&0x01)<<7; //奇校验位	
	PID = (ID|P0|P1);	
	return PID;   
}
