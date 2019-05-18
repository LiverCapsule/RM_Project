#include "CanBusTask.h"
#include "SuperviseTask.h"
#include "BSP_CAN.h"
#include "BSP_Data.h"
#include "DriverLib_PID.h"
#include "Data_MiniPC.h"


#include "Data_Judge.h"

volatile Encoder TurntableEncoder = {0, 0, 0, 0, 0, 0, 0, 0, 0};

Measure Chassis_Motor1_Measure = {0,0,0,0,0,0,0};
Measure Chassis_Motor2_Measure = {0,0,0,0,0,0,0};
Measure Chassis_Motor3_Measure = {0,0,0,0,0,0,0};
Measure Chassis_Motor4_Measure = {0,0,0,0,0,0,0};

Measure LiftChain_Motor1_Measure = {0,0,0,0,0,0,0};//1,2,3,4���˳��͵���һ�������Ͻǿ�ʼ˳ʱ��
Measure LiftChain_Motor2_Measure = {0,0,0,0,0,0,0};
Measure LiftChain_Motor3_Measure = {0,0,0,0,0,0,0};
Measure LiftChain_Motor4_Measure = {0,0,0,0,0,0,0};

Measure LiftChain_Motor5_Measure = {0,0,0,0,0,0,0};//5��6λ̧��е�۵��������
Measure LiftChain_Motor6_Measure = {0,0,0,0,0,0,0};

Measure Guide_Motor1_Measure = {0,0,0,0,0,0,0};
Measure Guide_Motor2_Measure = {0,0,0,0,0,0,0};

Measure FlipArm_Motor_Measure = {0,0,0,0,0,0,0};//7Ϊ�����ƶ���
Measure	MoveArm_Motor_Measure = {0,0,0,0,0,0,0};

//����3��
//
/**
  * @brief  �������ֵ,����ת��Ϊ�����ĽǶ�							//[0][1]��е�Ƕ�
  * @param  msg�����CAN�ش�����Ϣ v�������ṹ��				//[2][3]ʵ��ת�ص�������ֵ
  * @retval void																			//[4][5]ת�ص�������ֵ
  */
void get_measure(Measure *mea, Can_Msg *msg)//�鿴C620˵����
{
	
	mea->angle = (uint16_t)(msg->data[0] << 8 | msg->data[1]);
	mea->speed_rpm = (int16_t)(msg->data[2] << 8 | msg->data[3]);
	mea->real_current = (int16_t)((msg->data[4] << 8) | (msg->data[5]));
	mea->Temperature = msg->data[6];
	
	if((mea->angle - mea->lastangle) < -6400)
	{
		mea->round_cnt++;
	}
	else if((mea->angle - mea->lastangle) > 6400)
	{
		mea->round_cnt--;
	}
	mea->ecd_angle = mea->round_cnt *360 + (float)(mea->angle - mea->initial_angle)*360/8192;
	
	mea->lastangle = mea->angle;
}





uint32_t can_count;









void CAN1_Msg_Process(void)
{
	can_count++;
	CAN_Res_FrameCounter++;//���ܻ��е����⣬��ʱ�ȼӵ�����
	switch(CAN1_Receive.rx_header.StdId)
	{
			case CAN_BUS1_LIFTCHAINMOTOR1_FEEDBACK_MSG_ID:
			{
				ChassisFrameCounter[0]++;
				if(can_count < 200)
				{
					LiftChain_Motor1_Measure.initial_angle = LiftChain_Motor1_Measure.angle;
  			}
				get_measure(&LiftChain_Motor1_Measure, &CAN1_Receive.msg);

			}
			break;
			case CAN_BUS1_LIFTCHAINMOTOR2_FEEDBACK_MSG_ID:
			{
				ChassisFrameCounter[1]++;
				if(can_count < 200)
				{
					LiftChain_Motor2_Measure.initial_angle = LiftChain_Motor2_Measure.angle;
				}
				get_measure(&LiftChain_Motor2_Measure, &CAN1_Receive.msg);

			}
			break;

			case CAN_BUS1_GUIDEMOTOR1_FEEDBACK_MSG_ID:
			{
				get_measure(&Guide_Motor1_Measure, &CAN1_Receive.msg);

			}break;
			case CAN_BUS1_GUIDEMOTOR2_FEEDBACK_MSG_ID:
			{
				get_measure(&Guide_Motor2_Measure, &CAN1_Receive.msg);

			}break;
			
			default:
			{
	
			}
			break;
		}
}

extern uint16_t CAN_RS_FrameCounter;

void CAN2_Msg_Process(void)
{
	can_count++;
//	CAN2_Res_FrameCounter++;//���ܻ��е����⣬��ʱ�ȼӵ�����
	switch(CAN2_Receive.rx_header.StdId)
	{
			case CAN_BUS2_CHASSISMOTOR1_FEEDBACK_MSG_ID:
			{
				CAN_RS_FrameCounter++;
				ChassisFrameCounter[0]++;
				get_measure(&Chassis_Motor1_Measure, &CAN2_Receive.msg);
			}
			break;
			case CAN_BUS2_CHASSISMOTOR2_FEEDBACK_MSG_ID:
			{
				ChassisFrameCounter[1]++;
				get_measure(&Chassis_Motor2_Measure, &CAN2_Receive.msg);
			}
			break;
			case CAN_BUS2_CHASSISMOTOR3_FEEDBACK_MSG_ID:
			{
				ChassisFrameCounter[2]++;
				get_measure(&Chassis_Motor3_Measure, &CAN2_Receive.msg);
			}
			break;
			case CAN_BUS2_CHASSISMOTOR4_FEEDBACK_MSG_ID:
			{
				ChassisFrameCounter[3]++;
				get_measure(&Chassis_Motor4_Measure, &CAN2_Receive.msg);
			}
			break;
			
			case CAN_BUS2_LIFTCHAINMOTOR3_FEEDBACK_MSG_ID:
			{
				if(can_count < 200)
				{
					LiftChain_Motor5_Measure.initial_angle = LiftChain_Motor5_Measure.angle;
				}
				get_measure(&LiftChain_Motor5_Measure, &CAN2_Receive.msg);
			}
			break;
			
			case CAN_BUS2_LIFTCHAINMOTOR4_FEEDBACK_MSG_ID:
			{
				if(can_count < 200)
				{
					LiftChain_Motor6_Measure.initial_angle = LiftChain_Motor6_Measure.angle;
				}
				get_measure(&LiftChain_Motor6_Measure, &CAN2_Receive.msg);
			}
			break;
			
			case CAN_BUS2_FLIPARMMOTOR_FEEDBACK_MSG_ID:
			{
				if(can_count < 200)
				{
					FlipArm_Motor_Measure.initial_angle = FlipArm_Motor_Measure.angle;
				}
				get_measure(&FlipArm_Motor_Measure, &CAN2_Receive.msg);

			}
			break;
			
			case CAN_BUS2_MOVEARMMOTOR_FEEDBACK_MSG_ID:
			{
				if(can_count < 200)
				{
					MoveArm_Motor_Measure.initial_angle = MoveArm_Motor_Measure.angle;
				}
				get_measure(&MoveArm_Motor_Measure, &CAN2_Receive.msg);
			}
			break;
			default:
			{
	
			}
			break;
		}
}






void CAN2_Send_LM(int16_t iq_1,int16_t iq_2,int16_t iq_3,int16_t iq_4)//�����е�۸õ���can2��1 2 3 4
{
	CAN2_ReadyToSend.tx_header.StdId = 0x1ff;
	CAN2_ReadyToSend.msg.data[0] = (unsigned char)( iq_1 >>8);
	CAN2_ReadyToSend.msg.data[1] = (unsigned char)iq_1;
	CAN2_ReadyToSend.msg.data[2] = (unsigned char)( iq_2 >>8);
	CAN2_ReadyToSend.msg.data[3] = (unsigned char)iq_2;
	CAN2_ReadyToSend.msg.data[4] = (unsigned char)( iq_3 >>8);
	CAN2_ReadyToSend.msg.data[5] = (unsigned char)iq_3;
	CAN2_ReadyToSend.msg.data[6] = (unsigned char)( iq_4 >>8);
	CAN2_ReadyToSend.msg.data[7] = (unsigned char)iq_4;
	
	CAN_bufferPush(&Que_CAN2_Tx,CAN2_ReadyToSend);
}

void CAN2_Send_CM(int16_t iq_1,int16_t iq_2,int16_t iq_3,int16_t iq_4)
{
	CAN2_ReadyToSend.tx_header.StdId = 0x200;
	CAN2_ReadyToSend.msg.data[0] = (unsigned char)( iq_1 >>8);
	CAN2_ReadyToSend.msg.data[1] = (unsigned char)iq_1;
	CAN2_ReadyToSend.msg.data[2] = (unsigned char)( iq_2 >>8);
	CAN2_ReadyToSend.msg.data[3] = (unsigned char)iq_2;
	CAN2_ReadyToSend.msg.data[4] = (unsigned char)( iq_3 >>8);
	CAN2_ReadyToSend.msg.data[5] = (unsigned char)iq_3;
	CAN2_ReadyToSend.msg.data[6] = (unsigned char)( iq_4 >>8);
	CAN2_ReadyToSend.msg.data[7] = (unsigned char)iq_4;
	
	CAN_bufferPush(&Que_CAN2_Tx,CAN2_ReadyToSend);
	
}

void CAN1_Send_LM(int16_t iq_1,int16_t iq_2)//,int16_t iq_3,int16_t iq_4)//���������ĵ���can1��1 2
{
	CAN1_ReadyToSend.tx_header.StdId = 0x200;
	CAN1_ReadyToSend.msg.data[0] = (unsigned char)( iq_1 >>8);
	CAN1_ReadyToSend.msg.data[1] = (unsigned char)iq_1;
	CAN1_ReadyToSend.msg.data[2] = (unsigned char)( iq_2 >>8);
	CAN1_ReadyToSend.msg.data[3] = (unsigned char)iq_2;
//	CAN1_ReadyToSend.msg.data[4] = (unsigned char)( iq_3 >>8);
//	CAN1_ReadyToSend.msg.data[5] = (unsigned char)iq_3;
//	CAN1_ReadyToSend.msg.data[6] = (unsigned char)( iq_4 >>8);
//	CAN1_ReadyToSend.msg.data[7] = (unsigned char)iq_4;
	
	CAN_bufferPush(&Que_CAN1_Tx,CAN1_ReadyToSend);

}



void CAN1_Send_GM(int16_t iq_1,int16_t iq_2)
{
	CAN1_ReadyToSend.tx_header.StdId = 0x1FF;
	CAN1_ReadyToSend.msg.data[0] = (unsigned char)( iq_1 >>8);
	CAN1_ReadyToSend.msg.data[1] = (unsigned char)iq_1;
	CAN1_ReadyToSend.msg.data[2] = (unsigned char)( iq_2 >>8);
	CAN1_ReadyToSend.msg.data[3] = (unsigned char)iq_2;
	
	CAN_bufferPush(&Que_CAN1_Tx,CAN1_ReadyToSend);

}







uint32_t TxMailFreeNum = 0;






void Can_Send(void)
{		

	//��ʱ���ƶ����ȷ���
	Info_Rc_MiniPC();
	uint16_t testlen = CAN_bufferlen(&Que_CAN1_Tx);
	uint16_t testlen2 = CAN_bufferlen(&Que_CAN2_Tx);
//�������һ���߳��﷢�Ļ��ȷ������ݶ��˾ͻ᲻������
//�벻�����÷��������Ҿ�������can�߳����淢һ�Σ�ʣ�µĵ��ж��﷢
//ʱ�䴥��ģʽ������ԣ�����û��Ҫ�������ݾ�����
		CAN_bufferPop(&Que_CAN1_Tx,&CAN1_ReallySend);
		CAN_bufferPop(&Que_CAN2_Tx,&CAN2_ReallySend);


		HAL_CAN_AddTxMessage(&hcan1,&CAN1_ReallySend.tx_header,CAN1_ReallySend.msg.data,(uint32_t*)CAN_TX_MAILBOX0);//��0�������ˣ����Զ��ӵ���һ��
		HAL_CAN_AddTxMessage(&hcan2,&CAN2_ReallySend.tx_header,CAN2_ReallySend.msg.data,(uint32_t*)CAN_TX_MAILBOX0);//��0�������ˣ����Զ��ӵ���һ��

		CAN_Send_FrameCounter++;


		
		
	TxMailFreeNum = HAL_CAN_GetTxMailboxesFreeLevel(&hcan2);

}





/**
  * @brief CAN data sent in PMP interrupt.
  *
  * @retval None
  */

void HAL_CAN_TxMailbox0CompleteCallback(CAN_HandleTypeDef* hcan)
{
	if(hcan->Instance == CAN1)
	{
		uint16_t testlen = CAN_bufferlen(&Que_CAN1_Tx);
		if(testlen>0)//�������һ���߳��﷢�Ļ��ȷ������ݶ��˾ͻ᲻������
		{
			CAN_bufferPop(&Que_CAN1_Tx,&CAN1_ReallySend);
			HAL_CAN_AddTxMessage(&hcan1,&CAN1_ReallySend.tx_header,CAN1_ReallySend.msg.data,(uint32_t*)CAN_TX_MAILBOX0);
			
		}
	}
	if(hcan->Instance == CAN2)
	{
		uint16_t testlen = CAN_bufferlen(&Que_CAN2_Tx);
		if(testlen>0)//�������һ���߳��﷢�Ļ��ȷ������ݶ��˾ͻ᲻������
		{
			CAN_bufferPop(&Que_CAN2_Tx,&CAN2_ReallySend);
			HAL_CAN_AddTxMessage(&hcan2,&CAN2_ReallySend.tx_header,CAN2_ReallySend.msg.data,(uint32_t*)CAN_TX_MAILBOX0);
			
		}
	}
	
}


/**
  * @brief 
  *
  * @retval None
  */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	if(hcan->Instance == CAN1)
	{
		HAL_CAN_GetRxMessage(&hcan1,CAN_RX_FIFO0,&CAN1_Receive.rx_header,CAN1_Receive.msg.data);//
		CAN1_Msg_Process();	
	}
	else if(hcan->Instance == CAN2)
	{
		HAL_CAN_GetRxMessage(&hcan2,CAN_RX_FIFO0,&CAN2_Receive.rx_header,CAN2_Receive.msg.data);//����fifo��֪�ǲ���Ҫ��
		CAN2_Msg_Process();	
	}
													
}








/**
* @brief ����ʵ�ֵĹ��ܣ�ÿms��������
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Can_Send_Task */
uint16_t aa = 0;
uint16_t bb = 0;
void Can_Send_Task(void const * argument)
{

  /* USER CODE BEGIN Can_Send_Task */
	
	portTickType xLastWakeTime;
	xLastWakeTime = xTaskGetTickCount();

  /* Infinite loop */
  for(;;)
  {

			Can_Send();
			Info_Rc_Judge();
			Info_Sd_Judge();
			Info_Rc_MiniPC();//�ȷ�������
			vTaskDelayUntil(&xLastWakeTime,1/portTICK_RATE_MS);//��ʱ��������̬

		
  }
  /* USER CODE END Can_Send_Task */
}
