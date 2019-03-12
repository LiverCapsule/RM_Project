#include "CanBusTask.h"
#include "SuperviseTask.h"
#include "BSP_CAN.h"
#include "BSP_Data.h"
#include "DriverLib_PID.h"

volatile Encoder TurntableEncoder = {0, 0, 0, 0, 0, 0, 0, 0, 0};

Measure Motor1_Measure = {0,0,0,0,0,0,0};
Measure Motor2_Measure = {0,0,0,0,0,0,0};
Measure Motor3_Measure = {0,0,0,0,0,0,0};
Measure Motor4_Measure = {0,0,0,0,0,0,0};

Measure LBeltM_Measure = {0,0,0,0,0,0,0};
Measure RBeltM_Measure = {0,0,0,0,0,0,0};
Measure	ARMM_Measure = {0,0,0,0,0,0,0};
/**
  * @brief  处理编码值,将其转换为连续的角度							//[0][1]机械角度
  * @param  msg电机由CAN回传的信息 v编码器结构体				//[2][3]实际转矩电流测量值
  * @retval void																			//[4][5]转矩电流给定值
  */
void get_measure(Measure *mea, Can_Msg *msg)//查看C620说明书
{
	mea->angle = (uint16_t)(msg->data[0] << 8 | msg->data[1]);
	mea->speed_rpm = (int16_t)(msg->data[2] << 8 | msg->data[3]);
	mea->real_current = (int16_t)((msg->data[4] << 8) | (msg->data[5]));
	mea->Temperature = msg->data[6];
	
	if((mea->angle - mea->lastangle) < -6400)
	{
		mea->round_cnt--;
	}
	else if((mea->angle - mea->lastangle) > 6400)
	{
		mea->round_cnt++;
	}
	mea->ecd_angle = mea->round_cnt *360 + (float)(mea->angle)/8192;
	
	mea->lastangle = mea->angle;
}




void EncoderProcess(volatile Encoder *v, Can_Msg *msg)
{
	int i = 0;
	int32_t temp_sum = 0;
	v->last_raw_value = v->raw_value;
	v->raw_value = (msg->data[0] << 8) | msg->data[1];//机械角度
	v->diff = v->raw_value - v->last_raw_value;
	if (v->diff < -6400) //两次编码器的反馈值差别太大，表示圈数发生了改变
	{
		v->round_cnt++;
		v->ecd_raw_rate = v->diff + 8192;
	}
	else if (v->diff > 6400)
	{
		v->round_cnt--;
		v->ecd_raw_rate = v->diff - 8192;
	}
	else
	{
		v->ecd_raw_rate = v->diff;
	}
	//计算得到连续的编码器输出值
	v->ecd_value = v->raw_value + v->round_cnt * 8192;
	//计算得到角度值，范围正负无穷大
	v->ecd_angle = (float)(v->raw_value - v->ecd_bias) * 360.0f / 8192 + v->round_cnt * 360;
	v->rate_buf[v->buf_count++] = v->ecd_raw_rate;
	if (v->buf_count == RATE_BUF_SIZE)
	{
		v->buf_count = 0;
	}	
	//计算速度平均值
	for (i = 0; i < RATE_BUF_SIZE; i++)
	{
		temp_sum += v->rate_buf[i];
	}
	v->filter_rate = (int32_t)(temp_sum / RATE_BUF_SIZE);
}


/**
  * @needs	处理并发送云台、底盘、拨盘电机电流或电压值，接收并处理电调回传的电机信息，帧率均为1k，并为其他通信留空间
  * @brief  CAN1、CAN2
  * @note   CAN总线发送:底盘电机4*M3508 C620电调,发送 data∈(-16384,16384) 0x4000	20A
  * 				0x200或     云台电机自带电调GM3510,data∈(-29000,29000) 0x7148  				云台电机标识符需用0x1FF,ID5为YAWID6为PITCH 3510ID可以是5/6/7
	*         0x1FF(各对           			 RM6623,data∈(-5000,5000)   0x1388         
  *         应四个ID的 	拨盘电机RM2006  C610电调  发送 data∈(-10000,10000) 0x2710  10A
  * 				电调)1-4 5-8
	*        
  *         CAN总线接收:			C610	  	C620 0x200+ID号		GM6623 P 0x206	Y 0x206	  GM3510	0x204+1/2/3																												
  *         data[0]data[1]  机械角度		机械角度						机械角度										机械角度								0-8191 360°
	*        	data[2]data[3]	转子转速		转子转速						实际电流		13000						输出转矩
  *         data[4]data[5]	输出转矩		实际电流						给定电流
	*         data[6]data[7]						温度
  * @param  None  朱利豪
  */

//接收不需要怎么改
//发送的形式想好了，直接把id也push进去然后pop的时候注意就行了
//由频率决定发送的时机,想好了不知道可以不。
//还要做的就是CAN2


uint32_t can_count;


void Can_Msg_Process(void)
{
	can_count++;
	CAN_Res_FrameCounter++;//可能会有点问题，暂时先加到这里
	switch(CAN1_Receive.rx_header.StdId)
	{
			case CAN_BUS1_MOTOR1_FEEDBACK_MSG_ID:
			{
				ChassisFrameCounter[0]++;

				get_measure(&Motor1_Measure, &CAN1_Receive.msg);

			}
			break;
			case CAN_BUS1_MOTOR2_FEEDBACK_MSG_ID:
			{
				ChassisFrameCounter[1]++;
				get_measure(&Motor2_Measure, &CAN1_Receive.msg);

			}
			break;
			case CAN_BUS1_MOTOR3_FEEDBACK_MSG_ID:
			{
				ChassisFrameCounter[2]++;
				get_measure(&Motor3_Measure, &CAN1_Receive.msg);

			}
			break;
			case CAN_BUS1_MOTOR4_FEEDBACK_MSG_ID:
			{
				ChassisFrameCounter[3]++;
				get_measure(&Motor4_Measure, &CAN1_Receive.msg);
			}
			break;
			case CAN_BUS1_BELTMOTOR1_FEEDBACK_MSG_ID:
			{
				RbeltFrameCounter++;

				get_measure(&RBeltM_Measure, &CAN1_Receive.msg);

			}break;
			case CAN_BUS1_BELTMOTOR2_FEEDBACK_MSG_ID://左边轮子为206
			{
				LbeltFrameCounter++;

				get_measure(&LBeltM_Measure, &CAN1_Receive.msg);

			}break;
			case CAN_BUS1_ARMMOTOR_FEEDBACK_MSG_ID:
			{
				MotorArmFrameCounter++;
				
				get_measure(&ARMM_Measure,&CAN1_Receive.msg);
				EncoderProcess(&TurntableEncoder,&CAN1_Receive.msg);
			}break;
			default:
			{
	
			}
			break;
		}
}


void Can_Send_CM(int16_t iq_1,int16_t iq_2,int16_t iq_3,int16_t iq_4)
{
	CAN1_ReadyToSend.tx_header.StdId = 0x200;
	CAN1_ReadyToSend.msg.data[0] = (unsigned char)( iq_1 >>8);
	CAN1_ReadyToSend.msg.data[1] = (unsigned char)iq_1;
	CAN1_ReadyToSend.msg.data[2] = (unsigned char)( iq_2 >>8);
	CAN1_ReadyToSend.msg.data[3] = (unsigned char)iq_2;
	CAN1_ReadyToSend.msg.data[4] = (unsigned char)( iq_3 >>8);
	CAN1_ReadyToSend.msg.data[5] = (unsigned char)iq_3;
	CAN1_ReadyToSend.msg.data[6] = (unsigned char)( iq_4 >>8);
	CAN1_ReadyToSend.msg.data[7] = (unsigned char)iq_4;
	
	CAN_bufferPush(&Que_CAN1_Tx,CAN1_ReadyToSend);
	
}

void Can_Send_BM(int16_t iq_1,int16_t iq_2,int16_t iq_3,int16_t iq_4)
{
	CAN1_ReadyToSend.tx_header.StdId = 0x1FF;
	CAN1_ReadyToSend.msg.data[0] = (unsigned char)( iq_1 >>8);
	CAN1_ReadyToSend.msg.data[1] = (unsigned char)iq_1;
	CAN1_ReadyToSend.msg.data[2] = (unsigned char)( iq_2 >>8);
	CAN1_ReadyToSend.msg.data[3] = (unsigned char)iq_2;
	CAN1_ReadyToSend.msg.data[4] = (int16_t)(AMRotatePID.output)>>8;
	CAN1_ReadyToSend.msg.data[5] = (unsigned char)(AMRotatePID.output);
	CAN1_ReadyToSend.msg.data[6] = (unsigned char)( iq_4 >>8);
	CAN1_ReadyToSend.msg.data[7] = (unsigned char)iq_4;
	
	CAN_bufferPush(&Que_CAN1_Tx,CAN1_ReadyToSend);

}

uint32_t TxMailFreeNum = 0;

void Can_Send(void)
{		




	uint16_t testlen = CAN_bufferlen(&Que_CAN1_Tx);
//如果都在一个线程里发的话等发的数据多了就会不够用了
//想不出更好方法，暂且就这样，can线程里面发一次，剩下的到中断里发
//时间触发模式好像可以，但是没必要而且数据就少了
		CAN_bufferPop(&Que_CAN1_Tx,&CAN1_ReallySend);

		HAL_CAN_AddTxMessage(&hcan1,&CAN1_ReallySend.tx_header,CAN1_ReallySend.msg.data,(uint32_t*)CAN_TX_MAILBOX0);//若0邮箱满了，就自动加到下一个
		
		CAN_Send_FrameCounter++;


		
		
	TxMailFreeNum = HAL_CAN_GetTxMailboxesFreeLevel(&hcan1);

}





/**
  * @brief CAN data sent in PMP interrupt.
  *
  * @retval None
  */

void HAL_CAN_TxMailbox0CompleteCallback(CAN_HandleTypeDef* hcan)
{
	uint16_t testlen = CAN_bufferlen(&Que_CAN1_Tx);
		if(testlen>0)//如果都在一个线程里发的话等发的数据多了就会不够用了
	{
		CAN_bufferPop(&Que_CAN1_Tx,&CAN1_ReallySend);
		HAL_CAN_AddTxMessage(&hcan1,&CAN1_ReallySend.tx_header,CAN1_ReallySend.msg.data,(uint32_t*)CAN_TX_MAILBOX0);
		
	}
}


/**
  * @brief 
  *
  * @retval None
  */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	HAL_CAN_GetRxMessage(&hcan1,CAN_RX_FIFO0,&CAN1_Receive.rx_header,CAN1_Receive.msg.data);//
	
		
	Can_Msg_Process();	

													
}








/**
* @brief 所需实现的功能：每ms接收来自
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
			
			vTaskDelayUntil(&xLastWakeTime,1/portTICK_RATE_MS);//此时处于阻塞态

	

		
		
		
		
  }
  /* USER CODE END Can_Send_Task */
}
