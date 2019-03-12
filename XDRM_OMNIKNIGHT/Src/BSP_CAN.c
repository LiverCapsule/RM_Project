/**
  ******************************************************************************
  * File Name          : CAN.c
  * Description        : This file provides code for the configuration
  *                      of the CAN instances.
  ******************************************************************************
  * This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * Copyright (c) 2018 STMicroelectronics International N.V. 
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "BSP_CAN.h"


#include "BSP_GPIO.h"


/* USER CODE BEGIN 0 */


CanRxMsgTypeDef CAN1_Receive;
	
CanTxMsgTypeDef CAN1_ReadyToSend;
CanTxMsgTypeDef CAN1_ReallySend;



CAN_FilterTypeDef CAN1_Filter;
/* USER CODE END 0 */


CAN_HandleTypeDef hcan1;

/* CAN1 init function */
void MX_CAN1_Init(void)//本来一直也是初始化有问题，之后烧了一下之前代码又可以了。有问题的//st-link拔了之后又有问题了嘿嘿
{

  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 3;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_9TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_4TQ;
  hcan1.Init.TimeTriggeredMode = DISABLE;//非时间触发通信模式//不生成接收发送邮箱时间戳值
  hcan1.Init.AutoBusOff = ENABLE;//软件自动离线管理//一旦检测到128次11个连续隐性位，硬件自动总线关闭/////
  hcan1.Init.AutoWakeUp = DISABLE;//自动唤醒模式//软件通过can_mcr的sleep位清零发出请求后，进入睡眠
  hcan1.Init.AutoRetransmission = ENABLE;//是否禁止自动重发送//无论结果如何，只发送一次//另一种是直到标准消息发送成功
  hcan1.Init.ReceiveFifoLocked = DISABLE;//接受FIFO锁定模式//FIFO装满后，下一条覆盖前一条消息
  hcan1.Init.TransmitFifoPriority = DISABLE;//发送FIFO优先级//由消息标识符决定
	
  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }
	  HAL_CAN_Start(&hcan1);
//	Can1_Filter.SlaveStartFilterBank = ;一个can就不用管
	CAN1_Filter.FilterActivation = ENABLE;
	CAN1_Filter.FilterBank = 0;
	CAN1_Filter.FilterFIFOAssignment = CAN_FilterFIFO0;
	CAN1_Filter.FilterIdHigh = 0;
	CAN1_Filter.FilterIdLow = 0;
	CAN1_Filter.FilterMode = CAN_FILTERMODE_IDMASK;//标识符掩码，32位掩码32位标识符//另一种模式就可以是两个32位标识符。
	CAN1_Filter.FilterMaskIdHigh = 0;
	CAN1_Filter.FilterMaskIdLow = 0;
	CAN1_Filter.FilterScale = CAN_FILTERSCALE_32BIT;

	CAN1_ReadyToSend.tx_header.DLC = 0x08;
	CAN1_ReadyToSend.tx_header.IDE = CAN_ID_STD;
	CAN1_ReadyToSend.tx_header.RTR = CAN_RTR_DATA;
	CAN1_ReallySend.tx_header.DLC = 0x08;
	CAN1_ReallySend.tx_header.IDE = CAN_ID_STD;
	CAN1_ReallySend.tx_header.RTR = CAN_RTR_DATA;
	
	CAN1_Receive.rx_header.DLC = 0x08;//这个都一样就放这里吧。
	CAN1_Receive.rx_header.RTR = CAN_RTR_DATA;
	CAN1_Receive.rx_header.IDE = CAN_ID_STD;
	
	HAL_CAN_ConfigFilter(&hcan1,&CAN1_Filter);//这里写错，收不到。。

  HAL_CAN_ActivateNotification(&hcan1,CAN_IT_RX_FIFO0_MSG_PENDING);	//ojbk了// 感觉不是，是有挂起中断，但无挂起clear,是因为FMP不在被认为是一个flag位，是一个计数的，硬件自动加减   //首先can的挂起是相对于接收邮箱的
	HAL_CAN_ActivateNotification(&hcan1,CAN_IT_TX_MAILBOX_EMPTY);
	
				
}

void HAL_CAN_MspInit(CAN_HandleTypeDef* canHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct;
  if(canHandle->Instance==CAN1)
  {
  /* USER CODE BEGIN CAN1_MspInit 0 */

  /* USER CODE END CAN1_MspInit 0 */
    /* CAN1 clock enable */
    __HAL_RCC_CAN1_CLK_ENABLE();
  
    /**CAN1 GPIO Configuration    
    PB8     ------> CAN1_RX
    PB9     ------> CAN1_TX 
    */
    GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_0;//每个can口对应不同的引脚！差哪个头就用哪个引脚
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF9_CAN1;
    HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /* USER CODE BEGIN CAN1_MspInit 1 */

  /* USER CODE END CAN1_MspInit 1 */
  }
}

void HAL_CAN_MspDeInit(CAN_HandleTypeDef* canHandle)
{

  if(canHandle->Instance==CAN1)
  {
  /* USER CODE BEGIN CAN1_MspDeInit 0 */

  /* USER CODE END CAN1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_CAN1_CLK_DISABLE();
  
    /**CAN1 GPIO Configuration    
    PB8     ------> CAN1_RX
    PB9     ------> CAN1_TX 
    */
    HAL_GPIO_DeInit(GPIOD, GPIO_PIN_1|GPIO_PIN_0);

    /* CAN1 interrupt Deinit */
    HAL_NVIC_DisableIRQ(CAN1_TX_IRQn);
    HAL_NVIC_DisableIRQ(CAN1_RX0_IRQn);
  /* USER CODE BEGIN CAN1_MspDeInit 1 */

  /* USER CODE END CAN1_MspDeInit 1 */
  }
} 

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
